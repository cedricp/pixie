//
// ssdiffusion.cpp
//
// Copyright (c) 2004-2007 Pixar Animation Studios.
//
// The information in this file is provided for the exclusive use of the
// licensees of Pixar.
//
// Pixar DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
// INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN
// NO EVENT SHALL Pixar BE LIABLE FOR ANY SPECIAL, INDIRECT OR
// CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS
// OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
// OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE
// USE OR PERFORMANCE OF THIS SOFTWARE.
//
// Pixar Animation Studios
// 1200 Park Avenue
// Emeryville, CA 94608
//

//
// This file takes as input a point cloud of data points, each point
// having a position, normal, radius, and transmitted radiance, and
// optionally an albedo and diffuse mean free path length.  It
// computes a simulation of multiple subsurface scattering using a
// diffusion approximation (with an octree for efficiency).  The
// result is a new point cloud where each point has a position,
// normal, radius, and subsurface diffusion color.
//
// This implementation is inspired by the following three articles:
//
//   Henrik Wann Jensen, Steve Marschner, Marc Levoy, and Pat Hanrahan.
//   "A Practical Model for Subsurface Light Transport".
//   Proceedings of SIGGRAPH 2001, pages 511-518.  ACM, August 2001.
//
//   Henrik Wann Jensen and Juan Buhler.
//   "A Rapid Hierarchical Rendering Technique for Translucent Materials".
//   Proceedings of SIGGRAPH 2002, pages 576-581.  ACM, July 2002.
//
//   Christophe Hery.
//   "Implementing a Skin BSSRDF".
//   In "RenderMan, Theory and Practice",
//   SIGGRAPH 2003 Course Note #9, pages 73-88.  ACM, July 2003.
//

// Multithread support implemented at DWARF LABS 2010 Cedric PAILLE
// Original source code adapted for Pixie at DWARF LABS 2010 Cedric PAILLE

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <ptcapi.h>
#include <string.h>

#include <pthread.h>
#include <sys/sysinfo.h>

#include "ptcommon.h"


#define MAX_NUMTREAD 64
#define MAXPTSINNODE 8

pthread_mutex_t progresslock;


typedef struct MSARGS{
    struct  color alpha_prime; // reduced scattering albedo
    struct  color sigma_tr; // effective transport extinction coeffient
    struct  color l_u; //mean free path length: avg dist at which light scatters
    struct  color z_r, z_v; // height of "real" and virtual dipole light sources
    float   eta; // relative index of refraction, ior / 1
    float   F_dr, F_dt; // diffuse Fresnel reflection and transmission terms
    float   A;
    float   factor;
    int     paramFromFile,albedoFromFile;
    struct  color *Rd;
    int     dmfpFromFile;
    struct color *l_d;
    float   unitlength;
    float   ior;
    float   maxsolidangle;
    int     printProgress;
    int     threadnum,numthread;
} msargs;


static const int steps = 10000;
static struct   dataPointList *pointlist;
static struct   bbox ptcloudBBox = {{1e30, 1e30, 1e30}, {-1e30, -1e30, -1e30}};
static          octreeNode root;
static float    alphaPrimeTable[steps+1]; // for rapid inversion
static int      ntotalpoints;
static int      calcpoints = 0; // for debugging stats
static int      computedpoints = 0; // for debugging stats
static bool     warned = false;

static const float inv_pi = 1.0 / M_PI, four_pi = 4.0 * M_PI;


//
// Compute diffuse color (BRDF albedo) Rd
//
static float
computeRd(float A, float alpha_prime)
{
    float s, a, b, c, Rd;

    s = sqrtf(3.0f * (1.0f - alpha_prime));
    a = 0.5f * alpha_prime;
    b = 1.0f + expf(- 4.0f/3.0f * A * s);
    c = expf(-s);
    Rd = a * b * c;

    return Rd;
}


//
// Fill alpha' table
//
static void
fillAlphaPrimeTable(float A)
{
    float RdTable[steps+1];
    float alpha_prime, Rd;
    float w;
    int i, j;

    for (i = 0; i <= steps; i++) {
	alpha_prime = i / (float)steps;
	RdTable[i] = computeRd(A, alpha_prime);
    }

    for (i = 0; i <= steps; i++) {
	Rd = i / (float)steps;
	for (j = 0; RdTable[j] < Rd; j++) ;
	assert(j == 0 || (RdTable[j-1] <= Rd && Rd <= RdTable[j]));
	w = (j > 0) ? (Rd - RdTable[j-1]) / (RdTable[j] - RdTable[j-1]) : 0.0f;
	alphaPrimeTable[i] = (w * j + (1.0f - w) * (j-1)) / (float)steps;
    }
}


//
// Interpolate alpha' (from albedo Rd)
//
static inline float
interpolateAlphaPrime(float Rd)
{
    float w, alpha_prime;
    int i;

    i = (int)(Rd * steps);
    w = Rd * steps - i;
    alpha_prime = (1.0f - w) * alphaPrimeTable[i] + w * alphaPrimeTable[i+1];

    return alpha_prime;
}


//
// Insert data point in point list
//
static inline void
insertInPointList(struct xyz *p, struct xyz *n, float mpsize,
		  int datasize, float *data)
{
    static int i = 0;

    bboxEXPAND(&ptcloudBBox, *p);

    pointlist[i].position = *p;
    pointlist[i].normal = *n;
    pointlist[i].mpsize = mpsize; // micropolygon size (half diagonal length)
    pointlist[i].area = data[0];
    pointlist[i].radius = sqrtf(inv_pi * data[0]);
    pointlist[i].rad_t.r = data[1];
    pointlist[i].rad_t.g = data[2];
    pointlist[i].rad_t.b = data[3];
    if (datasize >= 10) {
	pointlist[i].albedo.r = data[4];
	pointlist[i].albedo.g = data[5];
	pointlist[i].albedo.b = data[6];
	pointlist[i].diffusemeanfreepath.r = data[7];
	pointlist[i].diffusemeanfreepath.g = data[8];
	pointlist[i].diffusemeanfreepath.b = data[9];
    } else if (datasize >= 7) { // (should also handle just dmfp in file?)
	pointlist[i].albedo.r = data[4];
	pointlist[i].albedo.g = data[5];
	pointlist[i].albedo.b = data[6];
    }
    ++i;
}


//
// Recursively distribute the data points in an octree node among its
// children.  Recursion ends when there are sufficiently few points in
// the node (at most 8).
//
static void
splitOctreeNode(struct octreeNode *node, int level)
{
    struct octreeNode *child;
    struct dataPointList *datapoint, tmppoint;
    struct xyz p;
    float w;
    float maxradius = 0.0;
    int nptsinchild[8], pos[8];
    int lastpoint = node->firstpoint + node->npoints;
    int i, j;
    int ch, c;
    int degenerate;

    // Ensure that children in child index table are initialized: we
    // may return early (if we've reached max level or there are no
    // points in this node), and some children may end up with
    // no points in them.
    for (c = 0; c < 2*2*2; c++)
	node->children[c] = NULL;

    node->bbox.min.x = node->bbox.min.y = node->bbox.min.z = 1e30;
    node->bbox.max.x = node->bbox.max.y = node->bbox.max.z = -1e30;
    node->centroid.x = node->centroid.y = node->centroid.z = 0.0f;
    node->sumArea = 0.0f;
    node->sumPower.r = node->sumPower.g = node->sumPower.b = 0.0f;

    // Compute the bounding box, area, centroid, and power sum
    for (i = node->firstpoint; i < lastpoint; i++) {
	datapoint = &pointlist[i];
	bboxEXPAND(&node->bbox, datapoint->position);
	node->centroid.x += datapoint->position.x;
	node->centroid.y += datapoint->position.y;
	node->centroid.z += datapoint->position.z;
	node->sumArea += datapoint->area;
	node->sumPower.r += datapoint->area * datapoint->rad_t.r;
	node->sumPower.g += datapoint->area * datapoint->rad_t.g;
	node->sumPower.b += datapoint->area * datapoint->rad_t.b;
	maxradius = MAX(maxradius, datapoint->radius);
    }

    w = 1.0f / node->npoints;
    node->centroid.x *= w;
    node->centroid.y *= w;
    node->centroid.z *= w;

    // Check for degenerate case: all points at same position
    degenerate = (node->bbox.max.x - node->bbox.min.x < 1e-6f) &&
 	         (node->bbox.max.y - node->bbox.min.y < 1e-6f) &&
	         (node->bbox.max.z - node->bbox.min.z < 1e-6f);

    // Expand bbox by largest point radius
    node->bbox.min.x -= maxradius;
    node->bbox.min.y -= maxradius;
    node->bbox.min.z -= maxradius;
    node->bbox.max.x += maxradius;
    node->bbox.max.y += maxradius;
    node->bbox.max.z += maxradius;

    // Return if there are less than MAXPTSINNODE (8) points in node or
    // if all the points are in the same position
    if (node->npoints <= MAXPTSINNODE || degenerate) {
	node->isleaf = true;
	return; // no need to split octree node: nothing more to do here
    }

    node->isleaf = false;

    for (c = 0; c < 8; c++)
	nptsinchild[c] = 0;

    // Determine how many points in the node's pointlist should move
    // to each of the (up to) 8 children
    for (i = node->firstpoint; i < lastpoint; i++) {
	// Determine which child node the point should be moved to
	p = pointlist[i].position;
	c = determineChild(&node->bbox, &p); // 0..7
	nptsinchild[c]++;
    }

    // Allocate and initialize up to 8 child nodes
    for (c = 0; c < 8; c++) {
	pos[c] = (c > 0) ? pos[c-1] + nptsinchild[c-1] : node->firstpoint;

	if (nptsinchild[c]) {
	    child = (octreeNode*) malloc(sizeof(octreeNode));
	    if (child == NULL) {
		printf("WARNING: too many octree nodes to fit in memory. Aborting.\n");
		exit(1);
	    }
	    child->npoints = nptsinchild[c];
	    child->firstpoint = pos[c];
	    node->children[c] = child;
	}
    }

    // Swap the points so that the points of each child are consecutive.
    // (Could probably be more efficient by using a real sorting
    // algorithm with the child (0-7) being the sorting key.)
    for (ch = 0; ch < 7; ch++) { // last child (7) gets trivially done
	child = node->children[ch];
	if (!child) continue;
	i = child->firstpoint;
	j = node->firstpoint + node->npoints - 1;
	while (i < j) {

	    // Find the next point to move
	    do {
		p = pointlist[i].position;
		c = determineChild(&node->bbox, &p); // 0..7
		i++;
	    } while (c == ch && i <= j);
	    i--;

	    // Determine which child node the point should be moved to
	    do {
		p = pointlist[j].position;
		c = determineChild(&node->bbox, &p); // 0..7
		j--;
	    } while (c != ch && j >= i);
	    j++;

	    if (i < j) {
		assert(node->firstpoint <= j && j < lastpoint);
		// Swap points i and j
		tmppoint = pointlist[i];
		pointlist[i] = pointlist[j];
		pointlist[j] = tmppoint;
	    }
	}
    }

    // Recurse
    for (c = 0; c < 2*2*2; c++) {
	if (node->children[c]) {
	    // Go to child node
	    splitOctreeNode(node->children[c], level+1);
	}
    }
}


//
// Convert diffuse color (albedo), diffuse mean free path, and ior to
// reduced scattering albedo, effective transport extinction
// coefficient, and mean free path length (the average dist at which
// light scatters).
//
static void
convertParams(struct color *Rd, // diffuse color (BRDF albedo)
	      struct color *dmfp, // diffuse mean free path (aka. l_d)
	      float unitlength, // multiplier on dmfp
	      struct color *alpha_prime, // reduced scattering albedo
	      struct color *sigma_tr, // effective transport extinction coeff.
	      struct color *l_u) // mean free path length
{
    struct color l_d = *dmfp;
    l_d.r *= unitlength;
    l_d.g *= unitlength;
    l_d.b *= unitlength;

    // Interpolate alpha' (from albedo Rd)
    alpha_prime->r = interpolateAlphaPrime(Rd->r);
    alpha_prime->g = interpolateAlphaPrime(Rd->g);
    alpha_prime->b = interpolateAlphaPrime(Rd->b);

    // compute effective transport extinction coeff sigma_tr (from dmfp l_d)
    sigma_tr->r = 1.0f / l_d.r;
    sigma_tr->g = 1.0f / l_d.g;
    sigma_tr->b = 1.0f / l_d.b;

    // Compute mean free path length l_u from diffuse mean free path l_d:
    // l_u = 1 / sigma'_t  with  sigma'_t = 1 / (l_d * sqrt(3 * (1 - alpha')))
    // gives  l_u = l_d * sqrt(3 * (1 - alpha')).
    l_u->r = l_d.r * sqrtf(3.0f * (1.0f - alpha_prime->r));
    l_u->g = l_d.g * sqrtf(3.0f * (1.0f - alpha_prime->g));
    l_u->b = l_d.b * sqrtf(3.0f * (1.0f - alpha_prime->b));
}


//
// Compute BSSRDF R_d using the dipole approximation: "real" point
// light source inside volume and virtual point light source outside
// Definition: diffuse BSSRDF R_d = dM_o / dPhi_i.
//
static inline float
computeBSSRDF(float r, // distance from illum point P_i to point P_o
	      float sigma_tr, // effective transport extinction coeff
	      float alpha_prime, // reduced albedo
	      float l_u, // mean free path length
	      float z_r, // depth of "real" dipole light source (= l_u)
	      float z_v) // height of virtual dipole light source
{
    float d_r, d_v; // distance from the dipole lights to point xthe surface
    float c1, c2;
    float factor;
    float R_d; // the BSSRDF result

    if (r < l_u) { // P_i and P_o are very close to each other:
	// Evaluate the BSSRDF with a minimum distance of l_u =
	// 1/sigma'_t to eliminate singularities (see Jensen et al.,
	// Proc. SIGGRAPH 01, p. 516).
	r = l_u;
    }

    d_r = sqrtf(r*r + z_r*z_r); // distance to "real" dipole source
    d_v = sqrtf(r*r + z_v*z_v); // distance to virtual dipole source
    assert(d_r > 0.0f && d_v > 0.0f);

    c1 = z_r * (sigma_tr + 1.0f/d_r);
    c2 = z_v * (sigma_tr + 1.0f/d_v);
    factor =  c1 * expf(-sigma_tr * d_r) / (d_r*d_r);
    factor += c2 * expf(-sigma_tr * d_v) / (d_v*d_v);
    factor *= 1.0f / four_pi;

    R_d = alpha_prime * factor;

    assert(!isnan(R_d));
    return R_d;
}


//
// Recursively traverse the octree and add up the radiosity due
// to all the nodes
//
static void
recTraverseTree(struct xyz *P_o, octreeNode *node, int level,
		struct color *sigma_tr, struct color *alpha_prime,
		struct color *l_u, struct color *z_r, struct color *z_v,
		float maxsolidangle, // aka. eps
		struct color *M_o) // incremented
{
    struct octreeNode *child;
    struct color Rd; // diffuse BSSRDF reflection coefficients
    struct color rad_t;
    struct xyz diff;
    float omega = 0.0; // max solid angle
    float area;
    float dist, dist2;
    int lastpoint, c, i;
    bool leaf = node->isleaf;

    // If this is a leaf node: loop over the individual points in it
    if (leaf) {
	lastpoint = node->firstpoint + node->npoints;
	for (i = node->firstpoint; i < lastpoint; i++) {
	    ++calcpoints;

	    // Compute diffusion from data point P_i to data point P_o
	    struct xyz *P_i = &pointlist[i].position;

	    xyzSUB(diff, *P_i, *P_o);
	    dist = xyzLENGTH(diff); // dist from illum point P_i to point P_o

	    Rd.r = computeBSSRDF(dist, sigma_tr->r, alpha_prime->r, l_u->r,
				 z_r->r, z_v->r);
	    Rd.g = computeBSSRDF(dist, sigma_tr->g, alpha_prime->g, l_u->g,
				 z_r->g, z_v->g);
	    Rd.b = computeBSSRDF(dist, sigma_tr->b, alpha_prime->b, l_u->b,
				 z_r->b, z_v->b);

	    rad_t = pointlist[i].rad_t;
	    area = pointlist[i].area;

	    // Increment the radiant exitance (radiosity) at point P_o.
	    // See [Jensen02] equation (13).
	    // Note that rad_t can be negative, so M_o can also turn negative.
	    // Also note that we omit the F_dt term here and in equation (14),
	    // corresponding to diffuse radiance, as suggested right below
	    // equation (14).
	    // Furthermore, the alpha' term is omitted since it shouldn't be
	    // there (according to Christophe Hery).  If it were, it would
	    // cancel out the alpha' term in the BSSRDF.
	    M_o->r += Rd.r * area * rad_t.r;
	    M_o->g += Rd.g * area * rad_t.g;
	    M_o->b += Rd.b * area * rad_t.b;
	}

    } else {

	// Compute the minimum distance between the point and the node bbox.
	// The distance is 0 if the point is inside the bbox.
	dist2 = pointBBoxDist2(P_o, &node->bbox);

	// Compute the maximum solid angle spanned by the points in this node.
	// ([Jensen02] p. 579 suggests estimating this using the distance
	// to the centroid of the points.  But that fails if e.g. the points
	// are in two clumps, one of which is very close to the point P_o.)
	if (dist2 > 0.0f) {
	    omega = node->sumArea / dist2;
	}

	// To recurse or not to recurse, that is the question ...
	// We decide using the heuristic of [Jensen02] p. 579, except that
	// the solid angle is computed more conservatively: distance to
	// the bbox, not to the centroid.
	if (dist2 == 0.0f || omega > maxsolidangle) { // angle too large
	    // Recursively visit children
	    for (c = 0; c < 8; c++) {
		child = node->children[c];
		if (child) {
		    recTraverseTree(P_o, child, level+1,
				    sigma_tr, alpha_prime, l_u, z_r, z_v,
				    maxsolidangle, M_o);
		}
	    }
	} else { // error low enough: use node as is

	    xyzSUB(diff, *P_o, node->centroid);
	    dist = xyzLENGTH(diff); // dist from centroid to point P_o

	    // Compute the BSSRDF coefficients
	    Rd.r = computeBSSRDF(dist, sigma_tr->r, alpha_prime->r, l_u->r,
				 z_r->r, z_v->r);
	    Rd.g = computeBSSRDF(dist, sigma_tr->g, alpha_prime->g, l_u->g,
				 z_r->g, z_v->g);
	    Rd.b = computeBSSRDF(dist, sigma_tr->b, alpha_prime->b, l_u->b,
				 z_r->b, z_v->b);

	    // Increment the radiant exitance (radiosity) at point P_o.
	    // See [Jensen02] equation (13).  The F_dt and alpha' terms
	    // are omitted (as above).
	    // Note that rad_t and hence sumPower can be negative, so M_o
	    // can also turn negative.
	    M_o->r += Rd.r * node->sumPower.r;
	    M_o->g += Rd.g * node->sumPower.g;
	    M_o->b += Rd.b * node->sumPower.b;
	}
    }
}


void*
doMultiScatteringThread(void* structargs){
    msargs* msa = (msargs*)structargs;

    struct xyz *P_o;
    struct color M_o; // radiosity (aka. radiant exitance)
    struct color L_o; // exitant radiance;

    int o;

    for ( o = msa->threadnum; o < ntotalpoints; o+=msa->numthread ) { // loop over all shading points

	// Compute diffusion to data point o
	P_o = &pointlist[o].position;
	M_o.r = M_o.g = M_o.b = 0.0;

	// Use albedo and diffusemeanfreepath of exit point Po if present
	if (msa->paramFromFile) {
	    if (msa->albedoFromFile)
		msa->Rd = &pointlist[o].albedo;
	    if (msa->dmfpFromFile)
		msa->l_d = &pointlist[o].diffusemeanfreepath;

	    if (msa->Rd->r > 0.999f || msa->Rd->g > 0.999f || msa->Rd->b > 0.999f) {
		if (!warned) {
		    fprintf(stderr, "ptfilter warning: albedo is 1 (or larger) at point %i.\n", o);
		    fprintf(stderr, "Albedo clamped to 0.999.  (No more warnings will be given for this file.)\n");
		    warned = true;
		}
		msa->Rd->r = MIN(msa->Rd->r, 0.999f);
		msa->Rd->g = MIN(msa->Rd->g, 0.999f);
		msa->Rd->b = MIN(msa->Rd->b, 0.999f);
	    }

	    convertParams(msa->Rd, msa->l_d, msa->unitlength, &msa->alpha_prime, &msa->sigma_tr, &msa->l_u);

	    if (msa->l_u.r == 0.0f || msa->l_u.g == 0.0f || msa->l_u.b == 0.0f) {
		fprintf(stderr, "ptfilter error: mean free path length is 0 at point %i.  Aborting.\n", o);
		return NULL; // failure
	    }

	    // Dipole placements:
	    // z_r = l_u is the depth of "real" dipole light (dist to surface)
	    // z_v = l_u * (1.0f + 1.333333f * A): height of virt. dipole light
	    msa->z_r = msa->l_u;
	    msa->factor = (1.0f + 1.333333f * msa->A);
	    msa->z_v.r = msa->l_u.r * msa->factor;
	    msa->z_v.g = msa->l_u.g * msa->factor;
	    msa->z_v.b = msa->l_u.b * msa->factor;
	}

	// hierarchical gathering of light (fast)
	recTraverseTree(P_o, &root, 0,
			&msa->sigma_tr, &msa->alpha_prime, &msa->l_u, &msa->z_r, &msa->z_v,
			msa->maxsolidangle, &M_o);

    computedpoints++;

	// If we wanted to be physically correct, we would convert
	// radiant exitance (radiosity) to exitant radiance by
	// dividing by pi.  But the PRMan shading language is always
	// very lax about pi factors, and the ss colors are already
	// very dark.  We are omitting the Fresnel factor F_t/F_dr as
	// suggested below equation (14) in [Jensen02].
	L_o = M_o;

	if (msa->printProgress >= 2 && (o % (1000+msa->threadnum) == 0)) {
	    printf("thread [%i] diffusion at point %i = (%f %f %f)   (%i calc. points)\n",msa->threadnum,
		   o, L_o.r, L_o.g, L_o.b, calcpoints);
	    fflush(stdout); // since Windows doesn't flush after newline
	}
    //if ( (o % (1000+msa->threadnum) == 0)) {
    //    pthread_mutex_lock( &progresslock );
    //    pthread_mutex_unlock( &progresslock );
    //}
	pointlist[o].ssdiffusion = L_o;
    }
    return NULL;
}

//
// Compute multiple scattering (using diffusion approximation) at all points.
// The names and greek letters are from [Jensen01] and [Jensen02].
// if paramsFromFile is true, the parameters Rd, l_d, and ior are ignored --
// those values are read from the point cloud file instead.
//
static void
doMultiScattering(int albedoFromFile,
		  struct color *Rd, // diffuse color (BRDF albedo)
		  int dmfpFromFile,
		  struct color *l_d, // diffuse mean free path (aka. dmfp)
		  float unitlength,
		  float ior, // index of refraction
		  float maxsolidangle, // aka. eps
		  int printProgress)
{
    struct color alpha_prime; // reduced scattering albedo
    struct color sigma_tr; // effective transport extinction coeffient
    struct color l_u; //mean free path length: avg dist at which light scatters
    struct color z_r, z_v; // height of "real" and virtual dipole light sources
    float eta; // relative index of refraction, ior / 1
    float F_dr, F_dt; // diffuse Fresnel reflection and transmission terms
    float A;
    float factor = 0;
    int paramFromFile = albedoFromFile || dmfpFromFile;
    int numThreads = get_nprocs();
    int rett;

    pthread_t msthread[MAX_NUMTREAD];


    // Diffuse Fresnel terms, etc.
    eta = ior;
    F_dr = -1.440f / (eta*eta) + 0.710f / eta + 0.668f + 0.0636 * eta;
    assert(0.0f <= F_dr && F_dr <= 1.0);
    F_dt = 1.0f - F_dr;
    A = (1.0f + F_dr) / (1.0f - F_dr);

    fillAlphaPrimeTable(A);

    if (!paramFromFile) {

	if (Rd->r > 0.999f || Rd->g > 0.999f || Rd->b > 0.999f) {
	    fprintf(stderr, "ptfilter warning: albedo is 1 (or larger).\n");
	    fprintf(stderr, "Albedo clamped to 0.999.\n");
	    Rd->r = MIN(Rd->r, 0.999f);
	    Rd->g = MIN(Rd->g, 0.999f);
	    Rd->b = MIN(Rd->b, 0.999f);
	}

	convertParams(Rd, l_d, unitlength, &alpha_prime, &sigma_tr, &l_u);

	if (printProgress) {
	    printf("Diffuse Fresnel terms: F_dr = %f, F_dt = %f\n", F_dr, F_dt);
	    printf("Reduced scattering albedo: alpha' = %f %f %f\n",
		   alpha_prime.r, alpha_prime.g, alpha_prime.b);
	    printf("Reduced extinction coefficient: sigma'_t = %f %f %f\n",
		   sigma_tr.r, sigma_tr.g, sigma_tr.b);
	    printf("Mean free path length: l_u = %f %f %f\n", l_u.r, l_u.g, l_u.b);
	    fflush(stdout); // since Windows doesn't flush after newline
	}

	if (l_u.r == 0.0f || l_u.g == 0.0f || l_u.b == 0.0f) {
	    fprintf(stderr, "ptfilter error: mean free path length is 0.  Aborting.\n");
	    return; // failure
	}

	// Dipole placements:
	// z_r = l_u is the depth of "real" dipole light (distance to surface)
	// z_v = l_u * (1.0f + 1.333333f * A) is height of virtual dipole light
	z_r = l_u;
	factor = (1.0f + 1.333333f * A);
	z_v.r = l_u.r * factor;
	z_v.g = l_u.g * factor;
	z_v.b = l_u.b * factor;
    }

    // create structure for sending to threads
    msargs datas[MAX_NUMTREAD];

    // Iterate the threads, fill them and lauch
    for (int nt = 0; nt < numThreads; nt++){
        datas[nt].A = A;
        datas[nt].albedoFromFile = albedoFromFile;
        datas[nt].dmfpFromFile = dmfpFromFile;
        datas[nt].eta = eta;
        datas[nt].factor = factor;
        datas[nt].F_dr = F_dr;
        datas[nt].F_dt = F_dt;
        datas[nt].ior = ior;
        datas[nt].l_d = l_d;
        datas[nt].maxsolidangle = maxsolidangle;
        datas[nt].paramFromFile = paramFromFile;
        datas[nt].printProgress = printProgress;
        datas[nt].Rd = Rd;
        datas[nt].unitlength = unitlength;
        datas[nt].threadnum = nt;
        datas[nt].numthread = numThreads;
        datas[nt].z_r = z_r;
        datas[nt].z_v = z_v;
        datas[nt].l_u = l_u;
        datas[nt].sigma_tr = sigma_tr;
        datas[nt].alpha_prime = alpha_prime;

        // Thread creation
        rett = pthread_create(&msthread[nt], NULL, doMultiScatteringThread, &datas[nt]);
        if (rett){
            perror("Thread creation problem exiting ...\n");
            exit(0);
            return;
        } else {
            if (printProgress > 1)
                printf("Thread [%i] OK\n",datas[nt].threadnum);
        }
    }

    // Wait for the threads to finish
    for (int nt = 0; nt < numThreads; nt++){
        if (printProgress > 1)
            printf("joining thread %i\n", nt);
        pthread_join(msthread[nt],NULL);
    }
}

//
// Compute subsurface scattering on a point cloud:
// 1) Read in point cloud file(s) with rad_t (and optionally albedo and
//    mean path length).
// 2) Sort points into an octree.
// 3) Compute multiple subsurface scattering (diffusion approximation) at
//    all points.
// 4) Write out a point cloud file with subsurface diffusion color.
//
// Called from the ptfilter program.
//
// The subsurface scattering is specified by the diffuse albedo,
// diffuse mean free path length, and ior.  The first two quantities
// can be fixed or read from the point cloud points.
//
int
SSDiffusion(char *inputfilenames[], char *outputfilename, int numfiles,
	    int albedoFromFile, struct color *albedo,
	    int dmfpFromFile, struct color *diffusemeanfreepath,
	    float unitlength, float ior,
	    float maxsolidangle, int printProgress)
{
    pthread_mutex_init( &progresslock, NULL);
    PtcPointCloud inptc[256], outptc;
    float w2e[16], w2n[16], format[3];
    struct xyz position, normal;
    float *data = NULL; // there can be many data even though we only use max 6
    float mpsize, delta;
    int npoints, datasize = 0, nvars;
    int f, p;
    const char *vartypes[256], *varnames[256]; // arrays of strings
    char *newvartypes[1], *newvarnames[1]; // arrays of strings

    if (printProgress) {
	if (albedoFromFile)
	    printf("Scattering albedo: will be read from point cloud file\n");
	else
	    printf("Scattering albedo: alpha = %f %f %f\n",
		   albedo->r, albedo->g, albedo->b);

	if (dmfpFromFile)
	    printf("Diffuse mean free path length: will be read from point cloud file\n");
	else
	    printf("Diffuse mean free path length: l_d = %f %f %f\n",
		   diffusemeanfreepath->r, diffusemeanfreepath->g,
		   diffusemeanfreepath->b);

	if (unitlength != 1.0f)
	    printf("Unitlength: %f\n", unitlength);

	printf("Index of refraction: ior = %f\n", ior);
	fflush(stdout); // since Windows doesn't flush after newline
    }

    warned = false;

    // Read point cloud file headers to find out how many points there are
    // (Also read the camera info: two transformation matrices and format)
    ntotalpoints = 0;
    for (f = 0; f < numfiles; f++) {
	char *fromfile = inputfilenames[f];
	// Open the point cloud file and read the header
	inptc[f] = PtcOpenPointCloudFile(fromfile,&nvars,vartypes,varnames);

	if(inptc[f] == NULL){
	    printf("File doesn't exist\n");
	    exit(0);
	}


	PtcGetPointCloudInfo(inptc[f], "npoints", &npoints);
        ntotalpoints += npoints;
    }

    // Get transformation matrices and image format from first file
    PtcGetPointCloudInfo(inptc[0], "world2eye", w2e);
    PtcGetPointCloudInfo(inptc[0], "world2ndc", w2n);
    //PtcGetPointCloudInfo(inptc[0], "format", format);
    PtcGetPointCloudInfo(inptc[0], "datasize", &datasize);

    if (albedoFromFile && dmfpFromFile)
	assert(datasize >= 10);
    else if (albedoFromFile || dmfpFromFile)
	assert(datasize >= 7);
    else
	assert(datasize >= 4);

    data = (float *) malloc(datasize * sizeof(float));

    // Allocate room for all data points
    pointlist = (struct dataPointList *)
	malloc(ntotalpoints * sizeof(struct dataPointList));
    if (pointlist == NULL) {
	printf("WARNING: too many data points to fit in memory. Aborting.\n");
	return -1;
    }

    //
    // Read all point positions from file and store in array.
    // This is done by calling insertInPointList() with each
    // data point.
    //
    for (f = 0; f < numfiles; f++) {

	PtcGetPointCloudInfo(inptc[f], "npoints", &npoints);

	for (p = 0; p < npoints; p++) {
	    // Read one set of point data from point cloud
	    PtcReadDataPoint(inptc[f], (float*)&position, (float*)&normal,
			     &mpsize, data);
	    // Insert the point data in point list
	    insertInPointList(&position, &normal, mpsize, datasize, data);
	}

	PtcClosePointCloudFile(inptc[f]);
    }

    // Expand point cloud bbox a little bit (and offset center from middle)
    delta = 0.001f * bboxDIAG(&ptcloudBBox);
    ptcloudBBox.min.x -= delta;
    ptcloudBBox.min.y -= delta;
    ptcloudBBox.min.z -= delta;
    ptcloudBBox.max.x += 2.0f * delta;
    ptcloudBBox.max.y += 2.0f * delta;
    ptcloudBBox.max.z += 2.0f * delta;

    // Recursively create octree by splitting nodes
    if (printProgress) {
	printf("Pass 1: create octree\n");
	fflush(stdout); // since Windows doesn't flush after newline
    }
    root.npoints = ntotalpoints;
    root.firstpoint = 0;
    splitOctreeNode(&root, 0);

    // Compute scattering at all points
    if (printProgress) {
	printf("Pass 2: compute scattering at all %i points\n", ntotalpoints);
	fflush(stdout); // since Windows doesn't flush after newline
    }
    doMultiScattering(albedoFromFile, albedo,
		      dmfpFromFile, diffusemeanfreepath, unitlength,
		      ior,
		      maxsolidangle, printProgress);

    // Write out a point cloud file with subsurface diffusion color
    nvars = 1;
    char colorstring[] = "color";
    char ssdiffusion[] = "_ssdiffusion";
    newvartypes[0] = strdup(colorstring);
    newvarnames[0] = strdup(ssdiffusion);
    outptc = PtcCreatePointCloudFile(outputfilename,
				     nvars, newvartypes, newvarnames,
				     w2e, w2n, format);

    for (p = 0; p < ntotalpoints; p++) {
	float *data = (float*) &pointlist[p].ssdiffusion;
	PtcWriteDataPoint(outptc,
			  (float*)&pointlist[p].position,
			  (float*)&pointlist[p].normal,
			  pointlist[p].mpsize,
			  data);
    }

    // write out last points, update header, close file, etc.
    PtcFinishPointCloudFile(outptc);

    return 0; // success (return code 0)
}

