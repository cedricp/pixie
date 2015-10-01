// ********************************************************
// ** DWARF-LABS GI shader
// ** (c) 2010 Cédric PAILLE
// ********************************************************


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

pthread_mutex_t progresslocksss;

static struct   dataGIPointList *pointlist;
static struct   bbox ptcloudBBox = {{1e30, 1e30, 1e30}, {-1e30, -1e30, -1e30}};
static int      calcpoints = 0; // for debugging stats
static int      computedpoints = 0; // for debugging stats
static int      ntotalpoints;

static const float inv_pi = 1.0 / M_PI, four_pi = 4.0 * M_PI;

typedef struct MSARGS{
    float   maxsolidangle;
    int     printProgress;
    int     threadnum,numthread;
} msargs;

static GIoctreeNode root;

//
// Insert data point in point list
//
static inline void
insertInGIPointList(struct xyz *p, struct xyz *n, float mpsize,
		  int datasize, float *data)
{
    static int i = 0;

    bboxEXPAND(&ptcloudBBox, *p);
    xyzCOPY(pointlist[i].position,*p);
    xyzCOPY(pointlist[i].normal,*n);
    xyzNORMALIZE(pointlist[i].normal);

    pointlist[i].mpsize     = mpsize;  // micropolygon size (half diagonal length)
    pointlist[i].area       = data[0]; // Area
    pointlist[i].radius     = sqrtf(inv_pi * data[0]);
    pointlist[i].radiance.r = data[1]; // red componant
    pointlist[i].radiance.g = data[2]; // green ""
    pointlist[i].radiance.b = data[3]; // blue  ""
    ++i;
}

static void
splitOctreeNode(struct GIoctreeNode *node, int level)
{
    struct  GIoctreeNode *child;
    struct  dataGIPointList *datapoint, tmppoint;
    struct  xyz p;
    float   w;
    float   maxradius = 0.0;
    int     nptsinchild[8], pos[8];
    int     lastpoint = node->firstpoint + node->npoints;
    int     i, j;
    int     ch, c;
    int     degenerate;

    node->dP = 0;

    for (c = 0; c < 2*2*2; c++)
        node->children[c] = NULL;

    node->bbox.min.x = node->bbox.min.y = node->bbox.min.z = 1e30;
    node->bbox.max.x = node->bbox.max.y = node->bbox.max.z = -1e30;

    xyzINIT(node->normal);
    xyzINIT(node->centroid);
    colorINIT(node->radiance);

    node->sumArea = 0.0f;

    for (i = node->firstpoint; i < lastpoint; i++) {
        datapoint = &pointlist[i];
        bboxEXPAND(&node->bbox, datapoint->position);
        // Compute centroid of points
        xyzADD( node->centroid, node->centroid, datapoint->position);

        // Compute average normal
        xyzADD( node->normal, node->normal, datapoint->normal) ;
    }

    // Average points & normal
    w = 1.0f / node->npoints;
    xyzSCALE( node->centroid, node->centroid , w );
    xyzNORMALIZE(node->normal);

    for (i = node->firstpoint; i < lastpoint; i++) {
        datapoint = &pointlist[i];
        struct color col;

        float area = MAX( datapoint->area * xyzDOTI( node->normal, datapoint->normal ), 0 );

        colorCOPY(col,datapoint->radiance);
        colorSCALE( col, col, area );
        colorADD ( node->radiance, node->radiance, col );

        node->sumArea += datapoint->area;
        maxradius = MAX( maxradius, datapoint->radius );
        node->dP += area;
    }

    // Normalize radiance
    colorSCALE(node->radiance, node->radiance, 1.0 / node->dP);

    // Normalize area
    node->dP = sqrtf( node->dP / M_PI );

    // Check for degenerate case: all points at same position
    degenerate =    (node->bbox.max.x - node->bbox.min.x < 1e-6f) &&
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
            child = (GIoctreeNode*) malloc(sizeof(GIoctreeNode));
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


static void formfactor(struct xyz* rcvp,struct xyz*  shootp,struct xyz* rcvn,
                       struct xyz* shootn,struct color* shoote,float shootarea,
                       struct color* delta,float &occ)
{
    xyz cP,P0,P1,nD;
    xyz D;
    float ff = 0;

    float t = (xyzDOTI( *shootp, *rcvn )) - (xyzDOTI( *rcvp, *rcvn));

    // behind the tangent plane
    if ( t < -shootarea ) return;

    if ( t < shootarea ){
        // Recompute sphere
        xyzSCALE( P0, *rcvn, shootarea);
        xyzADD  ( P0, P0, *shootp );
        xyzSCALE( P1, *rcvn, -t);
        xyzADD  ( P1, P1, *shootp );

        // Center of the sphere
        xyzADD  ( cP, P0, P1 );
        xyzSCALE( cP, cP, 0.5f );

        xyzSUB( D, P0, cP );
        shootarea = xyzLENGTH(D);

        xyzSUB( D, cP, *rcvp );

        // Seeing from behind ?
        if( xyzDOTI( D, *shootn ) >= 0) return;

        xyzCOPY(nD,D);
        xyzNORMALIZE( nD );

    } else {
        xyzSUB( D, *shootp, *rcvp);

        if( xyzDOTI( D, *shootn ) >= 0 ) return;

        xyzCOPY( nD, D );
        xyzNORMALIZE( nD );


    }

    ff = (-(xyzDOTI( nD, *shootn)) * xyzDOTI( nD, *rcvn ) * (shootarea * shootarea)) / ( xyzDOTI(D,D) + 1e-6f );

    // Compute delta
    colorSCALE( *delta, *shoote, ff );
    occ = ff;
}

//
// Recursively traverse the octree and shade GI
//
static void
recTraverseGITree(struct xyz *P_o,struct xyz *P_n,GIoctreeNode *node, int level, const float &maxsolidangle,struct color *rad,float &occ)
{
    struct  GIoctreeNode *child;
    int     lastpoint, c, i;
    bool    leaf = node->isleaf;

    // If this is a leaf node: loop over the individual points in it
    if (leaf) {

        lastpoint = node->firstpoint + node->npoints;
        struct color cres,cadd;
        float occlusion = 0.0;
        float occres = 0.0;
        colorINIT(cadd);

        for (i = node->firstpoint; i < lastpoint; i++) {
            ++calcpoints;
            /// Raytrace
            if ( xyzDOTI( *P_o, pointlist[i].normal ) <= xyzDOTI( pointlist[i].position, pointlist[i].normal ) ) continue;

            colorINIT(cres);
            occlusion = 0;
            formfactor(P_o, &pointlist[i].position, P_n,&pointlist[i].normal, &pointlist[i].radiance, pointlist[i].radius, &cres, occlusion);
            colorADD(cadd,cadd,cres);
            occres += occlusion;
        }
        colorADD(*rad,*rad,cadd);
        occ += occres;

    } else {
        xyz D;
        xyzSUB(D,node->centroid,*P_o);

        float distSqr = xyzDOTI(D,D) + 1e-6f;
        float dpArea  = M_PI * node->dP * node->dP;
        float lenD    = xyzLENGTH(D);

        if( ( lenD > node->dP ) && ( ( dpArea / distSqr ) < maxsolidangle) ){
            // error low enough: use node as is
            struct color cres;
            colorINIT(cres);
            float  occlusion = 0;

            colorINIT(cres);
            formfactor( P_o, &node->centroid, P_n, &node->normal, &node->radiance, node->dP, &cres, occlusion);
            colorADD(*rad,*rad,cres);
            occ += occlusion;
        } else {
            // Recursively visit children
            for (c = 0; c < 8; c++) {
                child = node->children[c];
                if (child)
                    recTraverseGITree(P_o,P_n,child, level+1,maxsolidangle,rad,occ);
            }
        }
    }
}

void*
doDiffusionThread(void* structargs){
    msargs* msa = (msargs*)structargs;

    struct  xyz *P_o,*P_n;
    struct  color rad;
    float   occ;

    int o;

    for ( o = msa->threadnum; o < ntotalpoints; o+=msa->numthread ) { // loop over all shading points

        P_o = &pointlist[o].position;
        P_n = &pointlist[o].normal;
        colorINIT(rad);
        occ = 0.0f;

        // hierarchical gathering of light (fast)
        recTraverseGITree(P_o, P_n, &root, 0, msa->maxsolidangle, &rad, occ);
        computedpoints++;

        if (msa->printProgress >= 2 && (o % (1000+msa->threadnum) == 0)) {
            printf("thread [%i] diffusion at point %i = (%f %f %f)   (%i calc. points)\n",msa->threadnum,
               o, rad.r, rad.g, rad.b, calcpoints);
            fflush(stdout);
        }

        if ( (o % (1000+msa->threadnum) == 0)) {
            pthread_mutex_lock( &progresslocksss );
            pthread_mutex_unlock( &progresslocksss );
        }

        pointlist[o].occlusion = occ;
        colorCOPY( pointlist[o].radiosity, rad );
    }
    return NULL;
}

static void
doDiffusion(float maxsolidangle,
		    int printProgress)
{

    int numThreads = get_nprocs();
    int rett;

    pthread_t msthread[MAX_NUMTREAD];

    // create structure for sending to threads
    msargs datas[MAX_NUMTREAD];

    // Iterate the threads, fill them and lauch
    for (int nt = 0; nt < numThreads; nt++){
        datas[nt].maxsolidangle = maxsolidangle;
        datas[nt].printProgress = printProgress;
        datas[nt].threadnum = nt;
        datas[nt].numthread = numThreads;

        // Thread creation
        rett = pthread_create(&msthread[nt], NULL, doDiffusionThread, &datas[nt]);
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

int GIDiffusion(char *inputfilenames[], char *outputfilename, int numfiles,
	    float maxsolidangle, int printProgress)
{
    pthread_mutex_init( &progresslocksss, NULL);
    PtcPointCloud inptc[256], outptc;
    float   w2e[16], w2n[16], format[3];
    struct  xyz position, normal;
    float   *data = NULL; // there can be many data even though we only use max 6
    float   mpsize, delta;
    int     npoints, datasize = 0, nvars;
    int     f, p;

    const char *vartypes[256], *varnames[256]; // arrays of strings
    char    *newvartypes[2], *newvarnames[2]; // arrays of strings


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
    PtcGetPointCloudInfo(inptc[0], "datasize", &datasize);

	assert(datasize >= 4);

    data = (float *) malloc(datasize * sizeof(float));

    // Allocate room for all data points
    pointlist = (struct dataGIPointList *)
	malloc(ntotalpoints * sizeof(struct dataGIPointList));
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
	    insertInGIPointList(&position, &normal, mpsize, datasize, data);
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
        printf("Pass 2: compute diffusion at all %i points\n", ntotalpoints);
        fflush(stdout); // since Windows doesn't flush after newline
    }

    doDiffusion(maxsolidangle, printProgress);

    // Write out a point cloud file with diffusion color
    nvars = 2;
    char colorstring[]          = "color";
    char ssdiffusion[]          = "_colorbleeding";
    char floatstring[]          = "float";
    char ambientocclusion[]     = "_ambientocclusion";

    newvartypes[0] = strdup(colorstring);
    newvarnames[0] = strdup(ssdiffusion);
    newvartypes[1] = strdup(floatstring);
    newvarnames[1] = strdup(ambientocclusion);

    outptc = PtcCreatePointCloudFile(outputfilename,
				     nvars, newvartypes, newvarnames,
				     w2e, w2n, format);

    for (p = 0; p < ntotalpoints; p++) {
        float data[4];
        data[0] = pointlist[p].radiosity.r;
        data[1] = pointlist[p].radiosity.g;
        data[2] = pointlist[p].radiosity.b;
        data[3] = pointlist[p].occlusion;
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

