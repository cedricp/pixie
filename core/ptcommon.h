

#ifndef PTCOMMON_H
#define PTCOMMON_H

#define MAX(a,b) a > b ? a : b
#define MIN(a,b) a < b ? a : b

// Standard types
struct xyz	 { float x, y, z; };
struct color	 { float r, g, b; };
struct bbox      { struct xyz min, max; };

// Vector subtraction: r = a - b
#define xyzSUB(r, a, b) { (r).x = (a).x - (b).x; \
                          (r).y = (a).y - (b).y; \
                          (r).z = (a).z - (b).z; }

// Vector addition: r = a + b
#define xyzADD(r, a, b) { (r).x = (a).x + (b).x; \
                          (r).y = (a).y + (b).y; \
                          (r).z = (a).z + (b).z; }

#define colorADD(re, a, c)  (re).r = (a).r + (c).r; \
                            (re).g = (a).g + (c).g; \
                            (re).b = (a).b + (c).b;


// Length of vector
#define xyzLENGTH(v) (sqrtf((v).x*(v).x + (v).y*(v).y + (v).z*(v).z))

// Length of vector
#define xyzDOT(r,v,w) (r) = (v).x*(w).x + (v).y*(w).y + (v).z*(w).z

#define xyzDOTI(v,w) ((v).x*(w).x + (v).y*(w).y + (v).z*(w).z)

// Vector normalize
#define xyzNORMALIZE(v) float invlen = 1/xyzLENGTH(v); \
                        (v).x *= invlen;\
                        (v).y *= invlen;\
                        (v).z *= invlen;\

#define xyzSCALE(r,v,s)     (r).x = (v).x * s;\
                            (r).y = (v).y * s;\
                            (r).z = (v).z * s;\

#define colorSCALE(re,v,s)   (re).r = (v).r * s;\
                             (re).g = (v).g * s;\
                             (re).b = (v).b * s;\

#define colorSCALEV(re,v,s)   (re).r = (v).r * (s).r;\
                              (re).g = (v).g * (s).g;\
                              (re).b = (v).b * (s).b;\

#define xyzINIT(v)  (v).x = 0.0f;\
                    (v).y = 0.0f;\
                    (v).z = 0.0f;\

#define colorINIT(v)    (v).r = 0.0f;\
                        (v).g = 0.0f;\
                        (v).b = 0.0f;\

#define xyzCOPY(v,w)    (v).x = (w).x;\
                        (v).y = (w).y;\
                        (v).z = (w).z;\


#define colorCOPY(v,w)  (v).r = (w).r;\
                        (v).g = (w).g;\
                        (v).b = (w).b;\

// Length of the diagonal of a bounding box
#define bboxDIAG(bb) \
    (sqrtf(((bb)->max.x - (bb)->min.x) * ((bb)->max.x - (bb)->min.x) + \
	   ((bb)->max.y - (bb)->min.y) * ((bb)->max.y - (bb)->min.y) + \
	   ((bb)->max.z - (bb)->min.z) * ((bb)->max.z - (bb)->min.z)))

// Make the bounding box contain point pt
#define bboxEXPAND(bbox, pt) { \
    if ((pt).x < (bbox)->min.x) (bbox)->min.x = (pt).x; \
    if ((pt).y < (bbox)->min.y) (bbox)->min.y = (pt).y; \
    if ((pt).z < (bbox)->min.z) (bbox)->min.z = (pt).z; \
    if ((pt).x > (bbox)->max.x) (bbox)->max.x = (pt).x; \
    if ((pt).y > (bbox)->max.y) (bbox)->max.y = (pt).y; \
    if ((pt).z > (bbox)->max.z) (bbox)->max.z = (pt).z; }



struct dataPointList {
    struct xyz position;
    struct xyz normal;
    float mpsize; // micropolygon edge length (aka. radius in point cloud file)
    float area, radius; // having both is redundant but convenient
    struct color rad_t; // the radiance leaving the inside of the surface
                        // (due to diffusely transmitted irradiance)
    struct color albedo;
    struct color diffusemeanfreepath;
    struct color ssdiffusion; // result
};

struct dataGIPointList {
    struct xyz position;
    struct xyz normal;
    float mpsize; // micropolygon edge length (aka. radius in point cloud file)
    float area, radius; // having both is redundant but convenient
    struct color radiance; // the radiance leaving the inside of the surface
                        // (due to diffusely transmitted irradiance)
    struct color radiosity;
    float  occlusion;
};

struct octreeNode {
    int npoints; // number of data points in this octree node
    int firstpoint; // index of first data point
    bool isleaf; // is this octree node a leaf?
    struct bbox bbox;
    struct xyz centroid; // average position of the points in node (could be
                         // weighted by the power at each point)
    float sumArea;
    struct color sumPower; // sum of rad_t * area of the points in this node
    struct octreeNode *children[8]; // eight children
};


struct GIoctreeNode {
    int     npoints; // number of data points in this octree node
    int     firstpoint; // index of first data point
    bool    isleaf; // is this octree node a leaf?
    struct  bbox bbox;
    struct  xyz centroid; // average position of the points in node (could be
                         // weighted by the power at each point)
    struct  xyz normal;
    struct  color radiance;
    float   sumArea;
    float   dP;
    //float   maxDeviationN;
    struct  GIoctreeNode *children[8]; // eight children
};

//
// Compute the distance from point p to axis-aligned bounding box
//
static float
pointBBoxDist2(struct xyz *p, struct bbox *bbox)
{
    struct xyz c, s; // bbox center and half-side lengths
    struct xyz d; // separation vector
    float dist2;

    // Compute bbox center c
    c.x = 0.5f * (bbox->max.x + bbox->min.x);
    c.y = 0.5f * (bbox->max.y + bbox->min.y);
    c.z = 0.5f * (bbox->max.z + bbox->min.z);

    // Compute bbox half-side lengths s
    s.x = 0.5f * (bbox->max.x - bbox->min.x);
    s.y = 0.5f * (bbox->max.y - bbox->min.y);
    s.z = 0.5f * (bbox->max.z - bbox->min.z);

    // Initial separation vector: d = vector bbox center c to point p
    xyzSUB(d, *p, c); // d = p - c

    // Update separation vector
    if (d.x <= -s.x)
	d.x += s.x;   // outside min bound
    else if (d.x >= s.x)
	d.x -= s.x;   // outside max bound
    else
	d.x = 0.0f;   // inside bounds

    if (d.y <= -s.y)
	d.y += s.y;   // outside min bound
    else if (d.y >= s.y)
	d.y -= s.y;   // outside max bound
    else
	d.y = 0.0f;   // inside bounds

    if (d.z <= -s.z)
	d.z += s.z;   // outside min bound
    else if (d.z >= s.z)
	d.z -= s.z;   // outside max bound
    else
	d.z = 0.0f;   // inside bounds

    dist2 = d.x*d.x + d.y*d.y + d.z*d.z; // dot product d . d
    return dist2;
}



//
// Determine which octree node child the point p belongs in
//
static int
determineChild(struct bbox *bbox, struct xyz *p)
{
    struct xyz midpoint;
    int i, j, k, child;

    assert(bbox->min.x <= p->x && p->x <= bbox->max.x);
    assert(bbox->min.y <= p->y && p->y <= bbox->max.y);
    assert(bbox->min.z <= p->z && p->z <= bbox->max.z);

    // Compute the midpoint of brick bbox
    midpoint.x = 0.5f * (bbox->max.x + bbox->min.x);
    midpoint.y = 0.5f * (bbox->max.y + bbox->min.y);
    midpoint.z = 0.5f * (bbox->max.z + bbox->min.z);

    // Compute which child point p is in: i,j,k
    i = (p->x >= midpoint.x) ? 1 : 0;
    j = (p->y >= midpoint.y) ? 2 : 0;
    k = (p->z >= midpoint.z) ? 4 : 0;
    child = i + j + k;
    assert(0 <= child && child <= 7);
    return child;
}


#endif
