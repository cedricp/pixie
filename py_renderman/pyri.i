// ********************************************************************************************************
// **
// ** RenderMan Interface Wrapper for Python
// ** (c) 2012 DwarfLabs - Cedric PAILLE
// **
// ********************************************************************************************************


%module ri

%{
#include "ri.h"

    static int tmp_val;

    static	char			*toLowerCase(char *s) {
        int	i;
        int	l	=	(int) strlen(s);

        for (i=0;i<l;i++) {
            if ((s[i] >= 'A') && (s[i] <= 'Z'))
                s[i]	=	'a'+s[i]-'A';
        }

        return s;
    }

    const	char	*RI_BEZIERBASIS				=	"bezier";
    const	char	*RI_BSPLINEBASIS			=	"b-spline";
    const	char	*RI_POWERBASIS				=	"power";
    const	char	*RI_CATMULLROMBASIS			=	"catmull-rom";
    const	char	*RI_HERMITEBASIS			=	"hermite";

    const char*		RI_BOXFILTER				=	"box";
    const char*		RI_TRIANGLEFILTER			=	"triangle";
    const char*		RI_GAUSSIANFILTER			=	"gaussian";
    const char*		RI_SINCFILTER				=	"sinc";
    const char*		RI_CATMULLROMFILTER			=	"catmull-rom";
    const char*		RI_BLACKMANHARRISFILTER		=	"blackman-harris";
    const char*		RI_MITCHELLFILTER			=	"mitchell";
    const char*		RI_BESSELFILTER				=   "bessel";
    const char*		RI_DISKFILTER				=   "disk";
    const char*		RI_CUSTOM					=	"custom";

    static	int		getBasis(RtBasis **a,char *n) {
        char	*name	=	toLowerCase(n);

        if (strcmp(name,RI_BEZIERBASIS) == 0)
                a[0]	=	&RiBezierBasis;
            else if (strcmp(name,RI_BSPLINEBASIS) == 0)
                a[0]	=	&RiBSplineBasis;
            else if (strcmp(name,RI_CATMULLROMBASIS) == 0)
                a[0]	=	&RiCatmullRomBasis;
            else if (strcmp(name,RI_HERMITEBASIS) == 0)
                a[0]	=	&RiHermiteBasis;
            else if (strcmp(name,RI_POWERBASIS) == 0)
                a[0]	=	&RiPowerBasis;
            else {
                return	0;
        }

        return	1;
    }

    RtFilterFunc			getFilter(const char *name) {
        if (strcmp(name,RI_GAUSSIANFILTER) == 0) {
            return	RiGaussianFilter;
        } else if (strcmp(name,RI_BOXFILTER) == 0) {
            return	RiBoxFilter;
        } else if (strcmp(name,RI_TRIANGLEFILTER) == 0) {
            return	RiTriangleFilter;
        } else if (strcmp(name,RI_SINCFILTER) == 0) {
            return	RiSincFilter;
        } else if (strcmp(name,RI_CATMULLROMFILTER) == 0) {
            return	RiCatmullRomFilter;
        } else if (strcmp(name,RI_BLACKMANHARRISFILTER) == 0) {
            return	RiBlackmanHarrisFilter;
        } else if (strcmp(name,RI_MITCHELLFILTER) == 0) {
            return	RiMitchellFilter;
        } else if (strcmp(name,RI_BESSELFILTER) == 0) {
            return  RiBesselFilter;
        } else if (strcmp(name,RI_DISKFILTER) == 0) {
            return  RiDiskFilter;
        }

        return	RiGaussianFilter;
    }


    void MakeTexture (const char *pic, const char *tex, const char* swrap, const char* twrap, const char* filterfunc, float swidth, float twidth, int n, const char *toks[], const void* parms[]){
        RtFilterFunc ff;
        ff = getFilter(filterfunc);
        RiMakeTextureV( pic, tex, swrap, twrap, ff, swidth, twidth, n, toks, parms);
    }

    void MakeBump (const char *pic, const char *tex, const char* swrap, const char* twrap, const char* filterfunc, float swidth, float twidth, int n, const char *toks[], const void* parms[]){
        RtFilterFunc ff;
        ff = getFilter(filterfunc);
        RiMakeBumpV( pic, tex, swrap, twrap, ff, swidth, twidth, n, toks, parms);
    }

    void MakeLatLongEnvironment (const char *pic, const char *tex, const char* filterfunc, float swidth, float twidth, int n, const char *toks[], const void* parms[]){
        RtFilterFunc ff;
        ff = getFilter(filterfunc);
        RiMakeLatLongEnvironmentV( pic, tex, ff, swidth, twidth, n, toks, parms);
    }

    void MakeSphericalEnvironment (const char *pic, const char *tex, const char* filterfunc, float swidth, float twidth, int n, const char *toks[], const void* parms[]){
        RtFilterFunc ff;
        ff = getFilter(filterfunc);
        RiMakeSphericalEnvironmentV( pic, tex, ff, swidth, twidth, n, toks, parms);
    }

    void MakeCubeFaceEnvironment (const char *px, const char *nx, const char *py, const char *ny, const char *pz, const char *nz, const char *tex, float fov,
                                   const char* filterfunc, float swidth, float twidth, int n, const char *toks[], const void* parms[]){
        RtFilterFunc ff;
        ff = getFilter(filterfunc);
        RiMakeCubeFaceEnvironmentV( px, nx, py, ny, pz, nz, tex, fov, ff, swidth, twidth, n, toks, parms);
    }

    void MakeShadow (const char *pic, const char *tex, int n, const char *toks[], const void* parms[]){
        RiMakeShadowV( pic, tex, n, toks, parms);
    }

    void MakeBrickMap(const char *src, const char *dest, int n, const char *toks[], const void* parms[]){
        const char** filein[1];
        filein[0] = src;
        RiMakeBrickMapV( 1, filein, dest, n, toks, parms);
    }

    void PtFilter(const char *filterType, const char* filename, const char* outfilename, int n, const char *toks[], const void* parms[]){
        RiPtFilterV( filterType, filename, outfilename, n, toks, parms);
    }

    void ObjectInstance (const void* handle){
        RiObjectInstance(  handle );
    }

    void MotionBegin (int N, float times[]){
            RiMotionBeginV( N, times);
    }

    void MotionEnd (void){
        RiMotionEnd();
    }

    const void* ObjectBegin (void){
        const void* inst = RiObjectBegin();
    }

    void ObjectEnd(void){
        RiObjectEnd();
    }

    void Points (int npts,  int n, const char *toks[], const void* parms[]){
        RiPointsV(npts, n, toks, parms);
    }

    void Basis (const char* ubas, int ustep, const char* vbas, int vstep){

        RtBasis *ubasis, *vbasis;
        if ( !getBasis(&ubasis, ubas) && !getBasis(&vbasis, vbas) ){
            PyErr_SetString(PyExc_ValueError, "Bad basis token");
            return NULL;
        }
        RiBasis( ubasis, ustep, vbasis, vstep);
    }

    void Sphere (float radius, float zmin, float zmax, float thetamax, int n, const char* tokens[], const void* params[]){
        RiSphereV( radius, zmin, zmax, thetamax, n, tokens, params);
    }

    void Cone (float height, float radius, float thetamax, int n, const char* tokens[], const void* params[]){
        RiConeV( height, radius, thetamax, n, tokens, params);
    }

    void Cylinder (float radius, float zmin, float zmax, float thetamax, int n, const char* tokens[], const void* params[]){
        RiCylinderV (radius, zmin, zmax, thetamax, n, tokens, params);
    }


    void Disk (float height, float radius, float thetamax, int n, const char* tokens[], const void* params[]){
        RiDiskV( height, radius, thetamax, n, tokens, params);
    }

    void Torus (float majorrad, float minorrad, float phimin, float phimax, float thetamax, int n, const char* tokens[], const void* params[]){
        RiTorusV (majorrad, minorrad, phimin, phimax, thetamax, n, tokens, params);
    }

    void Curves (const char* degree, int ncurves, int nverts[], const char* wrap, int n, const char* tokens[], const void* params[]){
        RiCurves( degree, ncurves, nverts, wrap, n, tokens, params);
    }

    void Geometry (const char* type, int n, const char* tokens[], const void* params[]){
        RiGeometryV( type, n, tokens, params);
    }

    void Projection (const char *ptype, int tokn, const char *tokens[], const void* params[])
    {
        RiProjectionV(ptype, tokn, tokens, params);
    }
    
    void AbcArchive (const char *abcname, int tokn, const char *tokens[], const void* params[])
    {
        RiAbcArchiveV(abcname, tokn, tokens, params);
    }

    void Begin(const char* name)
    {
        RiBegin(name);
    }

    void End()
    {
        RiEnd();
    }

    void FrameBegin(int number)
    {
        RiFrameBegin(number);
    }

    void FrameEnd()
    {
        RiFrameEnd();
    }

    void WorldBegin()
    {
        RiWorldBegin();
    }

    void WorldEnd()
    {
        RiWorldEnd();
    }

    void Display(const char *dname, const char* type, const char* mode, int tokn, const char *tokens[], const void* params[])
    {
        RiDisplay( dname, type, mode, tokn, tokens, params);
    }

    void Format (int xres, int yres, float aspect)
    {
        RiFormat( xres, yres, aspect);
    }

    void FrameAspectRatio (float aspect)
    {
        RiFrameAspectRatio(aspect);
    }

    void ScreenWindow (float left, float right, float bot, float top)
    {
        RiScreenWindow (left, right, bot, top);
    }

    void CropWindow (float xmin, float xmax, float ymin, float ymax)
    {
        RiCropWindow ( xmin, xmax, ymin, ymax);
    }

    void Clipping (float dither, float yon)
    {
        RiClipping (dither, yon);
    }

    void ClippingPlane(float x,float y,float z,float nx,float ny,float nz)
    {
        RiClippingPlane( x, y, z, nx, ny, nz);
    }

    void DepthOfField (float fstop, float focallength, float focaldistance)
    {
        RiDepthOfField( fstop, focallength, focaldistance);
    }

    void Shutter (float smin, float smax)
    {
        RiShutter (smin, smax);
    }

    void PixelVariance (float variation)
    {
        RiPixelVariance(variation);
    }

    void PixelSamples (float xsamples, float ysamples)
    {
        RiPixelSamples( xsamples, ysamples);
    }

    void Exposure (float gain, float gamma)
    {
        RiExposure( gain, gamma );
    }

    void Imager (const char *name, int n, const char* tokens[], const void* params[])
    {
        RiImager (name, n, tokens, params);
    }

    void Quantize (const char* type, int one, int qmin, int qmax, float ampl)
    {
        RiQuantize (type, one, qmin, qmax, ampl);
    }

    void DisplayChannel (const char* channel,int n, const char* tokens[], const void* params[])
    {
        RiDisplayChannel( channel, n, tokens, params );
    }

    void Hider (const char* type, int n, const char* tokens[], const void* params[])
    {
        RiHiderV( type, n, tokens, params);
    }

    void Option (const char *name, int n, const char* tokens[], const void* params[])
    {
        RiOptionV(name, n, tokens, params);
    }

    void AttributeBegin (void)
    {
        RiAttributeBegin();
    }

    void AttributeEnd (void)
    {
        RiAttributeEnd();
    }

    void Color (float Csr, float Csg, float Csb)
    {
        float rgb[3];
        rgb[0] = Csr;
        rgb[1] = Csg;
        rgb[2] = Csb;
        RiColor(rgb);
    }

    void Opacity (float Csr, float Csg, float Csb)
    {
        float rgb[3];
        rgb[0] = Csr;
        rgb[1] = Csg;
        rgb[2] = Csb;
        RiOpacity(rgb);
    }

    void TextureCoordinates (float s1, float t1, float s2, float t2, float s3, float t3, float s4, float t4)
    {
        RiTextureCoordinates( s1, t1, s2, t2, s3, t3, s4, t4);
    }

    const void* LightSource (const char *name, int n, const char* tokens[], const void* params[])
    {
        const void* inst =  RiLightSourceV( name, n, tokens, params);
    }

    const void* AreaLightSource (const char *name, int n, const char* tokens[], const void* params[])
    {
        const void* inst =  RiAreaLightSourceV( name, n, tokens, params);
    }

    void Illuminate (const void* light, short onoff)
    {
        RiIlluminate( light, onoff);
    }

    void Surface (const char *name, int n, const char* tokens[], const void* params[])
    {
        RiSurfaceV( name, n, tokens, params);
    }

    void Atmosphere (const char *name, int n, const char* tokens[], const void* params[])
    {
        RiAtmosphereV( name, n, tokens, params);
    }

    void Interior (const char *name, int n, const char* tokens[], const void* params[])
    {
        RiInteriorV (name, n, tokens, params);
    }

    void Exterior (const char *name, int n, const char* tokens[], const void* params[])
    {
        RiExteriorV (name, n, tokens, params);
    }

    void ShadingRate (float size)
    {
        RiShadingRate( size );
    }

    void ShadingInterpolation (const char* type)
    {
        RiShadingInterpolation( type );
    }

    void Matte (short onoff)
    {
        RiMatte( onoff );
    }

    void Bound (float b1, float b2, float b3, float b4, float b5, float b6)
    {
        float bound[6];
        bound[0] = b1;
        bound[1] = b2;
        bound[2] = b3;
        bound[3] = b4;
        bound[4] = b5;
        bound[5] = b6;
        RiBound( bound );
    }
    void Detail (float b1, float b2, float b3, float b4, float b5, float b6)
    {
        float bound[6];
        bound[0] = b1;
        bound[1] = b2;
        bound[2] = b3;
        bound[3] = b4;
        bound[4] = b5;
        bound[5] = b6;
        RiDetail ( bound );
    }

    void DetailRange (float minvis, float lowtran, float uptran, float maxvis)
    {
        RiDetailRange ( minvis, lowtran, uptran, maxvis );
    }

    void GeometricApproximation (const char* type, float value)
    {
        RiGeometricApproximation( type, value );
    }

    void GeometricRepresentation (const char* type)
    {
        RiGeometricRepresentation( type );
    }

    void Orientation (const char* orientation)
    {
        RiOrientation ( orientation );
    }

    void ReverseOrientation (void)
    {
        RiReverseOrientation();
    }

    void Sides (int nsides)
    {
        RiSides (nsides);
    }

    void Identity (void)
    {
        RiIdentity();
    }

    void Transform (float t0, float t1, float t2, float t3, float t4, float t5, float t6, float t7, float t8, float t9, float t10, float t11, float t12, float t13, float t14, float t15 )
    {
        float matrix[4][4];
        matrix[0][0] = t0;
        matrix[0][1] = t1;
        matrix[0][2] = t2;
        matrix[0][3] = t3;
        matrix[1][0] = t4;
        matrix[1][1] = t5;
        matrix[1][2] = t6;
        matrix[1][3] = t7;;
        matrix[2][0] = t8;
        matrix[2][1] = t9;
        matrix[2][2] = t10;
        matrix[2][3] = t11;
        matrix[3][0] = t12;
        matrix[3][1] = t13;
        matrix[3][2] = t14;
        matrix[3][3] = t15;
        RiTransform(matrix);
    }
    void ConcatTransform (float t0, float t1, float t2, float t3, float t4, float t5, float t6, float t7, float t8, float t9, float t10, float t11, float t12, float t13, float t14, float t15)
    {
        float matrix[4][4];
        matrix[0][0] = t0;
        matrix[0][1] = t1;
        matrix[0][2] = t2;
        matrix[0][3] = t3;
        matrix[1][0] = t4;
        matrix[1][1] = t5;
        matrix[1][2] = t6;
        matrix[1][3] = t7;;
        matrix[2][0] = t8;
        matrix[2][1] = t9;
        matrix[2][2] = t10;
        matrix[2][3] = t11;
        matrix[3][0] = t12;
        matrix[3][1] = t13;
        matrix[3][2] = t14;
        matrix[3][3] = t15;
        RiConcatTransform(matrix);
    }

    void Perspective (float fov)
    {
        RiPerspective (fov);
    }

    void Translate (float dx, float dy, float dz)
    {
        RiTranslate (dx, dy, dz);
    }

    void Rotate (float angle, float dx, float dy, float dz)
    {
        RiRotate(angle, dx, dy, dz);
    }

    void Scale (float dx, float dy, float dz)
    {
        RiScale(dx,dy,dz);
    }

    void Skew (float angle, float dx1, float dy1, float dz1, float dx2, float dy2, float dz2)
    {
        RiSkew( angle, dx1, dy1, dz1, dx2, dy2, dz2);
    }

    void Deformation (const char *name, int n, const char* tokens[], const void* params[])
    {
        RiDeformationV (name, n, tokens, params);
    }

    void Displacement (const char *name, int n, const char* tokens[], const void* params[])
    {
        RiDisplacement( name, n, tokens, params );
    }

    void CoordinateSystem (const char* space)
    {
        RiCoordinateSystem( space );
    }

    void CoordSysTransform (const char* space)
    {
        RiCoordSysTransform( space );
    }

    void TransformBegin (void)
    {
        RiTransformBegin();
    }

    void TransformEnd (void)
    {
        RiTransformEnd();
    }

    void Attribute (const char *name, int n, const char* tokens[], const void* params[])
    {
        RiAttributeV (name, n, tokens, params);
    }

    void Polygon (int nvertices, int tokn, const char* tokens[], const void* params[])
    {
        RiPolygonV ( nvertices, tokn, tokens, params );
    }

    void GeneralPolygon (int n, int *nverts, int tokn, const char* tokens[], const void* params[])
    {
        RiGeneralPolygonV (n, nverts, tokn, tokens, params);
    }

    void PointsPolygons (int n, int *nverts, int *verts, int tokn, const char* tokens[], const void* params[])
    {
        RiPointsPolygonsV (n, nverts, verts, tokn, tokens, params);
    }

    void PointsGeneralPolygons (int n, int *nloops, int *nverts, int *verts, int tokn, const char* tokens[], const void* params[]){
        RiPointsGeneralPolygonsV( n, nloops, nverts, verts, tokn, tokens, params);
    }

    void SubdivisionMesh (const char* scheme, int nfaces, int nvertices[], int vertices[], int ntags, const char* tags[], int nargs[], int intargs[], float floatargs[], int n, const char *toks[], const void* parms[]){
        RiSubdivisionMeshV( scheme, nfaces, nvertices, vertices, ntags, tags, nargs, intargs, floatargs, n, toks, parms);
    }

%}

%typemap(in) (const char* ribname)
{
    $1 = PyString_AsString($input);
}

%typemap(default) (const char* ribname)
{
    $1 = NULL;
}

%typemap(default) (int n, const char *toks[])
{
    $1 =0;
    $2 = NULL;
}


%typemap(default) (const void* parms[])
{
    $1 = NULL;
}

%typemap(in) (int n, const char *toks[], const void* parms[])
{
    int i, j;
    void *valv;

    if (!PyDict_Check($input))
    {
        PyErr_SetString(PyExc_ValueError, "Expecting a dictionnary as argument");
        return NULL;
    }

    $1 = PyDict_Size($input);
    $2 = (char **) alloca(($1+1)*sizeof(char *));
    $3 = (void*)alloca($1*sizeof(void*));

    PyObject* dictitems = PyDict_Keys($input);
    PyObject* mydict = $input;

    for (i = 0; i < $1; i++)
    {
        PyObject *s = PyList_GetItem(dictitems,i);
        if (!PyString_Check(s))
        {
            free($2);
            PyErr_SetString(PyExc_ValueError, "List items must be strings");
            return NULL;
        }
        $2[i] = PyString_AsString(s);
    }
    $2[i] = 0;

    for (i = 0; i < $1; i++)
    {
        PyObject *list = PyDict_GetItem( mydict, PyList_GetItem(dictitems,i) );
        if ( !PyList_Check(list) )
        {
            PyErr_SetString(PyExc_ValueError, "Expecting a list in dictionary");
            return NULL;
        }

        const int listsize = PyList_Size(list);
        $3[i] = (void*)alloca(sizeof(void*)*listsize);
        float* farray = $3[i];
        int*   iarray = $3[i];
        char** carray = $3[i];

        for (j = 0; j < listsize; j++)
        {
            PyObject* item = PyList_GetItem(list,j);

            if (PyFloat_Check(item)){
                float f = PyFloat_AsDouble(item);
                valv = &f;
                memcpy( &(farray[j]), valv, sizeof(float) );
                farray[j] = f;
            } else if (PyString_Check(item)){
                carray[j] = PyString_AsString(item);
            } else if (PyLong_Check(item)){
                int in = PyLong_AsLong(item);
                valv = &in;
                memcpy( &(iarray[j]), valv, sizeof(int) );
            }
        }
    }
}


/*%typemap(freearg) (int n, const char *toks[], const void* parms[])
{
    if ($2) free($2);
}*/

%typemap(in) (int n, int *nverts)
{
    int i, count;
    tmp_val = 0;


    if (!PyList_Check($input))
    {
        PyErr_SetString(PyExc_ValueError, "Expecting a list");
        return NULL;
    }

    $1 = PyList_Size($input);

    $2 = alloca($1 * sizeof(int));


    for (i = 0; i < $1; i++)
    {
        PyObject *s = PyList_GetItem($input,i);
        $2[i] = PyInt_AsLong(s);
        tmp_val += $2[i];
    }
}


%typemap(in) (int *verts)
{
    int i, argsize;

    if (!PyList_Check($input))
    {
        PyErr_SetString(PyExc_ValueError, "Expecting a list");
        return NULL;
    }


    argsize = PyList_Size($input);

    if (tmp_val != argsize)
    {
        PyErr_SetString(PyExc_ValueError, "Nr of vertices incorrect");
        return NULL;
    }

    $1 = alloca(sizeof(int) * argsize);

    for (i = 0; i < argsize; i++)
    {
        PyObject *s = PyList_GetItem($input,i);
        $1[i] = PyInt_AsLong(s);
    }
}

%typemap(in) (int n, int *nloops)
{
    int i, count;
    tmp_val = 0;


    if (!PyList_Check($input))
    {
        PyErr_SetString(PyExc_ValueError, "Expecting a list");
        return NULL;
    }

    $1 = PyList_Size($input);

    $2 = alloca($1 * sizeof(int));


    for (i = 0; i < $1; i++)
    {
        PyObject *s = PyList_GetItem($input,i);
        $2[i] = PyInt_AsLong(s);
        tmp_val += $2[i];
    }
}

%typemap(in) (int *gpnverts)
{
    int i, argsize;

    if (!PyList_Check($input))
    {
        PyErr_SetString(PyExc_ValueError, "Expecting a list");
        return NULL;
    }


    argsize = PyList_Size($input);

    if (tmp_val != argsize)
    {
        PyErr_SetString(PyExc_ValueError, "Nr nverts incorrect");
        return NULL;
    }
    tmp_val = 0;

    $1 = alloca(sizeof(int) * argsize);

    for (i = 0; i < argsize; i++)
    {
        PyObject *s = PyList_GetItem($input,i);
        $1[i] = PyInt_AsLong(s);
        tmp_val += $1[i];
    }
}

%typemap(in) (int *gpverts)
{
    int i, argsize;

    if (!PyList_Check($input))
    {
        PyErr_SetString(PyExc_ValueError, "Expecting a list");
        return NULL;
    }


    argsize = PyList_Size($input);

    if (tmp_val != argsize)
    {
        PyErr_SetString(PyExc_ValueError, "Nr of verts incorrect");
        return NULL;
    }

    $1 = alloca(sizeof(int) * argsize);

    for (i = 0; i < argsize; i++)
    {
        PyObject *s = PyList_GetItem($input,i);
        $1[i] = PyInt_AsLong(s);
    }
}

%typemap(in) (int N, float times[]){
    int i, count;
    tmp_val = 0;


    if (!PyList_Check($input))
    {
        PyErr_SetString(PyExc_ValueError, "Expecting a list");
        return NULL;
    }

    $1 = PyList_Size($input);

    $2 = alloca($1 * sizeof(float));


    for (i = 0; i < $1; i++)
    {
        PyObject *s = PyList_GetItem($input,i);
        $2[i] = PyFloat_AsDouble(s);
        tmp_val += $2[i];
    }
}

%typemap(in) (int nfaces, int nvertices[])
{
    int i, count;
    tmp_val = 0;


    if (!PyList_Check($input))
    {
        PyErr_SetString(PyExc_ValueError, "Expecting a list");
        return NULL;
    }

    $1 = PyList_Size($input);

    $2 = alloca($1 * sizeof(int));


    for (i = 0; i < $1; i++)
    {
        PyObject *s = PyList_GetItem($input,i);
        $2[i] = PyInt_AsLong(s);
        tmp_val += $2[i];
    }
}

%typemap(in) (int vertices[])
{
    int i, argsize;

    if (!PyList_Check($input))
    {
        PyErr_SetString(PyExc_ValueError, "Expecting a list");
        return NULL;
    }


    argsize = PyList_Size($input);

    if (tmp_val != argsize)
    {
        PyErr_SetString(PyExc_ValueError, "Nr vertices incorrect");
        return NULL;
    }

    $1 = alloca(sizeof(int) * argsize);

    for (i = 0; i < argsize; i++)
    {
        PyObject *s = PyList_GetItem($input,i);
        $1[i] = PyInt_AsLong(s);
    }
}

%typemap(in) (int ntags, const char* tags[])
{
    int i, j;
    void *valv;

    if (!PyList_Check($input))
    {
        PyErr_SetString(PyExc_ValueError, "Expecting a list as argument");
        return NULL;
    }

    $1 = PyList_Size($input);
    $2 = (char **) alloca(($1+1)*sizeof(char *));

    for (i = 0; i < $1; i++)
    {
        PyObject *s = PyList_GetItem($input,i);
        if (!PyString_Check(s))
        {
            free($2);
            PyErr_SetString(PyExc_ValueError, "List items must be strings");
            return NULL;
        }
        $2[i] = PyString_AsString(s);
    }
    $2[i] = 0;
}

%typemap(in) (int nargs[])
{
    int i, argsize;

    if (!PyList_Check($input))
    {
        PyErr_SetString(PyExc_ValueError, "Expecting a list");
        return NULL;
    }


    argsize = PyList_Size($input);


    $1 = alloca(sizeof(int) * argsize);

    for (i = 0; i < argsize; i++)
    {
        PyObject *s = PyList_GetItem($input,i);
        $1[i] = PyInt_AsLong(s);
    }
}

%typemap(in) (int intargs[])
{
    int i, argsize;

    if (!PyList_Check($input))
    {
        PyErr_SetString(PyExc_ValueError, "Expecting a list");
        return NULL;
    }


    argsize = PyList_Size($input);


    $1 = alloca(sizeof(int) * argsize);

    for (i = 0; i < argsize; i++)
    {
        PyObject *s = PyList_GetItem($input,i);
        $1[i] = PyInt_AsLong(s);
    }
}

%typemap(in) (float floatargs[])
{
    int i, argsize;

    if (!PyList_Check($input))
    {
        PyErr_SetString(PyExc_ValueError, "Expecting a list");
        return NULL;
    }


    argsize = PyList_Size($input);


    $1 = alloca(sizeof(float) * argsize);

    for (i = 0; i < argsize; i++)
    {
        PyObject *s = PyList_GetItem($input,i);
        $1[i] = PyFloat_AsDouble(s);
    }
}


void Begin (const char* ribname);
void End (void);
void FrameBegin (int number);
void FrameEnd (void);
void WorldBegin (void);
void WorldEnd (void);
void Display(const char *dname, const char* type, const char* mode, int n, const char *toks[], const void* parms[]);
void Projection (const char *dname, int n, const char *toks[], const void* parms[]);
void AbcArchive (const char *fname, int n, const char *toks[], const void* parms[]);
void Format (int xres, int yres, float aspect);
void FrameAspectRatio (float aspect);
void ScreenWindow (float left, float right, float bot, float top);
void CropWindow (float xmin, float xmax, float ymin, float ymax);
void Clipping (float hither, float yon);
void ClippingPlane(float x,float y,float z,float nx,float ny,float nz);
void DepthOfField (float fstop, float focallength, float focaldistance);
void Shutter (float smin, float smax);
void PixelVariance (float variation);
void PixelSamples (float xsamples, float ysamples);
void Exposure (float gain, float gamma);
void Imager (const char *name, int n, const char *toks[], const void* parms[]);
void Quantize (const char* type, int one, int qmin, int qmax, float ampl);
void DisplayChannel (const char* channel,int n, const char *toks[], const void* parms[]);

void Hider (const char* type, int n, const char *toks[], const void* parms[]);
void Option (const char *name, int n, const char *toks[], const void* parms[]);

void AttributeBegin (void);
void AttributeEnd (void);
void Color (float Csr, float Csg, float Csb);
void Opacity (float Csr, float Csg, float Csb);
void TextureCoordinates (float s1, float t1, float s2, float t2, float s3, float t3, float s4, float t4);
const void* LightSource (const char *name, int n, const char *toks[], const void* parms[]);
const void* AreaLightSource (const char *name, int n, const char *toks[], const void* parms[]);

void Illuminate (const void* light, short onoff);
void Surface (const char *name, int n, const char *toks[], const void* parms[]);
void Atmosphere (const char *name, int n, const char *toks[], const void* parms[]);
void Interior (const char *name, int n, const char *toks[], const void* parms[]);
void Exterior (const char *name, int n, const char *toks[], const void* parms[]);
void ShadingRate (float size);
void ShadingInterpolation (const char* type);
void Matte (short onoff);

void Bound (float b1, float b2, float b3, float b4, float b5, float b6);
void Detail (float b1, float b2, float b3, float b4, float b5, float b6);
void DetailRange (float minvis, float lowtran, float uptran, float maxvis);
void GeometricApproximation (const char* type, float value);
void GeometricRepresentation (const char* type);
void Orientation (const char* orientation);
void ReverseOrientation (void);
void Sides (int nsides);


void Identity (void);
void Transform (float t0, float t1, float t2, float t3, float t4, float t5, float t6, float t7, float t8, float t9, float t10, float t11, float t12, float t13, float t14, float t15 );
void ConcatTransform (float t0, float t1, float t2, float t3, float t4, float t5, float t6, float t7, float t8, float t9, float t10, float t11, float t12, float t13, float t14, float t15);
void Perspective (float fov);
void Translate (float dx, float dy, float dz);
void Rotate (float angle, float dx, float dy, float dz);
void Scale (float dx, float dy, float dz);
void Skew (float angle, float dx1, float dy1, float dz1, float dx2, float dy2, float dz2);
void Deformation (const char *name, int n, const char *toks[], const void* parms[]);
void Displacement (const char *name, int n, const char *toks[], const void* parms[]);
void CoordinateSystem (const char* space);
void CoordSysTransform (const char* space);

void TransformBegin (void);
void TransformEnd (void);
void Attribute (const char *name, int n, const char *toks[], const void* parms[]);

void Polygon (int nvertices, int n, const char *toks[], const void* parms[]);
void GeneralPolygon (int n, int *nverts, int n, const char *toks[], const void* parms[]);
void PointsPolygons (int n, int *nverts, int *verts, int n, const char *toks[], const void* parms[]);
void PointsGeneralPolygons (int n, int *nloops, int *gpnverts, int *gpverts, int n, const char *toks[], const void* parms[]);

void Points (int npts,  int n, const char *toks[], const void* parms[]);

void SubdivisionMesh (const char* scheme, int nfaces, int nvertices[], int vertices[], int ntags, const char* tags[], int nargs[], int intargs[], float floatargs[], int n, const char *toks[], const void* parms[]);

void Basis (const char* ubasis, int ustep, const char* vbasis, int vstep);

void Sphere (float radius, float zmin, float zmax, float thetamax, int n, const char *toks[], const void* parms[]);
void Cone (float height, float radius, float thetamax, int n, const char *toks[], const void* parms[]);
void Cylinder (float radius, float zmin, float zmax, float thetamax, int n, const char *toks[], const void* parms[]);
void Disk (float height, float radius, float thetamax, int n, const char *toks[], const void* parms[]);
void Torus (float majorrad, float minorrad, float phimin, float phimax, float thetamax, int n, const char *toks[], const void* parms[]);
void Curves (const char* degree, int ncurves, int nverts[], const char* wrap, int n, const char *toks[], const void* parms[]);
void Geometry (const char* type, int n, const char *toks[], const void* parms[]);

const void* ObjectBegin (void);
void ObjectEnd (void);
void ObjectInstance (const void* handle);
void MotionBegin (int N, float times[]);
void MotionEnd (void);

void MakeTexture (const char *pic, const char *tex, const char* swrap, const char* twrap, const char* filterfunc, float swidth, float twidth, int n, const char *toks[], const void* parms[]);
void MakeBump (const char *pic, const char *tex, const char* swrap, const char* twrap, const char* filterfunc, float swidth, float twidth, int n, const char *toks[], const void* parms[]);
void MakeLatLongEnvironment (const char *pic, const char *tex, const char* filterfunc, float swidth, float twidth, int n, const char *toks[], const void* parms[]);
void MakeSphericalEnvironment (const char *pic, const char *tex, const char* filterfunc, float swidth, float twidth, int n, const char *toks[], const void* parms[]);
void MakeCubeFaceEnvironment (const char *px, const char *nx, const char *py, const char *ny, const char *pz, const char *nz, const char *tex, float fov, const char* filterfunc, float swidth, float twidth, int n, const char *toks[], const void* parms[]);
void MakeShadow (const char *pic, const char *tex, int n, const char *toks[], const void* parms[]);
void MakeBrickMap(const char *src, const char *dest, int n, const char *toks[], const void* parms[]);
void PtFilter(const char *filterType, const char* filename, const char* outfilename, int n, const char *toks[], const void* parms[]);


