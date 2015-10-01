
// ********************************************************************************************************
// **
// ** PTC Wrapper for Python
// ** (c) 2012 DwarfLabs - Cedric PAILLE
// **
// ********************************************************************************************************

%module ptc

%{
#include "ptcapi.h"

    class pointCloud{
    public:
        pointCloud(): ptc(0) {result = malloc(16*sizeof(float));}
        ~pointCloud(){
            free (result);
            if (ptc)
                PtcClosePointCloudFile(ptc);
        }

        bool read(char* filename){
            ptc = PtcOpenPointCloudFile(filename, &nvars, vartypes, varnames);
            if (ptc){
                int count = 0;

                for (int i = 0; i < nvars; i++){
                    if (strcmp( "color", vartypes[i]) == 0)  {datasize[i] = 3; offset[i] = count; count += 3;}
                    else if (strcmp( "vector", vartypes[i]) == 0) {datasize[i] = 3; offset[i] = count; count += 3;}
                    else if (strcmp( "normal", vartypes[i]) == 0) {datasize[i] = 3; offset[i] = count; count += 3;}
                    else if (strcmp( "point", vartypes[i]) == 0)  {datasize[i] = 3; offset[i] = count; count += 3;}
                    else if (strcmp( "matrix", vartypes[i]) == 0) {datasize[i] = 16; offset[i] = count; count += 16;}
                    else if (strcmp( "float", vartypes[i]) == 0)  {datasize[i] = 1; offset[i] = count; count += 1;}
                    else {
                        PyErr_SetString(PyExc_ValueError, "Unexpected type in pointcloud, closing");
                        PtcClosePointCloudFile(ptc);
                        return NULL;
                    }
                }
            }
            if (ptc) return true; else return false;
        }

        void close(){
            if (ptc)
                PtcClosePointCloudFile(ptc);
            ptc = NULL;
        }

        void* getPoint(){
            if(!ptc) return Py_None;
            PtcReadDataPoint(ptc, point, normal, &radius, data);
            return (void*)this;
        }

        bool setPoint( float *pnt, float *norm, float radius, float *dat ){
            if (!ptc) return false;
            PtcWriteDataPoint( ptc, pnt, norm, radius, dat );
            return true;
        }

        bool write(char* filename, int nvars, char **vartpes, char **varnmes, float *world2eye, float *world2ndc){
            ptc = PtcCreatePointCloudFile(filename, nvars, vartpes, varnmes, world2eye, world2ndc, NULL);
            if (!ptc) return false;
            return true;
        }

        float* getNPoints(){
            if (!ptc) return NULL;
            PtcGetPointCloudInfo( ptc, "npoints", result);
            resultSize = 1;
            return (float*)result;
        }

        float* getBbox(){
            if (!ptc) return NULL;
            PtcGetPointCloudInfo( ptc, "bbox", result);
            resultSize = 6;
            return (float*)result;
        }

        float* getWorld2Eye(){
            if (!ptc) return NULL;
            PtcGetPointCloudInfo( ptc, "world2eye", result);
            resultSize = 16;
            return (float*)result;
        }

        float* getWorld2Ndc(){
            if (!ptc) return NULL;
            PtcGetPointCloudInfo( ptc, "world2ndc", result);
            resultSize = 16;
            return (float*)result;
        }


        int getNVars(){ if(!ptc) return 0; return nvars; }

        const char** getVarInfo(){ if(!ptc) return 0; return vartypes; }


        float point[3], normal[3], radius;


        PtcPointCloud ptc;
        int nvars;
        const char *vartypes[50], *varnames[50];
        int datasize[50];
        int offset[50];
        float data[256];
        int resultSize;
        void* result;
    };
%}

%typemap(out) (const char**){
    int nvars = arg1->nvars;
    PyObject* dict = PyDict_New();

    for (int i = 0; i < nvars;i++){
        PyObject *key = PyString_FromString(arg1->varnames[i]);
        PyObject *val = PyString_FromString(arg1->vartypes[i]);
        PyDict_SetItem( dict, key, val );
    }

    $result = dict;
}

%typemap(out) (void*){
    int nvars = arg1->nvars;

    PyObject* dict = PyDict_New();

    PyObject* keyp = PyString_FromString("point");
    PyObject* keyn = PyString_FromString("normal");
    PyObject* keyr = PyString_FromString("radius");
    PyObject* keyd = PyString_FromString("data");

    PyObject* valtp = PyTuple_New(3);
    PyObject* valtn = PyTuple_New(3);

    for (int i = 0; i < 3; i++){
        PyObject* val = PyFloat_FromDouble(arg1->point[i]);
        PyTuple_SetItem( valtp, i, val);
    }

    for (int i = 0; i < 3; i++){
        PyObject* val = PyFloat_FromDouble(arg1->normal[i]);
        PyTuple_SetItem( valtn, i, val);
    }


    PyObject* datadict = PyDict_New();

    for (int i = 0; i < nvars; i++){
        PyObject* datakey = PyString_FromString(arg1->varnames[i]);
        PyObject* dataval = PyTuple_New(arg1->datasize[i]);
        for (int j = 0; j < arg1->datasize[i]; j++){
            float *dt = arg1->data;
            dt += arg1->offset[i] + j;

            PyObject * value = PyFloat_FromDouble( (float)(*dt) );
            PyTuple_SetItem(dataval, j,  value);
        }
        PyDict_SetItem(datadict, datakey, dataval);
    }

    PyDict_SetItem( dict, keyp, valtp);
    PyDict_SetItem( dict, keyn, valtn);
    PyDict_SetItem( dict, keyr, PyFloat_FromDouble(arg1->radius));
    PyDict_SetItem( dict, keyd, datadict);

    $result = dict;

}

%typemap(in) (float *norm){
    if ( !PyTuple_Check($input)){
        PyErr_SetString(PyExc_ValueError, "Expected a tuple as argument 1");
        return NULL;
    }

    $1 = (float*)alloca(3*sizeof(float));

    $1[0] = PyFloat_AsDouble(PyTuple_GetItem($input, 0));
    $1[1] = PyFloat_AsDouble(PyTuple_GetItem($input, 1));
    $1[2] = PyFloat_AsDouble(PyTuple_GetItem($input, 2));
}

%typemap(in) (float *pnts){
    if ( !PyTuple_Check($input)){
        PyErr_SetString(PyExc_ValueError, "Expected a tuple as argument 2");
        return NULL;
    }

    $1 = (float*)alloca(3*sizeof(float));

    $1[0] = PyFloat_AsDouble(PyTuple_GetItem($input, 0));
    $1[1] = PyFloat_AsDouble(PyTuple_GetItem($input, 1));
    $1[2] = PyFloat_AsDouble(PyTuple_GetItem($input, 2));
}

%typemap(in) (float *data){
    if ( !PyDict_Check($input)){
        PyErr_SetString(PyExc_ValueError, "Expected a dictionnary as argument 4");
        return NULL;
    }

    int dictlen = PyDict_Size($input);

    int datasize = 0;

    for(int i = 0; i < arg1->nvars; i++){
        datasize += arg1->datasize[i];
    }

    $1 = (float*)alloca(datasize*sizeof(float));

    PyObject* keylist = PyDict_Keys($input);

    for (int i = 0; i < dictlen; i++){

        char* key = PyString_AsString(PyList_GetItem(keylist, i));

        for (int j = 0; j < arg1->nvars; j++){
            if ( strcmp(key, arg1->varnames[j]) == 0){
                int dataoffset = arg1->offset[j];
                int datasize   = arg1->datasize[j];

                PyObject* valtuple = PyDict_GetItem($input, PyList_GetItem(keylist, j) );

                if (!PyTuple_Check(valtuple)){
                    PyErr_SetString(PyExc_ValueError, "Expected a map (string,tuple) in data dict (argument 4)");
                    return NULL;
                }

                if (PyTuple_Size(valtuple) != datasize){
                    PyErr_SetString(PyExc_ValueError, "Unexpected length in tuple arg 4");
                    return NULL;
                }

                for (int k = 0; k < datasize; k++){
                    $1[dataoffset+k] = PyFloat_AsDouble( PyTuple_GetItem( valtuple, k) );
                }
            }
        }

    }

}

%typemap(in) (int nvars, char **vartpes, char **varnmes)
{
    if (!PyDict_Check($input)){
        PyErr_SetString(PyExc_ValueError, "Expected a tuple as argument");
        return NULL;
    }

    $1 = PyDict_Size($input);
    arg1->nvars = $1;

    $2 = (char**) alloca( $1 * sizeof(char*));
    $3 = (char**) alloca( $1 * sizeof(char*));

    PyObject* keylist = PyDict_Keys($input);
    int offs
     = 0;
    for (int i = 0; i < $1; i++){
        PyObject* val = PyDict_GetItem($input, PyList_GetItem(keylist, i));
        arg1->varnames[i] = $3[i] = PyString_AsString( PyList_GetItem(keylist, i) );
        arg1->vartypes[i] = $2[i] = PyString_AsString( val );

        if ( strcmp( $2[i], "matrix" ) ==  0){
            arg1->offset[i] = offs;
            arg1->datasize[i] = 16;
            offs += 16;
        }

        if ( strcmp( $2[i], "float" ) ==  0){
            arg1->offset[i] = offs;
            arg1->datasize[i] = 1;
            offs += 1;
        }

        if ( (strcmp( $2[i], "point" ) ==  0) || (strcmp( $2[i], "normal" ) ==  0) || (strcmp( $2[i], "vector" ) ==  0) || (strcmp( $2[i], "color" ) ==  0) ){
            arg1->offset[i] = offs;
            arg1->datasize[i] = 3;
            offs += 3;
        }

    }
}

%typemap(in) (float *world2eye){
    if (!PyTuple_Check($input)){
        PyErr_SetString(PyExc_ValueError, "Expected a tuple as argument");
        return NULL;
    }
    $1 = (float*)alloca( 16* sizeof(float));

    for (int i = 0; i < 16; i++){
        $1[i] = PyFloat_AsDouble( PyTuple_GetItem($input, i) );
    }
}

%typemap(in) (float *world2ndc){
    if (!PyTuple_Check($input)){
        PyErr_SetString(PyExc_ValueError, "Expected a tuple as argument");
        return NULL;
    }
    $1 = (float*)alloca( 16* sizeof(float));

    for (int i = 0; i < 16; i++){
        $1[i] = PyFloat_AsDouble( PyTuple_GetItem($input, i) );
    }
}

%typemap(out) float*
{
    PyObject* ret = PyTuple_New(arg1->resultSize);

    for (int i = 0; i < arg1->resultSize; i++){
        float* farray = (float*)arg1->result;
        PyTuple_SetItem(ret, i, PyFloat_FromDouble( farray[i] ) );
    }

    $result = ret;
}

%typemap(out) int*
{
    PyObject* ret = PyInt_FromLong( ((int*)arg1->result)[0] );

    $result = ret;
}

class pointCloud{
public:
    pointCloud();
    ~pointCloud();

    bool  read(char* filename);
    bool  write(char* filename, int nvars, char **vartpes, char **varnmes, float *world2eye, float *world2ndc);
    void  close();
    void* getPoint();
    bool  setPoint( float *pnts, float *norm, float radius, float *data );
    int   getNVars();
    const char** getVarInfo();

    int*   getNPoints();
    float* getBbox();
    float* getWorld2Eye();
    float* getWorld2Ndc();

// ------ Datas --------
    float point[3], normal[3], radius;
    PtcPointCloud ptc;
    int nvars;
    const char *vartypes[256], *varnames[256];
    float data[256];

};
