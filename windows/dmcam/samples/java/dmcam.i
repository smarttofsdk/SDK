%include "stdint.i"
%include "carrays.i"
%include "typemaps.i"
%include "various.i"
/*  mod name */
%module dmcam

%rename("%(strip:[dmcam_])s") "";
/* ignore api with callback  */
%ignore dmcam_cap_set_callback_frame_ready;

%{
    #include "dmcam.h"
%}

#ifdef SWIGJAVA
%javaconst(1);
%include "arrays_java.i"

%pragma(java) jniclasscode=%{
    static {
        System.loadLibrary("dmcam_java");
        init(null);
    }
/*%array_class(dmcam_param_item_t, dmcam_param_item_list)*/

%}

%array_class(dmcam_dev_t, dmcamDevArray);
%array_class(dmcam_param_item_t, dmcamParamArray);
/*%cstring_output_maxsize(uint8_t *frame_data, uint32_t frame_dlen);*/
/*  frame buffer setting, raw2xxx  */
%apply unsigned char* NIOBUFFER{ uint8_t *frame_data }
%apply unsigned char* NIOBUFFER{ uint8_t *src}
%apply float* INOUT{ float *dst }
/*  dmcam_cmap_xx  */
%apply unsigned char* NIOBUFFER{ uint8_t *dst}
%apply float* INOUT{const float *src}

// depth_to_pcl 
%apply float* INOUT{ float *pcl}
%apply unsigned char* NIOBUFFER{ uint8_t *srcdata}
#endif
#ifdef SWIGPYTHON
/* rename api containing callback functions */
%rename(cap_set_callback_on_frame_ready) py_dmcam_cap_set_callback_on_frame_ready;
%rename(cap_set_callback_on_error) py_dmcam_cap_set_callback_on_error;

/*  api parameter convert */
%include <pybuffer.i>
/*%pybuffer_string(const char *filename);*/
%pybuffer_mutable_binary(uint8_t *frame_data, uint32_t frame_dlen);
%pybuffer_binary(uint8_t *src, int src_len);

%{
    #include "dmcam.h"
    /* -------- modify dmcam_cap_frame_callback ------ */
    /*static PyObject *(PyObject*)dev->user_data0 = NULL;*/
    static void on_cap_frdy(dmcam_dev_t* dev, dmcam_frame_t* frame)
    {
        PyObject *arglist, *result, *devobj, *frameobj;

        SWIG_PYTHON_THREAD_BEGIN_BLOCK;
        devobj = SWIG_NewPointerObj((void*)dev, SWIGTYPE_p_dmcam_dev_t, 0);
        frameobj = SWIG_NewPointerObj((void*)frame, SWIGTYPE_p_dmcam_frame_t, 0);

        arglist = Py_BuildValue("(OO)", devobj, frameobj);
        result =  PyEval_CallObject((PyObject*)dev->user_data0, arglist);
        /*printf("frame @ %p, %d, %d\n", frame, frame->frame_fbpos, frame->frame_count);*/
        Py_DECREF(devobj);
        Py_DECREF(frameobj);
        Py_DECREF(arglist);
        Py_XDECREF(result);
        SWIG_PYTHON_THREAD_END_BLOCK;

        return /*void*/;
    }
    static bool on_cap_error(dmcam_dev_t* dev, int err, void* err_arg)
    {
        bool ret;
        PyObject *arglist, *result, *devobj, *err_argobj;

        SWIG_PYTHON_THREAD_BEGIN_BLOCK;
        devobj = SWIG_NewPointerObj((void*)dev, SWIGTYPE_p_dmcam_dev_t, 0);
        err_argobj = SWIG_NewPointerObj(err_arg, SWIGTYPE_p_void, 0);

        arglist = Py_BuildValue("(OiO)", devobj, err, err_argobj);
        result =  PyEval_CallObject((PyObject*)dev->user_data1, arglist);

        ret = result == Py_True;

        Py_DECREF(devobj);
        Py_DECREF(err_argobj);
        Py_DECREF(arglist);
        Py_XDECREF(result);

        SWIG_PYTHON_THREAD_END_BLOCK;

        return ret;
    }

    void py_dmcam_cap_set_callback_on_frame_ready(dmcam_dev_t* dev, PyObject *PyFunc)
    {
        SWIG_PYTHON_THREAD_BEGIN_BLOCK;
        Py_XDECREF((PyObject*)dev->user_data0);          /* Dispose of previous callback */
        Py_XINCREF(PyFunc);         /* Add a reference to new callback */
        dev->user_data0 = (void*)PyFunc;         /* Remember new callback */
        SWIG_PYTHON_THREAD_END_BLOCK;
        /*  call c to register callback  */
        dmcam_cap_set_callback_on_frame_ready(dev, on_cap_frdy);
        /*{*/
            /*// test*/
            /*dmcam_frame_t f = { 0 };*/
            /*on_cap_frdy(dev, &f);*/
        /*}*/
    }
    void py_dmcam_cap_set_callback_on_error(dmcam_dev_t* dev, PyObject *PyFunc)
    {
        SWIG_PYTHON_THREAD_BEGIN_BLOCK;
        Py_XDECREF((PyObject*)dev->user_data1);          /* Dispose of previous callback */
        Py_XINCREF(PyFunc);         /* Add a reference to new callback */
        dev->user_data1 = (void*)PyFunc;         /* Remember new callback */
        SWIG_PYTHON_THREAD_END_BLOCK;
        /*  call c to register callback  */
        dmcam_cap_set_callback_on_error(dev, on_cap_error);
        /*{*/
            /*// test*/
            /*dmcam_frame_t f = { 0 };*/
            /*on_cap_error(dev, &f);*/
        /*}*/
    }
%}


%typemap(in) PyObject *PyFunc {
  if (!PyCallable_Check($input)) {
      PyErr_SetString(PyExc_TypeError, "Need a callable object!");
      return NULL;
  }
  $1 = $input;
}
/*  python wrapper functions  */
void py_dmcam_cap_set_callback_on_frame_ready(dmcam_dev_t* dev, PyObject *PyFunc);
void py_dmcam_cap_set_callback_on_error(dmcam_dev_t* dev, PyObject *PyFunc);

/* ---- swig python numpy parts ----- */
%{
    #define SWIG_FILE_WITH_INIT
%}

%include "numpy.i"

%init %{
    import_array();
%}

%numpy_typemaps(int8_t       , NPY_BYTE     , int)
%numpy_typemaps(uint8_t     , NPY_UBYTE    , int)
%numpy_typemaps(int16_t             , NPY_SHORT    , int)
%numpy_typemaps(uint16_t    , NPY_USHORT   , int)
%numpy_typemaps(int32_t               , NPY_INT      , int)
%numpy_typemaps(uint32_t      , NPY_UINT     , int)
/*%numpy_typemaps(long              , NPY_LONG     , int)*/
/*%numpy_typemaps(unsigned long     , NPY_ULONG    , int)*/
%numpy_typemaps(int64_t         , NPY_LONGLONG , int)
%numpy_typemaps(uint64_t, NPY_ULONGLONG, int)
%numpy_typemaps(float             , NPY_FLOAT    , int)
%numpy_typemaps(double            , NPY_DOUBLE   , int)
/* ---- swig numpy.i ----- */

/*  dmcam_dev_list  */
/*%array_class(dmcam_dev_t, dmcam_dev_array)*/
/*%array_class(dmcam_param_item_t, dmcam_param_item_list)*/

/*  dmcam_raw2xxxx funcs  */
%apply (float* ARGOUT_ARRAY1, int DIM1) {(float *dst, int dst_len)}
/*  dmcam_reg_batch_read/write */
%apply (uint32_t* IN_ARRAY1, int DIM1) {(const uint32_t *reg_vals, uint16_t reg_vals_len)}
%apply (uint32_t* ARGOUT_ARRAY1, int DIM1) {(uint32_t *reg_vals, uint16_t reg_vals_len)}

%apply (float* ARGOUT_ARRAY1, int DIM1) {(float *pcl, int pcl_len)}
%apply (float* IN_ARRAY1, int DIM1) {(const float *dist, int dist_len)}
/*%apply (float* IN_ARRAY2, int DIM1, int DIM2) {(const float *srcdata, int img_h, int img_w)}*/

/*  dmcam cmap apis */
%apply (uint8_t* ARGOUT_ARRAY1, int DIM1) {(uint8_t *dst, int dst_len)}
%apply (float* IN_ARRAY1, int DIM1) {(const float *src, int src_len)}

/*  param_batch_get  */
%typemap(in) (dmcam_param_item_t *param_items, int item_cnt) {
    int i, seq_len;
    if (!PySequence_Check($input)) {
        PyErr_SetString(PyExc_ValueError, "Expected a sequence");
        SWIG_fail;
    }
    seq_len = PySequence_Length($input);
    $1 = (dmcam_param_item_t*) malloc(seq_len * sizeof(dmcam_param_item_t));
    memset($1, 0, seq_len * sizeof(dmcam_param_item_t));
    for (i = 0; i < seq_len; i++) {
        PyObject *o = PySequence_GetItem($input, i);
        if (PyNumber_Check(o)) {
            $1[i].param_id = (dmcam_dev_param_e) PyInt_AsLong(o);
        } else {
            free($1);
            PyErr_SetString(PyExc_ValueError, "Sequence elements must be numbers");
            SWIG_fail;
        }
    }
    $2 = seq_len;
}
%typemap(freearg) (dmcam_param_item_t *param_items, int item_cnt) {
    if ($1) free($1);
}
%typemap(argout) (dmcam_param_item_t *param_items, int item_cnt) {
    /* clear previous result  */
    Py_DECREF($result);
    if (result == true) {
        int i;
        int seq_len = PySequence_Length($input);
        /*$result = SWIG_NewPointerObj($1, SWIGTYPE_p_dmcam_param_item_list, SWIG_POINTER_NEW);*/
        $result = PyTuple_New(seq_len);
        for (i = 0; i < seq_len; i++) {
            PyObject* param_val_obj = SWIG_NewPointerObj((void*)memcpy((void*)malloc(sizeof(dmcam_param_val_u)),&$1[i].param_val,sizeof(dmcam_param_val_u)), 
                    SWIGTYPE_p_dmcam_param_val_u, SWIG_POINTER_OWN |  0 );
            PyTuple_SetItem($result, i, param_val_obj);
        }
    } else {
        /*if ($1) free($1);*/
        $result = SWIG_Py_Void();
    }
}

/*  param_batch_set  */
%typemap(in) (const dmcam_param_item_t *param_items, int item_cnt) {
    int dict_len, i = 0;
    PyObject *key, *value;
    Py_ssize_t pos = 0;

    if (!PyDict_Check($input)) {
        PyErr_SetString(PyExc_ValueError, "Expected a map (dict)");
        SWIG_fail;
    }
    dict_len = PyDict_Size($input);
    $1 = (dmcam_param_item_t*) malloc(dict_len * sizeof(dmcam_param_item_t));
    memset($1, 0, dict_len * sizeof(dmcam_param_item_t));

    while (PyDict_Next($input, &pos, &key, &value)) {
        int res1;
        void* p_param_val;
        $1[i].param_id = (dmcam_dev_param_e) PyInt_AsLong(key);
        $1[i].param_val_len = sizeof(dmcam_param_val_u);
        res1 = SWIG_ConvertPtr(value, &p_param_val, SWIGTYPE_p_dmcam_param_val_u, 0 |  0 );
        if (!SWIG_IsOK(res1)) {
            SWIG_exception_fail(SWIG_ArgError(res1), "Dict value should be dmcam_param_val_u object"); 
        }
        memcpy(&$1[i].param_val, p_param_val, sizeof(dmcam_param_val_u));
        i++;
    }
    $2 = dict_len;
}
%typemap(freearg) (const dmcam_param_item_t *param_items, int item_cnt) {
    if ($1) free($1);
}

/*  serial[3] acess */
%typemap(out) uint32_t [ANY] {
  int i;
  $result = PyList_New($1_dim0);
  for (i = 0; i < $1_dim0; i++) {
    PyObject *o = PyLong_FromUnsignedLong($1[i]);
    PyList_SetItem($result,i,o);
  }
}

/*  dmcam_dev_list  */
%typemap(in, numinputs=0) (dmcam_dev_t *dev_list, int dev_list_num) {
    int seq_len = 16; // max 16 devices
    /*if (!PyNumber_Check($input)) {*/
        /*PyErr_SetString(PyExc_ValueError, "Expected a number");*/
        /*SWIG_fail;*/
    /*}*/
    /*seq_len = PyInt_AsLong($input);*/
    $1 = (dmcam_dev_t*) malloc(seq_len * sizeof(dmcam_dev_t));
    memset($1, 0, seq_len * sizeof(dmcam_dev_t));
    $2 = seq_len;
}
%typemap(freearg) (dmcam_dev_t *dev_list, int dev_list_num) {
    if ($1) free($1);
}
%typemap(argout) (dmcam_dev_t *dev_list, int dev_list_num) {
    /* clear previous result  */
    Py_DECREF($result);
    if (result > 0) {
        int i;
        /*$result = SWIG_NewPointerObj($1, SWIGTYPE_p_dmcam_param_item_list, SWIG_POINTER_NEW);*/
        $result = PyTuple_New(result);
        for (i = 0; i < result; i++) {
            PyObject* param_val_obj = SWIG_NewPointerObj((void*)memcpy((void*)malloc(sizeof(dmcam_dev_t)),&$1[i],sizeof(dmcam_dev_t)), 
                    SWIGTYPE_p_dmcam_dev_t, SWIG_POINTER_OWN |  0 );
            PyTuple_SetItem($result, i, param_val_obj);
        }
    } else {
        /*if ($1) free($1);*/
        $result = SWIG_Py_Void();
    }
}
#endif
/*  Parse the header file to generate wrappers */
%include "../../src/dmcam.h"
/*%include "../src/dmcam_dfu.h"*/

