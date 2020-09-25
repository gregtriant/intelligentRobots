#define _CFFI_

/* We try to define Py_LIMITED_API before including Python.h.

   Mess: we can only define it if Py_DEBUG, Py_TRACE_REFS and
   Py_REF_DEBUG are not defined.  This is a best-effort approximation:
   we can learn about Py_DEBUG from pyconfig.h, but it is unclear if
   the same works for the other two macros.  Py_DEBUG implies them,
   but not the other way around.
*/
#ifndef _CFFI_USE_EMBEDDING
#  include <pyconfig.h>
#  if !defined(Py_DEBUG) && !defined(Py_TRACE_REFS) && !defined(Py_REF_DEBUG)
#    define Py_LIMITED_API
#  endif
#endif

#include <Python.h>
#ifdef __cplusplus
extern "C" {
#endif
#include <stddef.h>

/* This part is from file 'cffi/parse_c_type.h'.  It is copied at the
   beginning of C sources generated by CFFI's ffi.set_source(). */

typedef void *_cffi_opcode_t;

#define _CFFI_OP(opcode, arg)   (_cffi_opcode_t)(opcode | (((uintptr_t)(arg)) << 8))
#define _CFFI_GETOP(cffi_opcode)    ((unsigned char)(uintptr_t)cffi_opcode)
#define _CFFI_GETARG(cffi_opcode)   (((intptr_t)cffi_opcode) >> 8)

#define _CFFI_OP_PRIMITIVE       1
#define _CFFI_OP_POINTER         3
#define _CFFI_OP_ARRAY           5
#define _CFFI_OP_OPEN_ARRAY      7
#define _CFFI_OP_STRUCT_UNION    9
#define _CFFI_OP_ENUM           11
#define _CFFI_OP_FUNCTION       13
#define _CFFI_OP_FUNCTION_END   15
#define _CFFI_OP_NOOP           17
#define _CFFI_OP_BITFIELD       19
#define _CFFI_OP_TYPENAME       21
#define _CFFI_OP_CPYTHON_BLTN_V 23   // varargs
#define _CFFI_OP_CPYTHON_BLTN_N 25   // noargs
#define _CFFI_OP_CPYTHON_BLTN_O 27   // O  (i.e. a single arg)
#define _CFFI_OP_CONSTANT       29
#define _CFFI_OP_CONSTANT_INT   31
#define _CFFI_OP_GLOBAL_VAR     33
#define _CFFI_OP_DLOPEN_FUNC    35
#define _CFFI_OP_DLOPEN_CONST   37
#define _CFFI_OP_GLOBAL_VAR_F   39
#define _CFFI_OP_EXTERN_PYTHON  41

#define _CFFI_PRIM_VOID          0
#define _CFFI_PRIM_BOOL          1
#define _CFFI_PRIM_CHAR          2
#define _CFFI_PRIM_SCHAR         3
#define _CFFI_PRIM_UCHAR         4
#define _CFFI_PRIM_SHORT         5
#define _CFFI_PRIM_USHORT        6
#define _CFFI_PRIM_INT           7
#define _CFFI_PRIM_UINT          8
#define _CFFI_PRIM_LONG          9
#define _CFFI_PRIM_ULONG        10
#define _CFFI_PRIM_LONGLONG     11
#define _CFFI_PRIM_ULONGLONG    12
#define _CFFI_PRIM_FLOAT        13
#define _CFFI_PRIM_DOUBLE       14
#define _CFFI_PRIM_LONGDOUBLE   15

#define _CFFI_PRIM_WCHAR        16
#define _CFFI_PRIM_INT8         17
#define _CFFI_PRIM_UINT8        18
#define _CFFI_PRIM_INT16        19
#define _CFFI_PRIM_UINT16       20
#define _CFFI_PRIM_INT32        21
#define _CFFI_PRIM_UINT32       22
#define _CFFI_PRIM_INT64        23
#define _CFFI_PRIM_UINT64       24
#define _CFFI_PRIM_INTPTR       25
#define _CFFI_PRIM_UINTPTR      26
#define _CFFI_PRIM_PTRDIFF      27
#define _CFFI_PRIM_SIZE         28
#define _CFFI_PRIM_SSIZE        29
#define _CFFI_PRIM_INT_LEAST8   30
#define _CFFI_PRIM_UINT_LEAST8  31
#define _CFFI_PRIM_INT_LEAST16  32
#define _CFFI_PRIM_UINT_LEAST16 33
#define _CFFI_PRIM_INT_LEAST32  34
#define _CFFI_PRIM_UINT_LEAST32 35
#define _CFFI_PRIM_INT_LEAST64  36
#define _CFFI_PRIM_UINT_LEAST64 37
#define _CFFI_PRIM_INT_FAST8    38
#define _CFFI_PRIM_UINT_FAST8   39
#define _CFFI_PRIM_INT_FAST16   40
#define _CFFI_PRIM_UINT_FAST16  41
#define _CFFI_PRIM_INT_FAST32   42
#define _CFFI_PRIM_UINT_FAST32  43
#define _CFFI_PRIM_INT_FAST64   44
#define _CFFI_PRIM_UINT_FAST64  45
#define _CFFI_PRIM_INTMAX       46
#define _CFFI_PRIM_UINTMAX      47

#define _CFFI__NUM_PRIM         48
#define _CFFI__UNKNOWN_PRIM           (-1)
#define _CFFI__UNKNOWN_FLOAT_PRIM     (-2)
#define _CFFI__UNKNOWN_LONG_DOUBLE    (-3)

#define _CFFI__IO_FILE_STRUCT         (-1)


struct _cffi_global_s {
    const char *name;
    void *address;
    _cffi_opcode_t type_op;
    void *size_or_direct_fn;  // OP_GLOBAL_VAR: size, or 0 if unknown
                              // OP_CPYTHON_BLTN_*: addr of direct function
};

struct _cffi_getconst_s {
    unsigned long long value;
    const struct _cffi_type_context_s *ctx;
    int gindex;
};

struct _cffi_struct_union_s {
    const char *name;
    int type_index;          // -> _cffi_types, on a OP_STRUCT_UNION
    int flags;               // _CFFI_F_* flags below
    size_t size;
    int alignment;
    int first_field_index;   // -> _cffi_fields array
    int num_fields;
};
#define _CFFI_F_UNION         0x01   // is a union, not a struct
#define _CFFI_F_CHECK_FIELDS  0x02   // complain if fields are not in the
                                     // "standard layout" or if some are missing
#define _CFFI_F_PACKED        0x04   // for CHECK_FIELDS, assume a packed struct
#define _CFFI_F_EXTERNAL      0x08   // in some other ffi.include()
#define _CFFI_F_OPAQUE        0x10   // opaque

struct _cffi_field_s {
    const char *name;
    size_t field_offset;
    size_t field_size;
    _cffi_opcode_t field_type_op;
};

struct _cffi_enum_s {
    const char *name;
    int type_index;          // -> _cffi_types, on a OP_ENUM
    int type_prim;           // _CFFI_PRIM_xxx
    const char *enumerators; // comma-delimited string
};

struct _cffi_typename_s {
    const char *name;
    int type_index;   /* if opaque, points to a possibly artificial
                         OP_STRUCT which is itself opaque */
};

struct _cffi_type_context_s {
    _cffi_opcode_t *types;
    const struct _cffi_global_s *globals;
    const struct _cffi_field_s *fields;
    const struct _cffi_struct_union_s *struct_unions;
    const struct _cffi_enum_s *enums;
    const struct _cffi_typename_s *typenames;
    int num_globals;
    int num_struct_unions;
    int num_enums;
    int num_typenames;
    const char *const *includes;
    int num_types;
    int flags;      /* future extension */
};

struct _cffi_parse_info_s {
    const struct _cffi_type_context_s *ctx;
    _cffi_opcode_t *output;
    unsigned int output_size;
    size_t error_location;
    const char *error_message;
};

struct _cffi_externpy_s {
    const char *name;
    size_t size_of_result;
    void *reserved1, *reserved2;
};

#ifdef _CFFI_INTERNAL
static int parse_c_type(struct _cffi_parse_info_s *info, const char *input);
static int search_in_globals(const struct _cffi_type_context_s *ctx,
                             const char *search, size_t search_len);
static int search_in_struct_unions(const struct _cffi_type_context_s *ctx,
                                   const char *search, size_t search_len);
#endif

/* this block of #ifs should be kept exactly identical between
   c/_cffi_backend.c, cffi/vengine_cpy.py, cffi/vengine_gen.py
   and cffi/_cffi_include.h */
#if defined(_MSC_VER)
# include <malloc.h>   /* for alloca() */
# if _MSC_VER < 1600   /* MSVC < 2010 */
   typedef __int8 int8_t;
   typedef __int16 int16_t;
   typedef __int32 int32_t;
   typedef __int64 int64_t;
   typedef unsigned __int8 uint8_t;
   typedef unsigned __int16 uint16_t;
   typedef unsigned __int32 uint32_t;
   typedef unsigned __int64 uint64_t;
   typedef __int8 int_least8_t;
   typedef __int16 int_least16_t;
   typedef __int32 int_least32_t;
   typedef __int64 int_least64_t;
   typedef unsigned __int8 uint_least8_t;
   typedef unsigned __int16 uint_least16_t;
   typedef unsigned __int32 uint_least32_t;
   typedef unsigned __int64 uint_least64_t;
   typedef __int8 int_fast8_t;
   typedef __int16 int_fast16_t;
   typedef __int32 int_fast32_t;
   typedef __int64 int_fast64_t;
   typedef unsigned __int8 uint_fast8_t;
   typedef unsigned __int16 uint_fast16_t;
   typedef unsigned __int32 uint_fast32_t;
   typedef unsigned __int64 uint_fast64_t;
   typedef __int64 intmax_t;
   typedef unsigned __int64 uintmax_t;
# else
#  include <stdint.h>
# endif
# if _MSC_VER < 1800   /* MSVC < 2013 */
#  ifndef __cplusplus
    typedef unsigned char _Bool;
#  endif
# endif
#else
# include <stdint.h>
# if (defined (__SVR4) && defined (__sun)) || defined(_AIX) || defined(__hpux)
#  include <alloca.h>
# endif
#endif

#ifdef __GNUC__
# define _CFFI_UNUSED_FN  __attribute__((unused))
#else
# define _CFFI_UNUSED_FN  /* nothing */
#endif

#ifdef __cplusplus
# ifndef _Bool
   typedef bool _Bool;   /* semi-hackish: C++ has no _Bool; bool is builtin */
# endif
#endif

/**********  CPython-specific section  **********/
#ifndef PYPY_VERSION


#if PY_MAJOR_VERSION >= 3
# define PyInt_FromLong PyLong_FromLong
#endif

#define _cffi_from_c_double PyFloat_FromDouble
#define _cffi_from_c_float PyFloat_FromDouble
#define _cffi_from_c_long PyInt_FromLong
#define _cffi_from_c_ulong PyLong_FromUnsignedLong
#define _cffi_from_c_longlong PyLong_FromLongLong
#define _cffi_from_c_ulonglong PyLong_FromUnsignedLongLong

#define _cffi_to_c_double PyFloat_AsDouble
#define _cffi_to_c_float PyFloat_AsDouble

#define _cffi_from_c_int(x, type)                                        \
    (((type)-1) > 0 ? /* unsigned */                                     \
        (sizeof(type) < sizeof(long) ?                                   \
            PyInt_FromLong((long)x) :                                    \
         sizeof(type) == sizeof(long) ?                                  \
            PyLong_FromUnsignedLong((unsigned long)x) :                  \
            PyLong_FromUnsignedLongLong((unsigned long long)x)) :        \
        (sizeof(type) <= sizeof(long) ?                                  \
            PyInt_FromLong((long)x) :                                    \
            PyLong_FromLongLong((long long)x)))

#define _cffi_to_c_int(o, type)                                          \
    ((type)(                                                             \
     sizeof(type) == 1 ? (((type)-1) > 0 ? (type)_cffi_to_c_u8(o)        \
                                         : (type)_cffi_to_c_i8(o)) :     \
     sizeof(type) == 2 ? (((type)-1) > 0 ? (type)_cffi_to_c_u16(o)       \
                                         : (type)_cffi_to_c_i16(o)) :    \
     sizeof(type) == 4 ? (((type)-1) > 0 ? (type)_cffi_to_c_u32(o)       \
                                         : (type)_cffi_to_c_i32(o)) :    \
     sizeof(type) == 8 ? (((type)-1) > 0 ? (type)_cffi_to_c_u64(o)       \
                                         : (type)_cffi_to_c_i64(o)) :    \
     (Py_FatalError("unsupported size for type " #type), (type)0)))

#define _cffi_to_c_i8                                                    \
                 ((int(*)(PyObject *))_cffi_exports[1])
#define _cffi_to_c_u8                                                    \
                 ((int(*)(PyObject *))_cffi_exports[2])
#define _cffi_to_c_i16                                                   \
                 ((int(*)(PyObject *))_cffi_exports[3])
#define _cffi_to_c_u16                                                   \
                 ((int(*)(PyObject *))_cffi_exports[4])
#define _cffi_to_c_i32                                                   \
                 ((int(*)(PyObject *))_cffi_exports[5])
#define _cffi_to_c_u32                                                   \
                 ((unsigned int(*)(PyObject *))_cffi_exports[6])
#define _cffi_to_c_i64                                                   \
                 ((long long(*)(PyObject *))_cffi_exports[7])
#define _cffi_to_c_u64                                                   \
                 ((unsigned long long(*)(PyObject *))_cffi_exports[8])
#define _cffi_to_c_char                                                  \
                 ((int(*)(PyObject *))_cffi_exports[9])
#define _cffi_from_c_pointer                                             \
    ((PyObject *(*)(char *, CTypeDescrObject *))_cffi_exports[10])
#define _cffi_to_c_pointer                                               \
    ((char *(*)(PyObject *, CTypeDescrObject *))_cffi_exports[11])
#define _cffi_get_struct_layout                                          \
    not used any more
#define _cffi_restore_errno                                              \
    ((void(*)(void))_cffi_exports[13])
#define _cffi_save_errno                                                 \
    ((void(*)(void))_cffi_exports[14])
#define _cffi_from_c_char                                                \
    ((PyObject *(*)(char))_cffi_exports[15])
#define _cffi_from_c_deref                                               \
    ((PyObject *(*)(char *, CTypeDescrObject *))_cffi_exports[16])
#define _cffi_to_c                                                       \
    ((int(*)(char *, CTypeDescrObject *, PyObject *))_cffi_exports[17])
#define _cffi_from_c_struct                                              \
    ((PyObject *(*)(char *, CTypeDescrObject *))_cffi_exports[18])
#define _cffi_to_c_wchar_t                                               \
    ((wchar_t(*)(PyObject *))_cffi_exports[19])
#define _cffi_from_c_wchar_t                                             \
    ((PyObject *(*)(wchar_t))_cffi_exports[20])
#define _cffi_to_c_long_double                                           \
    ((long double(*)(PyObject *))_cffi_exports[21])
#define _cffi_to_c__Bool                                                 \
    ((_Bool(*)(PyObject *))_cffi_exports[22])
#define _cffi_prepare_pointer_call_argument                              \
    ((Py_ssize_t(*)(CTypeDescrObject *, PyObject *, char **))_cffi_exports[23])
#define _cffi_convert_array_from_object                                  \
    ((int(*)(char *, CTypeDescrObject *, PyObject *))_cffi_exports[24])
#define _CFFI_CPIDX  25
#define _cffi_call_python                                                \
    ((void(*)(struct _cffi_externpy_s *, char *))_cffi_exports[_CFFI_CPIDX])
#define _CFFI_NUM_EXPORTS 26

typedef struct _ctypedescr CTypeDescrObject;

static void *_cffi_exports[_CFFI_NUM_EXPORTS];

#define _cffi_type(index)   (                           \
    assert((((uintptr_t)_cffi_types[index]) & 1) == 0), \
    (CTypeDescrObject *)_cffi_types[index])

static PyObject *_cffi_init(const char *module_name, Py_ssize_t version,
                            const struct _cffi_type_context_s *ctx)
{
    PyObject *module, *o_arg, *new_module;
    void *raw[] = {
        (void *)module_name,
        (void *)version,
        (void *)_cffi_exports,
        (void *)ctx,
    };

    module = PyImport_ImportModule("_cffi_backend");
    if (module == NULL)
        goto failure;

    o_arg = PyLong_FromVoidPtr((void *)raw);
    if (o_arg == NULL)
        goto failure;

    new_module = PyObject_CallMethod(
        module, (char *)"_init_cffi_1_0_external_module", (char *)"O", o_arg);

    Py_DECREF(o_arg);
    Py_DECREF(module);
    return new_module;

  failure:
    Py_XDECREF(module);
    return NULL;
}

/**********  end CPython-specific section  **********/
#else
_CFFI_UNUSED_FN
static void (*_cffi_call_python_org)(struct _cffi_externpy_s *, char *);
# define _cffi_call_python  _cffi_call_python_org
#endif


#define _cffi_array_len(array)   (sizeof(array) / sizeof((array)[0]))

#define _cffi_prim_int(size, sign)                                      \
    ((size) == 1 ? ((sign) ? _CFFI_PRIM_INT8  : _CFFI_PRIM_UINT8)  :    \
     (size) == 2 ? ((sign) ? _CFFI_PRIM_INT16 : _CFFI_PRIM_UINT16) :    \
     (size) == 4 ? ((sign) ? _CFFI_PRIM_INT32 : _CFFI_PRIM_UINT32) :    \
     (size) == 8 ? ((sign) ? _CFFI_PRIM_INT64 : _CFFI_PRIM_UINT64) :    \
     _CFFI__UNKNOWN_PRIM)

#define _cffi_prim_float(size)                                          \
    ((size) == sizeof(float) ? _CFFI_PRIM_FLOAT :                       \
     (size) == sizeof(double) ? _CFFI_PRIM_DOUBLE :                     \
     (size) == sizeof(long double) ? _CFFI__UNKNOWN_LONG_DOUBLE :       \
     _CFFI__UNKNOWN_FLOAT_PRIM)

#define _cffi_check_int(got, got_nonpos, expected)      \
    ((got_nonpos) == (expected <= 0) &&                 \
     (got) == (unsigned long long)expected)

#ifdef MS_WIN32
# define _cffi_stdcall  __stdcall
#else
# define _cffi_stdcall  /* nothing */
#endif

#ifdef __cplusplus
}
#endif

/************************************************************/


        #include <stdio.h>

        static void brushfireFromObstacles(int ** input, int ** output, int width, int height,
            int min_x, int max_x, int min_y, int max_y)
        {
            int i = 0;
            int j = 0;
            int step = 0;
            char changed = 1;
            while(changed == 1)
            {
                changed = 0;
                for(i = min_x ; i < max_x - 1 ; i = i + 1)
                {
                    for(j = min_y ; j < max_y - 1 ; j = j + 1)
                    {
                        if(output[i][j] == -1 && input[i][j] < 49) // Free space
                        {
                            if(
                                output[i - 1][j] == step ||
                                output[i + 1][j] == step ||
                                output[i - 1][j - 1] == step ||
                                output[i + 1][j - 1] == step ||
                                output[i - 1][j + 1] == step ||
                                output[i + 1][j + 1] == step ||
                                output[i][j - 1] == step ||
                                output[i][j + 1] == step
                            )
                            {
                                output[i][j] = step + 1;
                                changed = 1;
                            }
                        }
                    }
                }
                step = step + 1;
            }
        }

        static void thinning(int ** in, int ** out, int width, int height, int min_x, int max_x, int min_y, int max_y)
        {
            int i, j, steps = 0;
            char changed = 1;

            while(changed == 1)
            {
                steps = steps + 1;
                changed = 0;

                for(i = min_x ; i < max_x ; i = i + 1)
                    for(j = min_y ; j < max_y ; j = j + 1)
                        in[i][j] = out[i][j];

                // 000
                // x1x
                // 111
                for(i = min_x ; i < max_x - 1  ; i = i + 1)
                {
                    for(j = min_y ; j < max_y - 1 ; j = j + 1)
                    {
                        if(
                            in[i-1][j-1]==0 && in[i][j-1]==0 && in[i+1][j-1]==0
                                            && in[i][j]==1                      &&
                            in[i-1][j+1]==1 && in[i][j+1]==1 && in[i+1][j+1]==1
                        )
                        {
                            out[i][j] = 0;
                            changed = 1;
                        }
                    }
                }
                for(i = min_x ; i < max_x ; i = i + 1)
                    for(j = min_y ; j < max_y ; j = j + 1)
                        in[i][j] = out[i][j];

                // x00
                // 110
                // x1x
                for(i = min_x ; i < max_x - 1 ; i = i + 1)
                {
                    for(j = min_y ; j < max_y - 1 ; j = j + 1)
                    {
                        if(
                            1               && in[i][j-1]==0    && in[i+1][j-1]==0 &&
                            in[i-1][j]==1   && in[i][j]==1      && in[i+1][j]==0  &&
                            1               && in[i][j+1]==1    && 1
                        )
                        {
                            out[i][j] = 0;
                            changed = 1;
                        }
                    }
                }
                for(i = min_x ; i < max_x ; i = i + 1)
                    for(j = min_y ; j < max_y ; j = j + 1)
                        in[i][j] = out[i][j];

                // 1x0
                // 110
                // 1-0
                for(i = min_x ; i < max_x -1 ; i = i + 1)
                {
                    for(j = min_y ; j < max_y-1 ; j = j + 1)
                    {
                        if(
                            in[i-1][j-1]==1 && 1            && in[i+1][j-1]==0 &&
                            in[i-1][j]==1   && in[i][j]==1  && in[i+1][j]==0  &&
                            in[i-1][j+1]==1 && 1            && in[i+1][j+1]==0
                        )
                        {
                            out[i][j] = 0;
                            changed = 1;
                        }
                    }
                }
                for(i = min_x ; i < max_x ; i = i + 1)
                    for(j = min_y ; j < max_y ; j = j + 1)
                        in[i][j] = out[i][j];

                // x1x
                // 110
                // -00
                for(i = min_x ; i < max_x-1 ; i = i + 1)
                {
                    for(j = min_y ; j < max_y-1 ; j = j + 1)
                    {
                        if(
                            1               && in[i][j-1]==1    && 1                &&
                            in[i-1][j]==1   && in[i][j]==1      && in[i+1][j]==0    &&
                            1               && in[i][j+1]==0    && in[i+1][j+1]==0
                        )
                        {
                            out[i][j] = 0;
                            changed = 1;
                        }
                    }
                }
                for(i = min_x ; i < max_x ; i = i + 1)
                    for(j = min_y ; j < max_y ; j = j + 1)
                        in[i][j] = out[i][j];

                // 111
                // -1-
                // 000
                for(i = min_x ; i < max_x-1 ; i = i + 1)
                {
                    for(j = min_y ; j < max_y-1 ; j = j + 1)
                    {
                        if(
                            in[i-1][j-1]==1 && in[i][j-1]==1    && in[i+1][j-1]==1  &&
                            1               && in[i][j]==1      && 1                &&
                            in[i-1][j+1]==0 && in[i][j+1]==0    && in[i+1][j+1]==0
                        )
                        {
                            out[i][j] = 0;
                            changed = 1;
                        }
                    }
                }
                for(i = min_x ; i < max_x ; i = i + 1)
                    for(j = min_y ; j < max_y ; j = j + 1)
                        in[i][j] = out[i][j];

                // -1-
                // 011
                // 00-
                for(i = min_x ; i < max_x-1 ; i = i + 1)
                {
                    for(j = min_y ; j < max_y-1 ; j = j + 1)
                    {
                        if(
                            1               && in[i][j-1]==1    && 1                &&
                            in[i-1][j]==0   && in[i][j]==1      && in[i+1][j]==1    &&
                            in[i-1][j+1]==0 && in[i][j+1]==0    && 1
                        )
                        {
                            out[i][j] = 0;
                            changed = 1;
                        }
                    }
                }
                for(i = min_x ; i < max_y ; i = i + 1)
                    for(j = min_y ; j < max_y ; j = j + 1)
                        in[i][j] = out[i][j];

                // 0-1
                // 011
                // 0-1
                for(i = min_x ; i < max_x-1 ; i = i + 1)
                {
                    for(j = min_y ; j < max_y-1 ; j = j + 1)
                    {
                        if(
                            in[i-1][j-1]==0 && 1                && in[i+1][j-1]==1  &&
                            in[i-1][j]==0   && in[i][j]==1      && in[i+1][j]==1    &&
                            in[i-1][j+1]==0 && 1                && in[i+1][j+1]==1
                        )
                        {
                            out[i][j] = 0;
                            changed = 1;
                        }
                    }
                }
                for(i = min_x ; i < max_x ; i = i + 1)
                    for(j = min_y ; j < max_y ; j = j + 1)
                        in[i][j] = out[i][j];

                // 00-
                // 011
                // -1-
                for(i = min_x ; i < max_x-1 ; i = i + 1)
                {
                    for(j = min_y ; j < max_y-1 ; j = j + 1)
                    {
                        if(
                            in[i-1][j-1]==0 && in[i][j-1]==0    && 1  &&
                            in[i-1][j]==0   && in[i][j]==1      && in[i+1][j]==1    &&
                            1               && in[i][j+1]==1    && 1
                        )
                        {
                            out[i][j] = 0;
                            changed = 1;
                        }
                    }
                }
            } // END WHILE
        }

        static void prune(int ** in, int ** out, int width, int height, int min_x, int max_x, int min_y, int max_y, int iterations)
        {
            int i, j, steps = 0;
            int sum;

            for(steps = 0 ; steps < iterations ; steps = steps + 1)
            {
                for(i = min_x ; i < max_x ; i = i + 1)
                    for(j = min_y ; j < max_y ; j = j + 1)
                        in[i][j] = out[i][j];

                for(i = min_x ; i < max_x - 1  ; i = i + 1)
                {
                    for(j = min_y ; j < max_y - 1 ; j = j + 1)
                    {
                        sum = 
                            in[i-1][j-1] + in[i-1][j] + in[i-1][j+1] +    
                            in[i][j-1] + in[i][j] + in[i][j+1] +    
                            in[i+1][j-1] + in[i+1][j] + in[i+1][j+1];
                        if(sum == 2)
                        {
                            out[i][j] = 0;
                        }
                    }
                }
            }
        }

    

/************************************************************/

static void *_cffi_types[] = {
/*  0 */ _CFFI_OP(_CFFI_OP_FUNCTION, 22), // void()(int * *, int * *, int, int, int, int, int, int)
/*  1 */ _CFFI_OP(_CFFI_OP_POINTER, 21), // int * *
/*  2 */ _CFFI_OP(_CFFI_OP_NOOP, 1),
/*  3 */ _CFFI_OP(_CFFI_OP_PRIMITIVE, 7), // int
/*  4 */ _CFFI_OP(_CFFI_OP_PRIMITIVE, 7),
/*  5 */ _CFFI_OP(_CFFI_OP_PRIMITIVE, 7),
/*  6 */ _CFFI_OP(_CFFI_OP_PRIMITIVE, 7),
/*  7 */ _CFFI_OP(_CFFI_OP_PRIMITIVE, 7),
/*  8 */ _CFFI_OP(_CFFI_OP_PRIMITIVE, 7),
/*  9 */ _CFFI_OP(_CFFI_OP_FUNCTION_END, 0),
/* 10 */ _CFFI_OP(_CFFI_OP_FUNCTION, 22), // void()(int * *, int * *, int, int, int, int, int, int, int)
/* 11 */ _CFFI_OP(_CFFI_OP_NOOP, 1),
/* 12 */ _CFFI_OP(_CFFI_OP_NOOP, 1),
/* 13 */ _CFFI_OP(_CFFI_OP_PRIMITIVE, 7),
/* 14 */ _CFFI_OP(_CFFI_OP_PRIMITIVE, 7),
/* 15 */ _CFFI_OP(_CFFI_OP_PRIMITIVE, 7),
/* 16 */ _CFFI_OP(_CFFI_OP_PRIMITIVE, 7),
/* 17 */ _CFFI_OP(_CFFI_OP_PRIMITIVE, 7),
/* 18 */ _CFFI_OP(_CFFI_OP_PRIMITIVE, 7),
/* 19 */ _CFFI_OP(_CFFI_OP_PRIMITIVE, 7),
/* 20 */ _CFFI_OP(_CFFI_OP_FUNCTION_END, 0),
/* 21 */ _CFFI_OP(_CFFI_OP_POINTER, 3), // int *
/* 22 */ _CFFI_OP(_CFFI_OP_PRIMITIVE, 0), // void
};

static void _cffi_d_brushfireFromObstacles(int * * x0, int * * x1, int x2, int x3, int x4, int x5, int x6, int x7)
{
  brushfireFromObstacles(x0, x1, x2, x3, x4, x5, x6, x7);
}
#ifndef PYPY_VERSION
static PyObject *
_cffi_f_brushfireFromObstacles(PyObject *self, PyObject *args)
{
  int * * x0;
  int * * x1;
  int x2;
  int x3;
  int x4;
  int x5;
  int x6;
  int x7;
  Py_ssize_t datasize;
  PyObject *arg0;
  PyObject *arg1;
  PyObject *arg2;
  PyObject *arg3;
  PyObject *arg4;
  PyObject *arg5;
  PyObject *arg6;
  PyObject *arg7;

  if (!PyArg_UnpackTuple(args, "brushfireFromObstacles", 8, 8, &arg0, &arg1, &arg2, &arg3, &arg4, &arg5, &arg6, &arg7))
    return NULL;

  datasize = _cffi_prepare_pointer_call_argument(
      _cffi_type(1), arg0, (char **)&x0);
  if (datasize != 0) {
    if (datasize < 0)
      return NULL;
    x0 = (int * *)alloca((size_t)datasize);
    memset((void *)x0, 0, (size_t)datasize);
    if (_cffi_convert_array_from_object((char *)x0, _cffi_type(1), arg0) < 0)
      return NULL;
  }

  datasize = _cffi_prepare_pointer_call_argument(
      _cffi_type(1), arg1, (char **)&x1);
  if (datasize != 0) {
    if (datasize < 0)
      return NULL;
    x1 = (int * *)alloca((size_t)datasize);
    memset((void *)x1, 0, (size_t)datasize);
    if (_cffi_convert_array_from_object((char *)x1, _cffi_type(1), arg1) < 0)
      return NULL;
  }

  x2 = _cffi_to_c_int(arg2, int);
  if (x2 == (int)-1 && PyErr_Occurred())
    return NULL;

  x3 = _cffi_to_c_int(arg3, int);
  if (x3 == (int)-1 && PyErr_Occurred())
    return NULL;

  x4 = _cffi_to_c_int(arg4, int);
  if (x4 == (int)-1 && PyErr_Occurred())
    return NULL;

  x5 = _cffi_to_c_int(arg5, int);
  if (x5 == (int)-1 && PyErr_Occurred())
    return NULL;

  x6 = _cffi_to_c_int(arg6, int);
  if (x6 == (int)-1 && PyErr_Occurred())
    return NULL;

  x7 = _cffi_to_c_int(arg7, int);
  if (x7 == (int)-1 && PyErr_Occurred())
    return NULL;

  Py_BEGIN_ALLOW_THREADS
  _cffi_restore_errno();
  { brushfireFromObstacles(x0, x1, x2, x3, x4, x5, x6, x7); }
  _cffi_save_errno();
  Py_END_ALLOW_THREADS

  (void)self; /* unused */
  Py_INCREF(Py_None);
  return Py_None;
}
#else
#  define _cffi_f_brushfireFromObstacles _cffi_d_brushfireFromObstacles
#endif

static void _cffi_d_prune(int * * x0, int * * x1, int x2, int x3, int x4, int x5, int x6, int x7, int x8)
{
  prune(x0, x1, x2, x3, x4, x5, x6, x7, x8);
}
#ifndef PYPY_VERSION
static PyObject *
_cffi_f_prune(PyObject *self, PyObject *args)
{
  int * * x0;
  int * * x1;
  int x2;
  int x3;
  int x4;
  int x5;
  int x6;
  int x7;
  int x8;
  Py_ssize_t datasize;
  PyObject *arg0;
  PyObject *arg1;
  PyObject *arg2;
  PyObject *arg3;
  PyObject *arg4;
  PyObject *arg5;
  PyObject *arg6;
  PyObject *arg7;
  PyObject *arg8;

  if (!PyArg_UnpackTuple(args, "prune", 9, 9, &arg0, &arg1, &arg2, &arg3, &arg4, &arg5, &arg6, &arg7, &arg8))
    return NULL;

  datasize = _cffi_prepare_pointer_call_argument(
      _cffi_type(1), arg0, (char **)&x0);
  if (datasize != 0) {
    if (datasize < 0)
      return NULL;
    x0 = (int * *)alloca((size_t)datasize);
    memset((void *)x0, 0, (size_t)datasize);
    if (_cffi_convert_array_from_object((char *)x0, _cffi_type(1), arg0) < 0)
      return NULL;
  }

  datasize = _cffi_prepare_pointer_call_argument(
      _cffi_type(1), arg1, (char **)&x1);
  if (datasize != 0) {
    if (datasize < 0)
      return NULL;
    x1 = (int * *)alloca((size_t)datasize);
    memset((void *)x1, 0, (size_t)datasize);
    if (_cffi_convert_array_from_object((char *)x1, _cffi_type(1), arg1) < 0)
      return NULL;
  }

  x2 = _cffi_to_c_int(arg2, int);
  if (x2 == (int)-1 && PyErr_Occurred())
    return NULL;

  x3 = _cffi_to_c_int(arg3, int);
  if (x3 == (int)-1 && PyErr_Occurred())
    return NULL;

  x4 = _cffi_to_c_int(arg4, int);
  if (x4 == (int)-1 && PyErr_Occurred())
    return NULL;

  x5 = _cffi_to_c_int(arg5, int);
  if (x5 == (int)-1 && PyErr_Occurred())
    return NULL;

  x6 = _cffi_to_c_int(arg6, int);
  if (x6 == (int)-1 && PyErr_Occurred())
    return NULL;

  x7 = _cffi_to_c_int(arg7, int);
  if (x7 == (int)-1 && PyErr_Occurred())
    return NULL;

  x8 = _cffi_to_c_int(arg8, int);
  if (x8 == (int)-1 && PyErr_Occurred())
    return NULL;

  Py_BEGIN_ALLOW_THREADS
  _cffi_restore_errno();
  { prune(x0, x1, x2, x3, x4, x5, x6, x7, x8); }
  _cffi_save_errno();
  Py_END_ALLOW_THREADS

  (void)self; /* unused */
  Py_INCREF(Py_None);
  return Py_None;
}
#else
#  define _cffi_f_prune _cffi_d_prune
#endif

static void _cffi_d_thinning(int * * x0, int * * x1, int x2, int x3, int x4, int x5, int x6, int x7)
{
  thinning(x0, x1, x2, x3, x4, x5, x6, x7);
}
#ifndef PYPY_VERSION
static PyObject *
_cffi_f_thinning(PyObject *self, PyObject *args)
{
  int * * x0;
  int * * x1;
  int x2;
  int x3;
  int x4;
  int x5;
  int x6;
  int x7;
  Py_ssize_t datasize;
  PyObject *arg0;
  PyObject *arg1;
  PyObject *arg2;
  PyObject *arg3;
  PyObject *arg4;
  PyObject *arg5;
  PyObject *arg6;
  PyObject *arg7;

  if (!PyArg_UnpackTuple(args, "thinning", 8, 8, &arg0, &arg1, &arg2, &arg3, &arg4, &arg5, &arg6, &arg7))
    return NULL;

  datasize = _cffi_prepare_pointer_call_argument(
      _cffi_type(1), arg0, (char **)&x0);
  if (datasize != 0) {
    if (datasize < 0)
      return NULL;
    x0 = (int * *)alloca((size_t)datasize);
    memset((void *)x0, 0, (size_t)datasize);
    if (_cffi_convert_array_from_object((char *)x0, _cffi_type(1), arg0) < 0)
      return NULL;
  }

  datasize = _cffi_prepare_pointer_call_argument(
      _cffi_type(1), arg1, (char **)&x1);
  if (datasize != 0) {
    if (datasize < 0)
      return NULL;
    x1 = (int * *)alloca((size_t)datasize);
    memset((void *)x1, 0, (size_t)datasize);
    if (_cffi_convert_array_from_object((char *)x1, _cffi_type(1), arg1) < 0)
      return NULL;
  }

  x2 = _cffi_to_c_int(arg2, int);
  if (x2 == (int)-1 && PyErr_Occurred())
    return NULL;

  x3 = _cffi_to_c_int(arg3, int);
  if (x3 == (int)-1 && PyErr_Occurred())
    return NULL;

  x4 = _cffi_to_c_int(arg4, int);
  if (x4 == (int)-1 && PyErr_Occurred())
    return NULL;

  x5 = _cffi_to_c_int(arg5, int);
  if (x5 == (int)-1 && PyErr_Occurred())
    return NULL;

  x6 = _cffi_to_c_int(arg6, int);
  if (x6 == (int)-1 && PyErr_Occurred())
    return NULL;

  x7 = _cffi_to_c_int(arg7, int);
  if (x7 == (int)-1 && PyErr_Occurred())
    return NULL;

  Py_BEGIN_ALLOW_THREADS
  _cffi_restore_errno();
  { thinning(x0, x1, x2, x3, x4, x5, x6, x7); }
  _cffi_save_errno();
  Py_END_ALLOW_THREADS

  (void)self; /* unused */
  Py_INCREF(Py_None);
  return Py_None;
}
#else
#  define _cffi_f_thinning _cffi_d_thinning
#endif

static const struct _cffi_global_s _cffi_globals[] = {
  { "brushfireFromObstacles", (void *)_cffi_f_brushfireFromObstacles, _CFFI_OP(_CFFI_OP_CPYTHON_BLTN_V, 0), (void *)_cffi_d_brushfireFromObstacles },
  { "prune", (void *)_cffi_f_prune, _CFFI_OP(_CFFI_OP_CPYTHON_BLTN_V, 10), (void *)_cffi_d_prune },
  { "thinning", (void *)_cffi_f_thinning, _CFFI_OP(_CFFI_OP_CPYTHON_BLTN_V, 0), (void *)_cffi_d_thinning },
};

static const struct _cffi_type_context_s _cffi_type_context = {
  _cffi_types,
  _cffi_globals,
  NULL,  /* no fields */
  NULL,  /* no struct_unions */
  NULL,  /* no enums */
  NULL,  /* no typenames */
  3,  /* num_globals */
  0,  /* num_struct_unions */
  0,  /* num_enums */
  0,  /* num_typenames */
  NULL,  /* no includes */
  23,  /* num_types */
  0,  /* flags */
};

#ifdef PYPY_VERSION
PyMODINIT_FUNC
_cffi_pypyinit__cpp_functions(const void *p[])
{
    p[0] = (const void *)0x2601;
    p[1] = &_cffi_type_context;
}
#  ifdef _MSC_VER
     PyMODINIT_FUNC
#  if PY_MAJOR_VERSION >= 3
     PyInit__cpp_functions(void) { return NULL; }
#  else
     init_cpp_functions(void) { }
#  endif
#  endif
#elif PY_MAJOR_VERSION >= 3
PyMODINIT_FUNC
PyInit__cpp_functions(void)
{
  return _cffi_init("_cpp_functions", 0x2601, &_cffi_type_context);
}
#else
PyMODINIT_FUNC
init_cpp_functions(void)
{
  _cffi_init("_cpp_functions", 0x2601, &_cffi_type_context);
}
#endif
