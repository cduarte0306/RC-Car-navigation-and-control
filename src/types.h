/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#ifndef TYPES_H
#define TYPES_H
    

/* Type enumeration */
#define TYPE_ID_VOID                   (0x00)
#define TYPE_ID_BOOL                   (0x01)
#define TYPE_ID_CHAR                   (0x02)
#define TYPE_ID_UINT8                  (0x03)
#define TYPE_ID_INT8                   (0x04)
#define TYPE_ID_UINT16                 (0x05)
#define TYPE_ID_INT16                  (0x06)
#define TYPE_ID_UINT32                 (0x07)
#define TYPE_ID_INT32                  (0x08)
#define TYPE_ID_UINT64                 (0x09)
#define TYPE_ID_INT64                  (0x0A)
#define TYPE_ID_F32                    (0x0B)
#define TYPE_ID_F64                    (0x0C)
  
    
/**
 * Note these values represent the number of counts
 * for each of the different bit systems
 *
 * ie. if x is type uint8, the actual max value it can take
 * is MAX_UINT8
**/
#ifndef MAX_UINT8
    #define MAX_UINT8      (255u)
#endif

#ifndef MAX_UINT12
    #define MAX_UINT12     (4095u)
#endif

#ifndef MAX_UINT14
    #define MAX_UINT14     (16383u)
#endif

#ifndef MAX_UINT16
    #define MAX_UINT16     (65535u)
#endif

#ifndef MAX_UINT32
    #define MAX_UINT32     (4294967295u)
#endif

#ifndef MAX_F32
    #define MAX_F32        (3.402823466e+38F)
#endif


/**
 * Note these values represent the maximum value of
 * a signed value
 *
 * ie. if x is type int8, the actual max value it can take
 * is MAX_INT8
**/
#ifndef MAX_INT8
    #define MAX_INT8      (127)
#endif

#ifndef MAX_INT12
    #define MAX_INT12     (2047)
#endif

#ifndef MAX_INT14
    #define MAX_INT14     (8191)
#endif

#ifndef MAX_INT16
    #define MAX_INT16     (32767)
#endif

#ifndef MAX_INT32
    #define MAX_INT32     (2147483647)
#endif


/**
 * Gives maximum possible value of a type given its size in bytes and whether it is a signed value
 */
#ifndef MAX_VAL_SIGNED
    #define MAX_VAL_SIGNED(var)     ( ((uint64) 1u << 8*( sizeof( var ) )) - 1 )
#endif
#ifndef MAX_VAL_UNSIGNED
    #define MAX_VAL_UNSIGNED(var)   MAX_VAL_SIGNED(var) >> 1
#endif
    
    
typedef unsigned char   uint8;
typedef unsigned short  uint16;
typedef unsigned long   uint32;
typedef signed   char   int8;
typedef signed   short  int16;
typedef signed   long   int32;
typedef          float  float32;


    
/**
 * For when data can be either an int of a float,
 * This union allows easy interpretation between two possible data types.
**/
typedef union 
{
    /* Legacy union */
    float   f;
    uint32  u;
     int32  i;
} float_uint_t;

typedef union 
{ 
    float   f32;
    uint32  u32;
     int32  i32;
    uint16  u16;
     int16  i16;
     uint8  u8;
      int8  i8;
} val_type_t;


/**
 * @brief Compares two values based on provide type id to interpret the values as
 *
 * @param[in] type Type to interpret the value as
 * @param[in] val  First value to compare
 * @param[in] val  Second value to compare
 *
 * @return uint8
 * - (1) `TRUE` if values are equal
 * - (0) `FALSE` if values are not equal or type is unsupported
 */
static inline uint8 val_type_cmp(uint8 type, val_type_t val1, val_type_t val2)
{
    switch ( type )
    {
        case TYPE_ID_BOOL:
        case TYPE_ID_CHAR:
        case TYPE_ID_UINT8:  return val1.u8  == val2.u8;
        case TYPE_ID_INT8:   return val1.i8  == val2.i8;
        case TYPE_ID_INT16:  return val1.i16 == val2.i16;
        case TYPE_ID_UINT16: return val1.u16 == val2.u16;
        case TYPE_ID_INT32:  return val1.i32 == val2.i32;
        case TYPE_ID_UINT32: return val1.u32 == val2.u32;
        case TYPE_ID_F32:    return val1.f32 == val2.f32;
    };
    
    return 0;
}


/**
 * @brief Similiar to the `reinterpret_cast` in C++
 *   Based on given type id, it reinterprets the data as a 32 bit equivelent of the provided type.
 *
 * @param[in] type Type to reinterpret the value as
 * @param[in] val  Value to reinterpret
 *
 * @return val_type_t
 *   32-bit reinterpretation of `val`
 */
static inline val_type_t get_type(uint8 type, val_type_t val)
{
    switch ( type )
    {
        case TYPE_ID_BOOL:
        case TYPE_ID_CHAR:
        case TYPE_ID_UINT8:  return (val_type_t) { .u32 = val.u8  };
        case TYPE_ID_INT8:   return (val_type_t) { .i32 = val.i8  };
        case TYPE_ID_INT16:  return (val_type_t) { .i32 = val.i16 };
        case TYPE_ID_UINT16: return (val_type_t) { .u32 = val.u16 };
        case TYPE_ID_INT32:  return (val_type_t) { .i32 = val.i32 };
        case TYPE_ID_UINT32: return (val_type_t) { .u32 = val.u32 };
        case TYPE_ID_F32:    return (val_type_t) { .f32 = val.f32 };
    };
    
    return val;
}



/**
 * @brief Resolves max value based on passed type id
 *
 * @param[in] type Type to get max value for
 *
 * @return float max value the type can support, or 0 if type is unhandled
 */
static inline float max_val(uint8 type)
{
    switch ( type )
    {
        case TYPE_ID_BOOL:
        case TYPE_ID_CHAR:
        case TYPE_ID_UINT8:  return MAX_UINT8;
        case TYPE_ID_INT8:   return MAX_INT8;
        case TYPE_ID_INT16:  return MAX_INT16;
        case TYPE_ID_UINT16: return MAX_UINT16;
        case TYPE_ID_INT32:  return MAX_INT32;
        case TYPE_ID_UINT32: return MAX_UINT32;
        case TYPE_ID_F32:    return MAX_F32;
    };
    
    return 0;
}

    
#endif  /* TYPES_H */
/* [] END OF FILE */
