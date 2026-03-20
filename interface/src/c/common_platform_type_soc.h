// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_COMMON_PLATFORM_TYPE_SOC_H_
#define _IFLYAUTO_COMMON_PLATFORM_TYPE_SOC_H_

/******************************************************************************
**                      Includes                                             **
******************************************************************************/
#include <stdint.h>

/******************************************************************************
**                      Global Macro Definitions                             **
******************************************************************************/
#define PLATFORM_VENDOR_ID (17u)
#define PLATFORM_AR_RELEASE_MAJOR_VERSION (4u)
#define PLATFORM_AR_RELEASE_MINOR_VERSION (2u)
#define PLATFORM_AR_RELEASE_REVISION_VERSION (2u)
#define PLATFORM_SW_MAJOR_VERSION (1u)
#define PLATFORM_SW_MINOR_VERSION (0u)
#define PLATFORM_SW_PATCH_VERSION (0u)

/* CPU register width type definition */
#define CPU_TYPE_8 (8u)
#define CPU_TYPE_16 (16u)
#define CPU_TYPE_32 (32u)
#define CPU_TYPE_64 (64u)
/* Register width of CPU*/
#define CPU_TYPE CPU_TYPE_64 /* 64 bit  */

/* Bit order type definition*/
#define MSB_FIRST (0u) /* Big Endian bit ordering     */
#define LSB_FIRST (1u) /* Little Endian bit ordering  */
/* Bit order of Register level*/
#define CPU_BIT_ORDER LSB_FIRST /* Little Endian */

/* Byte order type definition*/
#define HIGH_BYTE_FIRST (0u) /* Big Endian byte ordering    */
#define LOW_BYTE_FIRST (1u)  /* Little Endian byte ordering */
/* Byte order on Memory level*/
#define CPU_BYTE_ORDER LOW_BYTE_FIRST /* Little Endian */

/* TRUE, FALSE symbol for Boolean types*/
#ifndef TRUE
#define TRUE (1u)
#endif
#ifndef FALSE
#define FALSE (0u)
#endif

/* unsigned char with a bit length that is the shortest one natively supported
  by the platform.*/
typedef unsigned char boolean; /* for use with TRUE/FALSE      */

/* 8bit unsigned :  0 .. 255 [0X00 .. 0XFF]*/
typedef uint8_t uint8;

/* 16bit unsigned:  0..65535 [0x0000..0xFFFF]*/
typedef uint16_t uint16;

/* 32bit unsigned:  0..4294967295 [0x00000000..0xFFFFFFFF]*/
typedef uint32_t uint32;

/* 64bit unsigned
 *          0..18446744073709551615   [0x0000000000000000..0xFFFFFFFFFFFFFFFF]*/
typedef uint64_t uint64;

/* 8bit signed, 7 bit + 1 bit sign -128..+127 [0x80..0x7F]*/
typedef int8_t sint8;
typedef int8_t int8;

/* 16bit signed, 15 bit + 1 bit sign -32768..+32767 [0x8000..0x7FFF]*/
typedef int16_t sint16;
typedef int16_t int16;

/* 32bit signed, 31 bit + 1 bit sign
 -2147483648..+2147483647 [0x80000000..0x7FFFFFFF]*/
typedef int32_t sint32;
typedef int32_t int32;
/*
 * 64bit signed, 63 bit + 1 bit sign
 * -9223372036854775808..9223372036854775807
 * [0x8000000000000000..0x7FFFFFFFFFFFFFFF]
 */
typedef int64_t sint64;

/* IEEE754-2008 single precision
 * -3.4028235e+38..+3.4028235e+38*/
typedef float float32; /* IEEE754-2008 single precision */

/* IEEE754-2008 double precision
 * -1.7976931348623157e+308..+1.7976931348623157e+308*/
typedef double float64; /* IEEE754-2008 double precision */

#endif /* _IFLYAUTO_COMMON_PLATFORM_TYPE_SOC_H_ */
