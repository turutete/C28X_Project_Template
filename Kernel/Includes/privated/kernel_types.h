/*
 * kernel_types.h
 *
 *  Created on: 29 oct. 2022
 *  Author: Dr. Carlos Romero
 */

#ifndef KERNEL_INCLUDES_PRIVATED_KERNEL_TYPES_H_
#define KERNEL_INCLUDES_PRIVATED_KERNEL_TYPES_H_

#include    "f28004x_device.h"                      // f28004x Headerfile Include File

#define SI      1
#define NO      0
#define ERROR   -1
#define OK      0
#define KO      -1
#define ON      1
#define OFF     0

// Tipos de variables MISRA en minúsculas
typedef Uint16      uint16;
typedef Uint32      uint32;
typedef Uint64      uint64;


typedef struct
{
    int16 real;
    int16 imag;
} complex16;

typedef struct
{
    int32 real;
    int32 imag;
} complex32;

typedef struct
{
    int64 real;
    int64 imag;
} complex64;

typedef struct
{
    float32 real;
    float32 imag;
}fcomplex32;

typedef struct
{
    float64 real;
    float64 imag;
}fcomplex64;


#endif /* KERNEL_INCLUDES_PRIVATED_KERNEL_TYPES_H_ */
