/*
 * core.h
 *
 *  Created on: 12 nov. 2022
 *  Author: Dr. Carlos Romero
 */

#ifndef INCLUDES_PRIVATED_CORE_H_
#define INCLUDES_PRIVATED_CORE_H_

#include    <kernel_types.h>
#include    <system.h>
#include    <diagnosys.h>

#define DCDC_POWERON_TRIALS     5                   // Reintentos de lectura de DCDCSTS.SWSEQDON=1. No hay una especificaci�n
                                                    // concreta en el datasheet. Probablemente deba ser inmediato.
#define DCDC_POWERON_80us       (80*CLOCKIN_MHZ)    //  Valor aproximado para contador de 80us en DCDC poweron



enum DCDC_1200mV_VALUES
{
    DCDC_EXTERNAL,
    DCDC_INTERNAL,
    LDO_INTERNAL
};
struct  KERNEL_STATUS
{
    uint32 DCDC_1200mV:2;
    uint32 CLA1_VIOLATION:1;
    uint32 DMA_WRITE_VIOLATION:1;
    uint32 RESET_BY_WATCHDOG:1;
    uint32 RESET_BY_UPGRADING:1;
    uint32 UPGRADING_IN_PROGRESS:1;
    uint32 RESERVED:25;
};

typedef struct
{
    //Atributos públicos
    union
    {
        uint32 all;
        struct KERNEL_STATUS bits;

    } kernel_status;

    //Métodos públicos
    void (* Pre_Kernel)(void);        // Método preparatorio de kernel
    void (* Kernel)(void);            // Método público Kernel

} clase_kernel;

#endif

