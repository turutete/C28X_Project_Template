/*
 * diagnosys.h
 *
 *  Created on: 13 nov. 2022
 *  Author: Dr. Carlos Romero
 */

#ifndef INCLUDES_PRIVATED_DIAGNOSYS_H_
#define INCLUDES_PRIVATED_DIAGNOSYS_H_

#include    <kernel_types.h>

#define SIZE_SYSTEM_LOG     32      // Tamaño del log del sistema (nº de eventos máximo que almacena

enum SYSTEM_LOG_EVENTS
{
    POWER_ON_OK,
    POWER_ON_KO,
    UNEXPECTED_PROCESSOR,
    WATCHDOG_RESET,
    NMI_RESET,
    EXTERNAL_RESET,
    POWER_ON_RESET,
    KERNEL_STARTED,
    APPLICATION_STARTED,
    SW_UPGRADING_STARTED,
    SW_UPGRADING_DOWNLOADING,
    SW_UPGRADING_FLASHING,
    SW_UPGRADING_PAUSED,
    SW_UPGRADING_ENDED,
    SW_UPGRADING_ABORTED,
    WATCHDOG_ENABLED,
    WATCHDOG_DISABLED

};

typedef struct
{
    uint16 num_events;
    void (* Init_System_Log)(void);
    void (* Write_Event)(enum SYSTEM_LOG_EVENTS);
    void (* Reset_System_Log)(void);
} clase_diagnosys;


// Clase pública
extern clase_diagnosys Diagnosys;

// Método privado sólo para el Kernel
extern void Init_System_Log(void);

#ifdef  UNIT_TESTS
extern void Script_Diagnosys (void);
#endif

#endif /* INCLUDES_PRIVATED_DIAGNOSYS_H_ */
