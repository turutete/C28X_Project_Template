/*
 * realtime.h
 *
 *  Created on: 20 nov. 2022
 *  Author: Dr. Carlos Romero
 */

#ifndef INCLUDES_PRIVATED_REALTIME_H_
#define INCLUDES_PRIVATED_REALTIME_H_

#include    <kernel_types.h>
#include    <system.h>
#include    <f28004x_cputimer.h>

#ifdef  UNIT_TESTS
#include <stdio.h>
#endif

#define     MAX_CONCURRENT_TIMERS       8           // Número máximo de servicios de timer concurrentes
#define     ONE_ms_IN_us                1000        // 1000 us = 1 ms

typedef enum
{
    UNUSED,
    PAUSED,
    ONGOING,
    TIMEOUT
} TIMER_STATUS;

typedef enum
{
    POLLING,
    INTERRUPT
} TIMER_TYPE;

typedef struct
{
    uint16 hora;
    uint16 min;
    uint16 sec;
    uint16 msec;
} TIME_STAMPING;


typedef struct
{
  TIMER_STATUS estado;
  TIMER_TYPE tipo;
  uint32 timeout;
  TIME_STAMPING tinicial;
  TIME_STAMPING tfinal;
  void (* callback)(void);
} TIMER_OBJECT;

typedef int16 TIMER_OBJECT_ID;

typedef struct
{
    void (* InitRealtime)(void);
    void (* ResetClock)(void);
    void (* StopClock)(void);
    void (* StartClock)(void);
    void (* SetClock)(TIME_STAMPING);
    void (* GetClock)(TIME_STAMPING *);
    TIMER_OBJECT_ID (* GetTimer)(TIMER_TYPE);
    void (* ReleaseTimer)(TIMER_OBJECT_ID);
    void (* SetTimer) (TIMER_OBJECT_ID, uint32);
    void (* StartTimer)(TIMER_OBJECT_ID);
    void (* StopTimer)(TIMER_OBJECT_ID);
    void (* ResetTimer)(TIMER_OBJECT_ID);
    uint16 (* CheckTimer) (TIMER_OBJECT_ID);
     void (* SetCallback)(TIMER_OBJECT_ID,void (* pcallback)(void));
} clase_realtime;

// Declaraciones públicas
extern clase_realtime RealTime;
extern interrupt void Kernel_Realtime(void);


#ifdef  UNIT_TESTS
extern void Script_RealTime (void);
#endif

#endif /* INCLUDES_PRIVATED_REALTIME_H_ */
