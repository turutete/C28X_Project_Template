/** \page   page_realtime   RealTime
 *
 *  \version    1.0.0.0
 *
 *  Atributos:
 *  ----------
 *
 *  * TIME_STAMPING system_clock
 *  * TIMER_OBJECT * ptimer
 *
 * Métodos:
 * --------
 *
 * * Init_Realtime()
 * * Reset_Clock()
 * * Stop_Clock()
 * * Start_Clock()
 * * Set_Clock(TIME_STAMPING)
 * * Get_Clock(TIME_STAMPING *)
 * * TIMER_OBJECT_ID Get_Timer(TYPE_OF_TIMER)
 * * Release_Timer(TIMER_OBJECT_ID)
 * * Set_Timer(TIMER_OBJECT_ID, uint16)
 * * Start_Timer(TIMER_OBJECT_ID)
 * * Stop_Timer(TIMER_OBJECT_ID)
 * * Reset_Timer(TIMER_OBJECT_ID)
 * * uint16 Check_Timer(TIMER_OBJECT_ID)
 * * Set_Callback(TIMER_OBJECT_ID,void (* callback)(void))
 *
 *
 * Descripción
 * -----------
 *
 * RealTime es un componente del Kernel que gestiona el reloj de tiempo real del sistema.
 *
 * Utiliza la interrupción del timer **TIMER2** que está reservado en la familia F28XX para timer del sistema operativo.
 *
 * Teniendo en cuenta la definición del sistema **CLOCKSYS_MHZ**, configura el periodo del TIMER2 para generar interrupciones
 * periódicas cada 1ms.
 *
 * La ISR del TIMER2 actualiza el objeto *TIME_STAMPING* del sistema (hora:min:sec:msec), que es el reloj de tiempo real
 * del sistema.
 *
 * RealTime permite a cualquier componente SW del Kernel o de la aplicación utilizar la información del reloj de tiempo real,
 * así como configurarlo.
 *
 * \attention ***IMPORTANTE***: Kernel no configura el valor del reloj del sistema. Tras un reset o powerup el reloj del sistema
 *  establece el reloj del sistema a 0:0:0:0. Si la aplicación requiere utilizar marcas de tiempo, debe configurar
 *  el reloj del sistema con la hora deseada
 *
 *  \attention Sólo debe existir un componente en la aplicación que configure el reloj del sistema, para evitar posibles
 *  sobreescrituras
 *
 *  Objetos
 *  --------
 *
 *   TIME_STAMPING
 *   {
 *   - uint16 hora;
 *   - uint16 min;
 *   - uint16 sec;
 *   - uint16 mesc;
 *   }
 *
 *   TIMER_OBJECT
 *   {
 *   - uint16 estado;     (UNUSED, PAUSED, ONGOING, TIMEOUT)
 *   - uint16 tipo;       (POLLING, INTERRUPT)
 *   - uint32 timeout;    (timeout period in ms of the timer)
 *   - TIME_STAMPING tinicial;    (time stamping of the starting time)
 *   - TIME_STAMPING tfinal;      (time stamping of the computed ending time)
 *   - void (* callback)(void);   (if using an INTERRUPT timer, this field points to the callback function)
 *   }
 *
 *
 * Servicio de temporización
 * -------------------------
 * RealTime ofrece servicio de temporización, para cualquier componente de SW que lo requiera. Soporta un máximo de
 * ***MAX_CONCURRENT_TIMERS***.
 *
 * Para utilizar un servicio de timer, debe seguirse los siguientes pasos:
 *
 * 1. Solicitar un temporizador mediante el método *uint16 Get_Timer(TYPE_OF_TIMER)*.
 *  * TYPE_OF_TIMER especifica el tipo de servicio de timer que se desea utilizar:
 *      * POLLING: Es un temporizador de cuenta atrás. Cuando expira, el campo *estado* del objeto toma el valor *TIMEOUT*.
 *  El componente SW que utiliza el timer debe comprobar si ha expirado leyendo este campo. Si no ha expirado, valdrá
 *  *ONGOING*.
 *
 *      * INTERRUPT: Es un temporizador de cuenta atrás. Cuando expira, el campo *estado* del objeto toma el valor *TIMEOUT*,
 *  y se ejecuta la función callback configurada.
 *
 * 2. El método Get_Timer devuelve el identificador del objeto, o bien *ERROR* si no hay disponible ningún temporizador. El
 * componente SW que solicita el servicio debe comprobar que el identificador devuelto es válido o es ERROR. Si este
 * evento se produce, RealTime generará un mensaje de error en el log del sistema.
 *
 * 3. Una vez conseguido un servicio de timer, debe configurarse. Para ello, se utilizará e método *Set_Timer()* para
 * establecer el timeout del timer.
 *
 * 4. Finalmente, el componente SW deberá iniciar la ejecución del temporizador mediante el método *Start_Timer()*.
 *
 * 5. Si se requiere pausar el timer se utilizará el método *Pause_Timer()*. Pausar el timer para el decremento del mismo,
 * pero no resetea su configuración. Si se desea reanudar el timer, se podra hacer volviendo a usar *Start_Timer()*.
 *
 * 6. El método *Reset_Timer()* para el timer y resetea sus valores.
 *
 *  Historial de Cambios
 *  --------------------
 *  | Fecha | Autores | Versión | Descripción |
 *  |:--: |:--:|:--:|:--:|
 *  | 20/11/2022| Dr. Carlos Romero |1.0.0.0|Primera edición|
 *
 *  ToDos
 *  -----
 *
 *  \todo   Lista de características futuras
 *
 *  Bugs Conocidos
 *  --------------
 *
 *  \bug        Sin bugs conocidos
 *
 *  Notas para el Desarrollador
 *  ----------------------------
 *  \warning    Sin notas de alertas para el desarrollador
 *
 *
 *  \copyright  ZIGOR R&D AIE
 *
 */

#include "realtime.h"

// Definición de atributos privados

TIME_STAMPING reloj_sistema;                            // Reloj del sistema

TIMER_OBJECT_ID ind_timer_service;
uint16 timers_used;

TIMER_OBJECT timer_service[MAX_CONCURRENT_TIMERS];      // Buffer de objetos timers


// Definición de métodos privados
interrupt void Kernel_Realtime(void);
uint32 Diff_Time(TIME_STAMPING, TIME_STAMPING);

// Definición de métodos públicos
void Init_Realtime(void);
void Reset_Clock(void);
void Stop_Clock(void);
void Start_Clock(void);
void Set_Clock(TIME_STAMPING);
void Get_Clock(TIME_STAMPING *);
TIMER_OBJECT_ID Get_Timer(TIMER_TYPE);
void Release_Timer(TIMER_OBJECT_ID);
void Set_Timer (TIMER_OBJECT_ID, uint32);
void Start_Timer(TIMER_OBJECT_ID);
void Stop_Timer(TIMER_OBJECT_ID);
void Reset_Timer(TIMER_OBJECT_ID);
uint16 Check_Timer(TIMER_OBJECT_ID);
void Set_Callback(TIMER_OBJECT_ID,void (* pcallback)(void));

// definición de la clase pública
clase_realtime RealTime;

// Desarrollo de métodos

interrupt void Kernel_Realtime(void)
{
    uint16 index;
    uint32 difftime;

    reloj_sistema.msec++;
    if(1000==reloj_sistema.msec)
    {
        reloj_sistema.msec=0;
        reloj_sistema.sec++;
        if(60==reloj_sistema.sec)
        {
            reloj_sistema.sec=0;
            reloj_sistema.min++;
            if(60==reloj_sistema.min)
            {
                reloj_sistema.min=0;
                reloj_sistema.hora++;
                if(24==reloj_sistema.hora)
                    reloj_sistema.hora=0;
            }
        }
#ifdef  UNIT_TESTS
        printf("%d:%d:%d\n",reloj_sistema.hora,reloj_sistema.min,reloj_sistema.sec);
#endif
    }
    // Control de timers
    for(index=0;index<MAX_CONCURRENT_TIMERS;index++)
    {
        if(timer_service[index].estado==ONGOING)
        {
            Get_Clock(&timer_service[index].tfinal);
            difftime=Diff_Time(timer_service[index].tfinal,timer_service[index].tinicial);
            if(timer_service[index].timeout>=difftime)
            {
                timer_service[index].estado=TIMEOUT;
                if(timer_service[index].tipo==INTERRUPT)
                {
                    // ToDO: Lanzar interrupción SW de Servicio de Temporización
                }
            }
        }
    }

    // Actualiza temporizado de tareas periódicas y semáforos
    // (ToDo)

}


void Init_Realtime(void)
{
    uint16 index;
    uint32 timhl;
    uint16 timh;
    uint16 timl;

    // Inicialización de atributos
    reloj_sistema.hora=0;
    reloj_sistema.min=0;
    reloj_sistema.sec=0;
    reloj_sistema.msec=0;
    ind_timer_service=0;
    timers_used=0;

    for(index=0;index<MAX_CONCURRENT_TIMERS;index++)
    {
        timer_service[index].estado=UNUSED;
        timer_service[index].tipo=POLLING;
        timer_service[index].timeout=0;
        timer_service[index].tinicial.hora=0;
        timer_service[index].tinicial.min=0;
        timer_service[index].tinicial.sec=0;
        timer_service[index].tinicial.msec=0;
        timer_service[index].tfinal.hora=0;
        timer_service[index].tfinal.min=0;
        timer_service[index].tfinal.sec=0;
        timer_service[index].tfinal.msec=0;
        timer_service[index].callback=NULL;
    }

    // Configuración de TIMER 2 para periodo fijo de 1ms=1000us
    timhl=((uint32)ONE_ms_IN_us * (uint32)CLOCKSYS_MHZ)-1;
    timh=(uint16)((timhl>>16)&0x0000FFFF);
    timl=(uint16)(timhl&0x0000FFFF);

    CpuTimer2Regs.TCR.bit.TSS=1;        // Stop TIMER2
    CpuTimer2Regs.TCR.bit.TIE=1;        // Habilita interrupción TIMER 2

    CpuTimer2Regs.PRD.bit.MSW=timh;     // Carga el periodo del TIMER2
    CpuTimer2Regs.PRD.bit.LSW=timl;

    CpuTimer2Regs.TCR.bit.TRB=1;        // Carga TIM=PRD

    // Inicialización de métodos

    RealTime.InitRealtime=Init_Realtime;
    RealTime.ResetClock=Reset_Clock;
    RealTime.StopClock=Stop_Clock;
    RealTime.StartClock=Start_Clock;
    RealTime.SetClock=Set_Clock;
    RealTime.GetClock=Get_Clock;
    RealTime.GetTimer=Get_Timer;
    RealTime.ReleaseTimer=Release_Timer;
    RealTime.SetTimer=Set_Timer;
    RealTime.StartTimer=Start_Timer;
    RealTime.StopTimer=Stop_Timer;
    RealTime.ResetTimer=Reset_Timer;
    RealTime.CheckTimer=Check_Timer;
    RealTime.SetCallback=Set_Callback;

}

void Reset_Clock(void)
{
    Stop_Clock();
    reloj_sistema.hora=0;
    reloj_sistema.min=0;
    reloj_sistema.sec=0;
    reloj_sistema.msec=0;
}

void Stop_Clock(void)
{
    CpuTimer2Regs.TCR.bit.TSS=1;            // Stop TIMER2
}

void Start_Clock(void)
{
    CpuTimer2Regs.TCR.bit.TSS=0;            // Start TIMER2
}

void Set_Clock(TIME_STAMPING tsynchro)
{
    Stop_Clock();
    if(tsynchro.hora <24 && tsynchro.min<60 && tsynchro.sec<60 && tsynchro.msec<1000)
    {
        reloj_sistema.hora=tsynchro.hora;
        reloj_sistema.min=tsynchro.min;
        reloj_sistema.sec=tsynchro.sec;
        reloj_sistema.msec=tsynchro.msec;
    }
    Start_Clock();
}

void Get_Clock(TIME_STAMPING * ptime)
{
    DINT;
    ptime->msec=reloj_sistema.msec;
    ptime->sec=reloj_sistema.sec;
    ptime->min=reloj_sistema.min;
    ptime->hora=reloj_sistema.hora;
    EINT;
}

TIMER_OBJECT_ID Get_Timer(TIMER_TYPE tipo)
{
    uint16 index=0;
    TIMER_OBJECT_ID id=ERROR;
    uint16 flag=0;

    // Busca timer no usado
    while(0==flag)
    {
        if(timer_service[index].estado==UNUSED)
        {
            timer_service[index].estado=PAUSED;
            id=(TIMER_OBJECT_ID)(index);
            timer_service[index].tipo=tipo;
            flag=1;
        }
        else
        {
            index++;
            if(index==MAX_CONCURRENT_TIMERS)
                flag=1;
        }
    }
    return(id);
}

void Release_Timer(TIMER_OBJECT_ID id)
{
    if(0<=id && MAX_CONCURRENT_TIMERS>id)
        timer_service[id].estado=UNUSED;
}
void Set_Timer (TIMER_OBJECT_ID id, uint32 tout)
{
    if(0<=id && MAX_CONCURRENT_TIMERS>id)
        timer_service[id].timeout=tout;
}

void Start_Timer(TIMER_OBJECT_ID id)
{
    if(0<=id && MAX_CONCURRENT_TIMERS>id)
    {
        Get_Clock(&timer_service[id].tinicial);
        Get_Clock(&timer_service[id].tfinal);
        timer_service[id].estado=ONGOING;
    }
}

void Stop_Timer(TIMER_OBJECT_ID id)
{
    if(0<=id && MAX_CONCURRENT_TIMERS>id)
        timer_service[id].estado=PAUSED;
}

void Reset_Timer(TIMER_OBJECT_ID id)
{
    if(0<=id && MAX_CONCURRENT_TIMERS>id)
    {
        timer_service[id].estado=PAUSED;
        Get_Clock(&timer_service[id].tinicial);
        Get_Clock(&timer_service[id].tfinal);
    }
}

uint16 Check_Timer(TIMER_OBJECT_ID id)
{
    uint16 flag=NO;

    if(0<=id && MAX_CONCURRENT_TIMERS>id)
        if(timer_service[id].estado==TIMEOUT)
            flag=SI;
    return(flag);
}

void Set_Callback(TIMER_OBJECT_ID id,void (* pisr)(void))
{
    if(0<=id && MAX_CONCURRENT_TIMERS>id)
        timer_service[id].callback=pisr;

}

uint32 Diff_Time(TIME_STAMPING tnew, TIME_STAMPING told)
{
    uint32 difftime;
    uint32 t2,t1;

    t2=((uint32)(tnew.hora)*(uint32)(3600)+(uint32)(tnew.min*60)+(uint32)(tnew.sec));
    t1=((uint32)(told.hora)*(uint32)(3600)+(uint32)(told.min*60)+(uint32)(told.sec));

    if(tnew.hora<told.hora)
        t2+=86400;

    difftime=t2-t1;

    return(difftime);
}



#ifdef  UNIT_TESTS
#include    <stdio.h>

void Script_RealTime(void);
void Callback(void);


void Script_RealTime(void)
{
    uint16 veces=0;
    uint16 index,flag;
    TIME_STAMPING t1,t2;
    TIMER_OBJECT_ID id[MAX_CONCURRENT_TIMERS+1];

    // Prueba el funcionamiento de la rutina de interrupción estando las DINT
    RealTime.InitRealtime();

    RealTime.GetClock(&t1);

    for(veces=0;veces<1000;veces++)
        Kernel_Realtime();

    RealTime.GetClock(&t2);

    if(t1.msec!=0 || t1.sec!=0 || t1.min!=0 || t1.hora!=0)
        printf("FALLO GetClock()\n");

    if(t2.msec!=1 || t2.sec!=0 || t2.min!=0 || t2.hora!=0)
            printf("FALLO Kernel_Realtime()\n");

    // Prueba de configuración horaria y valores límite
    t1.hora=23;
    t1.min=59;
    t1.sec=59;
    t1.msec=999;

    RealTime.SetClock(t1);

    if(reloj_sistema.hora!=23 || reloj_sistema.min!=59 || reloj_sistema.sec!=59 || reloj_sistema.msec!=999)
        printf("FALLO Set_Clock()\n");

    Kernel_Realtime();

    if(reloj_sistema.hora!=0 || reloj_sistema.min!=0 || reloj_sistema.sec!=0 || reloj_sistema.msec!=0)
        printf("FALLO Boundary Kernel_RealTime()\n");

    // Prueba de reserva de servicio de temporización

    for(index=0;index<(MAX_CONCURRENT_TIMERS+1);index++)
        id[index]=RealTime.GetTimer(POLLING);

    flag=0;
    for(index=0;index<MAX_CONCURRENT_TIMERS;index++)
        if(id[index]=index)
            flag=1;
    if(id[MAX_CONCURRENT_TIMERS]!=ERROR)
        flag=1;

    if(1==flag)
        printf("FALLO Get_Timer()\n");
    else
    {
        flag=0;
        // Start, Stop, timeout, reset y release
        RealTime.ResetTimer(3);
        RealTime.StartTimer(3);
        for(veces=0;veces<9;veces++)
            Kernel_Realtime();

        if(RealTime.CheckTimer(3)!=NO)
            flag=1;

        Kernel_Realtime();
        if(RealTime.CheckTimer(3)!=SI)
            flag=1;

        if(1==flag)
            printf("FALLO Timeout\n");

        flag=0;
        RealTime.ResetTimer(3);
        RealTime.SetTimer(3,2);
        RealTime.StartTimer(3);

        Kernel_Realtime();
        RealTime.StopTimer(3);
        Kernel_Realtime();

        if(RealTime.CheckTimer(3)!=NO)
            flag=1;

        RealTime.StartTimer(3);
        Kernel_Realtime();

        if(RealTime.CheckTimer(3)!=SI)
            flag=1;

        if(1==flag)
            printf("FALLO Start-Stop timer\n");

        RealTime.ReleaseTimer(3);

        id[3]=RealTime.GetTimer(INTERRUPT);
        if(3!=id[3])
            printf("FALLO reasignaci�n de timer\n");
        else
        {
            RealTime.SetCallback(3,Callback);
            RealTime.SetTimer(3,1);
            RealTime.StartTimer(3);
            Kernel_Realtime();
            printf("Callback Timer mensaje ok?\n");
        }


    }

}

void Callback(void)
{
    printf("Callback Timer OK\n");
}

#endif
