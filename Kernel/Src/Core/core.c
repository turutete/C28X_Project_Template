/** \page   page_kernel   Kernel
 *
 *
 *
 *  Kernel es un conjunto de aplicaciones comunes a cualquier aplicación para la familia C2000 de TI, que ofrecen
 *  métodos y servicios que facilitan la implementación de cualquier aplicación de tiempo real.
 *
 *  Está compuesto por los siguientes módulos:
 *
 *  -\subpage    page_config
 *  -\subpage    page_core
 *  -\subpage    page_realtime
 *  -\subpage    page_scheduler
 *  -\subpage    page_threads
 *  -\subpage    page_hal
 *  -\subpage    page_boot
 *  -\subpage    page_diagnosys
 *  -\subpage    page_upgrader
 *  -\subpage    page_services
 *
 *  En particular, este Kernel ofrece los siguientes servicios:
 *
 *  * Configuración del microprocesador (Flash, Memorias, Periféricos, Relojes)
 *  * Implementa la capa HAL (Hardware Abstraction Level), suministrando métodos para la utilización de los recursos HW
 *      del microcontrolador (periféricos).
 *  * Implementa diferentes tipos de hilos (threads), y mecanismos de sincronización entre hilos.
 *  * Implementa un planificador de tareas que puede configurarse para ser o no preemtivo.
 *  * Implementa un mecanismo de actualización de SW
 *  * Implementa un mecanismo de diagnóstico, para permitir la identificación de problemas en ejecución.
 *  * Implementa diversos servicios, que pueden ser utilizados por la aplicación
 *
 *  Aplicaciones con Kernel
 *  -----------------------
 *  El proyecto *_Project_Template_* es una plantilla para crear aplicaciones empotradas para la famlia C28X
 *  de Texas Instruments. De esta manera, para crear un nuevo proyecto, basta:
 *
 *  1. Clonar la versión más actualizada del proyecto *_Project_Template_* del repositorio oficial en el Workspace de nuestro
 *  ordenador.
 *  2. Configurar en ***system.h*** las definiciones que se ajusten al HW real en el que se ejecutará la aplicación
 *  3. Desarrollar la aplicación dentro de la carpeta  ***Application***, de acuerdo con las reglas de
 *  desarrollo de código de UTI-SW.
 *
 *  \image  html    arquitectura_proyecto.jpg
 *
 *  El usuario sólo debe modificar el contenido del fichero ***system.h***. No debe modificar ningún otro
 *  fichero de **Kernel**.
 *
 *  En la carpeta ***Application/Src*** se ecuentra el fichero *main.c*, que lo suministra también la
 *  plantilla. En este fichero se encuentra la función main(), que incluye 2 llamadas a
 *  métodos públicos del Kernel, *_kernel.Pre_kernel()_* y *_kernel.Kernel()_*. Estas 2 llamadas deben
 *  conservarse, y en el misma posición donde están, es decir, lo primero que debe efectuarse tras un power up
 *  de la aplicación es una llamada a kernel.Prekernel(), y la última línea de main() debe ser
 *  la llamada a kernel.Kernel().
 *
 *  Entre ambas llamadas, el usuario deberá incluir las llamadas a métodos públicos de creación de
 *  tareas que necesite la aplicación, así como los mecanismos de sincronización entre tareas.
 *
 *  Tareas
 *  ======
 *  El diseño de aplicaciones con kernel debe efectuarse mediante el uso de ***Tareas***, prioridades
 *  y mecanismos de sincronización, suministrados y gestionados por el Kernel.
 *
 *  Tipos de Tareas
 *  ---------------
 *
 *  *HWI*: Interrupciones HW. Son las rutinas de servicio de interrupción que ofrece la familia de
 *  microcontroladores C28X. Todas estas rutinas son parte de la capa ***HAL*** del kernel. Estas tareas
 *  se lanzan mediante el mecanismo de interrupciones asíncronas del procesador. Cada una de estas rutinas
 *  de interrupción tienen asociados **semáforos**, que permiten sincronizar su ejecución con la activación
 *  de tareas de la aplicación.
 *
 *  *SWI*:  Interrupciones SW. Son rutinas de interrupción utilizadas por el Kernel para sincronizar
 *  tareas internas del kernel.
 *
 *  *PRD*:   Tareas periódicas. Son tareas de la aplicación, que se ejecutan periódicamente.
 *
 *  *TSK*: Son tareas de la aplicación, que se activan mediente mecanismos de sincronización con
 *  procesos o tareas del kernel, o bien con otras tareas de la aplicación.
 *
 *  Sincronismo entre Tareas
 *  ------------------------
 *  El Kernel ofrece los siguientes mecanismos de sincronización entre tareas.
 *
 *  *Semáforos*: Este mecanismo permite a cualquier tarea, de la aplicación o del kernel, crear un objeto
 *  semáforo, y permite que otra tarea se suscriba a este semáforo. El semáforo tiene un estado que puede
 *  ser *ON* o *OFF*. El estado *ON* sólo lo puede poner la tarea que crea el semáforo. El estado
 *  es *OFF* tras crear el semáforo, o bien tras ejecutarse la tarea suscrita.
 *
 *  El mecanismo de semáforos permite gestionar a un mismo semáforo diveras suscripciones. El máximo
 *  número de suscriptores viene descrito por la constante ***MAX_NUMERO_SUSCRIPTORES_SEMAFORO***, en el
 *  fichero  *semaphores.h*.
 *
 *  Mutex
 *  ------
 *  Este mecanismo suspende la ejecución de una tarea el tiempo (en ms) indicado. Es decir, cuando en una tarea
 *  se activa un mecanismo de mutex, la tarea deja de ejecutarse hasta pasado el tiempo indicado.
 *
 *  \page       page_core   Core
 *
 *
 *  CORE es un componente del Kernel, que realiza la inicialización del sistema tras un power-up o reset
 *  e imlementa la máquina de estados del funcionamiento del Kernel.
 *
 *  Implementa 2 métodos públicos, que **deben ser ejecutados en la función main()**:
 *
 *  * Pre_Kernel()
 *  * Kernel()
 *
 *  De esta forma, en la función ***main()*** la primera instrucción que se realiza es una llamada a ***Pre_Kernel()***,
 *  y la última instrucción que se efectúa es una llamada a ***Kernel()***.
 *
 *  Son 2 métodos sin parámetros de llamada, ni tampoco retornan ningún valor. En el caso de producirse alguna excepción
 *  durante su ejecución, dichas excepciones se reflejarán en la variable de estado del Kernel, ***kernel.kernel_status***,
 *  así como en el logger del sistema.
 *
 *  Pre_Kernel()
 *  ------------
 *
 *  Este método supervisa el estado del microcontrolador e inicializa el sistema:
 *
 *  1. Resetea el Log del sistema
 *  2. Comprueba si el sistema está configurado para usar DCDC de 1.2V externo o interno
 *  3. Si es DCDC interno, activa el DCDC, y si no es posible arrancarlo, deja el LDO
 *  4. Comprueba el estado del microcontrolador
 *  5. Comprueba cuál fue el motivo del power on y lo anota en el log del sistema
 *  6. Inicializa la Flash
 *  7. Inicializa relojes del sistema
 *  8. Configura las distintas RAMs
 *  9. Copia en RAM el código definido en Flash para ser ejecutado en RAM
 *  10. Inicializa periféricos
 *  11. Configura el Watchdog
 *  12. Crea la tabla de vectores de interrupción
 *  12. Inicializa la clase Kernel
 *
 *  En caso de que al comprobar el estado del microcontrolador identificara un fallo catastrófico, aborta el arranque
 *  y ejecuta el procedimiento de *Fallo en el arranque*
 *
 *  Kernel()
 *  --------
 *
 *  (Pendiente de explicar)
 *
 *
 *  Historial de Cambios
 *  --------------------
 *  | Fecha | Autores | Versión | Descripción |
 *  |:--: |:--:|:--:|:--:|
 *  | 10/12/2022| Dr. Carlos Romero |1.0.0.0|Primera edición|
 *
 *  ToDos
 *  -----
 *
 *  \todo   Hacer ISR de NMI
 *  \todo   Detectar causa de NMI por CLOCKFAIL
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


#include <core.h>

// Definiciones locales
#define CPU1_ID     1
#define CPU2_ID     2

//  Establecimiento de variables en la sección .kernelvar reservada para el kernel en RAMM1
#pragma DATA_SECTION(kernel,".kernelvar")

// Establecimiento de métodos en la sección .flashinit del kernel
#pragma CODE_SECTION(Configure_Flash,".flashinit")

// Definición de atributos públicos y privados
clase_kernel kernel;
device_info dispositivo;

// Declaración de métodos públicos
void Pre_Kernel(void);
void Kernel(void);

// Declaración de métodos privados
void Power_On(void);
int16 Check_Processor(void);
void Check_Reset_Cause(void);
void Init_Flash(void);
void Configure_Flash(void);
void Configure_Clocks(void);
void Configure_PLL_Registers(void);
void Configure_RAM(void);
void Copy_Flash_to_RAM(void);
void Init_Peripherals(void);
void Configure_Watchdog(void);
void Create_Vector_Table(void);
void Boot_Failure(void);
void Init_Kernel_Object(int16);


#ifndef UNIT_TESTS
// Métodos de core.c
void Pre_Kernel(void)
{
    int16 estado;

    Diagnosys.Init_System_Log();

    Power_On();

    estado=Check_Processor();

    if(estado!=ERROR)
    {
        Check_Reset_Cause();
        Init_Flash();
        Configure_Clocks();
        Configure_RAM();
        Copy_Flash_to_RAM();
        Init_Peripherals();
        Configure_Watchdog();
        Create_Vector_Table();
        Init_Kernel_Object(estado);
    }
    else
    {
        Diagnosys.Write_Event(UNEXPECTED_PROCESSOR);
        Boot_Failure();
    }

}

#else       // Test Unitarios
void Pre_Kernel(void)
{
    // Pre_Kernel para debug
    int16 estado;

    Script_Diagnosys();


    estado=Check_Processor();

    if(estado!=ERROR)
    {
        printf("Identificación Procesador OK\n");
        Check_Reset_Cause();
        Init_Flash();
        Configure_Clocks();
        Configure_RAM();
        Copy_Flash_to_RAM();
        Init_Peripherals();
        Configure_Watchdog();
        Create_Vector_Table();
        Init_Kernel_Object(estado);
    }
    else
    {
        printf("Identificación Procesador KO\n");

    }



}

#endif

// Métodos Privados

void Kernel(void)
{

}

void Power_On(void)
{
 #if (DCDC_1200mV_EXTERNO==false)

    int16 counter;
    int16 flag;
    uint16 value;

    EALLOW;
    AnalogSubsysRegs.DCDCCTL.bit.DCDCEN=ON;
    flag=OFF;
    counter=DCDC_POWERON_TRIALS;
    while (flag==OFF)
    {
        value=AnalogSubsysRegs.DCDCSTS.bit.SWSEQDONE;
        if (value==OK)
            flag=OK;
        else
        {
            counter--;
            if(counter<=0)
            {
                flag=ERROR;
            }
        }
    }

    if(flag==OK)
    {
        value=AnalogSubsysRegs.DCDCSTS.bit.INDDETECT;
        if(value==OFF)
            flag=ERROR;
        else
        {
            counter=DCDC_POWERON_80us;
            while(counter>=0)
                counter--;
        }
    }

    if(flag==OK)
    {
        kernel.kernel_status.bits.DCDC_1200mV=DCDC_INTERNAL;
        Diagnosys.Write_Event(POWER_ON_OK);
    }
    else
    {
        kernel.kernel_status.bits.DCDC_1200mV=LDO_INTERNAL;
        Diagnosys.Write_Event(POWER_ON_KO);
    }

    EDIS;



#else
    kernel.kernel_status.bits.DCDC_1200mV=DCDC_EXTERNAL;
    Diagnosys.Write_Event(POWER_ON_OK);

#endif

}


int16 Check_Processor(void)
{
    int16 resultado=OK;

    dispositivo.device_id=DevCfgRegs.PARTIDH.all;
    dispositivo.revision_id=DevCfgRegs.REVID.all;
    dispositivo.serial_number=DevCfgRegs.REVID.all;


    switch(dispositivo.device_id)
    {
    case 0x01FF0500:
        dispositivo.device=F280049C;
        dispositivo.cpu_id=CPU1_ID;
        if(dispositivo.revision_id>=0x00000002)
            dispositivo.serial_number=DevCfgRegs.REVID.all;
        else
            dispositivo.serial_number=0x00000000;
        break;
    case 0x00F90300:
        dispositivo.device=F28379D;
        dispositivo.cpu_id=CPU1_ID;
        if(dispositivo.revision_id>=0x00000003)
            dispositivo.serial_number=DevCfgRegs.REVID.all;
        else
            dispositivo.serial_number=0x00000000;
        break;

    case 0x02F90300:
        dispositivo.device=F28379D;
        dispositivo.cpu_id=CPU2_ID;
        if(dispositivo.revision_id>=0x00000003)
            dispositivo.serial_number=DevCfgRegs.REVID.all;
        else
            dispositivo.serial_number=0x00000000;
        break;

    default:
        dispositivo.device=UNSUPPORTED;
        dispositivo.cpu_id=CPU1_ID;
        dispositivo.serial_number=0x00000000;
        resultador=ERROR;
        break;
    }


    return(resultado);

}

void Check_Reset_Cause(void)
{
    if(CpuSysRegs.RESC.bit.POR)
        Diagnosys.Write_Event(POWER_ON_RESET);
    else
    {
        if(CpuSysRegs.RESC.bit.NMIWDRSn)
            Diagnosys.Write_Event(NMI_RESET);
        if(CpuSysRegs.RESC.bit.WDRSn)
            Diagnosys.Write_Event(WATCHDOG_RESET);
        if(CpuSysRegs.RESC.bit.XRSn)
            Diagnosys.Write_Event(EXTERNAL_RESET);
    }

    CpuSysRegs.RESCCLR.all=0x0000010F;


}

void Init_Flash(void)
{
    // Copia el código inicializador de la flash en RAM
    uint32 * pcode_read;
    uint32 * pcode_write;
    uint16  size;
    uint16  veces;

    pcode_read=_FlashinitLoadStart;
    pcode_write=_FlashinitRunStart;
    size=_FlashinitLoadSize;

    for(veces=0;veces<size;veces++)
        *(pcode_write++)=*(pcode_read++);

    // Y lo ejecuta
    Configure_Flash();
}

void Configure_Flash(void)
{
    uint16 clk=CLOCKSYS_MHZ;

    Flash0CtrlRegs.FRD_INTF_CTRL.bit.DATA_CACHE_EN=0;
    Flash0CtrlRegs.FRD_INTF_CTRL.bit.PREFETCH_EN=0;

    if(dispositivo.device==F280049C)
    {
        if(clk>80)
            Flash0CtrlRegs.FRDCNTL.bit.RWAIT=4;
        else if (clk>60)
            Flash0CtrlRegs.FRDCNTL.bit.RWAIT=3;
        else if (clk>40)
            Flash0CtrlRegs.FRDCNTL.bit.RWAIT=2;
        else if (clk>20)
            Flash0CtrlRegs.FRDCNTL.bit.RWAIT=1;
        else
            Flash0CtrlRegs.FRDCNTL.bit.RWAIT=0;
    }
    if(dispositivo.device==F28379D)
    {
#if CLOCK_EXTERN==true
        if(clk>150)
            Flash0CtrlRegs.FRDCNTL.bit.RWAIT=3;
        else if (clk>100)
            Flash0CtrlRegs.FRDCNTL.bit.RWAIT=2;
        else if (clk>50)
            Flash0CtrlRegs.FRDCNTL.bit.RWAIT=1;
        else
            Flash0CtrlRegs.FRDCNTL.bit.RWAIT=0;

#else
        if(clk>145)
            Flash0CtrlRegs.FRDCNTL.bit.RWAIT=3;
        else if (clk>97)
            Flash0CtrlRegs.FRDCNTL.bit.RWAIT=2;
        else if (clk>48)
            Flash0CtrlRegs.FRDCNTL.bit.RWAIT=1;
        else
            Flash0CtrlRegs.FRDCNTL.bit.RWAIT=0;
#endif

    }


    Flash0CtrlRegs.FRD_INTF_CTRL.bit.DATA_CACHE_EN=1;
    Flash0CtrlRegs.FRD_INTF_CTRL.bit.PREFETCH_EN=1;

    Flash0CtrlRegs.FBAC.bit.BAGP=0;                         // Verificar el valor

    Flash0CtrlRegs.FBFALLBACK.bit.BNKPWR0=3;                // Verificar valor
    Flash0CtrlRegs.FBFALLBACK.bit.BNKPWR1=3;

}

void Configure_Clocks(void)
{
    uint16 veces;
    int16 flag=0;
    uint16 trial=0;
    uint16 clksource=INTOSC2;

    EALLOW;
#if CLOCK_EXTERN==true
    // External clock source
    ClkCfgRegs.XTALCR.bit.OSCOFF=0;
#if SINGLE_ENDED==true
    ClkCfgRegs.XTALCR.bit.SE=1;
#endif

    while(flag==0)
    {
        for(veces=0;veces<4;veces++)
        {
            ClkCfgRegs.X1CNT.bit.CLR=1;
            while(ClkCfgRegs.X1CNT.bit.X1CNT!=0x3FF);
        }
        ClkCfgRegs.CLKSRCCTL1.bit.OSCCLKSRCSEL=1;

        if(ClkCfgRegs.MCDCR.bit.MCLKSTS==0)
            flag=1;
        else
        {
            trial++;
            if(trial>=TRIAL_10ms)
            {
                flag=-1;
            }
        }
    }

    if(flag==-1)
    {
        // Error del reloj de tiempo real
        Diagnosys.Write_Event(EXTERNAL_CLOCK_KO);
        // Configura el reloj interno INTOSC2
        ClkCfgRegs.XTALCR.bit.OSCOFF=1;
        ClkCfgRegs.XTALCR.bit.SE=0;
        clksource=INTOSC2;
    }
    else
    {
        Diagnosys.Write_Event(EXTERNAL_CLOCK_OK);
        ClkCfgRegs.CLKSRCCTL1.bit.INTOSC2OFF=0;         // Deshabilita INTOSC2 porque no se usa
        clksource=XTAL;
    }


#else
    // INTOSC2 (opción por defecto)
    ClkCfgRegs.XTALCR.bit.OSCOFF=1;
    ClkCfgRegs.XTALCR.bit.SE=0;
#endif


    // Configuración del PLL a la máxima velocidad posible en el microcontrolador
    ClkCfgRegs.CLKSRCCTL1.bit.OSCCLKSRCSEL=clksource;
    asm('RPTK 349');        // El manual de referencia indica un minimo de 300 ciclos. Esperamos 350 ciclos
    asm('NOP');

    Configure_PLL_Registers();

    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV=0;

    // Habilitación de relojes de periféricos
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC=OFF;
    CpuSysRegs.PCLKCR0.bit.HRPWM=ON;
    CpuSysRegs.PCLKCR0.bit.DMA=ON;
    CpuSysRegs.PCLKCR0.bit.CPUTIMER2=ON;
    CpuSysRegs.PCLKCR0.bit.CPUTIMER1=ON;
    CpuSysRegs.PCLKCR0.bit.CPUTIMER0=ON;
    CpuSysRegs.PCLKCR0.bit.CLA1=ON;
#ifdef _F28379D
    CpuSysRegs.PCLKCR0.bit.GTBCLKSYNC=ON;
#endif

#if EMIF_PERIPHERAL==true
    CpuSysRegs.PCLKCR1.bit.EMIF1=ON;
    CpuSysRegs.PCLKCR1.bit.EMIF2=ON;
#endif

#if ePWM_PERIPHERAL==true
    CpuSysRegs.PCLKCR2.bit.EPWM1=ON;
    CpuSysRegs.PCLKCR2.bit.EPWM2=ON;
    CpuSysRegs.PCLKCR2.bit.EPWM3=ON;
    CpuSysRegs.PCLKCR2.bit.EPWM4=ON;
    CpuSysRegs.PCLKCR2.bit.EPWM5=ON;
    CpuSysRegs.PCLKCR2.bit.EPWM6=ON;
    CpuSysRegs.PCLKCR2.bit.EPWM7=ON;
    CpuSysRegs.PCLKCR2.bit.EPWM8=ON;
#endif

#if eCAP_PERIPHERAL==true
    CpuSysRegs.PCLKCR3.bit.ECAP1=ON;
    CpuSysRegs.PCLKCR3.bit.ECAP2=ON;
    CpuSysRegs.PCLKCR3.bit.ECAP3=ON;
    CpuSysRegs.PCLKCR3.bit.ECAP4=ON;
    CpuSysRegs.PCLKCR3.bit.ECAP5=ON;
    CpuSysRegs.PCLKCR3.bit.ECAP6=ON;
    CpuSysRegs.PCLKCR3.bit.ECAP7=ON;
#endif

#if eQEP_PERIPHERAL==true
    CpuSysRegs.PCLKCR4.bit.EQEP1=ON;
    CpuSysRegs.PCLKCR4.bit.EQEP2=ON;
#endif


#if SD_PERIPHERAL==true
    CpuSysRegs.PCLKCR6.bit.SD1=ON;
#endif

#if SCI_PERIPHERAL==true
    CpuSysRegs.PCLKCR7.bit.SCI_A=ON;
    CpuSysRegs.PCLKCR7.bit.SCI_B=ON;
#endif

#if SPI_PERIPHERAL==true
    CpuSysRegs.PCLKCR8.bit.SPI_A=ON;
    CpuSysRegs.PCLKCR8.bit.SPI_B=ON;
#endif

#if I2C_PERIPHERAL==true
    CpuSysRegs.PCLKCR9.bit.I2C_A=ON;
#endif

#if CAN_PERIPHERAL==true
    CpuSysRegs.PCLKCR10.bit.CAN_A=ON;
    CpuSysRegs.PCLKCR10.bit.CAN_B=ON;
#endif

#if MCBSP_PERIPHERAL==true
    CpuSysRegs.PCLKCR11.bit.McBSP_A=ON;
    CpuSysRegs.PCLKCR11.bit.McBSP_B=ON;
#endif

#if USB_PERIPHERAL==true
    CpuSysRegs.PCLKCR11.bit.USB_A=ON;
#endif

#if UPP_PERIPHERAL=true
    CpuSysRegs.PCLKCR12.bit.uPP_A=ON;
#endif


#if ADC_PERIPHERAL==true
    CpuSysRegs.PCLKCR13.bit.ADC_A=ON;
    CpuSysRegs.PCLKCR13.bit.ADC_B=ON;
    CpuSysRegs.PCLKCR13.bit.ADC_C=ON;
    CpuSysRegs.PCLKCR13.bit.ADC_D=OFF;
#endif

#if CMPSS_PERIPHERAL==true
    CpuSysRegs.PCLKCR14.bit.CMPSS1=ON;
    CpuSysRegs.PCLKCR14.bit.CMPSS2=ON;
    CpuSysRegs.PCLKCR14.bit.CMPSS3=ON;
    CpuSysRegs.PCLKCR14.bit.CMPSS4=ON;
    CpuSysRegs.PCLKCR14.bit.CMPSS5=ON;
    CpuSysRegs.PCLKCR14.bit.CMPSS6=ON;
    CpuSysRegs.PCLKCR14.bit.CMPSS7=ON;
#endif

#if PGA_PERIPHERAL==true
    CpuSysRegs.PCLKCR15.bit.PGA1=ON;
    CpuSysRegs.PCLKCR15.bit.PGA2=ON;
    CpuSysRegs.PCLKCR15.bit.PGA3=ON;
    CpuSysRegs.PCLKCR15.bit.PGA4=ON;
    CpuSysRegs.PCLKCR15.bit.PGA5=ON;
    CpuSysRegs.PCLKCR15.bit.PGA6=ON;
    CpuSysRegs.PCLKCR15.bit.PGA7=ON;
#endif

#if DAC_PERIPHERAL==true
    CpuSysRegs.PCLKCR16.bit.DAC_A=ON;
    CpuSysRegs.PCLKCR16.bit.DAC_B=ON;
#endif

#if CLB_PERIPHERAL==true
    CpuSysRegs.PCLKCR17.bit.CLB1=ON;
    CpuSysRegs.PCLKCR17.bit.CLB2=ON;
    CpuSysRegs.PCLKCR17.bit.CLB3=ON;
    CpuSysRegs.PCLKCR17.bit.CLB4=ON;
#endif

#if FSI_PERIPHERAL==true
    CpuSysRegs.PCLKCR18.bit.FSIRX_A=ON;
    CpuSysRegs.PCLKCR18.bit.FSITX_A=ON;
#endif

#if LIN_PERIHERAL==true
    CpuSysRegs.PCLKCR19.bit.LIN_A=ON;
#endif

#if PMBUS_PERIPHERAL==true
    CpuSysRegs.PCLKCR20.bit.PMBUS_A=ON;
#endif

#if DCC_PERIPHERAL==true
    CpuSysRegs.PCLKCR21.bit.DCC_0=ON;
#endif


    EDIS;
}

void Configure_PLL_Registers(void)
{
    float32 div_flotante, parte_fracional;
    uint16 imult, fmult, div_entera;
    uint16 numerador, denominador;

    numerador=(uint16)CLOCKSYS_MHZ;
    denominador=(uint16)CLOCKIN_MHZ;

    div_entera=numerador/denominador;
    div_flotante=(float)numerador/(float)denominador;

    imult=div_entera;
    parte_fraccional=div_flotante-(float)div_entera;

    if(parte_fraccional>0.875)
    {
        fmult=0;
        imult++;

    }
    else if (parte_fraccional>0.625)
    {
        fmult=3;
    }
    else if (parte_fraccional> 0.375)
    {
        fmult=2;
    }
    else if (parte_fraccional>0.125)
    {
        fmult=1;
    }
    else
    {
        fmult=0;
    }

    ClkCfgRegs.SYSPLLMULT.bit.FMULT=fmult;
    ClkCfgRegs.SYSPLLMULT.bit.IMULT=imult;
    ClkCfgRegs.SYSPLLMULT.bit.ODIV=0;
    ClkCfgRegs.SYSCLKDIVSEL.bit.PLLSYSCLKDIV=0;

    ClkCfgRegs.SYSPLLCTL1.bit.PLLCLKEN=1;

}

void Configure_RAM(void)
{

}

void Copy_Flash_to_RAM(void)
{

}

void Init_Peripherals(void)
{

}

void Configure_Watchdog(void)
{

}

void Create_Vector_Table(void)
{
    DINT;
    PieCtrlRegs.PIECTRL.bit.ENPIE=1;
    EALLOW;

    // Configuración de la tabla de vectores de interrupción del PIE
    //PieVectTable.x=&isr_name;


    // Habilitación de los PIEIER.x

    //Habilitación de bits en IER

    // Habilitación de interrupciones en los periféricos


    EINT;

}

void Boot_Failure(void)
{

}

void Init_Kernel_Object(int16 estado)
{

}


