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
 *  El proyecto *Project_Template* es una plantilla para crear aplicaciones empotradas para la famlia C28X
 *  de Texas Instruments. De esta manera, para crear un nuevo proyecto, basta:
 *
 *  1. Clonar la versión más actualizada del proyecto *Project_Template* del repositorio oficial en el Workspace de nuestro
 *  ordenador.
 *  2. Configurar ***system.h*** las definiciones que se ajusten al HW real en el que se ejecutará la aplicación
 *  3. Desarrollar la aplicación dentro de la carpeta  ***Application***, de acuerdo con las reglas de
 *  desarrollo de código de UTI-SW.
 *
 *  \image  html    arquitectura_proyecto.jpg
 *
 *  El usuario sólo debe modificar el contenido del fichero ***system.h***. No debe modificar ningún otro
 *  fichero de *Kernel*.
 *
 *  En la carpeta ***Application/Src*** se ecuentra el fichero *main.c*, que lo suministra también la
 *  plantilla. En este fichero se encuentra la función main(), que incluye 2 llamadas a
 *  métodos públicos del Kernel, *kernel.Pre_kernel()* y *kernel.Kernel()*. Estas 2 llamadas deben
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
 *  de interrupción tienen asociados *semáforos*, que permiten sincronizar su ejecución con la activación
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
 *  *Semáforos*: Este mecanismo permite a cualquier tarea, de la aplicación o del kernel, a crear un objeto
 *  semáforo, y permite que otra tarea se suscriba a este semáforo. El semáforo tiene un estado que puede
 *  ser *ON* o *OFF*. El estado *ON* sólo lo puede poner la tarea que crea el semáforo. El estado
 *  es *OFF* tras crear el semáforo, o bien tras ejecutarse la tarea suscrita.
 *
 *  El mecanismo de semáforos permite gestionar a un mismo semáforo diveras suscripciones. El máximo
 *  número de suscriptores viene descrito por la constante ***MAX_NUMERO_SUSCRIPTORES_SEMAFORO***, en el
 *  fichero  *semaphores.h*.
 *
 *
 *  \page       page_core   Core
 *
 *
 *  CORE es un componente del Kernel, que realiza la inicialización del sistema tras un power-up o reset
 *  e imlementa la máquina de estados del funcionamiento del Kernel.
 *
 *  Implementa 2 métodos públicos, que *deben ser ejecutados en la función main()*:
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


#include <core.h>


// Definición de atributos públicos y privados
clase_kernel kernel;

// Declaración de métodos públicos
void Pre_Kernel(void);
void Kernel(void);

// Declaración de métodos privados
void Power_On(void);
int16 Check_Processor(void);
void Check_Reset_Cause(void);
void Init_Flash(void);
void Configure_Clocks(void);
void Configure_RAM(void);
void Copy_Flash_to_RAM(void);
void Init_Peripherals(void);
void Configure_Watchdog(void);
void Boot_Failure(void);
void Init_Kernel_Object(int16);


// Definición del objeto kernel
clase_kernel kernel;

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
        Init_Kernel_Object(estado);
    }
    else
    {
        Diagnosys.Write_Event(UNEXPECTED_PROCESSOR);
        Boot_Failure();

    }



}


void Kernel(void)
{

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
        Init_Kernel_Object(estado);
    }
    else
    {
        printf("Identificación Procesador KO\n");

    }



}

#endif

// Métodos Privados

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
    uint32 valor;
    int16 resultado=ERROR;

    valor=DevCfgRegs.PARTIDH.all;

#if (TMS320F28004X==true)
    if(valor==F280049 || valor==F280048 || valor==F280045 || valor==F280041 || valor== F280040)
        resultado=OK;
#endif


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

}

void Configure_Clocks(void)
{

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

void Boot_Failure(void)
{

}

void Init_Kernel_Object(int16 estado)
{

}


