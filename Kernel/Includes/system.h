/*
 * System.h
 *
 *  Created on: 31 oct. 2022
 *  Author: Dr. Carlos Romero
 *
 *  Este fichero de encabezamiento define etiquetas que utiliza la función Pre_Kernel() para configurar el sistema.
 *
 *  El valor de estas etiquetas debe modificarse para cada aplicación, de acuerdo al HW del producto.
 *
 *  El desarrollador deberá seguir las indicaciones de los comentarios en el fichero. La no observancia
 *  de estos comentarios puede llevar a compilaciones incompatibles con el HW real, o fallos de compilación.
 */

#ifndef INCLUDES_SYSTEM_H_
#define INCLUDES_SYSTEM_H_


#define CLOCKIN_MHZ         10      // Frecuencia del reloj o cristal externo en MHz. Si el valor es 0, se utiliza
                                    // el reloj interno del microprocesador

#define CLOCKSYS_MHZ        100

// Definir si la fuente del reloj de Sistema es externa (true=se usa fuente externa. false=no se usa fuente externa)
#define CLOCK_EXTERN        false

// Si se usa fuente externa, definir si es singled ended o dual ended (cristal o resonador) (true= opción seleccionada. false= opción no seleccionada)
#if CLOCK_EXTERN==true
#define SINGLE_ENDED        true
#endif


// Configurar la fuente de 1.2V externa (Poner a SI si se usa DCDC externo para 1.2V y NO en caso contrario.
// IMPORTANTE: Ver datasheet para el dispositivo y nº de pines para poner configuración compatible)
#define DCDC_1200mV_EXTERNO     true


// Seleccionar periféricos que se que usa la aplicación (true: Se usa false: no se usa)
#define ePWM_PERIPHERAL         false
#define eCAP_PERIPHERAL         false
#define eQEP_PERIPHERAL         false
#define SD_PERIPHERAL           false
#define ADC_PERIPHERAL          false
#define GPIO_PERIPHERAL         false
#define IXBAR_PERIPHERAL        false
#define OXBAR_PERIPHERAL        false
#define CMPSS_PERIPHERAL        false
#define DAC_PERIPHERAL          false
#define SPI_PERIPHERAL          false
#define CAN_PERIPHERAL          false
#define SCI_PERIPHERAL          false
#define I2C_PERIPHERAL          false
#define NMI_WD_PERIPHERAL       true
#define PGA_PERIPHERAL          false
#define PWMXBAR_PERIPHERAL      false
#define PMBUS_PERIPHERAL        false
#define FSI_PERIPHERAL          false
#define LIN_PERIPHERAL          false
#define WINDOWEDWD_PERIPHERAL   false
#define HRPWM_PERIPHERAL        false
#define USB_PERIPHERAL          false
#define MCBSP_PERIPHERAL        false
#define UPP_PERIPHERAL          false
#define EMIF_PERIPHERAL         false


enum    CLOCK_SOURCES
{
    INTOSC2,
    XTAL,
    INTOSC1
};


#endif /* INCLUDES_SYSTEM_H_ */
