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


// SELECCIONAR EL MICROPROCESADOR. SÓLO UNO DE ELLOS PUEDE ESTAR A true. EL RESTO DEBE SER false
#define     TMS320F2837XD   false
#define     TMS320F2837XS   false
#define     TMS320F28004X   true

#define CLOCKIN_MHZ         10      // Frecuencia del reloj o cristal externo en MHz. Si el valor es 0, se utiliza
                                    // el reloj interno del microprocesador


// Por defecto se selecciona la frecuencia máxima de reloj de la CPU de la familia. Pero es posible poner un valor
// menor. Pero nunca deberá ponerse un valor superior al indicado en el datasheet del procesador.

#if (TMS320F28004X==SI)
#define CLOCKSYS_MHZ        100
#endif

#if (TMS320F2837XD==SI || TMS320F2837XS==SI)
#define CLOCKSYS_MHZ        200
#endif


// Definir si la fuente del reloj de Sistema es externa (true=se usa fuente externa. false=no se usa fuente externa)
#define CLOCK_EXTERN        false

// Si se usa fuente externa, definir si es singled ended o dual ended (cristal o resonador) (true= opción seleccionada. false= opción no seleccionada)
#if CLOCK_EXTERN==true
#define SINGLE_ENDED        true
#endif


// Configurar la fuente de 1.2V externa (Poner a SI si se usa DCDC externo para 1.2V y NO en caso contrario.
// IMPORTANTE: Ver datasheet para el dispositivo y nº de pines para poner configuración compatible)
#define DCDC_1200mV_EXTERNO     true



// NO MODIFICAR LAS DEFINICIONES O DECLARACIONES TRAS ESTA LÍNEA DEL FICHERO
#if (TMS320F28004X==true)
#define F280049     0x01FF0500
#define F280048     0x01FE0500
#define F280045     0x01FB0500
#define F280041     0x01F70500
#define F280040     0x01F60500
#endif




#endif /* INCLUDES_SYSTEM_H_ */
