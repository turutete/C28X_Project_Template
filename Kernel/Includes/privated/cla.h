/*
 * cla.h
 *
 *  Created on: 16 abr. 2023
 *      Author: Dr. Carlos Romero
 */

#ifndef KERNEL_INCLUDES_PRIVATED_CLA_H_
#define KERNEL_INCLUDES_PRIVATED_CLA_H_

#include <kernel_types.h>

typedef struct
{
    uint16 cla_enabler;                     // Habilitador de CLA
    uint16 task_id;                         // Identificador de la tarea de CLA en ejecución

    uint16 (* Add_function)(void *);        // Añade una función a la tabla de tareas de CLA
    uint16 (* Remove_function)(uint16);     // Elimina una función de la tabla de tareas de CLA
    void (* Copy_codeforcla)(void);         // Copia el codigo de CLA en RAM y la asigna para CLA como programa
    void (* Copy_dataforcla)(void);         // Copia datos inicializados en RAM y la asigna para CLA como datos

} clase_cla;

#endif /* KERNEL_INCLUDES_PRIVATED_CLA_H_ */
