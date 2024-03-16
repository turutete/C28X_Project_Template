/**
 * \page       page_aplicacion Funcionalidad
 *
 *  \author     autor1
 *  \author     autorN
 *  \version    a.b.c.d
 *  \date       dd/mm/aaaa
 *  \bug       [Zigormine id]: Asunto del ticket bug en zigormine
 *  \warning   Alertas de interés para desarrolladores
 *  \copyright ZIGOR R&D AIE
 *
 *  main()
 *  ======
 *
 *  Tras un reset o powerup del procesador se lanza el bootloader del procesador. Una vez finalizado el bootloader,
 *  el procesador salta a la función ***main()***.
 *
 *  Esta función realiza secuencialmente las siguientes tareas:
 *
 *  1. Lanza las tareas preparatorias del **Pre_Kernel()**
 *  2. Crea la pila de tareas
 *  3. Lanza el **Kernel()**
 *
 *  El **Kernel()** se encarga de ejecutar el conjunto de hilos (threads) propios del Kernel, así como los
 *  definidos para a aplicación, de acuerdo a sus prioridades, y mecanismos de sincronización entre tareas.
 *
 *
 */

/*
 * IMPORTANTE
 * ----------
 *
 * Modificar el código de este fichero main() únicamente en los apartados señalados, de acuerdo a las instrucciones
 *
 */

#include <kernel.h>

void main(void)
{

    kernel.Pre_Kernel();

    // Crear en esta sección las tareas estáticas de la aplicación mediante los métodos suministrados por el Kernel



    // Indicar en esta sección los mecanismos de sincronización entre tareas, mediante los métodos suministrados por el Kernel



    kernel.Kernel();
}

