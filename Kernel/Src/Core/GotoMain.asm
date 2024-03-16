*
*	GotoMain.asm
*
*	Este fichero en ensamblador configura el salto tras finalizar el proceso de boot de la ROM tras un reset a
*	la función de inicialización de variable c_int00 tradicional del lenguaje de programacion C.
*
*	Utiliza una sección llamada .codestart que debe estar definida en el fichero cmd de linkado que, a su vez, se
*	situa en la zona de memoria llamada BEGIN, que en los C28 es la direccion 0x00080000.
*
*	Historial de versiones
*
*	Versión		Autor							Fecha		Comentario
*	------------------------------------------------------------------------------------------------------------
*	1.0.0.0		Dr. Carlos Romero				12/02/2023	Primera versión
*
*

		.ref 	_c_int00

		.sect ".codestart"

_goto_cinit:
		LB		_c_int00




