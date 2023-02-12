*
*	GotoMain.asm
*
*	Este fichero en ensamblador configura el salto tras finalizar el proceso de boot de la ROM tras un reset a
*	la función de inicialización de variable c_int00 tradicional del lenguaje de prograamacion C
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




