
// KERNEL reserva RAMLS0-RAMLS4 para memoria de programa de CLA, y RAMLS5-RAMLS7 para memoria de datos
// de CLA.
// La sección CLASCRATCH es la pila de la CLA. En el código siguiente se define el tamaño de la pila, y
// se ajusta automáticamente el tamaño disponible en RAMLS5 para memoria de datos de CLA.
//	El tamaño máximo permitido para el tamaño de la pila de CLA es 0x800, que es el tamaño de RAMLS5.
//	Si el valor definido fuera mayor que este límite, se tome el valor límite.
//
//	KERNEL utiliza la memoria RAM M0 (0x30B=779) como memoria de programa. Es donde se ejecuta el código de actualización
//	de software (UPGRADER).

#define CLA_SCRATCHPAD_SIZE		0x100

#if	CLA_SCRATCHPAD_SIZE>0x800
#undef CLA_SCRATCHPAD_SIZE
#define	CLA_SCRATCHPAD_SIZE		0x800
#endif

#define	RAMLS5_FORCLA_SIZE		(0x800-CLA_SCRATCHPAD_SIZE)
#define	RAMLS5_FORCLA			(0x0000A800+CLA_SCRATCHPAD_SIZE)

MEMORY
{
PAGE 0 :
   RAMLS0          	: origin = 0x008000, length = 0x000800
   RAMLS1          	: origin = 0x008800, length = 0x000800
   RAMLS2      		: origin = 0x009000, length = 0x000800
   RAMLS3      		: origin = 0x009800, length = 0x000800
   RAMLS4      		: origin = 0x00A000, length = 0x000800

   RAMGS0      		: origin = 0x00C000, length = 0x002000
   RAMGS1      		: origin = 0x00E000, length = 0x002000

   RESET           	: origin = 0x3FFFC0, length = 0x000002

#ifdef __TI_COMPILER_VERSION__
   #if __TI_COMPILER_VERSION__ >= 20012000
GROUP {      /* GROUP memory ranges for crc/checksum of entire flash */
   #endif
#endif
  /* BEGIN is used for the "boot to Flash" bootloader mode   */

   BEGIN           	: origin = 0x080000, length = 0x000002

   /* Flash sectors */
   FLASH_MEMORY		:	origin = 0x080002, length = 0x001FFEE	/* on-chip Flash */
   /* origin = 0x09FFF0, length = 0x000010  Reserved as per the errata advisory */

#ifdef __TI_COMPILER_VERSION__
  #if __TI_COMPILER_VERSION__ >= 20012000
}  crc(_ccs_flash_checksum, algorithm=C28_CHECKSUM_16)
  #endif
#endif

PAGE 1 :

   BOOT_RSVD   	: origin = 0x000002, length = 0x0000F3     /* Part of M0, BOOT rom will use this for stack */
   RAMM0        : origin = 0x0000F5, length = 0x00030B
   RAMM1       	: origin = 0x000400, length = 0x000400     /* on-chip RAM block M1 */

   // BLOQUES DE MEMORIA UTILIZADAS POR LA CPU
   ADCARESULTS	: origin = 0x000B00, length = 0x000020
   ADCBRESULTS	: origin = 0x000B20, length = 0x000020
   ADCCRESULTS	: origin = 0x000B40, length = 0x000020
   CPU0TIMERS	: origin = 0x000C00, length = 0x000008
   CPU1TIMERS	: origin = 0x000C08, length = 0x000008
   CPU2TIMERS	: origin = 0x000C10, length = 0x000008
   PIE			: origin = 0x000CE0, length = 0x000020
   PIEVECTABLE	: origin = 0x000D00, length = 0x000200
   DMAREGS		: origin = 0x001000, length = 0x000200
   CLAREGS		: origin = 0x001400, length = 0x000080
   CLATOCPU		: origin = 0x001480, length = 0x000080
   CPUTOCLA		: origin = 0x001500, length = 0x000080
   CLB1CNF		: origin = 0x003000, length = 0x000100
   CLB1CTRL		: origin = 0x003100, length = 0x000100
   CLB1DATA		: origin = 0x003200, length = 0x000200
   CLB2CNF		: origin = 0x003400, length = 0x000100
   CLB2CTRL		: origin = 0x003500, length = 0x000100
   CLB2DATA		: origin = 0x003600, length = 0x000200
   CLB3CNF		: origin = 0x003800, length = 0x000100
   CLB3CTRL		: origin = 0x003900, length = 0x000100
   CLB3DATA		: origin = 0x003A00, length = 0x000200
   CLB4CNF		: origin = 0x003C00, length = 0x000100
   CLB4CTRL		: origin = 0x003D00, length = 0x000100
   CLB4DATA		: origin = 0x003E00, length = 0x000200
   EPWM1		: origin = 0x004000, length = 0x000100
   EPWM2		: origin = 0x004100, length = 0x000100
   EPWM3		: origin = 0x004200, length = 0x000100
   EPWM4		: origin = 0x004300, length = 0x000100
   EPWM5		: origin = 0x004400, length = 0x000100
   EPWM6		: origin = 0x004500, length = 0x000100
   EPWM7		: origin = 0x004600, length = 0x000100
   EPWM8		: origin = 0x004700, length = 0x000100
   EQEP1		: origin = 0x005100, length = 0x000040
   EQEP2		: origin = 0x005140, length = 0x000040
   ECAP1		: origin = 0x005200, length = 0x000020
   ECAP2		: origin = 0x005240, length = 0x000020
   ECAP3		: origin = 0x005280, length = 0x000020
   ECAP4		: origin = 0x0052C0, length = 0x000020
   ECAP5		: origin = 0x005300, length = 0x000020
   ECAP6		: origin = 0x005340, length = 0x000020
   HRCAP6		: origin = 0x005360, length = 0x000020
   ECAP7		: origin = 0x005380, length = 0x000020
   HRCAP7		: origin = 0x0053A0, length = 0x000020
   PGA1			: origin = 0x005B00, length = 0x000010
   PGA2			: origin = 0x005B10, length = 0x000010
   PGA3			: origin = 0x005B20, length = 0x000010
   PGA4			: origin = 0x005B30, length = 0x000010
   PGA5			: origin = 0x005B40, length = 0x000010
   PGA6			: origin = 0x005B50, length = 0x000010
   PGA7			: origin = 0x005B60, length = 0x000010
   DACA			: origin = 0x005C00, length = 0x000010
   DACB			: origin = 0x005C10, length = 0x000010
   CMPSS1		: origin = 0x005C80, length = 0x000020
   CMPSS2		: origin = 0x005CA0, length = 0x000020
   CMPSS3		: origin = 0x005CC0, length = 0x000020
   CMPSS4		: origin = 0x005CE0, length = 0x000020
   CMPSS5		: origin = 0x005D00, length = 0x000020
   CMPSS6		: origin = 0x005D20, length = 0x000020
   CMPSS7		: origin = 0x005D40, length = 0x000020
   SDFM1		: origin = 0x005E00, length = 0x000080
   SPIA			: origin = 0x006100, length = 0x000010
   SPIB			: origin = 0x006110, length = 0x000010
   CLAPROMCRC32	: origin = 0x0061C0, length = 0x000020
   PMBUSA		: origin = 0x006400, length = 0x000020
   FSITX		: origin = 0x006600, length = 0x000080
   FSIRX		: origin = 0x006680, length = 0x000080
   LINA			: origin = 0x006A00, length = 0x000100
   WATCHDOG		: origin = 0x007000, length = 0x000040
   NMIREGS		: origin = 0x007060, length = 0x000010
   XINTREGS		: origin = 0x007070, length = 0x000010
   SCIA			: origin = 0x007200, length = 0x000010
   SCIB			: origin = 0x007210, length = 0x000010
   I2CA			: origin = 0x007300, length = 0x000040
   ADCAREGS		: origin = 0x007400, length = 0x000080
   ADCBREGS		: origin = 0x007480, length = 0x000080
   ADCCREGS		: origin = 0x007500, length = 0x000080
   INPUTXBAR	: origin = 0x007900, length = 0x000020
   XBARREGS		: origin = 0x007920, length = 0x000020
   SYNCSOC		: origin = 0x007940, length = 0x000010
   DMACLA		: origin = 0x007980, length = 0x000040
   EPWMXBAR		: origin = 0x007A00, length = 0x000040
   CLBXBAR		: origin = 0x007A40, length = 0x000040
   OUTPUTXBAR	: origin = 0x007A80, length = 0x000040
   GPIOCTRL		: origin = 0x007C00, length = 0x000300
   GPIODATA		: origin = 0x007F00, length = 0x000100
   CANA			: origin = 0x048000, length = 0x000800
   CANB			: origin = 0x04A000, length = 0x000800
   DEVCONF		: origin = 0x05D000, length = 0x000180
   CLKCNF		: origin = 0x05D200, length = 0x000100
   CPUSYS		: origin = 0x05D300, length = 0x000100
   PERIPHAC		: origin = 0x05D500, length = 0x000200
   ANALOG		: origin = 0x05D700, length = 0x000100
   ROMPREFETCH	: origin = 0x05E608, length = 0x000002
   DCCREGS		: origin = 0x05E700, length = 0x000040
   ERADGLOBAL	: origin = 0x05E800, length = 0x000012
   ERADHWBP1	: origin = 0x05E900, length = 0x000008
   ERADHWBP2	: origin = 0x05E908, length = 0x000008
   ERADHWBP3	: origin = 0x05E910, length = 0x000008
   ERADHWBP4	: origin = 0x05E918, length = 0x000008
   ERADHWBP5	: origin = 0x05E920, length = 0x000008
   ERADHWBP6	: origin = 0x05E928, length = 0x000008
   ERADHWBP7	: origin = 0x05E930, length = 0x000008
   ERADHWBP8	: origin = 0x05E938, length = 0x000008
   ERADCOUNT1	: origin = 0x05E980, length = 0x000010
   ERADCOUNT2	: origin = 0x05E990, length = 0x000010
   ERADCOUNT3	: origin = 0x05E9A0, length = 0x000010
   ERADCOUNT4	: origin = 0x05E9B0, length = 0x000010
   DCSMB0Z1		: origin = 0x05F000, length = 0x000030
   DCSMB0Z2		: origin = 0x05F040, length = 0x000030
   DCSMCOM		: origin = 0x05F070, length = 0x000018
   DCSMB1Z1		: origin = 0x05F100, length = 0x000030
   DCSMB1Z2		: origin = 0x05F140, length = 0x000030
   MEMCNF		: origin = 0x05F400, length = 0x000080
   ACCESS		: origin = 0x05F4C0, length = 0x000040
   MEMERROR		: origin = 0x05F500, length = 0x000040
   ROMWAIT		: origin = 0x05F540, length = 0x000002
   FLASHCTRL	: origin = 0x05F800, length = 0x000300
   FLASHECC		: origin = 0x05FB00, length = 0x000040
   UIDREGS		: origin = 0x0703C0, length = 0x000010
   //---------------------------------------------------

   CLASCRATCH	: origin = 0x00A800, length = CLA_SCRATCHPAD_SIZE
   RAMLS5      	: origin = RAMLS5_FORCLA, length = RAMLS5_FORCLA_SIZE
   RAMLS6      	: origin = 0x00B000, length = 0x000800
   RAMLS7      	: origin = 0x00B800, length = 0x000800

   RAMGS2      	: origin = 0x010000, length = 0x002000
   RAMGS3      	: origin = 0x012000, length = 0x002000
}


SECTIONS
{
	// SECCIONES UTILIZADAS POR TI
   .codestart       : > BEGIN,     			PAGE = 0, ALIGN(4)
   .text            : >	FLASH_MEMORY,		PAGE = 0, ALIGN(4)
   .cinit           : > FLASH_MEMORY,		PAGE = 0, ALIGN(4)
   .pinit           : > FLASH_MEMORY,		PAGE = 0, ALIGN(4)
   .switch          : > FLASH_MEMORY,		PAGE = 0, ALIGN(4)
   .reset           : > RESET,     			PAGE = 0, TYPE = DSECT /* not used, */

   .stack           : > RAMM0,     			PAGE = 1
   .ebss            : > RAMGS2 | RAMGS3, 	PAGE = 1
   .esysmem         : > RAMGS2,    			PAGE = 1
   .econst          : > FLASH_MEMORY,		PAGE = 0, ALIGN(4)

	// SECCIONES UTILIZADAS POR CLA
   .bss_cla			: > RAMLS5 | RAMLS6 | RAMLS7,	PAGE = 1
   .const_cla		: > RAMLS5,				PAGE = 1
   CLAscratch		: > RAMLS5,				PAGE = 1
   CLA1_Prog		: {}	LOAD = FLASH_MEMORY,
                        	RUN = RAMLS0 | RAMLS1 | RAMLS2 | RAMLS3 | RAMLS4,
                        	LOAD_START(_ClainramLoadStart),
                        	LOAD_SIZE(_ClainramLoadSize),
                        	LOAD_END(_ClainramLoadEnd),
                        	RUN_START(_ClainramRunStart),
                        	RUN_SIZE(_ClainramRunSize),
                        	RUN_END(_ClainramRunEnd),
                        	PAGE = 0, ALIGN(4)

	// SECCIONES UTILIZADAS POR PROJECT_TEMPLATE
	.kernelvar		: > RAMM1,				PAGE = 1

	UNION 					RUN = RAMGS0 | RAMGS1
	{
	.cpuinram		:		LOAD = FLASH_MEMORY,	PAGE = 0,	ALIGN(4),
                        	LOAD_START(_CpuinramLoadStart),
                        	LOAD_SIZE(_CpuinramLoadSize),
                        	LOAD_END(_CpuinramLoadEnd),
                        	RUN_START(_CpuinramRunStart),
                        	RUN_SIZE(_CpuinramRunSize),
                        	RUN_END(_CpuinramRunEnd)

     .flashinit		:		LOAD = FLASH_MEMORY,	PAGE = 0, 	ALIGN(4),
                       		LOAD_START(_FlashinitLoadStart),
                        	LOAD_SIZE(_FlashinitLoadSize),
                        	LOAD_END(_FlashinitLoadEnd),
                        	RUN_START(_FlashinitRunStart),
                        	RUN_SIZE(_FlashinitRunSize),
                        	RUN_END(_FlashinitRunEnd)
 	}

   /* crc/checksum section configured as COPY section to avoid including in executable */
   .TI.memcrc       	: type = COPY

   // SECCIONES UTILIZADAS POR TI
//----------------------------------------
	AccessProtectionRegsFile	: > ACCESS,			PAGE = 1

	AdcaRegsFile				: > ADCAREGS,		PAGE = 1
	AdcbRegsFile				: > ADCBREGS,		PAGE = 1
	AdccRegsFile				: > ADCCREGS,		PAGE = 1

	AdcaResultRegsFile			: > ADCARESULTS,	PAGE = 1
	AdcbResultRegsFile			: > ADCBRESULTS,	PAGE = 1
	AdccResultRegsFile			: > ADCCRESULTS,	PAGE = 1

	AnalogSubsysRegsFile		: > ANALOG, 		PAGE = 1

	CanaRegsFile				: > CANA,			PAGE = 1
	CanbRegsFile				: > CANB,			PAGE = 1

	ClaPromCrc0RegsFile			: > CLAPROMCRC32,	PAGE = 1
	Cla1RegsFile				: > CLAREGS,		PAGE = 1

	Clb1DataExchRegsFile		: > CLB1DATA,		PAGE = 1
	Clb2DataExchRegsFile		: > CLB2DATA,		PAGE = 1
	Clb3DataExchRegsFile		: > CLB3DATA,		PAGE = 1
	Clb4DataExchRegsFile		: > CLB4DATA,		PAGE = 1
	Clb1LogicCfgRegsFile		: > CLB1CNF,		PAGE = 1
	Clb2LogicCfgRegsFile		: > CLB2CNF,		PAGE = 1
	Clb3LogicCfgRegsFile		: > CLB3CNF,		PAGE = 1
	Clb4LogicCfgRegsFile		: > CLB4CNF,		PAGE = 1
	Clb1LogicCtrlRegsFile		: > CLB1CTRL,		PAGE = 1
	Clb2LogicCtrlRegsFile		: > CLB2CTRL,		PAGE = 1
	Clb3LogicCtrlRegsFile		: > CLB3CTRL,		PAGE = 1
	Clb4LogicCtrlRegsFile		: > CLB4CTRL,		PAGE = 1

	CLBXbarRegsFile				: >	CLBXBAR,		PAGE = 1

	ClkCfgRegsFile				: > CLKCNF,			PAGE = 1

	Cmpss1RegsFile				: > CMPSS1,			PAGE = 1
	Cmpss2RegsFile				: > CMPSS2,			PAGE = 1
	Cmpss3RegsFile				: > CMPSS3,			PAGE = 1
	Cmpss4RegsFile				: > CMPSS4,			PAGE = 1
	Cmpss5RegsFile				: > CMPSS5,			PAGE = 1
	Cmpss6RegsFile				: > CMPSS6,			PAGE = 1
	Cmpss7RegsFile				: > CMPSS7,			PAGE = 1

	CpuTimer0RegsFile			: > CPU0TIMERS,		PAGE = 1
	CpuTimer1RegsFile			: > CPU1TIMERS,		PAGE = 1
	CpuTimer2RegsFile			: > CPU2TIMERS,		PAGE = 1

	CpuSysRegsFile				: > CPUSYS,			PAGE = 1

	DacaRegsFile				: > DACA,			PAGE = 1
	DacbRegsFile				: > DACB,			PAGE = 1

	Dcc0RegsFile				: > DCCREGS,		PAGE = 1

	DcsmBank0Z1RegsFile			: > DCSMB0Z1,		PAGE = 1
	DcsmBank0Z2RegsFile			: > DCSMB0Z2,		PAGE = 1
	DcsmBank1Z1RegsFile			: > DCSMB1Z1,		PAGE = 1
	DcsmBank1Z2RegsFile			: > DCSMB1Z2,		PAGE = 1
	DcsmCommonRegsFile			: > DCSMCOM,		PAGE = 1
	DevCfgRegsFile				: > DEVCONF,		PAGE = 1

	DmaClaSrcSelRegsFile		: > DMACLA,			PAGE = 1
	DmaRegsFile					: > DMAREGS,		PAGE = 1

	ECap1RegsFile				: >	ECAP1,			PAGE = 1
	ECap2RegsFile				: >	ECAP2,			PAGE = 1
	ECap3RegsFile				: >	ECAP3,			PAGE = 1
	ECap4RegsFile				: >	ECAP4,			PAGE = 1
	ECap5RegsFile				: >	ECAP5,			PAGE = 1
	ECap6RegsFile				: >	ECAP6,			PAGE = 1
	ECap7RegsFile				: >	ECAP7,			PAGE = 1


	EPwm1RegsFile				: > EPWM1,			PAGE = 1
	EPwm2RegsFile				: > EPWM2,			PAGE = 1
	EPwm3RegsFile				: > EPWM3,			PAGE = 1
	EPwm4RegsFile				: > EPWM4,			PAGE = 1
	EPwm5RegsFile				: > EPWM5,			PAGE = 1
	EPwm6RegsFile				: > EPWM6,			PAGE = 1
	EPwm7RegsFile				: > EPWM7,			PAGE = 1
	EPwm8RegsFile				: > EPWM8,			PAGE = 1

	EPwmXbarRegsFile			: > EPWMXBAR,		PAGE = 1

	EQep1RegsFile				: > EQEP1,			PAGE = 1
	EQep2RegsFile				: > EQEP2,			PAGE = 1


	EradCounter1RegsFile		: > ERADCOUNT1,		PAGE = 1
	EradCounter2RegsFile		: > ERADCOUNT2,		PAGE = 1
	EradCounter3RegsFile		: > ERADCOUNT3,		PAGE = 1
	EradCounter4RegsFile		: > ERADCOUNT4,		PAGE = 1

	EradGlobalRegsFile			: > ERADGLOBAL,		PAGE = 1

	EradHWBP1RegsFile			: > ERADHWBP1,		PAGE = 1
	EradHWBP2RegsFile			: > ERADHWBP2,		PAGE = 1
	EradHWBP3RegsFile			: > ERADHWBP3,		PAGE = 1
	EradHWBP4RegsFile			: > ERADHWBP4,		PAGE = 1
	EradHWBP5RegsFile			: > ERADHWBP5,		PAGE = 1
	EradHWBP6RegsFile			: > ERADHWBP6,		PAGE = 1
	EradHWBP7RegsFile			: > ERADHWBP7,		PAGE = 1
	EradHWBP8RegsFile			: > ERADHWBP8,		PAGE = 1

	Flash0CtrlRegsFile			: > FLASHCTRL,		PAGE = 1
	Flash0EccRegsFile			: > FLASHECC,		PAGE = 1

	FsiRxaRegsFile				: > FSIRX,			PAGE = 1
	FsiTxaRegsFile				: > FSITX,			PAGE = 1
	GpioCtrlRegsFile			: > GPIOCTRL,		PAGE = 1
	GpioDataRegsFile			: > GPIODATA,		PAGE = 1

	HRCap6RegsFile				: > HRCAP6,			PAGE = 1
	HRCap7RegsFile				: > HRCAP7,			PAGE = 1

	I2caRegsFile				: > I2CA,			PAGE = 1

	InputXbarRegsFile			: > INPUTXBAR,		PAGE = 1

	LinaRegsFile				: > LINA,			PAGE = 1

	MemoryErrorRegsFile			: > MEMERROR,		PAGE = 1
	MemCfgRegsFile				: > MEMCNF,			PAGE = 1

	NmiIntruptRegsFile			: > NMIREGS,		PAGE = 1

	OutputXbarRegsFile			: > OUTPUTXBAR,		PAGE = 1

	SysPeriphAcRegsFile			: > PERIPHAC,		PAGE = 1

	Pga1RegsFile				: > PGA1,			PAGE = 1
	Pga2RegsFile				: > PGA2,			PAGE = 1
	Pga3RegsFile				: > PGA3,			PAGE = 1
	Pga4RegsFile				: > PGA4,			PAGE = 1
	Pga5RegsFile				: > PGA5,			PAGE = 1
	Pga6RegsFile				: > PGA6,			PAGE = 1
	Pga7RegsFile				: > PGA7,			PAGE = 1

	PieCtrlRegsFile				: > PIE,			PAGE = 1
	PieVectTableFile			: > PIEVECTABLE,	PAGE = 1

	PmbusaRegsFile				: > PMBUSA,			PAGE = 1

	SciaRegsFile				: > SCIA,			PAGE = 1
	ScibRegsFile				: > SCIB,			PAGE = 1

	Sdfm1RegsFile				: > SDFM1,			PAGE = 1

	SpiaRegsFile				: > SPIA,			PAGE = 1
	SpibRegsFile				: > SPIB,			PAGE = 1

	SyncSocRegsFile				: > SYNCSOC,		PAGE = 1

	WdRegsFile					: > WATCHDOG,		PAGE = 1

	XbarRegsFile				: > XBARREGS,		PAGE = 1

	XintRegsFile				: > XINTREGS,		PAGE = 1

}
