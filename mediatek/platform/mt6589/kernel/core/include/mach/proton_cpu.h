#ifndef _PROTON_CPU_H
#define _PROTON_CPU_H

/***************************
* CPU Undervolt
****************************/
#define PROTON_UNDERVOLT 1

#if PROTON_UNDERVOLT
	#define DVFS_V0 (1056)  // mV  	HEX: 0x39
	#define DVFS_V1 (1050)  // mV  	HEX: 0x38
	#define DVFS_V2 ( 950)  // mV	HEX: 0x28
	#define DVFS_V3 ( 925)  // mV	HEX: 0x24
	#define DVFS_V4 ( 800)  // mV	HEX: 0x10
	
	/* HEX values */
	#define DVFS_PKV0 0x39 // 1.05625V
	#define DVFS_PKV1 0x38 // 1.050V
	#define DVFS_PKV2 0x28 // 0.950V
	#define DVFS_PKV3 0x24 // 0.925V
	#define DVFS_PKV4 0x10 // 0.800V
	
	#define DVFS_PKV5 0x30 // 1.000V
	#define DVFS_PKV6 0x28 // 0.950V
	#define DVFS_PKV7 0x10 // 0.800V
	
#else //default
	#define DVFS_V0 (1250) // mV
	#define DVFS_V1 (1200) // mV
	#define DVFS_V2 (1050) // mV
	#define DVFS_V3 ( 950) // mV
	#define DVFS_V4 ( 850) // mV
	
	/* HEX values */
	#define DVFS_PKV0 0x50 // 1.20V
	#define DVFS_PKV1 0x48 // 1.15V
	#define DVFS_PKV2 0x38 // 1.05V
	#define DVFS_PKV3 0x28 // 0.95V
	#define DVFS_PKV4 0x18 // 0.85V

	#define DVFS_PKV5 0x38 // 1.050V
	#define DVFS_PKV6 0x28 // 0.950V
	#define DVFS_PKV7 0x18 // 0.850V
#endif

	#define PK_VMAX 0x5D // 1.28125V
	#define PK_VMIN DVFS_PKV4 // 0.800V
	#define PK_VBOOT DVFS_PKV1 // 1.050V

/***************************
* GPU Overclock and Voltage
****************************/

#endif
