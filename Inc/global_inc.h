#ifndef _GLOBAL_INC_H
#define _GLOBAL_INC_H

#define LAUNCHER_17MM 0
#define LAUNCHER_42MM 1

#define RMUL 0
#define RMUC 1

#define UBC_SUPERCAP 1
#define SJTU_SUPERCAP 2 //not in use for 2025 RMU
#define MACRM_SUPERCAP 3
/********************* Only Modify this area (start) *********************/
#define LAUNCHER_TYPE LAUNCHER_42MM

#define SUPERCAP_TYPE MACRM_SUPERCAP
#define COMPETITION_TYPE RMUL

/********************* Only Modify this area (end) *********************/


/* Use the toolchain's fixed-width integer types. Defining them by hand here
   conflicts with <stdint.h> under GCC (arm-none-eabi). */
#include <stdint.h>

typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

#endif /* _GLOBAL_INC_H */
