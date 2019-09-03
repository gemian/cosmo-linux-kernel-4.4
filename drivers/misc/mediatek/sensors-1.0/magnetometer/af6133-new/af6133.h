/* include/linux/af6133.h - AF6133 compass driver
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * Definitions for AF6133 compass chip.
 */
#ifndef AF6133_H
#define AF6133_H

#include <linux/ioctl.h>

//#define SUPPORT_SOFTGYRO_FUNCTION

#define AF6133_BUFSIZE	6

#define AF6133J_PID	0x64

/* register definition */
#define REG_PCODE       0x00
#define REG_DATA	      0x03
#define REG_MEASURE	    0x0A
#define REG_RANGE	      0x0B
#define REG_I2C_CHECK   0x10
#define REG_SW_RESET	  0x11
#define REG_AVG		      0x13
#define REG_XY_SR	      0x14
#define REG_OTP_1D      0x1D
#define REG_TEMP        0x21
#define REG_DATA2	      0x23
#define REG_OSR		      0x2D
#define REG_SECURITY	  0x30
#define REG_WAITING	    0x32
#define REG_Z_SR	      0x33
#define REG_TEST0	      0x35
#define REG_TEST2	      0x37
#define MUX_SEL         0x34

/* factory test */
#define AF6133_OFFSET_MIN -1500
#define AF6133_OFFSET_MAX	 1500
#define AF6133_SENS_MIN		  200
#define AF6133_SENS_MAX		 1000

/* resolution */

// conversion of magnetic data (for AF6133) to uT units
#define CONVERT_M	1	
#define CONVERT_M_DIV	5		

// conversion of orientation data to degree units
#define CONVERT_O	1
#define CONVERT_O_DIV	100

#ifdef SUPPORT_SOFTGYRO_FUNCTION
// conversion of gyroscope data to rps units
#define CONVERT_GY	1
#define CONVERT_GY_DIV	1000		

// conversion of rotation vector
#define CONVERT_RV	1
#define CONVERT_RV_DIV	1000

// conversion of linear accleration data to m^2/s units
#define CONVERT_LA	1
#define CONVERT_LA_DIV	1000		

// conversion of gravity data to m^2/s units
#define CONVERT_GV	1
#define CONVERT_GV_DIV	1000

#endif

/* offset parameters */
#define MAG_MIN_OFFSET	447
#define MAG_OFFSET_LOOP	5

/* BIST parameters */
#define MAG_BIST_LOOP	5

#define BIST_COEFF_X	75 // 0.7476
#define BIST_COEFF_Y	84 // 0.8394
#define BIST_COEFF_Z	49 // 0.4853

#define BIST_BIAS_0	0
#define BIST_BIAS_1	0
#define BIST_BIAS_2	0
#define BIST_BIAS_3	0

#define BIST_GAIN_COEFF_X	(int32_t)(10000 * 500 / BIST_COEFF_X)
#define BIST_GAIN_COEFF_Y	(int32_t)(10000 * 500 / BIST_COEFF_Y)
#define BIST_GAIN_COEFF_Z	(int32_t)(10000 * 500 / BIST_COEFF_Z)

#define BIST_COMP_COEFF_0	(int16_t)(100 * BIST_COEFF_Y / BIST_COEFF_X)
#define BIST_COMP_COEFF_1	(int16_t)(100 * BIST_COEFF_X / BIST_COEFF_Y)
#define BIST_COMP_COEFF_2	(int16_t)(100 * BIST_COEFF_X / BIST_COEFF_Z)
#define BIST_COMP_COEFF_3	(int16_t)(100 * BIST_COEFF_Y / BIST_COEFF_Z)

#define BIST_COMP_BIAS_0	-1 //(int16_t)(sin(BIST_BIAS_0 * VTC_DEG2RAD)*100)
#define BIST_COMP_BIAS_1	-1 //(int16_t)(sin(BIST_BIAS_1 * VTC_DEG2RAD)*100)
#define BIST_COMP_BIAS_2	11 //(int16_t)(sin(BIST_BIAS_2 * VTC_DEG2RAD)*100)
#define BIST_COMP_BIAS_3	 1 //(int16_t)(sin(BIST_BIAS_3 * VTC_DEG2RAD)*100)

#endif

