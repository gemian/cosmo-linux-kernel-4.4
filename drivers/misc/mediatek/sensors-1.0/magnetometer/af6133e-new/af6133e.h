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

#define AF6133E_SET_OFFSET
#define AF6133E_TEMP_COMP

#define AF6133E_BUFSIZE    6

#define AF6133E_PID    0x68

/* Register Map */
#define REG_PCODE       0x00
#define REG_DATA        0x03
#define REG_MEASURE     0x0A
#define REG_RANGE       0x0B
#define REG_I2C_CHECK   0x10
#define REG_SW_RESET    0x11
#define REG_AVG         0x13
#define REG_SR_MODE     0x14
#define REG_AVG_2ND     0x15
#define REG_OSR         0x16
#define REG_LLPF        0x17
#define REG_OSC_FREQ    0x19
#define REG_TEMP        0x21
#define REG_ADC_GAIN    0x2B
#define REG_XY_WAITING  0x32
#define REG_Z_WAITING   0x33
#define REG_CHOPPER     0x35
#define REG_TEST2       0x37
#define REG_FLIPPING    0x38

/* factory test */
#define AF6133_OFFSET_MIN  -3000
#define AF6133_OFFSET_MAX   3000
#define AF6133_SENS_MIN      200
#define AF6133_SENS_MAX     1000

/* resolution */

// conversion of magnetic data (for AF6133E) to uT units
#define CONVERT_M        15	
#define CONVERT_M_DIV   100	

/* BIST parameters */
//#define MAG_BIST_LOOP	1

#define BIST_COEFF_X    79 // 0.794
#define BIST_COEFF_Y    82 // 0.817
#define BIST_COEFF_Z    79 // 0.785

/* Temp. Coeff. */
#ifdef AF6133E_TEMP_COMP
#define TEMP_RESOLUTION       908 //0.0908 * 1000
#define TEMP_SENS_COEFF_X     369 //0.00369 * 100000
#define TEMP_SENS_COEFF_Y     403 //0.00403 * 100000
#define TEMP_SENS_COEFF_Z     434 //0.00434 * 100000
#define TEMP_DELTA_THRESHOLD  2 //degree
#define TEMP_MF_NUM           5
#define TEMP_MF_IDX           (uint16_t)(TEMP_MF_NUM / 2)
#endif

/* BIST parameters */
#define BIST_RETRY_NUM   5
#define MAG_BIST_LOOP    5
#define GAIN_X_MIN      60
#define GAIN_X_MAX     600
#define GAIN_Y_MIN      60
#define GAIN_Y_MAX     600
#define GAIN_Z_MIN      60
#define GAIN_Z_MAX     600

#define BIST_COEFF_LIMIT_XY    9 //0.087 //5
#define BIST_COEFF_LIMIT_Z    70 //0.707 //45

#define BIST_GAIN_COEFF_X    (int32_t)(10000 * 666.666 / BIST_COEFF_X)
#define BIST_GAIN_COEFF_Y    (int32_t)(10000 * 666.666 / BIST_COEFF_Y)
#define BIST_GAIN_COEFF_Z    (int32_t)(10000 * 666.666 / BIST_COEFF_Z)

#define AF6133E_OFFSET_MIN    -3000
#define AF6133E_OFFSET_MAX     3000
#define AF6133E_SENS_MIN        200
#define AF6133E_SENS_MAX       1200

//abs
#define vtc_fabs(x)    ((x<0) ? (-x) : (x))

#endif

