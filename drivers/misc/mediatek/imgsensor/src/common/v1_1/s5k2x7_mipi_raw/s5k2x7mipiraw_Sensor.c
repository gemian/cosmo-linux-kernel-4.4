/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 s5k2x7mipiraw_sensor.c
 *
 * Project:
 * --------
 *	 ALPS MT6763
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *	PengtaoFan
 *  1511041520 modify code style from 2x7@mt6755 and move to mt6797
 *  1512291720 update setting from samsung
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k2x7mipiraw_Sensor.h"

/*===FEATURE SWITH===*/
 /* #define FPTPDAFSUPPORT    */
 /* #define FANPENGTAO    */

 /* #define NONCONTINUEMODE */
/*===FEATURE SWITH===*/

/****************************Modify Following Strings for Debug****************************/
#define PFX "S5K2X7"
#define LOG_INF_NEW(format, args...)    printk(PFX "[%s] " format, __func__, ##args)
#define LOG_INF LOG_INF_NEW
#define LOG_1 LOG_INF("S5K2X7,MIPI 4LANE\n")
#define SENSORDB LOG_INF
/****************************   Modify end    *******************************************/

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static imgsensor_info_struct imgsensor_info = {
	.sensor_id = S5K2X7_SENSOR_ID,

	.checksum_value = 0xcce1e01b,	/*checksum value for Camera Auto Test */

	.pre = {
		.pclk = 480000000,				/*record different mode's pclk */
		.linelength  = 6144,				/*record different mode's linelength */
		.framelength = 2600,			/*record different mode's framelength */
		.startx = 0,					          /* record different mode's startx of grabwindow */
		.starty = 0,					        /* record different mode's starty of grabwindow */
		.grabwindow_width  = 2832,		/*record different mode's width of grabwindow */
		.grabwindow_height = 2128,		/*record different mode's height of grabwindow */
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 480000000,				/* record different mode's pclk */
		.linelength  = 6144,				/* record different mode's linelength */
		.framelength = 2600,			/* record different mode's framelength */
		.startx = 0,					        /* record different mode's startx of grabwindow */
		.starty = 0,					        /* record different mode's starty of grabwindow */
		.grabwindow_width  = 2832,		/* record different mode's width of grabwindow */
		.grabwindow_height = 2128,		/* record different mode's height of grabwindow */
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
	},
	.cap1 = {							          /* capture for PIP 15ps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate */
		.pclk = 480000000,				/* record different mode's pclk */
		.linelength  = 6144,				/* record different mode's linelength */
		.framelength = 2600,			/* record different mode's framelength */
		.startx = 0,					        /* record different mode's startx of grabwindow */
		.starty = 0,					        /* record different mode's starty of grabwindow */
		.grabwindow_width  = 2832,		/* record different mode's width of grabwindow */
		.grabwindow_height = 2128,		/* record different mode's height of grabwindow */
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 150,
	},
	.normal_video = {
		.pclk = 480000000,				/* record different mode's pclk */
		.linelength  = 6144,				/* record different mode's linelength */
		.framelength = 2600,			/* record different mode's framelength */
		.startx = 0,					          /* record different mode's startx of grabwindow */
		.starty = 0,					        /* record different mode's starty of grabwindow */
		.grabwindow_width  = 2832,		/* record different mode's width of grabwindow */
		.grabwindow_height = 2128,		/* record different mode's height of grabwindow */
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 960000000,				/*record different mode's pclk */
		.linelength  = 3520,				/*record different mode's linelength */
		.framelength = 2272,			/*record different mode's framelength */
		.startx = 0,					          /* record different mode's startx of grabwindow */
		.starty = 0,					        /* record different mode's starty of grabwindow */
		.grabwindow_width  = 2832,		/*record different mode's width of grabwindow */
		.grabwindow_height = 2128,		/*record different mode's height of grabwindow */
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 1200,
	},
	.slim_video = {
		.pclk = 864000000,				/* record different mode's pclk */
		.linelength  = 9088,				/* record different mode's linelength */
		.framelength = 3168,			/* record different mode's framelength */
		.startx = 0,					          /* record different mode's startx of grabwindow */
		.starty = 0,					        /* record different mode's starty of grabwindow */
		.grabwindow_width  = 2832,		/* record different mode's width of grabwindow */
		.grabwindow_height = 2128,		/* record different mode's height of grabwindow */
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
	},


	.margin = 5,			/* sensor framelength & shutter margin */
	.min_shutter = 4,		/* min shutter */
	.max_frame_length = 0xFFFF-5,/* REG0x0202 <=REG0x0340-5//max framelength by sensor register's limitation */
	.ae_shut_delay_frame = 0,	/* shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2 */
	.ae_sensor_gain_delay_frame = 0,/* sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2 */
	.ae_ispGain_delay_frame = 2,/* isp gain delay frame for AE cycle */
	.ihdr_support = 0,	  /* 1, support; 0,not support */
	.ihdr_le_firstline = 0,  /* 1,le first ; 0, se first */
	.sensor_mode_num = 5,	  /* support sensor mode num ,don't support Slow motion */

	.cap_delay_frame = 3,		/* enter capture delay frame num */
	.pre_delay_frame = 3,		/* enter preview delay frame num */
	.video_delay_frame = 3,		/* enter video delay frame num */
	.hs_video_delay_frame = 3,	/* enter high speed video  delay frame num */
	.slim_video_delay_frame = 3,/* enter slim video delay frame num */

	.isp_driving_current = ISP_DRIVING_8MA, /* mclk driving current */
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,/* sensor_interface_type */
    .mipi_sensor_type = MIPI_OPHY_NCSI2, /*0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
    .mipi_settle_delay_mode = 1,/* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,//SENSOR_OUTPUT_FORMAT_RAW_4CELL_Gr,//SENSOR_OUTPUT_FORMAT_RAW_Gr,/* sensor output first pixel color */
	.mclk = 24,/*mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	.mipi_lane_num = SENSOR_MIPI_4_LANE,/* mipi lane num */
	.i2c_addr_table = {0x20, 0x5a, 0xff},/* record sensor support all write id addr, only supprt 4must end with 0xff */
    .i2c_speed = 300, /* i2c read/write speed */
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				/* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT, /* IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video */
	.shutter = 0x200,					/* current shutter */
	.gain = 0x200,						/* current gain */
	.dummy_pixel = 0,					/* current dummypixel */
	.dummy_line = 0,					/* current dummyline */
	.current_fps = 0,  /* full size current fps : 24fps for PIP, 30fps for Normal or ZSD */
	.autoflicker_en = KAL_FALSE,  /* auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker */
	.test_pattern = KAL_FALSE,		/* test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output */
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,/* current scenario id */
	.ihdr_en = KAL_FALSE, /* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x20,//0x5a, /* record current sensor's i2c write id */
};

 
/* Sensor output window information*/
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] = {

 { 5664, 4256,	  0,	0, 5664, 4256, 2832, 2128,   0,	0, 2832, 2128,	 0, 0, 2832, 2128}, /* Preview */
 { 5664, 4256,	  0,	0, 5664, 4256, 2832, 2128,   0,	0, 2832, 2128,	 0, 0, 2832, 2128}, /* capture */
 { 5664, 4256,	  0,	0, 5664, 4256, 2832, 2128,   0,	0, 2832, 2128,	 0, 0, 2832, 2128}, /* video */
 { 5664, 4256,	  0,	0, 5664, 4256, 2832, 2128,   0,	0, 2832, 2128,	 0, 0, 2832, 2128}, /* hight speed video */
 { 5664, 4256,	  0,	0, 5664, 4256, 2832, 2128,   0,	0, 2832, 2128,	 0, 0, 2832, 2128},/* slim video */
};

 static SET_PD_BLOCK_INFO_T imgsensor_pd_info =

{

};

static kal_uint16 read_cmos_sensor_byte(kal_uint16 addr)
{
    kal_uint16 get_byte = 0;
    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };


    iReadRegI2C(pu_send_cmd , 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);
    return get_byte;
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte = 0;
    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };


    iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
    return get_byte;
}

static void write_cmos_sensor_byte(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};


    iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};


    iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
	write_cmos_sensor(0x0342, imgsensor.line_length & 0xFFFF);
}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min framelength should enable(%d)\n", framerate, min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */




/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
		/* Extend frame length */
		write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
	}

	/*  Update Shutter */
	write_cmos_sensor(0X0202, shutter & 0xFFFF);
	LOG_INF("Exit! shutter = %d, framelength = %d\n", shutter, imgsensor.frame_length);

}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;

    reg_gain = gain/2;

	return (kal_uint16)reg_gain;
}

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	LOG_INF("set_gain %d\n", gain);

	if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
		LOG_INF("Error gain setting");
		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 16 * BASEGAIN)
			gain = 16 * BASEGAIN;
	}

    reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor(0x0204, (reg_gain&0xFFFF));
	return gain;
}	/*	set_gain  */



static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/
	spin_lock(&imgsensor_drv_lock);
    imgsensor.mirror = image_mirror;
    spin_unlock(&imgsensor_drv_lock);
	switch (image_mirror) {
		case IMAGE_NORMAL:
			write_cmos_sensor_byte(0x0101, 0x00);
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor_byte(0x0101, 0x01);
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor_byte(0x0101, 0x02);
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor_byte(0x0101, 0x03);
			break;
		default:
			LOG_INF("Error image_mirror setting\n");
	}

}

static void check_output_stream_off(void)
{
	kal_uint16 read_count = 0, read_register0005_value = 0;

	for (read_count = 0; read_count <= 4; read_count++)
	{
		read_register0005_value = read_cmos_sensor(0x0005);

		if (read_register0005_value == 0xff)
			break;
		mdelay(50);

		if (read_count == 4)
			LOG_INF("stream off error\n");
	}
}
/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}	/*	night_mode	*/
static void sensor_init(void)
{
  LOG_INF("E\n");
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0000, 0x0013);
	write_cmos_sensor(0x0000, 0x2187);
	write_cmos_sensor(0x6214, 0x7971);
	write_cmos_sensor(0x6218, 0x7150);
	mdelay(3);
	write_cmos_sensor(0x0A02, 0x7000);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x3DF4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0449);
	write_cmos_sensor(0x6F12, 0x0348);
	write_cmos_sensor(0x6F12, 0x044A);
	write_cmos_sensor(0x6F12, 0x0860);
	write_cmos_sensor(0x6F12, 0x101A);
	write_cmos_sensor(0x6F12, 0x8880);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x5EBA);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x44F4);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x3280);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x6500);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xF05F);
	write_cmos_sensor(0x6F12, 0x9146);
	write_cmos_sensor(0x6F12, 0x0E46);
	write_cmos_sensor(0x6F12, 0x0446);
	write_cmos_sensor(0x6F12, 0xDFF8);
	write_cmos_sensor(0x6F12, 0x08A4);
	write_cmos_sensor(0x6F12, 0x30E0);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xBAFA);
	write_cmos_sensor(0x6F12, 0x8346);
	write_cmos_sensor(0x6F12, 0x4FEA);
	write_cmos_sensor(0x6F12, 0x4835);
	write_cmos_sensor(0x6F12, 0x4046);
	write_cmos_sensor(0x6F12, 0xFE4F);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xB021);
	write_cmos_sensor(0x6F12, 0x05F5);
	write_cmos_sensor(0x6F12, 0x0055);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xB4FA);
	write_cmos_sensor(0x6F12, 0x388D);
	write_cmos_sensor(0x6F12, 0xF98C);
	write_cmos_sensor(0x6F12, 0x411A);
	write_cmos_sensor(0x6F12, 0x4046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xB3FA);
	write_cmos_sensor(0x6F12, 0x788D);
	write_cmos_sensor(0x6F12, 0xB989);
	write_cmos_sensor(0x6F12, 0x411A);
	write_cmos_sensor(0x6F12, 0x5046);
	write_cmos_sensor(0x6F12, 0xAAF8);
	write_cmos_sensor(0x6F12, 0x1A13);
	write_cmos_sensor(0x6F12, 0xB0F8);
	write_cmos_sensor(0x6F12, 0x1A13);
	write_cmos_sensor(0x6F12, 0x0029);
	write_cmos_sensor(0x6F12, 0xFBD1);
	write_cmos_sensor(0x6F12, 0x7021);
	write_cmos_sensor(0x6F12, 0xA0F8);
	write_cmos_sensor(0x6F12, 0x0E13);
	write_cmos_sensor(0x6F12, 0xC4F3);
	write_cmos_sensor(0x6F12, 0x0C01);
	write_cmos_sensor(0x6F12, 0xAE42);
	write_cmos_sensor(0x6F12, 0x00D2);
	write_cmos_sensor(0x6F12, 0x3546);
	write_cmos_sensor(0x6F12, 0x2D1B);
	write_cmos_sensor(0x6F12, 0x2B46);
	write_cmos_sensor(0x6F12, 0x4A46);
	write_cmos_sensor(0x6F12, 0x4046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xA0FA);
	write_cmos_sensor(0x6F12, 0x2C44);
	write_cmos_sensor(0x6F12, 0xA944);
	write_cmos_sensor(0x6F12, 0x5946);
	write_cmos_sensor(0x6F12, 0x4046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x9FFA);
	write_cmos_sensor(0x6F12, 0x600B);
	write_cmos_sensor(0x6F12, 0x5FEA);
	write_cmos_sensor(0x6F12, 0x0008);
	write_cmos_sensor(0x6F12, 0x01D1);
	write_cmos_sensor(0x6F12, 0xB442);
	write_cmos_sensor(0x6F12, 0xC8D3);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF09F);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0x8046);
	write_cmos_sensor(0x6F12, 0xE448);
	write_cmos_sensor(0x6F12, 0x1646);
	write_cmos_sensor(0x6F12, 0x0F46);
	write_cmos_sensor(0x6F12, 0x0068);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x84B2);
	write_cmos_sensor(0x6F12, 0x050C);
	write_cmos_sensor(0x6F12, 0x2146);
	write_cmos_sensor(0x6F12, 0x2846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x8EFA);
	write_cmos_sensor(0x6F12, 0x3246);
	write_cmos_sensor(0x6F12, 0x3946);
	write_cmos_sensor(0x6F12, 0x4046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x8EFA);
	write_cmos_sensor(0x6F12, 0xDD4B);
	write_cmos_sensor(0x6F12, 0x16B9);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x83F8);
	write_cmos_sensor(0x6F12, 0x8B00);
	write_cmos_sensor(0x6F12, 0x33F8);
	write_cmos_sensor(0x6F12, 0xF80F);
	write_cmos_sensor(0x6F12, 0x1989);
	write_cmos_sensor(0x6F12, 0x1A7C);
	write_cmos_sensor(0x6F12, 0x01FB);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x6F12, 0x13F8);
	write_cmos_sensor(0x6F12, 0x471C);
	write_cmos_sensor(0x6F12, 0x421A);
	write_cmos_sensor(0x6F12, 0xD749);
	write_cmos_sensor(0x6F12, 0x8A80);
	write_cmos_sensor(0x6F12, 0x5A88);
	write_cmos_sensor(0x6F12, 0x5E89);
	write_cmos_sensor(0x6F12, 0x9F7C);
	write_cmos_sensor(0x6F12, 0x06FB);
	write_cmos_sensor(0x6F12, 0x0722);
	write_cmos_sensor(0x6F12, 0x13F8);
	write_cmos_sensor(0x6F12, 0x456C);
	write_cmos_sensor(0x6F12, 0x961B);
	write_cmos_sensor(0x6F12, 0xCE80);
	write_cmos_sensor(0x6F12, 0x13F8);
	write_cmos_sensor(0x6F12, 0x3C3C);
	write_cmos_sensor(0x6F12, 0x032B);
	write_cmos_sensor(0x6F12, 0x03D1);
	write_cmos_sensor(0x6F12, 0x401E);
	write_cmos_sensor(0x6F12, 0x8880);
	write_cmos_sensor(0x6F12, 0x521E);
	write_cmos_sensor(0x6F12, 0xCA80);
	write_cmos_sensor(0x6F12, 0x2146);
	write_cmos_sensor(0x6F12, 0x2846);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x61BA);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0x0646);
	write_cmos_sensor(0x6F12, 0xC748);
	write_cmos_sensor(0x6F12, 0x0D46);
	write_cmos_sensor(0x6F12, 0x4268);
	write_cmos_sensor(0x6F12, 0x140C);
	write_cmos_sensor(0x6F12, 0x97B2);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x3946);
	write_cmos_sensor(0x6F12, 0x2046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x54FA);
	write_cmos_sensor(0x6F12, 0xC348);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0xBC10);
	write_cmos_sensor(0x6F12, 0x0329);
	write_cmos_sensor(0x6F12, 0x07D0);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0x0B11);
	write_cmos_sensor(0x6F12, 0x0129);
	write_cmos_sensor(0x6F12, 0x03D8);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0x0901);
	write_cmos_sensor(0x6F12, 0x0128);
	write_cmos_sensor(0x6F12, 0x01D9);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x2946);
	write_cmos_sensor(0x6F12, 0x3046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x4AFA);
	write_cmos_sensor(0x6F12, 0x3946);
	write_cmos_sensor(0x6F12, 0x2046);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x39BA);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0x0546);
	write_cmos_sensor(0x6F12, 0xB348);
	write_cmos_sensor(0x6F12, 0x0C46);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x8168);
	write_cmos_sensor(0x6F12, 0x0E0C);
	write_cmos_sensor(0x6F12, 0x8FB2);
	write_cmos_sensor(0x6F12, 0x3946);
	write_cmos_sensor(0x6F12, 0x3046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x2CFA);
	write_cmos_sensor(0x6F12, 0x2146);
	write_cmos_sensor(0x6F12, 0x2846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x37FA);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0x3946);
	write_cmos_sensor(0x6F12, 0x3046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x23FA);
	write_cmos_sensor(0x6F12, 0xAC4A);
	write_cmos_sensor(0x6F12, 0xAB48);
	write_cmos_sensor(0x6F12, 0x02EB);
	write_cmos_sensor(0x6F12, 0x8505);
	write_cmos_sensor(0x6F12, 0x0CB1);
	write_cmos_sensor(0x6F12, 0xC088);
	write_cmos_sensor(0x6F12, 0x0AE0);
	write_cmos_sensor(0x6F12, 0x4088);
	write_cmos_sensor(0x6F12, 0x08E0);
	write_cmos_sensor(0x6F12, 0x1368);
	write_cmos_sensor(0x6F12, 0x190C);
	write_cmos_sensor(0x6F12, 0x9BB2);
	write_cmos_sensor(0x6F12, 0x44B1);
	write_cmos_sensor(0x6F12, 0x8142);
	write_cmos_sensor(0x6F12, 0x04D9);
	write_cmos_sensor(0x6F12, 0x43EA);
	write_cmos_sensor(0x6F12, 0x0041);
	write_cmos_sensor(0x6F12, 0x02C2);
	write_cmos_sensor(0x6F12, 0xAA42);
	write_cmos_sensor(0x6F12, 0xF4D1);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF081);
	write_cmos_sensor(0x6F12, 0x8142);
	write_cmos_sensor(0x6F12, 0xF6D3);
	write_cmos_sensor(0x6F12, 0xFAE7);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0x0646);
	write_cmos_sensor(0x6F12, 0x9B48);
	write_cmos_sensor(0x6F12, 0x0D46);
	write_cmos_sensor(0x6F12, 0xC268);
	write_cmos_sensor(0x6F12, 0x140C);
	write_cmos_sensor(0x6F12, 0x97B2);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x3946);
	write_cmos_sensor(0x6F12, 0x2046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xFDF9);
	write_cmos_sensor(0x6F12, 0x2946);
	write_cmos_sensor(0x6F12, 0x3046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x0DFA);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0x3946);
	write_cmos_sensor(0x6F12, 0x2046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xF4F9);
	write_cmos_sensor(0x6F12, 0x9648);
	write_cmos_sensor(0x6F12, 0x964A);
	write_cmos_sensor(0x6F12, 0x30F8);
	write_cmos_sensor(0x6F12, 0x1E1F);
	write_cmos_sensor(0x6F12, 0x1180);
	write_cmos_sensor(0x6F12, 0x921C);
	write_cmos_sensor(0x6F12, 0x4188);
	write_cmos_sensor(0x6F12, 0x1180);
	write_cmos_sensor(0x6F12, 0x921C);
	write_cmos_sensor(0x6F12, 0x8188);
	write_cmos_sensor(0x6F12, 0x1180);
	write_cmos_sensor(0x6F12, 0x921C);
	write_cmos_sensor(0x6F12, 0xC188);
	write_cmos_sensor(0x6F12, 0x1180);
	write_cmos_sensor(0x6F12, 0x921C);
	write_cmos_sensor(0x6F12, 0x0189);
	write_cmos_sensor(0x6F12, 0x1180);
	write_cmos_sensor(0x6F12, 0x921C);
	write_cmos_sensor(0x6F12, 0x4189);
	write_cmos_sensor(0x6F12, 0x1180);
	write_cmos_sensor(0x6F12, 0x921C);
	write_cmos_sensor(0x6F12, 0x8189);
	write_cmos_sensor(0x6F12, 0x1180);
	write_cmos_sensor(0x6F12, 0x921C);
	write_cmos_sensor(0x6F12, 0xC189);
	write_cmos_sensor(0x6F12, 0x1180);
	write_cmos_sensor(0x6F12, 0x921C);
	write_cmos_sensor(0x6F12, 0x018A);
	write_cmos_sensor(0x6F12, 0x1180);
	write_cmos_sensor(0x6F12, 0x921C);
	write_cmos_sensor(0x6F12, 0x418A);
	write_cmos_sensor(0x6F12, 0x1180);
	write_cmos_sensor(0x6F12, 0x921C);
	write_cmos_sensor(0x6F12, 0x818A);
	write_cmos_sensor(0x6F12, 0x1180);
	write_cmos_sensor(0x6F12, 0x921C);
	write_cmos_sensor(0x6F12, 0xC18A);
	write_cmos_sensor(0x6F12, 0x1180);
	write_cmos_sensor(0x6F12, 0x921C);
	write_cmos_sensor(0x6F12, 0x018B);
	write_cmos_sensor(0x6F12, 0x1180);
	write_cmos_sensor(0x6F12, 0x921C);
	write_cmos_sensor(0x6F12, 0x418B);
	write_cmos_sensor(0x6F12, 0x1180);
	write_cmos_sensor(0x6F12, 0x921C);
	write_cmos_sensor(0x6F12, 0x818B);
	write_cmos_sensor(0x6F12, 0x1180);
	write_cmos_sensor(0x6F12, 0x921C);
	write_cmos_sensor(0x6F12, 0xC18B);
	write_cmos_sensor(0x6F12, 0x1180);
	write_cmos_sensor(0x6F12, 0x921C);
	write_cmos_sensor(0x6F12, 0x018C);
	write_cmos_sensor(0x6F12, 0x1180);
	write_cmos_sensor(0x6F12, 0x921C);
	write_cmos_sensor(0x6F12, 0x418C);
	write_cmos_sensor(0x6F12, 0x1180);
	write_cmos_sensor(0x6F12, 0x921C);
	write_cmos_sensor(0x6F12, 0x818C);
	write_cmos_sensor(0x6F12, 0x1180);
	write_cmos_sensor(0x6F12, 0x911C);
	write_cmos_sensor(0x6F12, 0xC08C);
	write_cmos_sensor(0x6F12, 0x0880);
	write_cmos_sensor(0x6F12, 0xA5E7);
	write_cmos_sensor(0x6F12, 0x7CB5);
	write_cmos_sensor(0x6F12, 0x0346);
	write_cmos_sensor(0x6F12, 0x7748);
	write_cmos_sensor(0x6F12, 0x448F);
	write_cmos_sensor(0x6F12, 0x058F);
	write_cmos_sensor(0x6F12, 0x4FF0);
	write_cmos_sensor(0x6F12, 0x8040);
	write_cmos_sensor(0x6F12, 0x8089);
	write_cmos_sensor(0x6F12, 0xA0F5);
	write_cmos_sensor(0x6F12, 0x2061);
	write_cmos_sensor(0x6F12, 0x5139);
	write_cmos_sensor(0x6F12, 0x02D1);
	write_cmos_sensor(0x6F12, 0x0824);
	write_cmos_sensor(0x6F12, 0x41F2);
	write_cmos_sensor(0x6F12, 0x5815);
	write_cmos_sensor(0x6F12, 0x6A46);
	write_cmos_sensor(0x6F12, 0x01A9);
	write_cmos_sensor(0x6F12, 0x1846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xBAF9);
	write_cmos_sensor(0x6F12, 0xDDE9);
	write_cmos_sensor(0x6F12, 0x0021);
	write_cmos_sensor(0x6F12, 0x2046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xBAF9);
	write_cmos_sensor(0x6F12, 0x38B1);
	write_cmos_sensor(0x6F12, 0xDDE9);
	write_cmos_sensor(0x6F12, 0x0021);
	write_cmos_sensor(0x6F12, 0x2846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xB4F9);
	write_cmos_sensor(0x6F12, 0x08B1);
	write_cmos_sensor(0x6F12, 0x0120);
	write_cmos_sensor(0x6F12, 0x7CBD);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x7CBD);
	write_cmos_sensor(0x6F12, 0xFEB5);
	write_cmos_sensor(0x6F12, 0x0446);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xB0F9);
	write_cmos_sensor(0x6F12, 0x2046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xB2F9);
	write_cmos_sensor(0x6F12, 0x4FF0);
	write_cmos_sensor(0x6F12, 0x8040);
	write_cmos_sensor(0x6F12, 0x8589);
	write_cmos_sensor(0x6F12, 0x94F8);
	write_cmos_sensor(0x6F12, 0x7700);
	write_cmos_sensor(0x6F12, 0x0190);
	write_cmos_sensor(0x6F12, 0x94F8);
	write_cmos_sensor(0x6F12, 0x7800);
	write_cmos_sensor(0x6F12, 0x0290);
	write_cmos_sensor(0x6F12, 0x94F8);
	write_cmos_sensor(0x6F12, 0x3C01);
	write_cmos_sensor(0x6F12, 0x5D4E);
	write_cmos_sensor(0x6F12, 0x40F6);
	write_cmos_sensor(0x6F12, 0x5127);
	write_cmos_sensor(0x6F12, 0xD8B1);
	write_cmos_sensor(0x6F12, 0x718F);
	write_cmos_sensor(0x6F12, 0xBD42);
	write_cmos_sensor(0x6F12, 0x04D1);
	write_cmos_sensor(0x6F12, 0x5B4A);
	write_cmos_sensor(0x6F12, 0x0821);
	write_cmos_sensor(0x6F12, 0x0B20);
	write_cmos_sensor(0x6F12, 0xA2F8);
	write_cmos_sensor(0x6F12, 0x5001);
	write_cmos_sensor(0x6F12, 0x94F8);
	write_cmos_sensor(0x6F12, 0x2220);
	write_cmos_sensor(0x6F12, 0x4FF2);
	write_cmos_sensor(0x6F12, 0x5010);
	write_cmos_sensor(0x6F12, 0x82B3);
	write_cmos_sensor(0x6F12, 0x96F8);
	write_cmos_sensor(0x6F12, 0x5420);
	write_cmos_sensor(0x6F12, 0x6AB3);
	write_cmos_sensor(0x6F12, 0x94F8);
	write_cmos_sensor(0x6F12, 0x7620);
	write_cmos_sensor(0x6F12, 0x042A);
	write_cmos_sensor(0x6F12, 0x21D0);
	write_cmos_sensor(0x6F12, 0x94F8);
	write_cmos_sensor(0x6F12, 0x3C31);
	write_cmos_sensor(0x6F12, 0x21F0);
	write_cmos_sensor(0x6F12, 0x0301);
	write_cmos_sensor(0x6F12, 0x5A43);
	write_cmos_sensor(0x6F12, 0x6B46);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x8CF9);
	write_cmos_sensor(0x6F12, 0x94F8);
	write_cmos_sensor(0x6F12, 0x3D01);
	write_cmos_sensor(0x6F12, 0x48B3);
	write_cmos_sensor(0x6F12, 0x94F8);
	write_cmos_sensor(0x6F12, 0x7620);
	write_cmos_sensor(0x6F12, 0x318F);
	write_cmos_sensor(0x6F12, 0x1346);
	write_cmos_sensor(0x6F12, 0x4243);
	write_cmos_sensor(0x6F12, 0xBD42);
	write_cmos_sensor(0x6F12, 0x01D1);
	write_cmos_sensor(0x6F12, 0x41F2);
	write_cmos_sensor(0x6F12, 0x5811);
	write_cmos_sensor(0x6F12, 0x94F8);
	write_cmos_sensor(0x6F12, 0x2250);
	write_cmos_sensor(0x6F12, 0x4FF2);
	write_cmos_sensor(0x6F12, 0x6010);
	write_cmos_sensor(0x6F12, 0x35B1);
	write_cmos_sensor(0x6F12, 0x96F8);
	write_cmos_sensor(0x6F12, 0x5450);
	write_cmos_sensor(0x6F12, 0x1DB1);
	write_cmos_sensor(0x6F12, 0x042B);
	write_cmos_sensor(0x6F12, 0x21F0);
	write_cmos_sensor(0x6F12, 0x0301);
	write_cmos_sensor(0x6F12, 0x10D0);
	write_cmos_sensor(0x6F12, 0x6B46);
	write_cmos_sensor(0x6F12, 0x10E0);
	write_cmos_sensor(0x6F12, 0x94F8);
	write_cmos_sensor(0x6F12, 0x3C21);
	write_cmos_sensor(0x6F12, 0x21F0);
	write_cmos_sensor(0x6F12, 0x0301);
	write_cmos_sensor(0x6F12, 0x9200);
	write_cmos_sensor(0x6F12, 0x6B46);
	write_cmos_sensor(0x6F12, 0x091F);
	write_cmos_sensor(0x6F12, 0xDBE7);
	write_cmos_sensor(0x6F12, 0xFFE7);
	write_cmos_sensor(0x6F12, 0x94F8);
	write_cmos_sensor(0x6F12, 0x7620);
	write_cmos_sensor(0x6F12, 0x94F8);
	write_cmos_sensor(0x6F12, 0x3C31);
	write_cmos_sensor(0x6F12, 0x5A43);
	write_cmos_sensor(0x6F12, 0xD3E7);
	write_cmos_sensor(0x6F12, 0x6B46);
	write_cmos_sensor(0x6F12, 0x091D);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x5FF9);
	write_cmos_sensor(0x6F12, 0x96F8);
	write_cmos_sensor(0x6F12, 0x5E00);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0x02D0);
	write_cmos_sensor(0x6F12, 0x2046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x5DF9);
	write_cmos_sensor(0x6F12, 0xFEBD);
	write_cmos_sensor(0x6F12, 0x70B5);
	write_cmos_sensor(0x6F12, 0x4FF4);
	write_cmos_sensor(0x6F12, 0x0141);
	write_cmos_sensor(0x6F12, 0x0C20);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x5BF9);
	write_cmos_sensor(0x6F12, 0x0446);
	write_cmos_sensor(0x6F12, 0x314D);
	write_cmos_sensor(0x6F12, 0xC005);
	write_cmos_sensor(0x6F12, 0x04D5);
	write_cmos_sensor(0x6F12, 0x2888);
	write_cmos_sensor(0x6F12, 0x401C);
	write_cmos_sensor(0x6F12, 0x2880);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x57F9);
	write_cmos_sensor(0x6F12, 0x2004);
	write_cmos_sensor(0x6F12, 0x06D5);
	write_cmos_sensor(0x6F12, 0x288C);
	write_cmos_sensor(0x6F12, 0x401C);
	write_cmos_sensor(0x6F12, 0x2884);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0x7040);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x53B9);
	write_cmos_sensor(0x6F12, 0x70BD);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0xDFF8);
	write_cmos_sensor(0x6F12, 0x8080);
	write_cmos_sensor(0x6F12, 0x0220);
	write_cmos_sensor(0x6F12, 0x4446);
	write_cmos_sensor(0x6F12, 0x14F8);
	write_cmos_sensor(0x6F12, 0x711F);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x3EF9);
	write_cmos_sensor(0x6F12, 0x86B2);
	write_cmos_sensor(0x6F12, 0x6078);
	write_cmos_sensor(0x6F12, 0x0127);
	write_cmos_sensor(0x6F12, 0x421E);
	write_cmos_sensor(0x6F12, 0x07FA);
	write_cmos_sensor(0x6F12, 0x02F1);
	write_cmos_sensor(0x6F12, 0x3142);
	write_cmos_sensor(0x6F12, 0x15D0);
	write_cmos_sensor(0x6F12, 0x204C);
	write_cmos_sensor(0x6F12, 0x658F);
	write_cmos_sensor(0x6F12, 0x41EA);
	write_cmos_sensor(0x6F12, 0x0502);
	write_cmos_sensor(0x6F12, 0x6287);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0x0322);
	write_cmos_sensor(0x6F12, 0x801E);
	write_cmos_sensor(0x6F12, 0x8240);
	write_cmos_sensor(0x6F12, 0xB4F8);
	write_cmos_sensor(0x6F12, 0x8E00);
	write_cmos_sensor(0x6F12, 0x20EA);
	write_cmos_sensor(0x6F12, 0x0203);
	write_cmos_sensor(0x6F12, 0xA4F8);
	write_cmos_sensor(0x6F12, 0x8E30);
	write_cmos_sensor(0x6F12, 0x1043);
	write_cmos_sensor(0x6F12, 0xA4F8);
	write_cmos_sensor(0x6F12, 0x8E00);
	write_cmos_sensor(0x6F12, 0x0220);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x21F9);
	write_cmos_sensor(0x6F12, 0x6587);
	write_cmos_sensor(0x6F12, 0x98F8);
	write_cmos_sensor(0x6F12, 0x7500);
	write_cmos_sensor(0x6F12, 0x3946);
	write_cmos_sensor(0x6F12, 0x401E);
	write_cmos_sensor(0x6F12, 0x8740);
	write_cmos_sensor(0x6F12, 0x3742);
	write_cmos_sensor(0x6F12, 0x2CD0);
	write_cmos_sensor(0x6F12, 0x114A);
	write_cmos_sensor(0x6F12, 0x908C);
	write_cmos_sensor(0x6F12, 0x401C);
	write_cmos_sensor(0x6F12, 0x9084);
	write_cmos_sensor(0x6F12, 0x1148);
	write_cmos_sensor(0x6F12, 0x0288);
	write_cmos_sensor(0x6F12, 0x012A);
	write_cmos_sensor(0x6F12, 0x24D1);
	write_cmos_sensor(0x6F12, 0x90F8);
	write_cmos_sensor(0x6F12, 0x5421);
	write_cmos_sensor(0x6F12, 0xFAB1);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0x1AE0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0x8000);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x3CA0);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x44C0);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x32C0);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x3B50);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0xA000);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x2F50);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0xAC30);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x09F0);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0xF000);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x2790);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0x7000);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x2570);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xFDB8);
	write_cmos_sensor(0x6F12, 0x80F8);
	write_cmos_sensor(0x6F12, 0x5511);
	write_cmos_sensor(0x6F12, 0xA4E6);
	write_cmos_sensor(0x6F12, 0x4079);
	write_cmos_sensor(0x6F12, 0x38B1);
	write_cmos_sensor(0x6F12, 0x1F23);
	write_cmos_sensor(0x6F12, 0x8343);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0xC701);
	write_cmos_sensor(0x6F12, 0x0220);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xF5B8);
	write_cmos_sensor(0x6F12, 0x1F21);
	write_cmos_sensor(0x6F12, 0x0220);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xF6B8);
	write_cmos_sensor(0x6F12, 0x70B5);
	write_cmos_sensor(0x6F12, 0x0446);
	write_cmos_sensor(0x6F12, 0x4279);
	write_cmos_sensor(0x6F12, 0x8079);
	write_cmos_sensor(0x6F12, 0x18B1);
	write_cmos_sensor(0x6F12, 0x0121);
	write_cmos_sensor(0x6F12, 0x401E);
	write_cmos_sensor(0x6F12, 0x8140);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0x0021);
	write_cmos_sensor(0x6F12, 0x3348);
	write_cmos_sensor(0x6F12, 0x0A43);
	write_cmos_sensor(0x6F12, 0x6271);
	write_cmos_sensor(0x6F12, 0x0069);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x86B2);
	write_cmos_sensor(0x6F12, 0x050C);
	write_cmos_sensor(0x6F12, 0x3146);
	write_cmos_sensor(0x6F12, 0x2846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x91F8);
	write_cmos_sensor(0x6F12, 0x2046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xE3F8);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0x3146);
	write_cmos_sensor(0x6F12, 0x2846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x89F8);
	write_cmos_sensor(0x6F12, 0x0120);
	write_cmos_sensor(0x6F12, 0x70BD);
	write_cmos_sensor(0x6F12, 0x10B5);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0xB341);
	write_cmos_sensor(0x6F12, 0x2748);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xDAF8);
	write_cmos_sensor(0x6F12, 0x274C);
	write_cmos_sensor(0x6F12, 0x4BF6);
	write_cmos_sensor(0x6F12, 0x5870);
	write_cmos_sensor(0x6F12, 0xE18C);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xD9F8);
	write_cmos_sensor(0x6F12, 0xE08C);
	write_cmos_sensor(0x6F12, 0x254A);
	write_cmos_sensor(0x6F12, 0x2449);
	write_cmos_sensor(0x6F12, 0x42F8);
	write_cmos_sensor(0x6F12, 0x2010);
	write_cmos_sensor(0x6F12, 0x401C);
	write_cmos_sensor(0x6F12, 0xE084);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x5541);
	write_cmos_sensor(0x6F12, 0x2248);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xC7F8);
	write_cmos_sensor(0x6F12, 0x1B4C);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0xEF31);
	write_cmos_sensor(0x6F12, 0x2060);
	write_cmos_sensor(0x6F12, 0x1F48);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xBFF8);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0xAB31);
	write_cmos_sensor(0x6F12, 0x6060);
	write_cmos_sensor(0x6F12, 0x1C48);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xB8F8);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x5D31);
	write_cmos_sensor(0x6F12, 0xA060);
	write_cmos_sensor(0x6F12, 0x1A48);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xB1F8);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0xBF21);
	write_cmos_sensor(0x6F12, 0xE060);
	write_cmos_sensor(0x6F12, 0x1748);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xAAF8);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x8721);
	write_cmos_sensor(0x6F12, 0x1548);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0xA4F8);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0xC511);
	write_cmos_sensor(0x6F12, 0x1348);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x9EF8);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0xE501);
	write_cmos_sensor(0x6F12, 0x1148);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x98F8);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0xD501);
	write_cmos_sensor(0x6F12, 0x0F48);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x92F8);
	write_cmos_sensor(0x6F12, 0x2061);
	write_cmos_sensor(0x6F12, 0x10BD);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x44C0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xE7BD);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x3280);
	write_cmos_sensor(0x6F12, 0xAFF3);
	write_cmos_sensor(0x6F12, 0x0080);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x3CF0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xA4CD);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x58A5);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x463F);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x5637);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xA01F);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xC677);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x049F);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0641);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xD7BB);
	write_cmos_sensor(0x6F12, 0x4EF2);
	write_cmos_sensor(0x6F12, 0x8F3C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4EF2);
	write_cmos_sensor(0x6F12, 0xED3C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4EF2);
	write_cmos_sensor(0x6F12, 0x814C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4EF2);
	write_cmos_sensor(0x6F12, 0xA95C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4EF2);
	write_cmos_sensor(0x6F12, 0xE33C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x40F6);
	write_cmos_sensor(0x6F12, 0x032C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4AF2);
	write_cmos_sensor(0x6F12, 0xCD4C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x45F6);
	write_cmos_sensor(0x6F12, 0xA50C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x44F2);
	write_cmos_sensor(0x6F12, 0x3F6C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x45F2);
	write_cmos_sensor(0x6F12, 0x376C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4CF2);
	write_cmos_sensor(0x6F12, 0x610C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4AF2);
	write_cmos_sensor(0x6F12, 0x070C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4CF2);
	write_cmos_sensor(0x6F12, 0x6B4C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4CF2);
	write_cmos_sensor(0x6F12, 0x275C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4CF2);
	write_cmos_sensor(0x6F12, 0x3F3C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4CF2);
	write_cmos_sensor(0x6F12, 0x094C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x40F2);
	write_cmos_sensor(0x6F12, 0xE91C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4BF2);
	write_cmos_sensor(0x6F12, 0x6B5C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x46F6);
	write_cmos_sensor(0x6F12, 0x6D0C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4FF2);
	write_cmos_sensor(0x6F12, 0xFD0C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x40F2);
	write_cmos_sensor(0x6F12, 0x532C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x40F2);
	write_cmos_sensor(0x6F12, 0xD12C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4DF2);
	write_cmos_sensor(0x6F12, 0xBB7C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x43F2);
	write_cmos_sensor(0x6F12, 0xBB1C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x010C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x43F2);
	write_cmos_sensor(0x6F12, 0x5B1C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x010C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6028, 0x4000); 
	write_cmos_sensor(0x112C, 0x4220);
	write_cmos_sensor(0x112E, 0x0000);
	write_cmos_sensor(0x3842, 0x240F);
	write_cmos_sensor(0xF404, 0x0FF3);
	write_cmos_sensor(0xF426, 0x00A0);
	write_cmos_sensor(0xF428, 0x44C2);
	write_cmos_sensor(0xF458, 0x0009);
	write_cmos_sensor(0xF45A, 0x000E);
	write_cmos_sensor(0xF45C, 0x000E);
	write_cmos_sensor(0xF45E, 0x0018);
	write_cmos_sensor(0xF460, 0x0010);
	write_cmos_sensor(0xF462, 0x0018);
	write_cmos_sensor(0x3850, 0x005A);
	write_cmos_sensor(0x3834, 0x0803);
	write_cmos_sensor(0x3264, 0x0009);
	write_cmos_sensor(0x3284, 0x0044);
	write_cmos_sensor(0x3290, 0x0000);
	write_cmos_sensor(0x3298, 0x0020);
	write_cmos_sensor(0x33E0, 0x00B9);
	write_cmos_sensor(0x33E4, 0x006F);
	write_cmos_sensor(0x33E8, 0x0076);
	write_cmos_sensor(0x33EC, 0x011A);
	write_cmos_sensor(0x33F0, 0x0004);
	write_cmos_sensor(0x33F4, 0x0000);
	write_cmos_sensor(0x33F8, 0x0000);
	write_cmos_sensor(0x33FC, 0x0000);
	write_cmos_sensor(0x3400, 0x00B9);
	write_cmos_sensor(0x3404, 0x006F);
	write_cmos_sensor(0x3408, 0x0076);
	write_cmos_sensor(0x340C, 0x011A);
	write_cmos_sensor(0x3410, 0x0004);
	write_cmos_sensor(0x3414, 0x0000);
	write_cmos_sensor(0x3418, 0x0000);
	write_cmos_sensor(0x341C, 0x0000);
	write_cmos_sensor(0xF4AE, 0x003C);
	write_cmos_sensor(0xF4B0, 0x1124); 
	write_cmos_sensor(0xF4B2, 0x003D);
	write_cmos_sensor(0xF4B4, 0x1125); 
	write_cmos_sensor(0xF4B6, 0x0044);
	write_cmos_sensor(0xF4B8, 0x112C);
	write_cmos_sensor(0xF4BA, 0x0045);
	write_cmos_sensor(0xF4BC, 0x112D);
	write_cmos_sensor(0xF4BE, 0x004C);
	write_cmos_sensor(0xF4C0, 0x1134); 
	write_cmos_sensor(0xF4C2, 0x004D);
	write_cmos_sensor(0xF4C4, 0x1135); 
	write_cmos_sensor(0xF486, 0x0480);
	write_cmos_sensor(0x31E4, 0x0000);
	write_cmos_sensor(0x31E2, 0x0000);
	write_cmos_sensor(0x31E0, 0x0000);
	write_cmos_sensor(0xF46C, 0x002D);
	write_cmos_sensor(0xF47A, 0x00FC);
	write_cmos_sensor(0xF47C, 0x0000);
	write_cmos_sensor(0x31F8, 0x0008);
	write_cmos_sensor(0x31FA, 0x1158);
	write_cmos_sensor(0xF160, 0x000B);
	write_cmos_sensor(0xF150, 0x000B);
	write_cmos_sensor(0x3050, 0x0000);
	write_cmos_sensor(0x3052, 0x0400);
	write_cmos_sensor(0x0B00, 0x0080);
	write_cmos_sensor(0x305E, 0x0000);
	write_cmos_sensor(0x3076, 0x0000);
	write_cmos_sensor(0x3054, 0x0000);
	write_cmos_sensor(0x0B04, 0x0101);
	write_cmos_sensor(0x0B08, 0x0000);
	write_cmos_sensor(0x0B0E, 0x0000);
	write_cmos_sensor(0xB134, 0x0180);
	write_cmos_sensor(0xB13C, 0x0400);
	write_cmos_sensor(0x3000, 0x0001);
	write_cmos_sensor(0x655E, 0x03E8);
	write_cmos_sensor(0x6592, 0x0010);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x20C0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x19C8);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x602A, 0x00EA);
	write_cmos_sensor(0x6F12, 0xFF08);
	write_cmos_sensor(0x602A, 0x00F6);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0x8000);
	write_cmos_sensor(0x6F12, 0xC000);
	write_cmos_sensor(0x6F12, 0xFFFF);
	write_cmos_sensor(0x602A, 0x02F4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0032);
	write_cmos_sensor(0x6F12, 0x0050);
	write_cmos_sensor(0x6F12, 0x0050);
	write_cmos_sensor(0x6F12, 0x00C8);
	write_cmos_sensor(0x6F12, 0x00C8);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x0118);
	write_cmos_sensor(0x6F12, 0x0030);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0x0050);
	write_cmos_sensor(0x6F12, 0x00B4);
	write_cmos_sensor(0x6F12, 0x00B4);
	write_cmos_sensor(0x6F12, 0x00B4);
	write_cmos_sensor(0x6F12, 0x00C8);
	write_cmos_sensor(0x6F12, 0x00FA);
	write_cmos_sensor(0x6F12, 0x00FA);
	write_cmos_sensor(0x6F12, 0x012C);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0x0050);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0025);
	write_cmos_sensor(0x6F12, 0x003C);
	write_cmos_sensor(0x6F12, 0x003C);
	write_cmos_sensor(0x6F12, 0x0096);
	write_cmos_sensor(0x6F12, 0x0096);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x015B);
	write_cmos_sensor(0x6F12, 0x0035);
	write_cmos_sensor(0x6F12, 0x0021);
	write_cmos_sensor(0x6F12, 0x0021);
	write_cmos_sensor(0x6F12, 0x0043);
	write_cmos_sensor(0x6F12, 0x0096);
	write_cmos_sensor(0x6F12, 0x0096);
	write_cmos_sensor(0x6F12, 0x0096);
	write_cmos_sensor(0x6F12, 0x00A7);
	write_cmos_sensor(0x6F12, 0x00D0);
	write_cmos_sensor(0x6F12, 0x00D0);
	write_cmos_sensor(0x6F12, 0x00FA);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0x0050);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0019);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0x0064);
	write_cmos_sensor(0x6F12, 0x0064);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x019D);
	write_cmos_sensor(0x6F12, 0x0039);
	write_cmos_sensor(0x6F12, 0x001B);
	write_cmos_sensor(0x6F12, 0x001B);
	write_cmos_sensor(0x6F12, 0x0035);
	write_cmos_sensor(0x6F12, 0x0078);
	write_cmos_sensor(0x6F12, 0x0078);
	write_cmos_sensor(0x6F12, 0x0078);
	write_cmos_sensor(0x6F12, 0x0085);
	write_cmos_sensor(0x6F12, 0x00A7);
	write_cmos_sensor(0x6F12, 0x00A7);
	write_cmos_sensor(0x6F12, 0x00C8);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0x0050);
	write_cmos_sensor(0x602A, 0x004A);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x03F4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x602A, 0x1360);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1C8A);
	write_cmos_sensor(0x6F12, 0x0011);
	write_cmos_sensor(0x6F12, 0x1010);
	write_cmos_sensor(0x602A, 0x1C92);
	write_cmos_sensor(0x6F12, 0x7700);
	write_cmos_sensor(0x602A, 0x1BA0);
	write_cmos_sensor(0x6F12, 0x0030);
	write_cmos_sensor(0x602A, 0x1C06);
	write_cmos_sensor(0x6F12, 0x0250);
	write_cmos_sensor(0x6F12, 0x8060);
	write_cmos_sensor(0x602A, 0x1BDE);
	write_cmos_sensor(0x6F12, 0x0001);

	write_cmos_sensor(0x0100, 0x0000);

}	/*	sensor_init  */


static void preview_setting(void)
{
  LOG_INF("E\n");

  write_cmos_sensor(0x0100, 0x0000);
  check_output_stream_off();
	write_cmos_sensor(0x6214, 0x7971);
	write_cmos_sensor(0x6218, 0x7150);
	write_cmos_sensor(0x0344, 0x0000);
	write_cmos_sensor(0x0346, 0x0000);
	write_cmos_sensor(0x0348, 0x161F);
	write_cmos_sensor(0x034A, 0x109F);
	write_cmos_sensor(0x034C, 0x0B10);
	write_cmos_sensor(0x034E, 0x0850);
	write_cmos_sensor(0x0340, 0x0A28);
	write_cmos_sensor(0x0342, 0x1800);
	write_cmos_sensor(0x3000, 0x0001);
	write_cmos_sensor(0x3002, 0x0100);
	write_cmos_sensor(0x0900, 0x0111);
	write_cmos_sensor(0x0380, 0x0002);
	write_cmos_sensor(0x0382, 0x0002);
	write_cmos_sensor(0x0384, 0x0002);
	write_cmos_sensor(0x0386, 0x0002);
	write_cmos_sensor(0x3070, 0x0000);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x03E0);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x03E2);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x03E4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x004A);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x00E0);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x3072, 0x0000);
	write_cmos_sensor(0x0B04, 0x0101);
	write_cmos_sensor(0x306A, 0x0340);
	write_cmos_sensor(0x306E, 0x0000);
	write_cmos_sensor(0x3074, 0x0000);
	write_cmos_sensor(0x3004, 0x0001);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x013E, 0x0000);
	write_cmos_sensor(0x0300, 0x0003);
	write_cmos_sensor(0x0302, 0x0002);
	write_cmos_sensor(0x0304, 0x0004);
	write_cmos_sensor(0x0306, 0x00F0);
	write_cmos_sensor(0x0308, 0x0008);
	write_cmos_sensor(0x030A, 0x0001);
	write_cmos_sensor(0x030C, 0x0004);
	write_cmos_sensor(0x030E, 0x010A);
	write_cmos_sensor(0x300A, 0x0001);
	write_cmos_sensor(0x300C, 0x0001);
	write_cmos_sensor(0x324A, 0x0300);
	write_cmos_sensor(0x31C0, 0x0004);
	write_cmos_sensor(0x319E, 0x0100);
	write_cmos_sensor(0x31A0, 0x0050);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x1C04);
	write_cmos_sensor(0x6F12, 0x0A01);
	write_cmos_sensor(0x602A, 0x1BA2);
	write_cmos_sensor(0x6F12, 0x0081);
	write_cmos_sensor(0x0200, 0x0000);
	write_cmos_sensor(0x0202, 0x0620);
	write_cmos_sensor(0x021E, 0x0040);
	write_cmos_sensor(0x021C, 0x0000);
	write_cmos_sensor(0x0204, 0x0020);
	write_cmos_sensor(0x0216, 0x0000);
	write_cmos_sensor(0x3258, 0x0036);
	write_cmos_sensor(0x3260, 0x002A);
	write_cmos_sensor(0x3268, 0x0001);
	write_cmos_sensor(0x3270, 0x0024);
	write_cmos_sensor(0x3288, 0x0030);
	write_cmos_sensor(0x32A0, 0x0024);
	write_cmos_sensor(0x32C4, 0x0029);
	write_cmos_sensor(0x337C, 0x0066);
	write_cmos_sensor(0x3458, 0x002E);
	write_cmos_sensor(0x382E, 0x060F);
	write_cmos_sensor(0x3830, 0x0805);
	write_cmos_sensor(0x3832, 0x0605);
	write_cmos_sensor(0x3834, 0x0803);
	write_cmos_sensor(0x3840, 0x0064);
	write_cmos_sensor(0x3844, 0x00BF);
	write_cmos_sensor(0xF404, 0x0FF3);
	write_cmos_sensor(0xF424, 0x0000);
	write_cmos_sensor(0x0612, 0x0000);
	write_cmos_sensor(0x31C6, 0x1620);
	write_cmos_sensor(0x0408, 0x0000);
	write_cmos_sensor(0x040C, 0x0000);
	write_cmos_sensor(0x0100, 0x0100);

}	/*	preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);
	if (currefps == 300) {

		preview_setting();

	} else if (currefps == 240)
	{
		LOG_INF("else if (currefps == 240)\n");
  write_cmos_sensor(0x0100, 0x0000);

  write_cmos_sensor(0x0100, 0x0100);

	} else if (currefps == 150)
	{
	LOG_INF("else if (currefps == 150)\n");
	write_cmos_sensor(0x0100, 0x0000);

	write_cmos_sensor(0x0100, 0x0100);

	} else {
		LOG_INF("else  150fps\n");
	write_cmos_sensor_byte(0x0100, 0x00);

	write_cmos_sensor_byte(0x0100, 0x01);
	}
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);
  preview_setting();
}

static void hs_video_setting(void)
{
	LOG_INF("E\n");
  write_cmos_sensor(0x0100, 0x0000);
	check_output_stream_off();
	write_cmos_sensor(0x6214, 0x7971);
	write_cmos_sensor(0x6218, 0x7150);
	write_cmos_sensor(0x0344, 0x0000);
	write_cmos_sensor(0x0346, 0x0000);
	write_cmos_sensor(0x0348, 0x161F);
	write_cmos_sensor(0x034A, 0x109F);
	write_cmos_sensor(0x034C, 0x0B10);
	write_cmos_sensor(0x034E, 0x0850);
	write_cmos_sensor(0x0340, 0x08E0);
	write_cmos_sensor(0x0342, 0x0DC0);
	write_cmos_sensor(0x3000, 0x0001);
	write_cmos_sensor(0x3002, 0x0100);
	write_cmos_sensor(0x0900, 0x0111);
	write_cmos_sensor(0x0380, 0x0002);
	write_cmos_sensor(0x0382, 0x0002);
	write_cmos_sensor(0x0384, 0x0002);
	write_cmos_sensor(0x0386, 0x0002);
	write_cmos_sensor(0x3070, 0x0000);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x03E0);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x03E2);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x03E4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x004A);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x00E0);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x3072, 0x0000);
	write_cmos_sensor(0x0B04, 0x0101);
	write_cmos_sensor(0x306A, 0x0340);
	write_cmos_sensor(0x306E, 0x0000);
	write_cmos_sensor(0x3074, 0x0000);
	write_cmos_sensor(0x3004, 0x0001);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x013E, 0x0000);
	write_cmos_sensor(0x0300, 0x0003);
	write_cmos_sensor(0x0302, 0x0001);
	write_cmos_sensor(0x0304, 0x0004);
	write_cmos_sensor(0x0306, 0x00F0);
	write_cmos_sensor(0x0308, 0x0008);
	write_cmos_sensor(0x030A, 0x0001);
	write_cmos_sensor(0x030C, 0x0004);
	write_cmos_sensor(0x030E, 0x0178);
	write_cmos_sensor(0x300A, 0x0001);
	write_cmos_sensor(0x300C, 0x0000);
	write_cmos_sensor(0x324A, 0x0300);
	write_cmos_sensor(0x31C0, 0x0004);
	write_cmos_sensor(0x319E, 0x0100);
	write_cmos_sensor(0x31A0, 0x0030);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x1C04);
	write_cmos_sensor(0x6F12, 0x0A01);
	write_cmos_sensor(0x602A, 0x1BA2);
	write_cmos_sensor(0x6F12, 0x0081);
	write_cmos_sensor(0x0200, 0x0000);
	write_cmos_sensor(0x0202, 0x0620);
	write_cmos_sensor(0x021E, 0x0040);
	write_cmos_sensor(0x021C, 0x0000);
	write_cmos_sensor(0x0204, 0x0020);
	write_cmos_sensor(0x0216, 0x0000);
	write_cmos_sensor(0x3258, 0x0036);
	write_cmos_sensor(0x3260, 0x002A);
	write_cmos_sensor(0x3268, 0x0001);
	write_cmos_sensor(0x3270, 0x0024);
	write_cmos_sensor(0x3288, 0x0030);
	write_cmos_sensor(0x32A0, 0x0024);
	write_cmos_sensor(0x32C4, 0x0029);
	write_cmos_sensor(0x337C, 0x0066);
	write_cmos_sensor(0x3458, 0x002E);
	write_cmos_sensor(0x382E, 0x060F);
	write_cmos_sensor(0x3830, 0x0805);
	write_cmos_sensor(0x3832, 0x0605);
	write_cmos_sensor(0x3834, 0x0803);
	write_cmos_sensor(0x3840, 0x0064);
	write_cmos_sensor(0x3844, 0x00FF);
	write_cmos_sensor(0xF404, 0x0FF3);
	write_cmos_sensor(0xF424, 0x0000);
	write_cmos_sensor(0x0612, 0x0000);
  write_cmos_sensor(0x0100, 0x0100);

}

static void slim_video_setting(void)
{
	LOG_INF("E\n");
  write_cmos_sensor(0x0100, 0x0000);
  check_output_stream_off();
	write_cmos_sensor(0x6214, 0x7971);
	write_cmos_sensor(0x6218, 0x7150);
	write_cmos_sensor(0x0344, 0x0000);
	write_cmos_sensor(0x0346, 0x0000);
	write_cmos_sensor(0x0348, 0x161F);
	write_cmos_sensor(0x034A, 0x109F);
	write_cmos_sensor(0x034C, 0x0B10);
	write_cmos_sensor(0x034E, 0x0850);
	write_cmos_sensor(0x0340, 0x0C60);
	write_cmos_sensor(0x0342, 0x2380);
	write_cmos_sensor(0x3000, 0x0001);
	write_cmos_sensor(0x3002, 0x0100);
	write_cmos_sensor(0x0900, 0x0111);
	write_cmos_sensor(0x0380, 0x0002);
	write_cmos_sensor(0x0382, 0x0002);
	write_cmos_sensor(0x0384, 0x0002);
	write_cmos_sensor(0x0386, 0x0002);
	write_cmos_sensor(0x3070, 0x0000);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x03E0);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x03E2);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x03E4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x004A);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x00E0);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x3072, 0x0000);
	write_cmos_sensor(0x0B04, 0x0101);
	write_cmos_sensor(0x306A, 0x0340);
	write_cmos_sensor(0x306E, 0x0000);
	write_cmos_sensor(0x3074, 0x0000);
	write_cmos_sensor(0x3004, 0x0001);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x013E, 0x0000);
	write_cmos_sensor(0x0300, 0x0003);
	write_cmos_sensor(0x0302, 0x0001);
	write_cmos_sensor(0x0304, 0x0004);
	write_cmos_sensor(0x0306, 0x00D8);
	write_cmos_sensor(0x0308, 0x0008);
	write_cmos_sensor(0x030A, 0x0001);
	write_cmos_sensor(0x030C, 0x0004);
	write_cmos_sensor(0x030E, 0x0117);
	write_cmos_sensor(0x300A, 0x0001);
	write_cmos_sensor(0x300C, 0x0001);
	write_cmos_sensor(0x324A, 0x0300);
	write_cmos_sensor(0x31C0, 0x0004);
	write_cmos_sensor(0x319E, 0x0100);
	write_cmos_sensor(0x31A0, 0x0030);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x1C04);
	write_cmos_sensor(0x6F12, 0x0A01);
	write_cmos_sensor(0x602A, 0x1BA2);
	write_cmos_sensor(0x6F12, 0x0081);
	write_cmos_sensor(0x0200, 0x0000);
	write_cmos_sensor(0x0202, 0x0620);
	write_cmos_sensor(0x021E, 0x0040);
	write_cmos_sensor(0x021C, 0x0000);
	write_cmos_sensor(0x0204, 0x0020);
	write_cmos_sensor(0x0216, 0x0000);
	write_cmos_sensor(0x3258, 0x0036);
	write_cmos_sensor(0x3260, 0x002A);
	write_cmos_sensor(0x3268, 0x0001);
	write_cmos_sensor(0x3270, 0x0024);
	write_cmos_sensor(0x3288, 0x0030);
	write_cmos_sensor(0x32A0, 0x0024);
	write_cmos_sensor(0x32C4, 0x0029);
	write_cmos_sensor(0x337C, 0x0066);
	write_cmos_sensor(0x3458, 0x002E);
	write_cmos_sensor(0x382E, 0x060F);
	write_cmos_sensor(0x3830, 0x0805);
	write_cmos_sensor(0x3832, 0x0605);
	write_cmos_sensor(0x3834, 0x0803);
	write_cmos_sensor(0x3840, 0x0064);
	write_cmos_sensor(0x3844, 0x00FF);
	write_cmos_sensor(0xF404, 0x0FF3);
	write_cmos_sensor(0xF424, 0x0000);
	write_cmos_sensor(0x0612, 0x0000);
  write_cmos_sensor(0x0100, 0x0100);

}


static kal_uint32 return_sensor_id(void)
{
    return ((read_cmos_sensor_byte(0x0000) << 8) | read_cmos_sensor_byte(0x0001));
}

/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	*sensorID : return the sensor ID
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;

    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
	spin_lock(&imgsensor_drv_lock);
	imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
	spin_unlock(&imgsensor_drv_lock);
	do {
	    *sensor_id = return_sensor_id();
	    if (*sensor_id == imgsensor_info.sensor_id) {
#ifdef CONFIG_MTK_CAM_CAL

#endif
				        LOG_INF("i2c write id: 0x%x, ReadOut sensor id: 0x%x, imgsensor_info.sensor_id:0x%x.\n", imgsensor.i2c_write_id, *sensor_id, imgsensor_info.sensor_id);
		return ERROR_NONE;
	    }
			      LOG_INF("Read sensor id fail, i2c write id: 0x%x, ReadOut sensor id: 0x%x, imgsensor_info.sensor_id:0x%x.\n", imgsensor.i2c_write_id, *sensor_id, imgsensor_info.sensor_id);
	    retry--;
	} while (retry > 0);
	i++;
	retry = 1;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
    /* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
    *sensor_id = 0xFFFFFFFF;
    return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
    /* const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2}; */
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint32 sensor_id = 0;
	  LOG_1;
    /* sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address */
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
	spin_lock(&imgsensor_drv_lock);
	  imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
	  spin_unlock(&imgsensor_drv_lock);
	  do {
	      sensor_id = return_sensor_id();
	      if (sensor_id == imgsensor_info.sensor_id) {
		LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
		break;
	     }
	    LOG_INF("Read sensor id fail, id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
	     retry--;
	  } while (retry > 0);
	  i++;
	  if (sensor_id == imgsensor_info.sensor_id)
	      break;
	  retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id)
	  return ERROR_SENSOR_CONNECT_FAIL;

    /* initail sequence write in  */
    sensor_init();

    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en = KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_en = KAL_FALSE;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}   /*  open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/

	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	/* imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	set_mirror_flip(IMAGE_NORMAL);
	mdelay(10);
	/*
	#ifdef FANPENGTAO
	int i = 0;

	for (i = 0; i < 10; i++) {
		LOG_INF("delay time = %d, the frame no = %d\n", i*10, read_cmos_sensor(0x0005));
		mdelay(10);
	}
	#endif*/
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap.max_framerate) {
		LOG_INF("capture30fps: use cap30FPS's setting: %d fps!\n", imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;

	} else
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {

		LOG_INF("cap115fps: use cap1's setting: %d fps!\n", imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;

	} else  {
		LOG_INF("Warning:=== current_fps %d fps is not support, so use cap1's setting\n", imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	set_mirror_flip(IMAGE_NORMAL);
	mdelay(10);


	return ERROR_NONE;
}	/* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	set_mirror_flip(IMAGE_NORMAL);


	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(IMAGE_NORMAL);

	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;
    sensor_resolution->SensorCustom1Width  = imgsensor_info.custom1.grabwindow_width;
    sensor_resolution->SensorCustom1Height     = imgsensor_info.custom1.grabwindow_height;

    sensor_resolution->SensorCustom2Width  = imgsensor_info.custom2.grabwindow_width;
    sensor_resolution->SensorCustom2Height     = imgsensor_info.custom2.grabwindow_height;

    sensor_resolution->SensorCustom3Width  = imgsensor_info.custom3.grabwindow_width;
    sensor_resolution->SensorCustom3Height     = imgsensor_info.custom3.grabwindow_height;

    sensor_resolution->SensorCustom4Width  = imgsensor_info.custom4.grabwindow_width;
    sensor_resolution->SensorCustom4Height     = imgsensor_info.custom4.grabwindow_height;

    sensor_resolution->SensorCustom5Width  = imgsensor_info.custom5.grabwindow_width;
    sensor_resolution->SensorCustom5Height     = imgsensor_info.custom5.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);


	/* sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; */ /* not use */
	/* sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10;  */ /* not use */
	/* imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate;  */ /* not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; /* inverse with datasheet */
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;


	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  /* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;
	#ifdef FPTPDAFSUPPORT
	sensor_info->PDAF_Support = 1;
	#else
	sensor_info->PDAF_Support = 0;
	#endif

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			  sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			  sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			  sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

			  break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			  sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
			  sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;
			  break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			  sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
			  sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
			  sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;
			  break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			  sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
			  sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
			  sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
			  break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			  sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
			  sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
			  sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
			  break;
		default:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}

	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			  preview(image_window, sensor_config_data);
			  break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			  capture(image_window, sensor_config_data);
			  break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			  normal_video(image_window, sensor_config_data);
			  break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			  hs_video(image_window, sensor_config_data);
			  break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			  slim_video(image_window, sensor_config_data);
			  break;
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) /* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else /* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if (framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			if (framerate == 300)
			{
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);

			} else
			{
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			}
			if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter) set_dummy();
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter) set_dummy();

			break;
		default:  /* coding with  preview scenario by default */
				frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
				spin_lock(&imgsensor_drv_lock);
				imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
				imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
				imgsensor.min_frame_length = imgsensor.frame_length;
				spin_unlock(&imgsensor_drv_lock);

				LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
				break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			  *framerate = imgsensor_info.pre.max_framerate;
			  break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			  *framerate = imgsensor_info.normal_video.max_framerate;
			  break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			  *framerate = imgsensor_info.cap.max_framerate;
			  break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			  *framerate = imgsensor_info.hs_video.max_framerate;
			  break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			  *framerate = imgsensor_info.slim_video.max_framerate;
			  break;
		default:
			  break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {

		 write_cmos_sensor(0x0600, 0x0002);
	} else {

		 write_cmos_sensor(0x0600, 0x0000);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
			     UINT8 *feature_para, UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
    UINT16 *feature_data_16 = (UINT16 *) feature_para;
    UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
    UINT32 *feature_data_32 = (UINT32 *) feature_para;
    unsigned long long *feature_data = (unsigned long long *) feature_para;

    SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
	SET_PD_BLOCK_INFO_T *PDAFinfo;

    LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
    case SENSOR_FEATURE_GET_PERIOD:
	*feature_return_para_16++ = imgsensor.line_length;
	*feature_return_para_16 = imgsensor.frame_length;
        *feature_para_len = 4;
	break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
	*feature_return_para_32 = imgsensor.pclk;
        *feature_para_len = 4;
	break;
    case SENSOR_FEATURE_SET_ESHUTTER:
	set_shutter(*feature_data);
	break;
    case SENSOR_FEATURE_SET_NIGHTMODE:
        night_mode((BOOL) * feature_data);
	break;
    case SENSOR_FEATURE_SET_GAIN:
	set_gain((UINT16) *feature_data);
	break;
    case SENSOR_FEATURE_SET_FLASHLIGHT:
	break;
    case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
	break;
    case SENSOR_FEATURE_SET_REGISTER:
	write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
	break;
    case SENSOR_FEATURE_GET_REGISTER:
	sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
	break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
        *feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
        *feature_para_len = 4;
	break;
    case SENSOR_FEATURE_SET_VIDEO_MODE:
	set_video_mode(*feature_data);
	break;
    case SENSOR_FEATURE_CHECK_SENSOR_ID:
	get_imgsensor_id(feature_return_para_32);
	break;
    case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
        set_auto_flicker_mode((BOOL)*feature_data_16, *(feature_data_16+1));
	break;
    case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
	set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
	break;
    case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
	get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
	break;
    case SENSOR_FEATURE_SET_TEST_PATTERN:
	set_test_pattern_mode((BOOL)*feature_data);
	break;
    case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: /* for factory mode auto testing */
	*feature_return_para_32 = imgsensor_info.checksum_value;
        *feature_para_len = 4;
	break;
    case SENSOR_FEATURE_SET_FRAMERATE:
	LOG_INF("current fps :%d\n", (UINT32)*feature_data);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_fps = *feature_data;
	spin_unlock(&imgsensor_drv_lock);
	break;
    case SENSOR_FEATURE_SET_HDR:
	LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.ihdr_en = (BOOL)*feature_data;
	spin_unlock(&imgsensor_drv_lock);
	break;
    case SENSOR_FEATURE_GET_CROP_INFO:
	LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
	wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
	switch (*feature_data_32) {
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1], sizeof(SENSOR_WINSIZE_INFO_STRUCT));
	    break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2], sizeof(SENSOR_WINSIZE_INFO_STRUCT));
	    break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3], sizeof(SENSOR_WINSIZE_INFO_STRUCT));
	    break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
            memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4], sizeof(SENSOR_WINSIZE_INFO_STRUCT));
	    break;
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	default:
            memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof(SENSOR_WINSIZE_INFO_STRUCT));
	    break;
	}
	      break;
    case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
        LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n", (UINT16)*feature_data, (UINT16)*(feature_data+1), (UINT16)*(feature_data+2));
	/* ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2)); */
	break;
	/******************** PDAF START >>> *********/
		case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
			  LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%llu\n", *feature_data);
			  /* PDAF capacity enable or not, 2p8 only full size support PDAF */
			  switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					  *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
					  break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					  *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; /* video & capture use same setting */
					  break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					  *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					  break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					  *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					  break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					  *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					  break;
				default:
					  *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					  break;
			  }
			  break;
		case SENSOR_FEATURE_GET_PDAF_INFO:
			  LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%llu\n", *feature_data);
			  PDAFinfo = (SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));

			  switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			  memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info, sizeof(SET_PD_BLOCK_INFO_T));
				break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
				break;
			}
			break;
		case SENSOR_FEATURE_GET_PDAF_DATA:
			LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
			/* S5K2T7_read_eeprom((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2))); */
			break;
      /******************** PDAF END   <<< *********/
      default:
      break;
    }

    return ERROR_NONE;
}    /*    feature_control()  */


static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};


UINT32 S5K2X7_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}	/*	S5K2X7_MIPI_RAW_SensorInit	*/



