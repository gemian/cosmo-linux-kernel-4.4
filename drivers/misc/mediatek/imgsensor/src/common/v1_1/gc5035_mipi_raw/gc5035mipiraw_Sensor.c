/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 sp5509mipiraw_sensor.c
 *
 * Project:
 * --------

 * Description:

 * ------------
 *	 Source code of Sensor driver
 *
 *	PengtaoFan

 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#define DEBUG
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <asm/system.h>
//#include <linux/xlog.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "gc5035mipiraw_Sensor.h"

/****************************Modify Following Strings for Debug****************************/
#define PFX "gc5035_camera_sensor"
//#define LOG_WRN(format, args...) xlog_printk(ANDROID_LOG_WARN ,PFX, "[%S] " format, __FUNCTION__, ##args)
//#defineLOG_INF(format, args...) xlog_printk(ANDROID_LOG_INFO ,PFX, "[%s] " format, __FUNCTION__, ##args)
#define LOG_INF(fmt, args...)	pr_debug(PFX "[%s] " fmt, __FUNCTION__, ##args)
/****************************   Modify end    *******************************************/

static DEFINE_SPINLOCK(imgsensor_drv_lock);
static kal_uint32 Dgain_ratio = GC5035_SENSOR_DGAIN_BASE;

static imgsensor_info_struct imgsensor_info = { 
	.sensor_id = GC5035_SENSOR_ID,		//Sensor ID Value: 0x30C8//record sensor id defined in Kd_imgsensor.h
	
	.checksum_value = 0xcde448ca,		//checksum value for Camera Auto Test

	.pre = {
		.pclk = 87600000,				//record different mode's pclk
		.linelength  = 1460,				//record different mode's linelength
		.framelength = 2008,			//record different mode's framelength
		.startx= 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 1296,		//record different mode's width of grabwindow
		.grabwindow_height = 972,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
#if 0//def NONCONTINUEMODE
	.cap = {
		.pclk = 176000000,				//record different mode's pclk
		.linelength  = 2816,//5808,				//record different mode's linelength
		.framelength = 2083,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 2592,		//record different mode's width of grabwindow
		.grabwindow_height = 1944,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 14,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
#else //CONTINUEMODE
	.cap = {
		.pclk = 175200000,				//record different mode's pclk
		.linelength  = 2920,//5808,				//record different mode's linelength
		.framelength = 2008,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 2592,		//record different mode's width of grabwindow
		.grabwindow_height = 1944,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
#endif
#if 1 //fps 15
	.cap1 = {							//capture for PIP 15ps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
		.pclk = 141600000,				//record different mode's pclk
		.linelength  = 2920,//,				//record different mode's linelength
		.framelength = 2008,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 2592,		//record different mode's width of grabwindow
		.grabwindow_height = 1944,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 240,	
	},
#endif
	.normal_video = {
		.pclk = 87600000,				//record different mode's pclk
		.linelength  = 1460,//5808,				//record different mode's linelength
		.framelength = 2008,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 1296,		//record different mode's width of grabwindow
		.grabwindow_height = 972,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.hs_video = {
		.pclk = 175200000,				//record different mode's pclk
		.linelength  = 1896,				//record different mode's linelength
		.framelength = 1536,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 1280,		//record different mode's width of grabwindow
		.grabwindow_height = 720,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 600,	
	},
	.slim_video = {
		.pclk = 87600000,				//record different mode's pclk
		.linelength  = 1460,				//record different mode's linelength
		.framelength = 2008,			//record different mode's framelength
		.startx= 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 1280,		//record different mode's width of grabwindow
		.grabwindow_height = 720,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
  .custom1 = {
		.pclk = 175200000,				//record different mode's pclk
		.linelength  = 2920,//5808,				//record different mode's linelength
		.framelength = 2008,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 2592,		//record different mode's width of grabwindow
		.grabwindow_height = 1944,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
  .custom2 = {
		.pclk = 175200000,				//record different mode's pclk
		.linelength  = 2920,//5808,				//record different mode's linelength
		.framelength = 2008,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 2592,		//record different mode's width of grabwindow
		.grabwindow_height = 1944,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,		
	},
  .custom3 = {
		.pclk = 175200000,				//record different mode's pclk
		.linelength  = 2920,//5808,				//record different mode's linelength
		.framelength = 2008,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 2592,		//record different mode's width of grabwindow
		.grabwindow_height = 1944,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
  .custom4 = {
		.pclk = 175200000,				//record different mode's pclk
		.linelength  = 2920,//5808,				//record different mode's linelength
		.framelength = 2008,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 2592,		//record different mode's width of grabwindow
		.grabwindow_height = 1944,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,		
	},
  .custom5 = {
		.pclk = 175200000,				//record different mode's pclk
		.linelength  = 2920,//5808,				//record different mode's linelength
		.framelength = 2008,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 2592,		//record different mode's width of grabwindow
		.grabwindow_height = 1944,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},

	.margin = 16,			//sensor framelength & shutter margin
	.min_shutter = 4,		//min shutter
	.max_frame_length = 0x3fff,//REG0x0202 <=REG0x0340-5//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,	//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 3,	  //support sensor mode num ,don't support Slow motion
	
	.cap_delay_frame = 2,		//enter capture delay frame num
	.pre_delay_frame = 2, 		//enter preview delay frame num
	.video_delay_frame = 2,		//enter video delay frame num
	.hs_video_delay_frame = 2,	//enter high speed video  delay frame num
	.slim_video_delay_frame = 2,//enter slim video delay frame num
    .custom1_delay_frame = 2,
    .custom2_delay_frame = 2, 
    .custom3_delay_frame = 2, 
    .custom4_delay_frame = 2, 
    .custom5_delay_frame = 2,
	
	.isp_driving_current = ISP_DRIVING_8MA, //mclk driving current
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = 1,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,//SENSOR_OUTPUT_FORMAT_RAW_Gr,//sensor output first pixel color
	.mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num = SENSOR_MIPI_2_LANE,//mipi lane num
	.i2c_addr_table = {0x6e, 0x7e,0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
    .i2c_speed = 300, // i2c read/write speed
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x258,					//current shutter
	.gain = 0x40,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = KAL_FALSE, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x6e,//record current sensor's i2c write id
};


/* Sensor output window information*/

static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =	 
{
	{ 2592, 1944,   0,   0, 2592, 1944, 1296,  972, 0, 0, 1296,  972, 0, 0, 1296,  972 },
	{ 2592, 1944,   0,   0, 2592, 1944, 2592, 1944, 0, 0, 2592, 1944, 0, 0, 2592, 1944 },
	{ 2592, 1944,   0,   0, 2592, 1944, 1296,  972, 0, 0, 1296,  972, 0, 0, 1296,  972 },
	{ 2592, 1944, 656, 492, 1280,  960,  640,  480, 0, 0,  640,  480, 0, 0,  640,  480 },
	{ 2592, 1944,  16, 252, 2560, 1440, 1280,  720, 0, 0, 1280,  720, 0, 0, 1280,  720 }
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[1] = { (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 1, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}
static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pu_send_cmd[2] = { (char)(addr & 0xFF), (char)(para & 0xFF) };

	iWriteRegI2C(pu_send_cmd, 2, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	kal_uint32 frame_length = imgsensor.frame_length >> 2;

	frame_length = frame_length << 2;
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x41, (frame_length >> 8) & 0x3f);
	write_cmos_sensor(0x42, frame_length & 0xff);
	LOG_INF("Exit! framelength = %d\n", frame_length);
}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;

	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length)
		? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
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
	kal_uint16 realtime_fps = 0, cal_shutter = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/*if shutter bigger than frame_length, should extend frame length first*/
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
		(imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;

	if (imgsensor.autoflicker_en) {
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else
			set_max_framerate(realtime_fps, 0);
	} else
		set_max_framerate(realtime_fps, 0);

	cal_shutter = shutter >> 2;
	cal_shutter = cal_shutter << 2;
	Dgain_ratio = 256 * shutter / cal_shutter;

	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x03, (cal_shutter >> 8) & 0x3F);
	write_cmos_sensor(0x04, cal_shutter & 0xFF);

	LOG_INF("Exit! shutter = %d, framelength = %d\n", shutter, imgsensor.frame_length);
	LOG_INF("Exit! cal_shutter = %d, ", cal_shutter);
}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = gain << 2;

	if (reg_gain < GC5035_SENSOR_GAIN_BASE)
		reg_gain = GC5035_SENSOR_GAIN_BASE;
	else if (reg_gain > GC5035_SENSOR_GAIN_MAX)
		reg_gain = GC5035_SENSOR_GAIN_MAX;

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
	kal_uint32 temp_gain;
	kal_int16 gain_index;
	kal_uint16 GC5035_AGC_Param[GC5035_SENSOR_GAIN_MAP_SIZE][2] = {
		{  256,  0 },
		{  302,  1 },
		{  358,  2 },
		{  425,  3 },
		{  502,  8 },
		{  599,  9 },
		{  717, 10 },
		{  845, 11 },
		{  998, 12 },
		{ 1203, 13 },
		{ 1434, 14 },
		{ 1710, 15 },
		{ 1997, 16 },
		{ 2355, 17 },
		{ 2816, 18 },
		{ 3318, 19 },
		{ 3994, 20 },
	};

	reg_gain = gain2reg(gain);

	for (gain_index = GC5035_SENSOR_GAIN_MAX_VALID_INDEX - 1; gain_index >= 0; gain_index--)
		if (reg_gain >= GC5035_AGC_Param[gain_index][0])
			break;

	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xb6, GC5035_AGC_Param[gain_index][1]);
	temp_gain = reg_gain * Dgain_ratio / GC5035_AGC_Param[gain_index][0];
	write_cmos_sensor(0xb1, (temp_gain >> 8) & 0x0f);
	write_cmos_sensor(0xb2, temp_gain & 0xfc);
	LOG_INF("Exit! GC5035_AGC_Param[gain_index][1] = 0x%x, temp_gain = 0x%x, reg_gain = %d\n",
		GC5035_AGC_Param[gain_index][1], temp_gain, reg_gain);

	return reg_gain;
}	/*	set_gain  */

//[TODO]ihdr_write_shutter_gain not support for sp5509
static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le: 0x%x, se: 0x%x, gain: 0x%x\n", le, se, gain);	
}

#if 0
//[TODO]
static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	/********************************************************
	   *
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/
	spin_lock(&imgsensor_drv_lock);
    imgsensor.mirror= image_mirror; 
    spin_unlock(&imgsensor_drv_lock);
	switch (image_mirror) {
		case IMAGE_NORMAL:
			write_cmos_sensor(0x000e,0X0000); //B
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor(0x000e,0X0100); //Gb
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor(0x000e,0X0200); //Gr	
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor(0x000e,0X0300); //R
			break;
		default:
			LOG_INF("Error image_mirror setting\n");
	}

}
#endif
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
	/* SYSTEM */
	write_cmos_sensor(0xfc, 0x01);
	write_cmos_sensor(0xf4, 0x40);
	write_cmos_sensor(0xf5, 0xe9);
	write_cmos_sensor(0xf6, 0x14);
	write_cmos_sensor(0xf8, 0x49);
	write_cmos_sensor(0xf9, 0x82);
	write_cmos_sensor(0xfa, 0x00);
	write_cmos_sensor(0xfc, 0x81);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x36, 0x01);
	write_cmos_sensor(0xd3, 0x87);
	write_cmos_sensor(0x36, 0x00);
	write_cmos_sensor(0x33, 0x00);
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x01, 0xe7);
	write_cmos_sensor(0xf7, 0x01);
	write_cmos_sensor(0xfc, 0x8f);
	write_cmos_sensor(0xfc, 0x8f);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xee, 0x30);
	write_cmos_sensor(0x87, 0x18);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x90);
	write_cmos_sensor(0xfe, 0x00);

	/* Analog & CISCTL */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x05, 0x02);
	write_cmos_sensor(0x06, 0xda);
	write_cmos_sensor(0x9d, 0x0c);
	write_cmos_sensor(0x09, 0x00);
	write_cmos_sensor(0x0a, 0x04);
	write_cmos_sensor(0x0b, 0x00);
	write_cmos_sensor(0x0c, 0x03);
	write_cmos_sensor(0x0d, 0x07);
	write_cmos_sensor(0x0e, 0xa8);
	write_cmos_sensor(0x0f, 0x0a);
	write_cmos_sensor(0x10, 0x30);
	write_cmos_sensor(0x11, 0x02);
	write_cmos_sensor(0x17, GC5035_MIRROR);
	write_cmos_sensor(0x19, 0x05);
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x30, 0x03);
	write_cmos_sensor(0x31, 0x03);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xd9, 0xc0);
	write_cmos_sensor(0x1b, 0x20);
	write_cmos_sensor(0x21, 0x48);
	write_cmos_sensor(0x28, 0x22);
	write_cmos_sensor(0x29, 0x58);
	write_cmos_sensor(0x44, 0x20);
	write_cmos_sensor(0x4b, 0x10);
	write_cmos_sensor(0x4e, 0x1a);
	write_cmos_sensor(0x50, 0x11);
	write_cmos_sensor(0x52, 0x33);
	write_cmos_sensor(0x53, 0x44);
	write_cmos_sensor(0x55, 0x10);
	write_cmos_sensor(0x5b, 0x11);
	write_cmos_sensor(0xc5, 0x02);
	write_cmos_sensor(0x8c, 0x1a);
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x33, 0x05);
	write_cmos_sensor(0x32, 0x38);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x91, 0x80);
	write_cmos_sensor(0x92, 0x28);
	write_cmos_sensor(0x93, 0x20);
	write_cmos_sensor(0x95, 0xa0);
	write_cmos_sensor(0x96, 0xe0);
	write_cmos_sensor(0xd5, 0xfc);
	write_cmos_sensor(0x97, 0x28);
	write_cmos_sensor(0x16, 0x0c);
	write_cmos_sensor(0x1a, 0x1a);
	write_cmos_sensor(0x1f, 0x11);
	write_cmos_sensor(0x20, 0x10);
	write_cmos_sensor(0x46, 0xe3);
	write_cmos_sensor(0x4a, 0x04);
	write_cmos_sensor(0x54, GC5035_RSTDUMMY1);
	write_cmos_sensor(0x62, 0x00);
	write_cmos_sensor(0x72, 0xcf);
	write_cmos_sensor(0x73, 0xc9);
	write_cmos_sensor(0x7a, 0x05);
	write_cmos_sensor(0x7d, 0xcc);
	write_cmos_sensor(0x90, 0x00);
	write_cmos_sensor(0xce, 0x98);
	write_cmos_sensor(0xd0, 0xb2);
	write_cmos_sensor(0xd2, 0x40);
	write_cmos_sensor(0xe6, 0xe0);
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x12, 0x01);
	write_cmos_sensor(0x13, 0x01);
	write_cmos_sensor(0x14, 0x01);
	write_cmos_sensor(0x15, 0x02);
	write_cmos_sensor(0x22, GC5035_RSTDUMMY2);
	write_cmos_sensor(0x91, 0x00);
	write_cmos_sensor(0x92, 0x00);
	write_cmos_sensor(0x93, 0x00);
	write_cmos_sensor(0x94, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);

	/* Gain */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xb0, 0x6e);
	write_cmos_sensor(0xb1, 0x01);
	write_cmos_sensor(0xb2, 0x00);
	write_cmos_sensor(0xb3, 0x00);
	write_cmos_sensor(0xb4, 0x00);
	write_cmos_sensor(0xb6, 0x00);

	/* ISP */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x53, 0x00);
	write_cmos_sensor(0x89, 0x03);
	write_cmos_sensor(0x60, 0x40);

	/* BLK */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x42, 0x21);
	write_cmos_sensor(0x49, 0x03);
	write_cmos_sensor(0x4a, 0xff);
	write_cmos_sensor(0x4b, 0xc0);
	write_cmos_sensor(0x55, 0x00);

	/* Anti_blooming */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x41, 0x28);
	write_cmos_sensor(0x4c, 0x00);
	write_cmos_sensor(0x4d, 0x00);
	write_cmos_sensor(0x4e, 0x3c);
	write_cmos_sensor(0x44, 0x08);
	write_cmos_sensor(0x48, 0x01);

	/* Crop */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x91, 0x00);
	write_cmos_sensor(0x92, 0x08);
	write_cmos_sensor(0x93, 0x00);
	write_cmos_sensor(0x94, 0x07);
	write_cmos_sensor(0x95, 0x07);
	write_cmos_sensor(0x96, 0x98);
	write_cmos_sensor(0x97, 0x0a);
	write_cmos_sensor(0x98, 0x20);
	write_cmos_sensor(0x99, 0x00);

	/* MIPI */
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x02, 0x57);
	write_cmos_sensor(0x03, 0xb7);
	write_cmos_sensor(0x15, 0x14);
	write_cmos_sensor(0x18, 0x0f);
	write_cmos_sensor(0x21, 0x22);
	write_cmos_sensor(0x22, 0x06);
	write_cmos_sensor(0x23, 0x48);
	write_cmos_sensor(0x24, 0x12);
	write_cmos_sensor(0x25, 0x28);
	write_cmos_sensor(0x26, 0x08);
	write_cmos_sensor(0x29, 0x06);
	write_cmos_sensor(0x2a, 0x58);
	write_cmos_sensor(0x2b, 0x08);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x10);

	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x01);

}	/*	sensor_init  */


static void preview_setting(void)
{
	LOG_INF("E\n");
	/* System */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x01);
	write_cmos_sensor(0xfc, 0x01);
	write_cmos_sensor(0xf4, 0x40);
	write_cmos_sensor(0xf5, 0xe4);
	write_cmos_sensor(0xf6, 0x14);
	write_cmos_sensor(0xf8, 0x49);
	write_cmos_sensor(0xf9, 0x12);
	write_cmos_sensor(0xfa, 0x01);
	write_cmos_sensor(0xfc, 0x81);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x36, 0x01);
	write_cmos_sensor(0xd3, 0x87);
	write_cmos_sensor(0x36, 0x00);
	write_cmos_sensor(0x33, 0x20);
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x01, 0x87);
	write_cmos_sensor(0xf7, 0x11);
	write_cmos_sensor(0xfc, 0x8f);
	write_cmos_sensor(0xfc, 0x8f);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xee, 0x30);
	write_cmos_sensor(0x87, 0x18);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x90);
	write_cmos_sensor(0xfe, 0x00);

	/* Analog & CISCTL */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x05, 0x02);
	write_cmos_sensor(0x06, 0xda);
	write_cmos_sensor(0x9d, 0x0c);
	write_cmos_sensor(0x09, 0x00);
	write_cmos_sensor(0x0a, 0x04);
	write_cmos_sensor(0x0b, 0x00);
	write_cmos_sensor(0x0c, 0x03);
	write_cmos_sensor(0x0d, 0x07);
	write_cmos_sensor(0x0e, 0xa8);
	write_cmos_sensor(0x0f, 0x0a);
	write_cmos_sensor(0x10, 0x30);
	write_cmos_sensor(0x21, 0x60);
	write_cmos_sensor(0x29, 0x30);
	write_cmos_sensor(0x44, 0x18);
	write_cmos_sensor(0x4e, 0x20);
	write_cmos_sensor(0x8c, 0x20);
	write_cmos_sensor(0x91, 0x15);
	write_cmos_sensor(0x92, 0x3a);
	write_cmos_sensor(0x93, 0x20);
	write_cmos_sensor(0x95, 0x45);
	write_cmos_sensor(0x96, 0x35);
	write_cmos_sensor(0xd5, 0xf0);
	write_cmos_sensor(0x97, 0x20);
	write_cmos_sensor(0x1f, 0x19);
	write_cmos_sensor(0xce, 0x9b);
	write_cmos_sensor(0xd0, 0xb3);
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x14, 0x02);
	write_cmos_sensor(0x15, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);

	/* BLK */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x49, 0x00);
	write_cmos_sensor(0x4a, 0x01);
	write_cmos_sensor(0x4b, 0xf8);

	/* Anti_blooming */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x4e, 0x06);
	write_cmos_sensor(0x44, 0x02);

	/* Crop */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x91, 0x00);
	write_cmos_sensor(0x92, 0x04);
	write_cmos_sensor(0x93, 0x00);
	write_cmos_sensor(0x94, 0x03);
	write_cmos_sensor(0x95, 0x03);
	write_cmos_sensor(0x96, 0xcc);
	write_cmos_sensor(0x97, 0x05);
	write_cmos_sensor(0x98, 0x10);
	write_cmos_sensor(0x99, 0x00);

	/* MIPI */
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x02, 0x58);
	write_cmos_sensor(0x22, 0x03);
	write_cmos_sensor(0x26, 0x06);
	write_cmos_sensor(0x29, 0x03);
	write_cmos_sensor(0x2b, 0x06);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x10);

	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x91);
}	/*	preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps: %d\n", currefps);
	/* System */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x01);
	write_cmos_sensor(0xfc, 0x01);
	write_cmos_sensor(0xf4, 0x40);
	if (currefps == 240) { /* PIP */
		write_cmos_sensor(0xf5, 0xe7);
		write_cmos_sensor(0xf6, 0x14);
		write_cmos_sensor(0xf8, 0x3b);
	} else {
		write_cmos_sensor(0xf5, 0xe9);
		write_cmos_sensor(0xf6, 0x14);
		write_cmos_sensor(0xf8, 0x49);
	}
	write_cmos_sensor(0xf9, 0x82);
	write_cmos_sensor(0xfa, 0x00);
	write_cmos_sensor(0xfc, 0x81);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x36, 0x01);
	write_cmos_sensor(0xd3, 0x87);
	write_cmos_sensor(0x36, 0x00);
	write_cmos_sensor(0x33, 0x00);
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x01, 0xe7);
	write_cmos_sensor(0xf7, 0x01);
	write_cmos_sensor(0xfc, 0x8f);
	write_cmos_sensor(0xfc, 0x8f);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xee, 0x30);
	write_cmos_sensor(0x87, 0x18);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x90);
	write_cmos_sensor(0xfe, 0x00);

	/* Analog & CISCTL */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x05, 0x02);
	write_cmos_sensor(0x06, 0xda);
	write_cmos_sensor(0x9d, 0x0c);
	write_cmos_sensor(0x09, 0x00);
	write_cmos_sensor(0x0a, 0x04);
	write_cmos_sensor(0x0b, 0x00);
	write_cmos_sensor(0x0c, 0x03);
	write_cmos_sensor(0x0d, 0x07);
	write_cmos_sensor(0x0e, 0xa8);
	write_cmos_sensor(0x0f, 0x0a);
	write_cmos_sensor(0x10, 0x30);
	write_cmos_sensor(0x21, 0x48);
	write_cmos_sensor(0x29, 0x58);
	write_cmos_sensor(0x44, 0x20);
	write_cmos_sensor(0x4e, 0x1a);
	write_cmos_sensor(0x8c, 0x1a);
	write_cmos_sensor(0x91, 0x80);
	write_cmos_sensor(0x92, 0x28);
	write_cmos_sensor(0x93, 0x20);
	write_cmos_sensor(0x95, 0xa0);
	write_cmos_sensor(0x96, 0xe0);
	write_cmos_sensor(0xd5, 0xfc);
	write_cmos_sensor(0x97, 0x28);
	write_cmos_sensor(0x1f, 0x11);
	write_cmos_sensor(0xce, 0x98);
	write_cmos_sensor(0xd0, 0xb2);
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x14, 0x01);
	write_cmos_sensor(0x15, 0x02);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);

	/* BLK */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x49, 0x03);
	write_cmos_sensor(0x4a, 0xff);
	write_cmos_sensor(0x4b, 0xc0);

	/* Anti_blooming */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x4e, 0x3c);
	write_cmos_sensor(0x44, 0x08);

	/* Crop */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x91, 0x00);
	write_cmos_sensor(0x92, 0x08);
	write_cmos_sensor(0x93, 0x00);
	write_cmos_sensor(0x94, 0x07);
	write_cmos_sensor(0x95, 0x07);
	write_cmos_sensor(0x96, 0x98);
	write_cmos_sensor(0x97, 0x0a);
	write_cmos_sensor(0x98, 0x20);
	write_cmos_sensor(0x99, 0x00);

	/* MIPI */
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x02, 0x57);
	write_cmos_sensor(0x22, 0x06);
	write_cmos_sensor(0x26, 0x08);
	write_cmos_sensor(0x29, 0x06);
	write_cmos_sensor(0x2b, 0x08);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x10);

	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x91);
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps: %d\n", currefps);
	/* System */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x01);
	write_cmos_sensor(0xfc, 0x01);
	write_cmos_sensor(0xf4, 0x40);
	write_cmos_sensor(0xf5, 0xe4);
	write_cmos_sensor(0xf6, 0x14);
	write_cmos_sensor(0xf8, 0x49);
	write_cmos_sensor(0xf9, 0x12);
	write_cmos_sensor(0xfa, 0x01);
	write_cmos_sensor(0xfc, 0x81);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x36, 0x01);
	write_cmos_sensor(0xd3, 0x87);
	write_cmos_sensor(0x36, 0x00);
	write_cmos_sensor(0x33, 0x20);
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x01, 0x87);
	write_cmos_sensor(0xf7, 0x11);
	write_cmos_sensor(0xfc, 0x8f);
	write_cmos_sensor(0xfc, 0x8f);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xee, 0x30);
	write_cmos_sensor(0x87, 0x18);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x90);
	write_cmos_sensor(0xfe, 0x00);

	/* Analog & CISCTL */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x05, 0x02);
	write_cmos_sensor(0x06, 0xda);
	write_cmos_sensor(0x9d, 0x0c);
	write_cmos_sensor(0x09, 0x00);
	write_cmos_sensor(0x0a, 0x04);
	write_cmos_sensor(0x0b, 0x00);
	write_cmos_sensor(0x0c, 0x03);
	write_cmos_sensor(0x0d, 0x07);
	write_cmos_sensor(0x0e, 0xa8);
	write_cmos_sensor(0x0f, 0x0a);
	write_cmos_sensor(0x10, 0x30);
	write_cmos_sensor(0x21, 0x60);
	write_cmos_sensor(0x29, 0x30);
	write_cmos_sensor(0x44, 0x18);
	write_cmos_sensor(0x4e, 0x20);
	write_cmos_sensor(0x8c, 0x20);
	write_cmos_sensor(0x91, 0x15);
	write_cmos_sensor(0x92, 0x3a);
	write_cmos_sensor(0x93, 0x20);
	write_cmos_sensor(0x95, 0x45);
	write_cmos_sensor(0x96, 0x35);
	write_cmos_sensor(0xd5, 0xf0);
	write_cmos_sensor(0x97, 0x20);
	write_cmos_sensor(0x1f, 0x19);
	write_cmos_sensor(0xce, 0x9b);
	write_cmos_sensor(0xd0, 0xb3);
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x14, 0x02);
	write_cmos_sensor(0x15, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);

	/* BLK */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x49, 0x00);
	write_cmos_sensor(0x4a, 0x01);
	write_cmos_sensor(0x4b, 0xf8);

	/* Anti_blooming */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x4e, 0x06);
	write_cmos_sensor(0x44, 0x02);

	/* Crop */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x91, 0x00);
	write_cmos_sensor(0x92, 0x04);
	write_cmos_sensor(0x93, 0x00);
	write_cmos_sensor(0x94, 0x03);
	write_cmos_sensor(0x95, 0x03);
	write_cmos_sensor(0x96, 0xcc);
	write_cmos_sensor(0x97, 0x05);
	write_cmos_sensor(0x98, 0x10);
	write_cmos_sensor(0x99, 0x00);

	/* MIPI */
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x02, 0x58);
	write_cmos_sensor(0x22, 0x03);
	write_cmos_sensor(0x26, 0x06);
	write_cmos_sensor(0x29, 0x03);
	write_cmos_sensor(0x2b, 0x06);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x10);

	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x91);
}

static void hs_video_setting(void)
{
	LOG_INF("E\n");
	/* System */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x01);
	write_cmos_sensor(0xfc, 0x01);
	write_cmos_sensor(0xf4, 0x40);
	write_cmos_sensor(0xf5, 0xe9);
	write_cmos_sensor(0xf6, 0x14);
	write_cmos_sensor(0xf8, 0x49);
	write_cmos_sensor(0xf9, 0x82);
	write_cmos_sensor(0xfa, 0x00);
	write_cmos_sensor(0xfc, 0x81);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x36, 0x01);
	write_cmos_sensor(0xd3, 0x87);
	write_cmos_sensor(0x36, 0x00);
	write_cmos_sensor(0x33, 0x20);
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x01, 0x87);
	write_cmos_sensor(0xf7, 0x11);
	write_cmos_sensor(0xfc, 0x8f);
	write_cmos_sensor(0xfc, 0x8f);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xee, 0x30);
	write_cmos_sensor(0x87, 0x18);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x90);
	write_cmos_sensor(0xfe, 0x00);

	/* Analog & CISCTL */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x05, 0x03);
	write_cmos_sensor(0x06, 0xb4);
	write_cmos_sensor(0x9d, 0x20);
	write_cmos_sensor(0x09, 0x00);
	write_cmos_sensor(0x0a, 0xf4);
	write_cmos_sensor(0x0b, 0x00);
	write_cmos_sensor(0x0c, 0x03);
	write_cmos_sensor(0x0d, 0x05);
	write_cmos_sensor(0x0e, 0xc8);
	write_cmos_sensor(0x0f, 0x0a);
	write_cmos_sensor(0x10, 0x30);
	write_cmos_sensor(0xd9, 0xf8);
	write_cmos_sensor(0x21, 0xe0);
	write_cmos_sensor(0x29, 0x40);
	write_cmos_sensor(0x44, 0x30);
	write_cmos_sensor(0x4e, 0x20);
	write_cmos_sensor(0x8c, 0x20);
	write_cmos_sensor(0x91, 0x15);
	write_cmos_sensor(0x92, 0x3a);
	write_cmos_sensor(0x93, 0x20);
	write_cmos_sensor(0x95, 0x45);
	write_cmos_sensor(0x96, 0x35);
	write_cmos_sensor(0xd5, 0xf0);
	write_cmos_sensor(0x97, 0x20);
	write_cmos_sensor(0x1f, 0x19);
	write_cmos_sensor(0xce, 0x9b);
	write_cmos_sensor(0xd0, 0xb3);
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x14, 0x02);
	write_cmos_sensor(0x15, 0x00);
	write_cmos_sensor(0x91, 0x00);
	write_cmos_sensor(0x92, 0xf0);
	write_cmos_sensor(0x93, 0x00);
	write_cmos_sensor(0x94, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);

	/* BLK */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x49, 0x00);
	write_cmos_sensor(0x4a, 0x01);
	write_cmos_sensor(0x4b, 0xf8);

	/* Anti_blooming */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x4e, 0x06);
	write_cmos_sensor(0x44, 0x02);

	/* Crop */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x91, 0x00);
	write_cmos_sensor(0x92, 0x0a);
	write_cmos_sensor(0x93, 0x00);
	write_cmos_sensor(0x94, 0x0b);
	write_cmos_sensor(0x95, 0x02);
	write_cmos_sensor(0x96, 0xd0);
	write_cmos_sensor(0x97, 0x05);
	write_cmos_sensor(0x98, 0x00);
	write_cmos_sensor(0x99, 0x00);

	/* MIPI */
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x02, 0x58);
	write_cmos_sensor(0x22, 0x03);
	write_cmos_sensor(0x26, 0x06);
	write_cmos_sensor(0x29, 0x03);
	write_cmos_sensor(0x2b, 0x06);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x10);

	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x91);
}


static void slim_video_setting(void)
{
	LOG_INF("E\n");
	/* System */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x01);
	write_cmos_sensor(0xfc, 0x01);
	write_cmos_sensor(0xf4, 0x40);
	write_cmos_sensor(0xf5, 0xe4);
	write_cmos_sensor(0xf6, 0x14);
	write_cmos_sensor(0xf8, 0x49);
	write_cmos_sensor(0xf9, 0x12);
	write_cmos_sensor(0xfa, 0x01);
	write_cmos_sensor(0xfc, 0x81);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x36, 0x01);
	write_cmos_sensor(0xd3, 0x87);
	write_cmos_sensor(0x36, 0x00);
	write_cmos_sensor(0x33, 0x20);
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x01, 0x87);
	write_cmos_sensor(0xf7, 0x11);
	write_cmos_sensor(0xfc, 0x8f);
	write_cmos_sensor(0xfc, 0x8f);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xee, 0x30);
	write_cmos_sensor(0x87, 0x18);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x90);
	write_cmos_sensor(0xfe, 0x00);

	/* Analog & CISCTL */
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x05, 0x02);
	write_cmos_sensor(0x06, 0xda);
	write_cmos_sensor(0x9d, 0x0c);
	write_cmos_sensor(0x09, 0x00);
	write_cmos_sensor(0x0a, 0x04);
	write_cmos_sensor(0x0b, 0x00);
	write_cmos_sensor(0x0c, 0x03);
	write_cmos_sensor(0x0d, 0x07);
	write_cmos_sensor(0x0e, 0xa8);
	write_cmos_sensor(0x0f, 0x0a);
	write_cmos_sensor(0x10, 0x30);
	write_cmos_sensor(0x21, 0x60);
	write_cmos_sensor(0x29, 0x30);
	write_cmos_sensor(0x44, 0x18);
	write_cmos_sensor(0x4e, 0x20);
	write_cmos_sensor(0x8c, 0x20);
	write_cmos_sensor(0x91, 0x15);
	write_cmos_sensor(0x92, 0x3a);
	write_cmos_sensor(0x93, 0x20);
	write_cmos_sensor(0x95, 0x45);
	write_cmos_sensor(0x96, 0x35);
	write_cmos_sensor(0xd5, 0xf0);
	write_cmos_sensor(0x97, 0x20);
	write_cmos_sensor(0x1f, 0x19);
	write_cmos_sensor(0xce, 0x9b);
	write_cmos_sensor(0xd0, 0xb3);
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x14, 0x02);
	write_cmos_sensor(0x15, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x88);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfc, 0x8e);

	/* BLK */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x49, 0x00);
	write_cmos_sensor(0x4a, 0x01);
	write_cmos_sensor(0x4b, 0xf8);

	/* Anti_blooming */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x4e, 0x06);
	write_cmos_sensor(0x44, 0x02);

	/* Crop */
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x91, 0x00);
	write_cmos_sensor(0x92, 0x0a);
	write_cmos_sensor(0x93, 0x00);
	write_cmos_sensor(0x94, 0x0b);
	write_cmos_sensor(0x95, 0x02);
	write_cmos_sensor(0x96, 0xd0);
	write_cmos_sensor(0x97, 0x05);
	write_cmos_sensor(0x98, 0x00);
	write_cmos_sensor(0x99, 0x00);

	/* MIPI */
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x02, 0x58);
	write_cmos_sensor(0x22, 0x03);
	write_cmos_sensor(0x26, 0x06);
	write_cmos_sensor(0x29, 0x03);
	write_cmos_sensor(0x2b, 0x06);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x8c, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3e, 0x91);
}


static kal_uint32 return_sensor_id(void)
{
    return ((read_cmos_sensor(0xf0) << 8) | read_cmos_sensor(0xf1));
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
//extern int mainsubcam_flag;

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
				//gc5035_otp_identify();
				pr_debug("[gc5035_camera_sensor]get_imgsensor_id:i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			pr_debug("[gc5035_camera_sensor]get_imgsensor_id:Read sensor id fail, write id: 0x%x, id: 0x%x\n",
				imgsensor.i2c_write_id, *sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}

	if (*sensor_id != imgsensor_info.sensor_id) {
		/*if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF*/
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
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				pr_debug("[gc5035_camera_sensor]open:i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			pr_debug("[gc5035_camera_sensor]open:Read sensor id fail, write id: 0x%x, id: 0x%x\n",
				imgsensor.i2c_write_id, sensor_id);
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

	//gc5035_otp_function();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
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
	/* No Need to implement this function */
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
	imgsensor.autoflicker_en = KAL_TRUE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();

	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */
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
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
		/* PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M */
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_TRUE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
				imgsensor.current_fps, imgsensor_info.cap.max_framerate / 10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_TRUE;
	}
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
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
	imgsensor.autoflicker_en = KAL_TRUE;
	spin_unlock(&imgsensor_drv_lock);

	normal_video_setting(imgsensor.current_fps);
	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */
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
	imgsensor.autoflicker_en = KAL_TRUE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_TRUE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	return ERROR_NONE;
}
	
/*************************************************************************
* FUNCTION
* Custom1
*
* DESCRIPTION
*   This function start the sensor Custom1.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
    imgsensor.pclk = imgsensor_info.custom1.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom1.linelength;
    imgsensor.frame_length = imgsensor_info.custom1.framelength; 
    imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}   /*  Custom1   */

static kal_uint32 Custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
    imgsensor.pclk = imgsensor_info.custom2.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom2.linelength;
    imgsensor.frame_length = imgsensor_info.custom2.framelength; 
    imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}   /*  Custom2   */

static kal_uint32 Custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
    imgsensor.pclk = imgsensor_info.custom3.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom3.linelength;
    imgsensor.frame_length = imgsensor_info.custom3.framelength; 
    imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}   /*  Custom3   */

static kal_uint32 Custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
    imgsensor.pclk = imgsensor_info.custom4.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom4.linelength;
    imgsensor.frame_length = imgsensor_info.custom4.framelength; 
    imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}   /*  Custom4   */
static kal_uint32 Custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
    imgsensor.pclk = imgsensor_info.custom5.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom5.linelength;
    imgsensor.frame_length = imgsensor_info.custom5.framelength; 
    imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}   /*  Custom5   */
static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;

	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("Enter get_info!\n");
	LOG_INF("scenario_id = %d\n", scenario_id);

	/*sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10;*/ /*not use*/
	/*sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10;*/    /*not use*/
	/*imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate;*/     /*not use*/

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;                  /*not use*/
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;                         /*inverse with datasheet*/
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;                                           /*not use*/
	sensor_info->SensorResetActiveHigh = FALSE;                                           /*not use*/
	sensor_info->SensorResetDelayCount = 5;                                               /*not use*/

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0;                                             /*not use*/
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	/*The frame of setting shutter default 0 for TG int*/
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
	/*The frame of setting sensor gain*/
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;                                               /*not use*/
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;                                             /*not use*/
	sensor_info->SensorPixelClockCount = 3;                                               /*not use*/
	sensor_info->SensorDataLatchCount = 2;                                                /*not use*/

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;                                                 /*0 is default 1x*/
	sensor_info->SensorHightSampling = 0;                                                 /*0 is default 1x*/
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.cap.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
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
        case MSDK_SCENARIO_ID_CUSTOM1:
            Custom1(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            Custom2(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            Custom3(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            Custom4(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            Custom5(image_window, sensor_config_data); // Custom1
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
	/*This Function not used after ROME*/
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	/***********
	 *if (framerate == 0)	 //Dynamic frame rate
	 *	return ERROR_NONE;
	 *spin_lock(&imgsensor_drv_lock);
	 *if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
	 *	imgsensor.current_fps = 296;
	 *else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
	 *	imgsensor.current_fps = 146;
	 *else
	 *	imgsensor.current_fps = framerate;
	 *spin_unlock(&imgsensor_drv_lock);
	 *set_max_framerate(imgsensor.current_fps, 1);
	 ********/
	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) /* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else        /* Cancel Auto flick */
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
			set_dummy();
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;			
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:	
			if(framerate==300)
			{
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			}
			else
			{
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			}
			break;	
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;	
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
			set_dummy();
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ? (frame_length - imgsensor_info.custom2.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
			set_dummy();
            break; 
        case MSDK_SCENARIO_ID_CUSTOM3:
            frame_length = imgsensor_info.custom3.pclk / framerate * 10 / imgsensor_info.custom3.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength) ? (frame_length - imgsensor_info.custom3.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
			set_dummy();
            break; 
        case MSDK_SCENARIO_ID_CUSTOM4:
            frame_length = imgsensor_info.custom4.pclk / framerate * 10 / imgsensor_info.custom4.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom4.framelength) ? (frame_length - imgsensor_info.custom4.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom4.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
			set_dummy();
            break; 
        case MSDK_SCENARIO_ID_CUSTOM5:
            frame_length = imgsensor_info.custom5.pclk / framerate * 10 / imgsensor_info.custom5.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom5.framelength) ? (frame_length - imgsensor_info.custom5.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;	
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
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
        case MSDK_SCENARIO_ID_CUSTOM1:
            *framerate = imgsensor_info.custom1.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            *framerate = imgsensor_info.custom2.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            *framerate = imgsensor_info.custom3.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            *framerate = imgsensor_info.custom4.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            *framerate = imgsensor_info.custom5.max_framerate;
            break;
		default:
			break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	write_cmos_sensor(0xfe, 0x01);
	if (enable)
		write_cmos_sensor(0x8c, 0x11);
	else
		write_cmos_sensor(0x8c, 0x10);
	write_cmos_sensor(0xfe, 0x00);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
    //unsigned long long *feature_return_para=(unsigned long long *) feature_para;

    SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
	//SET_PD_BLOCK_INFO_T *PDAFinfo;

    LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
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
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
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
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
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
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
            }
			break;
        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            break;
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


UINT32 GC5035_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	sp5509_MIPI_RAW_SensorInit	*/



