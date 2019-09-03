/* drivers/i2c/chips/AF6133.c - AF6133 compass driver
 *
 * Copyright (C) 2013 VTC Technology Inc.
 * Author: Gary Huang <gary.huang@voltafield.com>
 *         George Tseng <george.tseng@voltafield.com>
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
 * For Android 8.0
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
#include <asm/atomic.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/hrtimer.h>
#include <hwmsensor.h>
#include <sensors_io.h>
#include <linux/types.h>


#include <hwmsen_helper.h>
#include "cust_mag.h"
#include "mag.h"
#include "af6133.h"


/* Add for auto detect feature */
static int af6133_local_init(void);
static int af6133_remove(void);
static int af6133_open_report_data(int en);
static int af6133_set_delay(u64 delay);
static int af6133_enable(int en);
static int af6133_get_data(int *x,int *y, int *z,int *status);

static int af6133_init_flag = -1;  //0:ok,,-1:fail

static struct mag_init_info af6133_init_info = {
        .name   = "af6133",	
        .init   = af6133_local_init,
        .uninit = af6133_remove,	
};

/* driver information */
#define AF6133_DEV_NAME         "af6133"
#define DRIVER_VERSION          "1.8.1"
#define DRIVER_RELEASE          "20180130_1013"
/* delay time (ms)*/
#define AF6133_DELAY_MIN	5
#define AF6133_DELAY_MAX 	100
#define AF6133_DELAY_SW_GYRO	10
/* debug message */
#define MSE_TAG                 "MSENSOR"
#define MSE_FUN(f)              printk(MSE_TAG" %s\r\n", __FUNCTION__)
#define MSE_ERR(fmt, args...)   printk(KERN_ERR MSE_TAG" %s %d : \r\n"fmt, __FUNCTION__, __LINE__, ##args)
#define MSE_LOG(fmt, args...)   printk(MSE_TAG fmt, ##args)
#define MSE_VER(fmt, args...)   ((void)0)
/******************************************************************************/
static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

static atomic_t open_flag = ATOMIC_INIT(0);
static atomic_t m_flag = ATOMIC_INIT(0);
static atomic_t o_flag = ATOMIC_INIT(0);
/*----------------------------------------------------------------------------*/
/* Maintain  cust info here */
struct mag_hw af6133_mag_cust;
static struct mag_hw *hw = &af6133_mag_cust;

/* For  driver get cust info */
struct mag_hw *af6133_get_cust_mag(void)
{
	return &af6133_mag_cust;
}
/*----------------------------------------------------------------------------*/
static struct i2c_client *af6133_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id af6133_i2c_id[] = {{AF6133_DEV_NAME,0},{}};
#if 1//def CONFIG_MTK_LEGACY
//static struct i2c_board_info i2c_af6133 __initdata = { I2C_BOARD_INFO("af6133", 0x0C)};  //7-bit address
#endif
/*----------------------------------------------------------------------------*/
static int af6133_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int af6133_i2c_remove(struct i2c_client *client);
static int af6133_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
/*songyl*/
//static int af6133_suspend(struct i2c_client *client, pm_message_t msg) ;
//static int af6133_resume(struct i2c_client *client);
static int af6133_suspend(struct device *dev);
static int af6133_resume(struct device *dev);
/*songyl*/

/*----------------------------------------------------------------------------*/
typedef enum {
	VTC_TRC_DEBUG  = 0x01,
} AMI_TRC;
/*----------------------------------------------------------------------------*/
#define CONTROL_DATA_LEN	3
/*----------------------------------------------------------------------------*/
int af6133_offset[3];
int af6133_gain[3];
int af6133_comp[4];
int af6133_bist_x[3];
int af6133_bist_y[3];
int af6133_bist_z[3];
int af6133_temp;
int af6133_controldata[CONTROL_DATA_LEN];
/*----------------------------------------------------------------------------*/
struct af6133_i2c_data {
	struct i2c_client *client;
	struct mag_hw *hw;
	struct hwmsen_convert cvt;
	atomic_t layout;   
	atomic_t trace;
};
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF 
static const struct of_device_id mag_of_match[] = {
	{.compatible = "mediatek,msensor"},
	{},
};
#endif

/*songyl*/
#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops af6133_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(af6133_suspend, af6133_resume)
};
#endif
/*songyl*/
/*----------------------------------------------------------------------------*/
static struct i2c_driver af6133_i2c_driver = {
    .driver = {
        //.owner = THIS_MODULE, 
        .name  = AF6133_DEV_NAME,
#ifdef CONFIG_OF 
        .of_match_table = mag_of_match,
#endif
    },
	.probe      = af6133_i2c_probe,
	.remove     = af6133_i2c_remove,
	.detect     = af6133_i2c_detect,
//	.suspend    = af6133_suspend,
//	.resume     = af6133_resume,
	.id_table   = af6133_i2c_id,
};

static DEFINE_MUTEX(af6133_mutex);
#define I2C_FLAG_WRITE	0
#define I2C_FLAG_READ	1

/*----------------------------------------------------------------------------*/
static atomic_t dev_open_count;
/*----------------------------------------------------------------------------*/

//========================================================================
static void af6133_power(struct mag_hw *hw, unsigned int on)
{
}
/*----------------------------------------------------------------------------*/
static int VTC_i2c_Rx(struct i2c_client *client, char *rxData, int length)
{
        uint8_t retry;
        struct i2c_msg msgs[] = 
        {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
 			.buf = rxData,
		},
		{
			.addr = client->addr,
                    	.flags = I2C_M_RD,
                    	.len = length,
                    	.buf = rxData,
            	},
        };

        for (retry = 0; retry < 3; retry++) 
        {
                if (i2c_transfer(client->adapter, msgs, 2) > 0)
                        break;
                else
                        mdelay(10);
        }

        if (retry >= 3) 
        {
                printk(KERN_ERR "%s: retry over 3\n", __func__);
                return -EIO;
        } 
        else
                return 0;
}
/*----------------------------------------------------------------------------*/
static int VTC_i2c_Tx(struct i2c_client *client, char *txData, int length)
{
        int retry;
        struct i2c_msg msg[] = 
        {
                {
                        .addr = client->addr,
                        .flags = 0,
                        .len = length,
                        .buf = txData,
                },
        };

        for (retry = 0; retry <= 3; retry++) 
        {
                if (i2c_transfer(client->adapter, msg, 1) > 0)
                        break;
                else
                        mdelay(10);
        }

        if (retry > 3) 
        {
                printk(KERN_ERR "%s: retry over 3\n", __func__);
                return -EIO;
        }
        else
                return 0;
}
/*----------------------------------------------------------------------------*/
int af6133_i2c_master_operate(struct i2c_client *client, char *buf, int count, int i2c_flag)
{

	if(i2c_flag == I2C_FLAG_READ)
	{
	  	return (VTC_i2c_Rx(client, buf, count>>8));
	}
	else if(i2c_flag == I2C_FLAG_WRITE)
	{
	  	return (VTC_i2c_Tx(client, buf, count));
  }
  
  return 0;
}

/*----------------------------------------------------------------------------
static int af6133_GetOpenStatus(void)
{
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) != 0));
	return atomic_read(&open_flag);
}
----------------------------------------------------------------------------*/
static int af6133_GetSensorOffset(int16_t *offset)
{
	/* AF6133 B2 SET/RESET & Offset Cancelation (1/3) */
	uint8_t databuf[6];
	uint8_t i;
	int16_t S_Data[3], R_Data[3];
	int32_t tmp[3];
 
	if(NULL == af6133_i2c_client)
	{
		return -1;
	}   

  	for(i=0;i<3;i++)
    		tmp[i] = 0;  
  	
  /* reset reading */
	databuf[0] = REG_XY_SR;
	databuf[1] = 0x08;
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);    	
  	
	databuf[0] = REG_Z_SR;
	databuf[1] = 0x44;
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  

  	for(i=0;i<MAG_OFFSET_LOOP;i++)
  	{
	
 		databuf[0] = REG_MEASURE;
		databuf[1] = 0x01;
		af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  	
		mdelay(4);

		databuf[0] = REG_DATA;
		af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
		S_Data[0] = (databuf[1] << 8) | databuf[0];
		S_Data[1] = (databuf[3] << 8) | databuf[2];
		S_Data[2] = (databuf[5] << 8) | databuf[4];
  
		tmp[0] += (int32_t)(S_Data[0]);
		tmp[1] += (int32_t)(S_Data[1]);
		tmp[2] += (int32_t)(S_Data[2]);
	}
  
  /* set reading */
	databuf[0] = REG_XY_SR;
	databuf[1] = 0x04; 
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);    	
  	
	databuf[0] = REG_Z_SR;
	databuf[1] = 0x24;
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  

  	for(i=0;i<MAG_OFFSET_LOOP;i++)
  	{
 		databuf[0] = REG_MEASURE;
		databuf[1] = 0x01;
		af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  	
		mdelay(4);

		databuf[0] = REG_DATA2;
		af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
		R_Data[0] = (databuf[1] << 8) | databuf[0];
		R_Data[1] = (databuf[3] << 8) | databuf[2];
		R_Data[2] = (databuf[5] << 8) | databuf[4];
  
		tmp[0] += (int32_t)(R_Data[0]);
		tmp[1] += (int32_t)(R_Data[1]);
		tmp[2] += (int32_t)(R_Data[2]);
	}
      	
  	for(i=0;i<3;i++)
  	{
		offset[i] = (int16_t)(tmp[i] / (MAG_OFFSET_LOOP*2));
	} 
 
	return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133_otp_set_offset(int16_t *index)
{
	/* AF6133 B2 SET/RESET & Offset Cancelation (2/3) */  	
	int16_t	i;
	uint8_t databuf[2];
  	uint8_t otp;
	uint8_t flag; 
 
	if(NULL == af6133_i2c_client)
	{
		return -1;
	}   

  	/* OTP write setting */
	databuf[0] = REG_TEST0;
	databuf[1] = 0xA3; 
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);    

	databuf[0] = REG_SECURITY;
	databuf[1] = 0x56; 
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  

	databuf[0] = REG_SECURITY;
	databuf[1] = 0x54; 
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  

	databuf[0] = REG_SECURITY;
	databuf[1] = 0x43; 
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  

	databuf[0] = REG_SECURITY;
	databuf[1] = 0xAA; 
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  
  
  	/* set offset OTP */
  	for(i=0;i<3;i++)
  	{
    		if(index[i] < 0)
    		{
      			flag = 1; 
      			index[i] = -index[i];
    		} 
    		else
    		{
    			flag = 0;
    		}
   
    		otp = (((uint8_t)(index[i])) & 0x0F) | (flag ? 0x10 : 0x00);

        databuf[0] = REG_OTP_1D + i;
        databuf[1] = otp; 
        af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE); 
  	}

	databuf[0] = REG_TEST0;
	databuf[1] = 0x83;     
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133_SetSensorOffset(void)
{  	
	int16_t	i;
	int16_t index[3];
  int16_t offset[3], total_offset[3];
	uint8_t databuf[6];
	int16_t mag_ASIC[3] = {0};
	int16_t mag_TEMP = 0;
 
	if(NULL == af6133_i2c_client)
	{
		return -1;
	}   
  	
  /* get sensor offset 1 */
	if(af6133_GetSensorOffset(offset))
	{
		return -1;
	}

	for(i=0;i<3;i++)
	{ 
		index[i] = offset[i] / MAG_MIN_OFFSET;
		total_offset[i] = offset[i];
  	
		if(index[i] % 2)
		index[i] += ((index[i]>0) ? 1 : -1);

		index[i] /= 2;  
	}
  
	if(index[0] || index[1] || index[2])
	{    
  	// reduce sensor offset 1
		if(af6133_otp_set_offset(index))
		{
			return -1;
		}
        
     /* get sensor offset 2 */
		if(af6133_GetSensorOffset(offset))
		{
			return -1;
		} 
		
		for(i=0;i<3;i++)
    { 
    	total_offset[i] += offset[i];
  	  index[i] = total_offset[i] / MAG_MIN_OFFSET;
  	
      if(index[i] % 2)
        index[i] += ((index[i]>0) ? 1 : -1);

      index[i] /= 2; 
      
      if(index[i] > 15) index[i] = 15;
      if(index[i] < -15) index[i] = -15;	
    }

    if(index[0] || index[1] || index[2])
    {  	  	
    	// reduce sensor offset 2
		  if(af6133_otp_set_offset(index))
		  {
			  return -1;
		  }
        
      //get sensor offset 3   
		  if(af6133_GetSensorOffset(offset))
		  {
			  return -1;
		  } 
    }      		 
	}
 
	/* update sensor offset */
  	for(i=0;i<3;i++)
  	{
    		af6133_offset[i] = offset[i];
  	}
   
  /* AF6133 B2 SET/RESET & Offset Cancelation (3/3) */
  /* Offset_ASIC_Start */
  databuf[0] = REG_XY_SR;
	databuf[1] = 0x08;     
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
	
	databuf[0] = REG_Z_SR;
	databuf[1] = 0x44;     
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
	
	databuf[0] = MUX_SEL;
	databuf[1] = 0x00;     
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
	
	for(i=0;i<5;i++)
	{
	  databuf[0] = REG_MEASURE;
	  databuf[1] = 0x01;     
	  af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
	
	  mdelay(4);
	
	  databuf[0] = REG_DATA;
	  af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x601, I2C_FLAG_READ);
	  
	  mag_ASIC[0] += (databuf[1] << 8) | databuf[0];
		mag_ASIC[1] += (databuf[3] << 8) | databuf[2];
		mag_ASIC[2] += (databuf[5] << 8) | databuf[4];
  }
  mag_ASIC[0] = mag_ASIC[0] / 5;
  mag_ASIC[1] = mag_ASIC[1] / 5;
  mag_ASIC[2] = mag_ASIC[2] / 5;
  /* Offset_ASIC_End */
  
  /* Offset_TEMP_Start */
  databuf[0] = REG_XY_SR;
	databuf[1] = 0x09;     
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
	
	for(i=0;i<5;i++)
	{
	  databuf[0] = REG_MEASURE;
	  databuf[1] = 0x01;     
	  af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
    
    mdelay(4);
    
	  databuf[0] = REG_TEMP;
	  af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x201, I2C_FLAG_READ);
	  
	  mag_TEMP += (databuf[1] << 8) | databuf[0];
  }
  mag_TEMP = mag_TEMP / 5;
	/* Offset_TEMP_End */
	
	databuf[0] = MUX_SEL;
	databuf[1] = 0x30;     
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
  
	af6133_temp = (databuf[1] << 8) | databuf[0];

	return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133_GetBistData(int16_t *bist_X, int16_t *bist_Y, int16_t *bist_Z)
{
	/* AF6133 B2 BIST for Sensitivity & Orthogonality (1/3) */  	
	int16_t	i;
	int16_t mag_P[3], mag_N[3];
	uint8_t databuf[6];
	int32_t tmp[3];
 
	if(NULL == af6133_i2c_client)
	{
		return -1;
	}
	
	databuf[0] = REG_XY_SR;
	databuf[1] = 0x04;     
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
	
	databuf[0] = REG_Z_SR;
	databuf[1] = 0x24;     
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);   

	databuf[0] = REG_TEST0;
	databuf[1] = 0x83;     
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
	
	databuf[0] = REG_TEST2;
	databuf[1] = 0x00;     
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

	/* Measure BIST X */
  	for(i=0;i<3;i++)
    		tmp[i] = 0;

	for(i=0;i<MAG_BIST_LOOP;i++)
	{
		databuf[0] = REG_MEASURE;
		databuf[1] = 0x45;     
		af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
		mdelay(8);

		databuf[0] = REG_DATA;
		af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
		mag_P[0] = (databuf[1] << 8) | databuf[0];
		mag_P[1] = (databuf[3] << 8) | databuf[2];
		mag_P[2] = (databuf[5] << 8) | databuf[4];
  
		databuf[0] = REG_DATA2;
		af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
		mag_N[0] = (databuf[1] << 8) | databuf[0];
		mag_N[1] = (databuf[3] << 8) | databuf[2];
		mag_N[2] = (databuf[5] << 8) | databuf[4];

    		tmp[0] += (int32_t)(mag_P[0] - mag_N[0]);
    		tmp[1] += (int32_t)(mag_N[1] - mag_P[1]);
    		tmp[2] += (int32_t)(mag_P[2] - mag_N[2]);
	}

  	for(i=0;i<3;i++)
    		bist_X[i] = -(int16_t)(tmp[i] / MAG_BIST_LOOP);

	/* Measure BIST Y */
  	for(i=0;i<3;i++)
    		tmp[i] = 0;

	for(i=0;i<MAG_BIST_LOOP;i++)
	{
		databuf[0] = REG_MEASURE;
		databuf[1] = 0x49;     
		af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
		mdelay(8);

		databuf[0] = REG_DATA;
		af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
		mag_P[0] = (databuf[1] << 8) | databuf[0];
		mag_P[1] = (databuf[3] << 8) | databuf[2];
		mag_P[2] = (databuf[5] << 8) | databuf[4];
  
		databuf[0] = REG_DATA2;
		af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
		mag_N[0] = (databuf[1] << 8) | databuf[0];
		mag_N[1] = (databuf[3] << 8) | databuf[2];
		mag_N[2] = (databuf[5] << 8) | databuf[4];

    		tmp[0] += (int32_t)(mag_P[0] - mag_N[0]);
    		tmp[1] += (int32_t)(mag_N[1] - mag_P[1]);
    		tmp[2] += (int32_t)(mag_N[2] - mag_P[2]);
	}

  	for(i=0;i<3;i++)
    		bist_Y[i] = -(int16_t)(tmp[i] / MAG_BIST_LOOP);

	/* Measure BIST Z */
  	for(i=0;i<3;i++)
    		tmp[i] = 0;

	for(i=0;i<MAG_BIST_LOOP;i++)
	{
		databuf[0] = REG_MEASURE;
		databuf[1] = 0x4D;     
		af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
		mdelay(8);

		databuf[0] = REG_DATA;
		af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
		mag_P[0] = (databuf[1] << 8) | databuf[0];
		mag_P[1] = (databuf[3] << 8) | databuf[2];
		mag_P[2] = (databuf[5] << 8) | databuf[4];
  
		databuf[0] = REG_DATA2;
		af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
		mag_N[0] = (databuf[1] << 8) | databuf[0];
		mag_N[1] = (databuf[3] << 8) | databuf[2];
		mag_N[2] = (databuf[5] << 8) | databuf[4];

    		tmp[0] += (int32_t)(mag_P[0] - mag_N[0]);
    		tmp[1] += (int32_t)(mag_P[1] - mag_N[1]);
    		tmp[2] += (int32_t)(mag_P[2] - mag_N[2]);
	}

  	for(i=0;i<3;i++)
    		bist_Z[i] = -(int16_t)(tmp[i] / MAG_BIST_LOOP);

	/* BIST current */
	databuf[0] = REG_TEST2;
	databuf[1] = 0x00;     
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
	
	/* Recovery Golden Mode */
	databuf[0] = REG_XY_SR;
	databuf[1] = 0x05;     
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
	
	databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;     
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133_SetBistData(void)
{  	
	int16_t	i;
	int16_t bistX[3], bistY[3], bistZ[3];
 
	if(NULL == af6133_i2c_client)
	{
		return -1;
	}   
  	
        /* get BIST data */
	if(af6133_GetBistData(bistX, bistY, bistZ))
	{
		return -1;
	}

	/* update BIST data */
	for(i=0;i<3;i++)
	{
		af6133_bist_x[i] = bistX[i];
		af6133_bist_y[i] = bistY[i];
		af6133_bist_z[i] = bistZ[i];
	}

	/* update senosr gain */
	af6133_gain[0] = (int16_t)(BIST_GAIN_COEFF_X / bistX[0]);
	af6133_gain[1] = (int16_t)(BIST_GAIN_COEFF_Y / bistY[1]);
	af6133_gain[2] = (int16_t)(BIST_GAIN_COEFF_Z / bistZ[2]);	

	/* update sens. comp. */
    	af6133_comp[0] = -(BIST_COMP_COEFF_0 * bistY[0] / bistX[0] + BIST_COMP_BIAS_0);
    	af6133_comp[1] = -(BIST_COMP_COEFF_1 * bistX[1] / bistY[1] + BIST_COMP_BIAS_1);
    	af6133_comp[2] = -(BIST_COMP_COEFF_2 * bistX[2] / bistZ[2] + BIST_COMP_BIAS_2);
    	af6133_comp[3] = -(BIST_COMP_COEFF_3 * bistY[2] / bistZ[2] + BIST_COMP_BIAS_3);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133_Chipset_Init(void)
{
	/* AF6133 B2 Golden Mode Setting (Temporary) */
	u8 databuf[2];
  
	databuf[0] = REG_PCODE;
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x101, I2C_FLAG_READ); 	
  
	if(AF6133J_PID != databuf[0])
	{
  		MSE_ERR("af6133 PCODE is incorrect: %d\n", databuf[0]);

  		return -3;
  	} 
  	else
  	{ 
  		printk("%s chip id:%#x\n",__func__,databuf[0]);
  	}
  
  databuf[0] = REG_RANGE;
  databuf[1] = 0x32; 
  af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE); 
  
	databuf[0] = REG_AVG;
	databuf[1] = 0x33; 
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  
  
	databuf[0] = REG_XY_SR;
  databuf[1] = 0x05;	
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  
  
	databuf[0] = REG_OSR;
	databuf[1] = 0x00;
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);  
  
	databuf[0] = REG_WAITING;
	databuf[1] = 0x00;     
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

	databuf[0] = REG_Z_SR;
	databuf[1] = 0x24;     
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

	databuf[0] = REG_TEST0;
	databuf[1] = 0x83;     
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

	databuf[0] = REG_TEST2;
	databuf[1] = 0x00;     
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

	databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;     
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
	
	databuf[0] = MUX_SEL;
	databuf[1] = 0x30;     
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

	mdelay(5);
  
  	return 0;
}
/*----------------------------------------------------------------------------*/
void af6133_Para_Init(void)
{
	int16_t i;

	for(i=0;i<3;i++)
	{
		af6133_offset[i] = 0;
		af6133_gain[i] = 100;
	}

	for(i=0;i<4;i++)
	{
		af6133_comp[i] = 0;
	}

	af6133_temp = 0;
}
/*----------------------------------------------------------------------------*/
static int af6133_ReadSensorData(int *buf)
{
	uint8_t databuf[6];
  int16_t output[3];
	int16_t i;
  
	if(NULL == af6133_i2c_client)
	{
		buf[0] = 0;
		buf[1] = 0;
		buf[2] = 0;
		return -2;
	}   
  	
	/* read sensor data */
	databuf[0] = REG_DATA2;
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
	output[0] = (databuf[1] << 8) | databuf[0];
	output[1] = (databuf[3] << 8) | databuf[2];
	output[2] = (databuf[5] << 8) | databuf[4];

	/* reduce sensor offset */  
	if(af6133_controldata[1] & 0x01)
	{
  		for(i=0;i<3;i++)
      			output[i] = output[i] - af6133_offset[i];
	}

	/* multiple sensor gain */  
	if(af6133_controldata[1] & 0x02)
	{
  		for(i=0;i<3;i++)
      			output[i] = (output[i] * af6133_gain[i]) / 100;
	}

	/* sens. comp. */  
	if(af6133_controldata[1] & 0x04)
	{
      		buf[0] = output[0] + (output[1] * af6133_comp[0]) / 100;
		buf[1] = output[1] + (output[0] * af6133_comp[1]) / 100;
		buf[2] = output[2] + (output[0] * af6133_comp[2]) / 100 + 
				     (output[1] * af6133_comp[3]) / 100;
	}
	else
	{
		buf[0] = output[0];
		buf[1] = output[1];
		buf[2] = output[2];
	}

	for(i=0;i<3;i++)
		buf[i] = -buf[i];

	/* next measurement */    
	databuf[0] = REG_MEASURE;
	databuf[1] = 0x01;
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
  
	return 0;
 }

/*----------------------------------------------------------------------------*/
static ssize_t show_daemon_name(struct device_driver *ddri, char *buf)
{
	char strbuf[10];
	sprintf(strbuf, "af6133d");
	return sprintf(buf, "%s", strbuf);		
}
/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char databuf[2];
	
	databuf[0] = REG_PCODE;
	af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x101, I2C_FLAG_READ); 	
  	
	if(AF6133J_PID != databuf[0])
	{
		printk("af6133 PCODE is incorrect: %d\n", databuf[0]);
	} 

	return sprintf(buf, "%X\n", databuf[0]);       
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	int databuf[3];
	af6133_ReadSensorData(databuf);
	return sprintf(buf, "%d %d %d\n", databuf[0],databuf[1],databuf[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_calidata_value(struct device_driver *ddri, char *buf)
{
	int status;
	int mag[3], tmp[3];
	char strbuf[32];
	
	af6133_get_data(&mag[0], &mag[1], &mag[2], &status);
	
	tmp[0] = mag[0] / CONVERT_M_DIV;
	tmp[1] = mag[1] / CONVERT_M_DIV;
	tmp[2] = mag[2] / CONVERT_M_DIV;
	 
	sprintf(strbuf, "%d, %d, %d\n", tmp[0],tmp[1], tmp[2]);	
	
	return sprintf(buf, "%s\n", strbuf);            
}
/*----------------------------------------------------------------------------*/

static ssize_t show_midcontrol_value(struct device_driver *ddri, char *buf)
{
	int tmp[3];
	char strbuf[32];
	tmp[0] = af6133_controldata[0];
	tmp[1] = af6133_controldata[1];
	tmp[2] = af6133_controldata[2];
	sprintf(strbuf, "%d %d %d\n", tmp[0],tmp[1], tmp[2]);	
	return sprintf(buf, "%s\n", strbuf);            
}
/*----------------------------------------------------------------------------*/

static ssize_t store_midcontrol_value(struct device_driver *ddri, const char *buf, size_t count)
{   
	int p[CONTROL_DATA_LEN];
	if(CONTROL_DATA_LEN == sscanf(buf, "%d %d %d",&p[0], &p[1], &p[2]))
	{
		memcpy(&af6133_controldata[0], &p, sizeof(int)*CONTROL_DATA_LEN);           
	}
	else
	{
		MSE_ERR("invalid format\n");     
	}
	//return sizeof(int)*CONTROL_DATA_LEN;    
	return strlen(buf);          
}
/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
    	struct af6133_i2c_data *data;

    	if(NULL == af6133_i2c_client)
	{
		MSE_ERR("af6133_i2c_client IS NULL !\n");
		return -1;
	}

    	data = i2c_get_clientdata(af6133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw->direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);            
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
    	struct af6133_i2c_data *data;
    	int layout=0;

	if(NULL == af6133_i2c_client)
	{
		MSE_ERR("af6133_i2c_client IS NULL !\n");
		return -1;
	}

    	data = i2c_get_clientdata(af6133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

	if(1 == sscanf(buf, "%d", &layout))
	{
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			MSE_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		}
		else if(!hwmsen_get_convert(data->hw->direction, &data->cvt))
		{
			MSE_ERR("invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		}
		else
		{
			MSE_ERR("invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	}
	else
	{
		MSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;            
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
    	struct af6133_i2c_data *data;
    	ssize_t len;

	if(NULL == af6133_i2c_client)
	{
		MSE_ERR("af6133_i2c_client IS NULL !\n");
		return -1;
	}
    
	data = i2c_get_clientdata(af6133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

	len = 0;
	if(data->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n", 
		data->hw->i2c_num, data->hw->direction, data->hw->power_id, data->hw->power_vol);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "OPEN: %d\n", atomic_read(&dev_open_count));
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
    	struct af6133_i2c_data *data;
	ssize_t res;

	if(NULL == af6133_i2c_client)
	{
		MSE_ERR("af6133_i2c_client IS NULL !\n");
		return -1;
	}

    	data = i2c_get_clientdata(af6133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}	

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&data->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
    	struct af6133_i2c_data *data;
    	int trace = 0;

	if(NULL == af6133_i2c_client)
	{
		MSE_ERR("af6133_i2c_client IS NULL !\n");
		return -1;
	}

    	data = i2c_get_clientdata(af6133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}
    
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&data->trace, trace);
	}
	else 
	{
		MSE_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}

	return count;    
}
/*----------------------------------------------------------------------------*/
/*----------------------------shipment test------------------------------------------------*/
/*!
 @return If @a testdata is in the range of between @a lolimit and @a hilimit,
 the return value is 1, otherwise -1.
 @param[in] testno   A pointer to a text string.
 @param[in] testname A pointer to a text string.
 @param[in] testdata A data to be tested.
 @param[in] lolimit  The maximum allowable value of @a testdata.
 @param[in] hilimit  The minimum allowable value of @a testdata.
 @param[in,out] pf_total
 */
int AF6133_TEST_DATA(const char testno[], const char testname[], const int testdata,
	const int lolimit, const int hilimit, int *pf_total)
{
	int pf;			/* Pass;1, Fail;-1 */

	if ((testno == NULL) && (strncmp(testname, "START", 5) == 0)) {
		MSE_LOG("--------------------------------------------------------------------\n");
		MSE_LOG(" Test No. Test Name	Fail	Test Data	[	 Low	High]\n");
		MSE_LOG("--------------------------------------------------------------------\n");
		pf = 1;
	} else if ((testno == NULL) && (strncmp(testname, "END", 3) == 0)) {
		MSE_LOG("--------------------------------------------------------------------\n");
		if (*pf_total == 1)
			MSE_LOG("Factory shipment test was passed.\n\n");
		else
			MSE_LOG("Factory shipment test was failed.\n\n");

		pf = 1;
	} else {
		if ((lolimit <= testdata) && (testdata <= hilimit))
			pf = 1;
		else
			pf = -1;

	/* display result */
	MSE_LOG(" %7s  %-10s	 %c	%9d	[%9d	%9d]\n",
		testno, testname, ((pf == 1) ? ('.') : ('F')), testdata,
		lolimit, hilimit);
	}

	/* Pass/Fail check */
	if (*pf_total != 0) {
		if ((*pf_total == 1) && (pf == 1))
			*pf_total = 1;		/* Pass */
		else
			*pf_total = -1;		/* Fail */
	}
	return pf;
}
/*----------------------------------------------------------------------------*/
/*!
 Execute "Onboard Function Test" (NOT includes "START" and "END" command).
 @retval 1 The test is passed successfully.
 @retval -1 The test is failed.
 @retval 0 The test is aborted by kind of system error.
 */
int FST_AF6133(void)
{
	int pf_total;  /* p/f flag for this subtest */
	int16_t bistX[3], bistY[3], bistZ[3];
	int16_t offset[3];
	//int16_t gain[3];
	int16_t value[3];
	int pid_flag=0;
	uint8_t databuf[2];

	/* *********************************************** */
	/* Reset Test Result */
	/* *********************************************** */
	pf_total = 1;

	/* *********************************************** */
	/* Test Start */
	/* *********************************************** */

	/* Reset device. */
	af6133_Chipset_Init();

	/* Read values from WIA. */
	databuf[0] = REG_PCODE;
	if (af6133_i2c_master_operate(af6133_i2c_client, databuf, 0x101, I2C_FLAG_READ) < 0) {
		MSE_ERR("af6133 I2C Error.\n");
		return 0;
	}

  	if(AF6133J_PID == databuf[0])
    		pid_flag = 1;

	/* TEST 1 */
	AF6133_TEST_DATA("T1", "AF6133 Product Code is correct", pid_flag, 1, 1, &pf_total);


	/* Check sensor offset */
	if(af6133_GetSensorOffset(offset))
	{
		MSE_ERR("af6133_GetSensorOffset return fail\n");
		return 0;
	}

	/* TEST 2 */
	AF6133_TEST_DATA("T2_1", "AF6133 x-axis offset", offset[0], AF6133_OFFSET_MIN, AF6133_OFFSET_MAX, &pf_total);
	AF6133_TEST_DATA("T2_2", "AF6133 y-axis offset", offset[1], AF6133_OFFSET_MIN, AF6133_OFFSET_MAX, &pf_total);
	AF6133_TEST_DATA("T2_3", "AF6133 z-axis offset", offset[2], AF6133_OFFSET_MIN, AF6133_OFFSET_MAX, &pf_total);

	if(af6133_GetBistData(bistX, bistY, bistZ))
	{
		MSE_ERR("af6133_GetBistData return fail\n");
		return 0;
	}

	value[0] = (int16_t)(bistX[0] * BIST_COEFF_X * af6133_gain[0]) / 10000;
	value[1] = (int16_t)(bistY[1] * BIST_COEFF_Y * af6133_gain[1]) / 10000;
	value[2] = (int16_t)(bistZ[2] * BIST_COEFF_Z * af6133_gain[2]) / 10000;

	/* TEST 3 */
	AF6133_TEST_DATA("T3_1", "AF6133 BIST test 1", value[0], AF6133_SENS_MIN, AF6133_SENS_MAX, &pf_total);
	AF6133_TEST_DATA("T3_2", "AF6133 BIST test 2", value[1], AF6133_SENS_MIN, AF6133_SENS_MAX, &pf_total);
	AF6133_TEST_DATA("T3_3", "AF6133 BIST test 3", value[2], AF6133_SENS_MIN, AF6133_SENS_MAX, &pf_total);

	return pf_total;
}
/*----------------------------------------------------------------------------*/
/*!
 Execute "Onboard Function Test" (includes "START" and "END" command).
 @retval 1 The test is passed successfully.
 @retval -1 The test is failed.
 @retval 0 The test is aborted by kind of system error.
 */
int AF6133_FctShipmntTestProcess_Body(void)
{
	int pf_total = 1;

	/* *********************************************** */
	/* Reset Test Result */
	/* *********************************************** */
	AF6133_TEST_DATA(NULL, "START", 0, 0, 0, &pf_total);

	/* *********************************************** */
	/* Step 1 to 2 */
	/* *********************************************** */
	pf_total = FST_AF6133();

	/* *********************************************** */
	/* Judge Test Result */
	/* *********************************************** */
	AF6133_TEST_DATA(NULL, "END", 0, 0, 0, &pf_total);

	return pf_total;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_shipment_test(struct device_driver *ddri, const char *buf, size_t count)
{
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_shipment_test(struct device_driver *ddri, char *buf)
{
	char result[10];
	int res = 0;

	res = AF6133_FctShipmntTestProcess_Body();
	if (1 == res) {
		MSE_LOG("shipment_test pass\n");
		strcpy(result, "y");
	} else if (-1 == res) {
		MSE_LOG("shipment_test fail\n");
		strcpy(result, "n");
	} else {
		MSE_LOG("shipment_test NaN\n");
		strcpy(result, "NaN");
	}

	return sprintf(buf, "%s\n", result);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_caliparas_value(struct device_driver *ddri, char *buf)
{
	int tmp[10];
	char strbuf[128];
	
	tmp[0] = af6133_offset[0];
	tmp[1] = af6133_offset[1];
	tmp[2] = af6133_offset[2];
	tmp[3] = af6133_gain[0];
	tmp[4] = af6133_gain[1];
	tmp[5] = af6133_gain[2];
	tmp[6] = af6133_comp[0];
	tmp[7] = af6133_comp[1];
	tmp[8] = af6133_comp[2];
	tmp[9] = af6133_comp[3];
	 
	sprintf(strbuf, "%d %d %d %d %d %d %d %d %d %d\n", tmp[0],tmp[1], tmp[2],
			tmp[3],tmp[4], tmp[5], tmp[6],tmp[7], tmp[8], tmp[9]);	
	return sprintf(buf, "%s\n", strbuf);            
}
/*----------------------------------------------------------------------------*/
static ssize_t show_bistdata_value(struct device_driver *ddri, char *buf)
{
	int tmp[9];
	char strbuf[128];
	
	tmp[0] = af6133_bist_x[0];
	tmp[1] = af6133_bist_x[1];
	tmp[2] = af6133_bist_x[2];
	tmp[3] = af6133_bist_y[0];
	tmp[4] = af6133_bist_y[1];
	tmp[5] = af6133_bist_y[2];
	tmp[6] = af6133_bist_z[0];
	tmp[7] = af6133_bist_z[1];
	tmp[8] = af6133_bist_z[2];
	
	sprintf(strbuf, "%d %d %d %d %d %d %d %d %d\n", tmp[0],tmp[1], tmp[2],
			tmp[3],tmp[4], tmp[5], tmp[6],tmp[7], tmp[8]);	
	return sprintf(buf, "%s\n", strbuf);            
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(daemon,      S_IRUGO, show_daemon_name, NULL);
static DRIVER_ATTR(chipinfo,    S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata,  S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(calidata,    S_IRUGO, show_calidata_value, NULL);
static DRIVER_ATTR(status,      S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(caliparas,   S_IRUGO, show_caliparas_value, NULL);
static DRIVER_ATTR(bistdata,    S_IRUGO, show_bistdata_value, NULL);
static DRIVER_ATTR(midcontrol,  S_IRUGO | S_IWUSR, show_midcontrol_value, store_midcontrol_value );
static DRIVER_ATTR(layout,      S_IRUGO | S_IWUSR, show_layout_value, store_layout_value );
static DRIVER_ATTR(trace,       S_IRUGO | S_IWUSR, show_trace_value, store_trace_value );
static DRIVER_ATTR(shipmenttest,S_IRUGO | S_IWUSR, show_shipment_test, store_shipment_test);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *af6133_attr_list[] = {
  	&driver_attr_daemon,
	&driver_attr_chipinfo,
	&driver_attr_sensordata,
	&driver_attr_calidata,
	&driver_attr_status,
	&driver_attr_caliparas,
	&driver_attr_bistdata,
	&driver_attr_midcontrol,
	&driver_attr_layout,
	&driver_attr_trace,
	&driver_attr_shipmenttest,
};
/*----------------------------------------------------------------------------*/
static int af6133_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(af6133_attr_list)/sizeof(af6133_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, af6133_attr_list[idx])))
		{            
			MSE_ERR("driver_create_file (%s) = %d\n", af6133_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int af6133_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(af6133_attr_list)/sizeof(af6133_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}


	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, af6133_attr_list[idx]);
	}


	return err;
}
/*----------------------------------------------------------------------------*/
static int af6133_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;
  struct af6133_i2c_data *data;
	
	if(NULL == af6133_i2c_client)
	{
		MSE_ERR("af6133_i2c_client IS NULL !\n");
		return -1;
	}

	data = i2c_get_clientdata(af6133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}
  
	value = (int)samplingPeriodNs / 1000 / 1000;
	
	MSE_LOG("af6133 mag set delay = (%d) ok.\n", value);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int af6133_flush(void)
{
	return mag_flush_report();
}
/*----------------------------------------------------------------------------*/
static int af6133_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
	int err;

	err = af6133_enable(enabledisable == true ? 1 : 0);
	if (err) {
		MSE_ERR("%s enable sensor failed!\n", __func__);
		return -1;
	}
	err = af6133_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		MSE_ERR("%s enable set batch failed!\n", __func__);
		return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133_factory_get_data(int32_t data[3], int *status)
{
	/* get raw data */
	return  af6133_get_data(&data[0], &data[1], &data[2], status);
}
/*----------------------------------------------------------------------------*/
static int af6133_factory_get_raw_data(int32_t data[3])
{
	MSE_LOG("do not support af6133_factory_get_raw_data!\n");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133_factory_enable_calibration(void)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133_factory_clear_cali(void)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133_factory_set_cali(int32_t data[3])
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133_factory_get_cali(int32_t data[3])
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133_factory_do_self_test(void)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct mag_factory_fops af6133_factory_fops = {
	.enable_sensor      = af6133_factory_enable_sensor,
	.get_data           = af6133_factory_get_data,
	.get_raw_data       = af6133_factory_get_raw_data,
	.enable_calibration = af6133_factory_enable_calibration,
	.clear_cali         = af6133_factory_clear_cali,
	.set_cali           = af6133_factory_set_cali,
	.get_cali           = af6133_factory_get_cali,
	.do_self_test       = af6133_factory_do_self_test,
};
/*----------------------------------------------------------------------------*/
static struct mag_factory_public af6133_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &af6133_factory_fops,
};
/*----------------------------------------------------------------------------*/
static int af6133_open_report_data(int en)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133_set_delay(u64 delay)
{
	int value = 0;

	struct af6133_i2c_data *data;
	
	if(NULL == af6133_i2c_client)
	{
		MSE_ERR("af6133_i2c_client IS NULL !\n");
		return -1;
	}

    	data = i2c_get_clientdata(af6133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}
	
	value = (int)(delay / 1000 / 1000);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, AF6133_DEV_NAME);
    return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133_enable(int en)
{
	int value = 0;
    	struct af6133_i2c_data *data ;

	if(NULL == af6133_i2c_client)
	{
		MSE_ERR("af6133_i2c_client IS NULL !\n");
		return -1;
	}

    	data = i2c_get_clientdata(af6133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

	value = en;

	if(value == 1)
	{
		atomic_set(&m_flag, 1);
		atomic_set(&open_flag, 1);
	}
	else
	{
		atomic_set(&m_flag, 0);
		if(atomic_read(&o_flag) == 0)
		{
			atomic_set(&open_flag, 0);
		}	
	}
	wake_up(&open_wq);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133_get_data(int *x,int *y, int *z,int *status)
{
  	struct af6133_i2c_data *data ;
	int databuf[3]={0};
	
	if(NULL == af6133_i2c_client)
	{
		MSE_ERR("af6133_i2c_client IS NULL !\n");
		return -1;
	}

    	data = i2c_get_clientdata(af6133_i2c_client);
	if(NULL == data)
	{
		MSE_ERR("data IS NULL !\n");
		return -1;
	}

	af6133_ReadSensorData(databuf);

	*x = databuf[0];
	*y = databuf[1];
	*z = databuf[2];

	if (atomic_read(&data->trace) & VTC_TRC_DEBUG) {
		MSE_LOG("%s get data: %d, %d, %d. divide %d, status %d!", __func__,
				*x, *y, *z, CONVERT_M_DIV, *status);
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
//static int af6133_suspend(struct i2c_client *client, pm_message_t msg) 
static int af6133_suspend(struct device *dev) 
{
		struct i2c_client *client = to_i2c_client(dev);  //
	struct af6133_i2c_data *obj = i2c_get_clientdata(client);
	MSE_FUN();    
#if 1
//	if(msg.event == PM_EVENT_SUSPEND)
	{   
		af6133_power(obj->hw, 0);   
			}
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
//static int af6133_resume(struct i2c_client *client)
static int af6133_resume(struct device *dev)
{
	int err;
	struct i2c_client *client = to_i2c_client(dev);
	struct af6133_i2c_data *obj = i2c_get_clientdata(client);
	MSE_FUN();
#if 1
	af6133_power(obj->hw, 1);

	if((err = af6133_Chipset_Init())!=0)
	{
		MSE_ERR("initialize client fail!!\n");
		return err;        
	}
#endif
	return 0;
}

/*----------------------------------------------------------------------------*/
static int af6133_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client = NULL;
	struct af6133_i2c_data *data = NULL;
	int err = 0;

	struct mag_control_path ctl_path ={0};
	struct mag_data_path dat_path = {0};

	printk("%s start\n",__func__);
	
	err = get_mag_dts_func(client->dev.of_node, hw);
	if (err < 0) 
	{
		MSE_ERR("get dts info fail\n");
		err = -EFAULT;
		goto exit;
	}

	if (!(data = kmalloc(sizeof(struct af6133_i2c_data), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct af6133_i2c_data));

	data->hw = hw;
	
	err = hwmsen_get_convert(data->hw->direction, &data->cvt);
	if(err)
	{
		MSE_ERR("invalid direction: %d\n", data->hw->direction);
		goto exit;
	}

	atomic_set(&data->layout, data->hw->direction);
	atomic_set(&data->trace, 0);
	
	init_waitqueue_head(&open_wq);

	data->client = client;
	new_client = data->client;
	i2c_set_clientdata(new_client, data);

	af6133_i2c_client = new_client;	

	af6133_Para_Init();

	err = af6133_Chipset_Init();
	if(err)
	{
		MSE_ERR("af6133 register initial fail\n");
		goto exit_init_failed;
	}
	
	err = af6133_SetSensorOffset();
	if(err)
	{
		MSE_ERR("af6133 sensor offset initial fail\n");
		goto exit_init_failed;
	}

	err = af6133_SetBistData();
	if(err)
	{
		MSE_ERR("af6133 sensor gain & comp. initial fail\n");
		goto exit_init_failed;
	}

	/* Register sysfs attribute */
	err = af6133_create_attr(&af6133_init_info.platform_diver_addr->driver);
	if(err)
	{
		MSE_ERR("create attribute err = %d\n", err);
		goto exit_sysfs_create_group_failed;
	}

	err = mag_factory_device_register(&af6133_factory_device);
	if(err)
	{
		MSE_ERR("factory device register failed, err = %d\n", err);
		goto exit_misc_device_register_failed;	
	}    

	ctl_path.is_use_common_factory = false;
	ctl_path.open_report_data = af6133_open_report_data;
	ctl_path.enable = af6133_enable;
	ctl_path.set_delay = af6133_set_delay;

	ctl_path.is_report_input_direct = false;
	ctl_path.is_support_batch = data->hw->is_batch_supported;
	
	ctl_path.batch = af6133_batch;
	ctl_path.flush = af6133_flush;
	//strlcpy(ctl_path.libinfo.libname, "vtclib", sizeof(ctl_path.libinfo.libname));
	strcpy(ctl_path.libinfo.libname, "vtclib");
	ctl_path.libinfo.layout = hw->direction;
  ctl_path.libinfo.deviceid = REG_PCODE;
		
	err = mag_register_control_path(&ctl_path);

	if(err < 0)
	{
		MSE_ERR("mag_register_control_path failed!\n");
		goto exit_misc_device_register_failed;
	}

	dat_path.div = CONVERT_M_DIV;

	dat_path.get_data = af6133_get_data;

	err = mag_register_data_path(&dat_path);
	if(err < 0)
	{
		MSE_ERR("mag_register_control_path failed!\n");
		goto exit_misc_device_register_failed;
	}  

	MSE_LOG("%s: probe done\n", __func__);

	af6133_init_flag=0;

	return 0;

  exit_sysfs_create_group_failed:   
  exit_init_failed:
  exit_misc_device_register_failed:
#ifdef SUPPORT_SOFTGYRO_FUNCTION
  exit_kfree:
#endif
  	kfree(data);
  exit:
	MSE_ERR("%s: err = %d\n", __func__, err);

	af6133_init_flag=-1;

	return err;
}
/*----------------------------------------------------------------------------*/
static int af6133_i2c_remove(struct i2c_client *client)
{
	int err;	

	if((err = af6133_delete_attr(&af6133_init_info.platform_diver_addr->driver)))
	{
		MSE_ERR("af6133_delete_attr fail: %d\n", err);
	}

	af6133_i2c_client = NULL;
	i2c_unregister_device(client);
	mag_factory_device_deregister(&af6133_factory_device);
	kfree(i2c_get_clientdata(client));	   
	return 0;
}
/*----------------------------------------------------------------------------*/
extern int aeon_gpio_set(const char *name);
static int af6133_local_init(void)
{
	printk("af6133_local_init");
	af6133_controldata[0] = 100;	// Loop delay (ms)
	af6133_controldata[1] = 7;  	// Use sensor offset, gain, comp.
	af6133_controldata[2] = 0;  	// resevered
	
	atomic_set(&dev_open_count, 0);
  
	if(i2c_add_driver(&af6133_i2c_driver))
	{
		MSE_ERR("add driver error\n");
		return -1;
	} 
	 
	if(-1 == af6133_init_flag)	
	{	   
		return -1;	
	}
	
	printk("%s done\n",__func__);
    
	return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133_remove(void)
{
	MSE_FUN(); 
	af6133_power(hw, 0);	
	atomic_set(&dev_open_count, 0);  
	i2c_del_driver(&af6133_i2c_driver);
	af6133_init_flag = -1;
	return 0;
}
/*----------------------------------------------------------------------------*/
static int __init af6133_init(void)
{
	int err;
	aeon_gpio_set("aeon_msensor_rst1");
	err = mag_driver_add(&af6133_init_info);
	
	if(err < 0)
	{
		MSE_ERR("mag_driver_add failed!\n");
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit af6133_exit(void)
{
	MSE_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(af6133_init);
module_exit(af6133_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Gary Huang");
MODULE_DESCRIPTION("AF6133 m-Sensor driver");
MODULE_LICENSE("VTC");
MODULE_VERSION(DRIVER_VERSION);

