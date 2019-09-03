/* drivers/i2c/chips/AF6133E.c - AF6133E compass driver
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
 * For Android 8.0, 9.0
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
#include "af6133e.h"

extern int aeon_gpio_set(const char *name);
/* Add for auto detect feature */
static int af6133e_local_init(void);
static int af6133e_remove(void);
static int af6133e_open_report_data(int en);
static int af6133e_set_delay(u64 delay);
static int af6133e_enable(int en);
static int af6133e_get_data(int *x,int *y, int *z,int *status);

static int af6133e_init_flag = -1;  //0:ok,,-1:fail

static struct mag_init_info af6133e_init_info = {
  .name   = "af6133e",
  .init   = af6133e_local_init,
  .uninit = af6133e_remove,
};

/* driver information */
#define AF6133E_DEV_NAME        "af6133e"
#define DRIVER_VERSION          "3.0.0"
#define DRIVER_RELEASE          "20190503"
/* delay time (ms)*/
#define AF6133E_DELAY_MIN        5
#define AF6133E_DELAY_MAX      100
#define AF6133E_DELAY_SW_GYRO   10
/* debug message */
#define MSE_TAG                 "MSENSOR"
#define MSE_FUN(f)              printk(MSE_TAG" %s\r\n", __FUNCTION__)
#define MSE_ERR(fmt, args...)   printk(KERN_ERR MSE_TAG" %s %d : \r\n"fmt, __FUNCTION__, __LINE__, ##args)
#define MSE_LOG(fmt, args...)   printk(MSE_TAG fmt, ##args)
#define MSE_VER(fmt, args...)   ((void)0)
/* function switch */
#define FUNC_MASK_BIAS          0x01
#define FUNC_MASK_GAIN          0x02
#define FUNC_MASK_COMP          0x04
#define FUNC_MASK_TEMP          0x08
/******************************************************************************/
static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

static atomic_t open_flag = ATOMIC_INIT(0);
static atomic_t m_flag = ATOMIC_INIT(0);
static atomic_t o_flag = ATOMIC_INIT(0);
/*----------------------------------------------------------------------------*/
static uint32_t vtc_int_sqrt(uint32_t x)
{
  uint32_t small = 1;
  uint32_t low, high;

  if(x < 0) return -1;

  if(x < 1)
  {
    low = 0;
    high = 1;
  }
  else
  {
    low = 1;
    high = x;
  }

  while((high - low) > small)
  {
    uint32_t mid = (low + high) / 2;

    if((mid * mid) > x)
      high = mid;
    else
      low = mid;
  }

  return low;
}
/*----------------------------------------------------------------------------*/
/* Maintain  cust info here */
struct mag_hw af6133e_mag_cust;
static struct mag_hw *hw = &af6133e_mag_cust;

/* For  driver get cust info */
struct mag_hw *af6133e_get_cust_mag(void)
{
  return &af6133e_mag_cust;
}
/*----------------------------------------------------------------------------*/
static struct i2c_client *af6133e_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id af6133e_i2c_id[] = {{AF6133E_DEV_NAME,0},{}};
/*----------------------------------------------------------------------------*/
static int af6133e_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int af6133e_i2c_remove(struct i2c_client *client);
static int af6133e_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);

/*----------------------------------------------------------------------------*/
typedef enum {
  VTC_TRC_DEBUG  = 0x01,
} AMI_TRC;
/*----------------------------------------------------------------------------*/
#define CONTROL_DATA_LEN 3
/*----------------------------------------------------------------------------*/
int16_t af6133e_offset[3];
int16_t af6133e_gain[3];
int16_t af6133e_comp[4];
int16_t af6133e_bist_x[3];
int16_t af6133e_bist_y[3];
int16_t af6133e_bist_z[3];
int af6133e_controldata[CONTROL_DATA_LEN];
int af6133e_odr;
#ifdef AF6133E_TEMP_COMP
int32_t af6133e_temp_t0;
int32_t af6133e_temp_dt;
int16_t af6133e_dt_pre;
uint16_t af6133e_t_count;
uint16_t af6133e_t_index;
int16_t af6133e_t_buf[TEMP_MF_NUM];
#endif
/*----------------------------------------------------------------------------*/
struct af6133e_i2c_data {
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
/*----------------------------------------------------------------------------*/
static struct i2c_driver af6133e_i2c_driver = {
  .driver = {
             //.owner = THIS_MODULE, 
             .name  = AF6133E_DEV_NAME,
#ifdef CONFIG_OF 
             .of_match_table = mag_of_match,
#endif
            },
  .probe      = af6133e_i2c_probe,
  .remove     = af6133e_i2c_remove,
  .detect     = af6133e_i2c_detect,
  //.suspend    = af6133e_suspend,
  //.resume     = af6133e_resume,
  .id_table   = af6133e_i2c_id,
};

static DEFINE_MUTEX(af6133e_mutex);
#define I2C_FLAG_WRITE  0
#define I2C_FLAG_READ   1

/*----------------------------------------------------------------------------*/
static atomic_t dev_open_count;
/*----------------------------------------------------------------------------*/

//========================================================================
static void af6133e_power(struct mag_hw *hw, unsigned int on)
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
  {
    return 0;
  }
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
  {
    return 0;
  }
}
/*----------------------------------------------------------------------------*/
int af6133e_i2c_master_operate(struct i2c_client *client, char *buf, int count, int i2c_flag)
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
static int af6133e_GetOpenStatus(void)
{
  wait_event_interruptible(open_wq, (atomic_read(&open_flag) != 0));
  return atomic_read(&open_flag);
}
----------------------------------------------------------------------------*/
static int af6133e_GetSensorOffset(int16_t *offset)
{
#ifdef AF6133E_SET_OFFSET
  uint8_t databuf[6];
#endif 
  if(NULL == af6133e_i2c_client)
  {
    return -1;
  }   

#ifdef AF6133E_SET_OFFSET
  databuf[0] = REG_AVG_2ND;
  databuf[1] = 0x10; 
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
  
  databuf[0] = REG_MEASURE;
  databuf[1] = 0x01; 
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
  
  mdelay(8);

  databuf[0] = REG_DATA;
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
  offset[0] = (databuf[1] << 8) | databuf[0];
  offset[1] = (databuf[3] << 8) | databuf[2];
  offset[2] = (databuf[5] << 8) | databuf[4];

  if(af6133e_controldata[2] & 0x01)
  {
    printk("%s offset data (REG_AVG_2ND = 0x10): %d, %d, %d,\n", __func__, offset[0], offset[1], offset[2]);
  }
  
  databuf[0] = REG_AVG_2ND;
  databuf[1] = 0x50; 
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
  
  databuf[0] = REG_MEASURE;
  databuf[1] = 0x01; 
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
  
  mdelay(8);
  
  databuf[0] = REG_DATA;
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
  offset[0] += (databuf[1] << 8) | databuf[0];
  offset[1] += (databuf[3] << 8) | databuf[2];
  offset[2] += (databuf[5] << 8) | databuf[4];

  if(af6133e_controldata[2] & 0x01)
  {
    printk("%s offset data (REG_AVG_2ND = 0x50): %d, %d, %d,\n", __func__, offset[0], offset[1], offset[2]);
  }
  
  offset[0] = 0;
  offset[1] = 0;
  offset[2] = offset[2] / 2;

  if(af6133e_controldata[2] & 0x01)
  {
      printk("%s offset data (final): %d, %d, %d,\n", __func__, offset[0], offset[1], offset[2]);
  }
#else
  offset[0] = 0;
  offset[1] = 0;
  offset[2] = 0;
#endif
 
  return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133e_SetSensorOffset(void)
{
#ifdef AF6133E_TEMP_COMP
  uint8_t databuf[6];
#endif
 
  if(NULL == af6133e_i2c_client)
  {
    return -1;
  }

  if(af6133e_GetSensorOffset(af6133e_offset))
  {
    return -2;
  }

#ifdef AF6133E_TEMP_COMP  
  databuf[0] = REG_TEMP;
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x201, I2C_FLAG_READ);

  af6133e_temp_t0 = (int32_t)((int16_t)((databuf[1] << 8) | databuf[0]));
  af6133e_temp_t0 = (af6133e_temp_t0 * TEMP_RESOLUTION) / 1000;
#endif
  
  return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133e_GetBistData(int16_t *bist_X, int16_t *bist_Y, int16_t *bist_Z)
{
  int16_t i;
  int16_t mag[3];
  uint8_t databuf[6];
  int32_t tmp_pos[3], tmp_neg[3];
  int16_t bist_loop_count = MAG_BIST_LOOP;
  int16_t bist_retry_count = BIST_RETRY_NUM;
  int16_t pos_count, neg_count;
 
  if(NULL == af6133e_i2c_client)
  {
    return -1;
  }

  // open 8M
  databuf[0] = REG_CHOPPER;
  databuf[1] = 0xF1; 
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

  /* Measure BIST X */
  for(i = 0; i < 3; i++)
  {
    tmp_neg[i] = 0;
    tmp_pos[i] = 0;
  }

  // XPos
  databuf[0] = REG_TEST2;
  databuf[1] = 0x04;     
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
  mdelay(1);
  bist_loop_count = MAG_BIST_LOOP;
  bist_retry_count = BIST_RETRY_NUM;

  do
  {
    databuf[0] = REG_MEASURE;
    databuf[1] = 0x01;     
    af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

    mdelay(6);

    databuf[0] = REG_DATA;
    af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
    mag[0] = (databuf[1] << 8) | databuf[0];
    mag[1] = (databuf[3] << 8) | databuf[2];
    mag[2] = (databuf[5] << 8) | databuf[4];
    
    if(af6133e_controldata[2] & 0x01)
    {
      printk("%s XPos: %d, %d, %d\n", __func__, mag[0], mag[1], mag[2]);
    }
    
    if(mag[0] <  16383 && mag[1] <  16383 && mag[2] <  16383 &&
       mag[0] > -16384 && mag[1] > -16384 && mag[2] > -16384)
    {
      tmp_pos[0] += (int32_t)(mag[0]);
      tmp_pos[1] += (int32_t)(mag[1]);
      tmp_pos[2] += (int32_t)(mag[2]);
      bist_loop_count--;
    }
    else
    {
      bist_retry_count--;
    }
  }
  while(bist_loop_count && bist_retry_count);
  
  pos_count = bist_loop_count;

  // XNeg
  databuf[0] = REG_TEST2;
  databuf[1] = 0x0C;     
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
  mdelay(1);
  bist_loop_count = MAG_BIST_LOOP;
  bist_retry_count = BIST_RETRY_NUM;

  do
  {
    databuf[0] = REG_MEASURE;
    databuf[1] = 0x01;     
    af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

    mdelay(6);

    databuf[0] = REG_DATA;
    af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
    mag[0] = (databuf[1] << 8) | databuf[0];
    mag[1] = (databuf[3] << 8) | databuf[2];
    mag[2] = (databuf[5] << 8) | databuf[4];
    
    if(af6133e_controldata[2] & 0x01)
    {
      printk("%s XNeg: %d, %d, %d\n", __func__, mag[0], mag[1], mag[2]);
    }
    
    if(mag[0] <  16383 && mag[1] <  16383 && mag[2] <  16383 &&
       mag[0] > -16384 && mag[1] > -16384 && mag[2] > -16384)
    {
      tmp_neg[0] += (int32_t)(mag[0]);
      tmp_neg[1] += (int32_t)(mag[1]);
      tmp_neg[2] += (int32_t)(mag[2]);
      bist_loop_count--;
    }
    else
    {
      bist_retry_count--;
    }
  }
  while(bist_loop_count && bist_retry_count);
  
  neg_count = bist_loop_count;
  
  if(pos_count != MAG_BIST_LOOP && neg_count != MAG_BIST_LOOP)
  {
    for(i=0;i<3;i++)
      bist_X[i] = (int16_t)(tmp_neg[i]/(MAG_BIST_LOOP-neg_count) - tmp_pos[i]/(MAG_BIST_LOOP-pos_count));
  }
  else
  {
    for(i=0;i<3;i++) bist_X[i] = 0;
    
    if(af6133e_controldata[2] & 0x01)
    {
      printk("%s bist_X fail: pos_count:%d, neg_count:%d\n", __func__, pos_count, neg_count);
    }
    goto BIST_END;
  }

  if(af6133e_controldata[2] & 0x01)
  {
    printk("%s bist_X: %d, %d, %d\n", __func__, bist_X[0], bist_X[1], bist_X[2]);
  }

  /* Measure BIST Y */
  for(i = 0; i < 3; i++)
  {
    tmp_neg[i] = 0;
    tmp_pos[i] = 0;
  }

  // YPos
  databuf[0] = REG_TEST2;
  databuf[1] = 0x02;     
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
  mdelay(1);
  bist_loop_count = MAG_BIST_LOOP;
  bist_retry_count = BIST_RETRY_NUM;
  
  do
  {
    databuf[0] = REG_MEASURE;
    databuf[1] = 0x01;     
    af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

    mdelay(6);

    databuf[0] = REG_DATA;
    af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
    mag[0] = (databuf[1] << 8) | databuf[0];
    mag[1] = (databuf[3] << 8) | databuf[2];
    mag[2] = (databuf[5] << 8) | databuf[4];
    
    if(af6133e_controldata[2] & 0x01)
    {
      printk("%s YPos: %d, %d, %d\n", __func__, mag[0], mag[1], mag[2]);
    }
    
    if(mag[0] <  16383 && mag[1] <  16383 && mag[2] <  16383 &&
       mag[0] > -16384 && mag[1] > -16384 && mag[2] > -16384)
    {
      tmp_pos[0] += (int32_t)(mag[0]);
      tmp_pos[1] += (int32_t)(mag[1]);
      tmp_pos[2] += (int32_t)(mag[2]);
      bist_loop_count--;
    }
    else
    {
      bist_retry_count--;
    }
  }
  while(bist_loop_count && bist_retry_count);
  
  pos_count = bist_loop_count;

  // YNeg
  databuf[0] = REG_TEST2;
  databuf[1] = 0x0A;     
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
  mdelay(1);
  bist_loop_count = MAG_BIST_LOOP;
  bist_retry_count = BIST_RETRY_NUM;
  
  do
  {
    databuf[0] = REG_MEASURE;
    databuf[1] = 0x01;     
    af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

    mdelay(6);

    databuf[0] = REG_DATA;
    af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
    mag[0] = (databuf[1] << 8) | databuf[0];
    mag[1] = (databuf[3] << 8) | databuf[2];
    mag[2] = (databuf[5] << 8) | databuf[4];
    
    if(af6133e_controldata[2] & 0x01)
    {
      printk("%s YNeg: %d, %d, %d\n", __func__, mag[0], mag[1], mag[2]);
    }
    
    if(mag[0] <  16383 && mag[1] <  16383 && mag[2] <  16383 &&
       mag[0] > -16384 && mag[1] > -16384 && mag[2] > -16384)
    {
      tmp_neg[0] += (int32_t)(mag[0]);
      tmp_neg[1] += (int32_t)(mag[1]);
      tmp_neg[2] += (int32_t)(mag[2]);
      bist_loop_count--;
    }
    else
    {
      bist_retry_count--;
    }
  }
  while(bist_loop_count && bist_retry_count);
  
  neg_count = bist_loop_count;
 
  if(pos_count != MAG_BIST_LOOP && neg_count != MAG_BIST_LOOP)
  {
    for(i=0;i<3;i++)
      bist_Y[i] = (int16_t)(tmp_neg[i]/(MAG_BIST_LOOP-neg_count) - tmp_pos[i]/(MAG_BIST_LOOP-pos_count));
  }
  else
  {
    for(i=0;i<3;i++) bist_Y[i] = 0;
    
    if(af6133e_controldata[2] & 0x01)
    {
      printk("%s bist_Y fail: pos_count:%d, neg_count:%d\n", __func__, pos_count, neg_count);
    }
    goto BIST_END;
  }

  if(af6133e_controldata[2] & 0x01)
  {
    printk("%s bist_Y: %d, %d, %d\n", __func__, bist_Y[0], bist_Y[1], bist_Y[2]);
  }
    
  /* Measure BIST Z */
  for(i = 0; i < 3; i++)
  {
    tmp_neg[i] = 0;
    tmp_pos[i] = 0;
  }

  // ZPos
  databuf[0] = REG_TEST2;
  databuf[1] = 0x01;     
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
  mdelay(1);
  bist_loop_count = MAG_BIST_LOOP;
  bist_retry_count = BIST_RETRY_NUM;

  do
  {
    databuf[0] = REG_MEASURE;
    databuf[1] = 0x01;     
    af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

    mdelay(6);

    databuf[0] = REG_DATA;
    af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
    mag[0] = (databuf[1] << 8) | databuf[0];
    mag[1] = (databuf[3] << 8) | databuf[2];
    mag[2] = (databuf[5] << 8) | databuf[4];
    
    if(af6133e_controldata[2] & 0x01)
    {
      printk("%s ZPos: %d, %d, %d\n", __func__, mag[0], mag[1], mag[2]);
    }
    
    if(mag[0] <  16383 && mag[1] <  16383 && mag[2] <  16383 &&
       mag[0] > -16384 && mag[1] > -16384 && mag[2] > -16384)
    {
      tmp_pos[0] += (int32_t)(mag[0]);
      tmp_pos[1] += (int32_t)(mag[1]);
      tmp_pos[2] += (int32_t)(mag[2]);
      bist_loop_count--;
    }
    else
    {
      bist_retry_count--;
    }
  }
  while(bist_loop_count && bist_retry_count);
  
  pos_count = bist_loop_count;

  // ZNeg
  databuf[0] = REG_TEST2;
  databuf[1] = 0x09;
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
  mdelay(1);
  bist_loop_count = MAG_BIST_LOOP;
  bist_retry_count = BIST_RETRY_NUM;

  do
  {
    databuf[0] = REG_MEASURE;
    databuf[1] = 0x01;     
    af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

    mdelay(6);

    databuf[0] = REG_DATA;
    af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
    mag[0] = (databuf[1] << 8) | databuf[0];
    mag[1] = (databuf[3] << 8) | databuf[2];
    mag[2] = (databuf[5] << 8) | databuf[4];
    
    if(af6133e_controldata[2] & 0x01)
    {
      printk("%s ZNeg: %d, %d, %d\n", __func__, mag[0], mag[1], mag[2]);
    }
    
    if(mag[0] <  16383 && mag[1] <  16383 && mag[2] <  16383 &&
       mag[0] > -16384 && mag[1] > -16384 && mag[2] > -16384)
    {
      tmp_neg[0] += (int32_t)(mag[0]);
      tmp_neg[1] += (int32_t)(mag[1]);
      tmp_neg[2] += (int32_t)(mag[2]);
      bist_loop_count--;
    }
    else
    {
      bist_retry_count--;
    }
  }
  while(bist_loop_count && bist_retry_count);
  
  neg_count = bist_loop_count;
 
  if(pos_count != MAG_BIST_LOOP && neg_count != MAG_BIST_LOOP)
  {
    for(i=0;i<3;i++)
      bist_Z[i] = (int16_t)(tmp_neg[i]/(MAG_BIST_LOOP-neg_count) - tmp_pos[i]/(MAG_BIST_LOOP-pos_count));
  }
  else
  {
    for(i=0;i<3;i++) bist_Z[i] = 0;
    
    if(af6133e_controldata[2] & 0x01)
    {
      printk("%s bist_Z fail: pos_count:%d, neg_count:%d\n", __func__, pos_count, neg_count);
    }
    goto BIST_END;
  }

  if(af6133e_controldata[2] & 0x01)
  {
    printk("%s bist_Z: %d, %d, %d\n", __func__, bist_Z[0], bist_Z[1], bist_Z[2]);
  }

BIST_END:

  // close 8M
  databuf[0] = REG_CHOPPER;  
  databuf[1] = 0xC1; 
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

  // Recovery Golden Mode
  databuf[0] = REG_TEST2;
  databuf[1] = 0x00;
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

  databuf[0] = REG_MEASURE;
  databuf[1] = 0x01;
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

  return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133e_SetBistData(void)
{  
  int16_t i;
  int16_t bistX[3], bistY[3], bistZ[3];
  int16_t gain[3];
  int16_t comp_coeff[4];
 
  if(NULL == af6133e_i2c_client)
  {
    return -1;
  }   
  
  /* get BIST data */
  if(af6133e_GetBistData(bistX, bistY, bistZ))
  {
    return -1;
  }

  /* update BIST data */
  for(i = 0; i < 3; i++)
  {
    af6133e_bist_x[i] = bistX[i];
    af6133e_bist_y[i] = bistY[i];
    af6133e_bist_z[i] = bistZ[i];
  }

  if(af6133e_controldata[2] & 0x01)
  {
    printk("%s bist_X: %d, %d, %d\n", __func__, af6133e_bist_x[0], af6133e_bist_x[1], af6133e_bist_x[2]);
    printk("%s bist_Y: %d, %d, %d\n", __func__, af6133e_bist_y[0], af6133e_bist_y[1], af6133e_bist_y[2]);
    printk("%s bist_Z: %d, %d, %d\n", __func__, af6133e_bist_z[0], af6133e_bist_z[1], af6133e_bist_z[2]);
  }

  if(af6133e_bist_x[0] > 0 && af6133e_bist_y[1] > 0 && af6133e_bist_z[2] > 0)
  {
    uint32_t SSensz = ((af6133e_bist_x[2]*BIST_COEFF_X) * (af6133e_bist_x[2]*BIST_COEFF_X) + 
                       (af6133e_bist_y[2]*BIST_COEFF_Y) * (af6133e_bist_y[2]*BIST_COEFF_Y)) /10000 +
                      ((af6133e_bist_z[2]*BIST_COEFF_Z)/100) * ((af6133e_bist_z[2]*BIST_COEFF_Z)/100);
    int16_t SSensz_i16;

    if(af6133e_controldata[2] & 0x01)
    {
      printk("%s SSensz: %d\n", __func__, SSensz);
    }

    SSensz = vtc_int_sqrt(SSensz);

    if(af6133e_controldata[2] & 0x01)
    {
      printk("%s SSensz = sqrt(SSensz): %d\n", __func__, SSensz);
    }

    SSensz_i16 = (int16_t)SSensz;

    if(af6133e_controldata[2] & 0x01)
    {
      printk("%s SSensz_i16: %d\n", __func__, SSensz_i16);
    }
    
    //calculate sens. compensation coeff.
    gain[0] = BIST_GAIN_COEFF_X / af6133e_bist_x[0];
    gain[1] = BIST_GAIN_COEFF_Y / af6133e_bist_y[1];
    gain[2] = BIST_GAIN_COEFF_Z / af6133e_bist_z[2];

    comp_coeff[0] = -(int16_t)((af6133e_bist_y[0]*BIST_COEFF_Y*100) / (af6133e_bist_x[0]*BIST_COEFF_X));
    comp_coeff[1] = -(int16_t)((af6133e_bist_x[1]*BIST_COEFF_X*100) / (af6133e_bist_y[1]*BIST_COEFF_Y));
    comp_coeff[2] = -(int16_t)((af6133e_bist_x[2]*BIST_COEFF_X) / SSensz_i16);
    comp_coeff[3] = -(int16_t)((af6133e_bist_y[2]*BIST_COEFF_Y) / SSensz_i16);

    if(af6133e_controldata[2] & 0x01)
    {
      printk("%s gain: %d\n", __func__, gain[0], gain[1], gain[2]);
      printk("%s comp_coeff: %d\n", __func__, comp_coeff[0], comp_coeff[1], comp_coeff[2], comp_coeff[3]);
    }
  }
  
  if( gain[0] < GAIN_X_MIN || gain[0] > GAIN_X_MAX ||
      gain[1] < GAIN_Y_MIN || gain[1] > GAIN_Y_MAX ||
      gain[2] < GAIN_Z_MIN || gain[2] > GAIN_Z_MAX ||
      (comp_coeff[0] + comp_coeff[1]) >  BIST_COEFF_LIMIT_XY ||
      (comp_coeff[0] + comp_coeff[1]) < -BIST_COEFF_LIMIT_XY ||  
      vtc_fabs(comp_coeff[2]) > BIST_COEFF_LIMIT_Z ||
      vtc_fabs(comp_coeff[3]) > BIST_COEFF_LIMIT_Z)
  {
    MSE_ERR("af6133e BISC parameters over range\n");
  }
  else
  {
    af6133e_gain[0] = gain[0];
    af6133e_gain[1] = gain[1];
    af6133e_gain[2] = gain[2];

    af6133e_comp[0] = comp_coeff[0];
    af6133e_comp[1] = comp_coeff[1];
    af6133e_comp[2] = comp_coeff[2];
    af6133e_comp[3] = comp_coeff[3];
  }

  if(af6133e_controldata[2] & 0x01)
  {
      printk("%s af6133e_gain: %d\n", __func__, af6133e_gain[0], af6133e_gain[1], af6133e_gain[2]);
      printk("%s af6133e_comp: %d\n", __func__, af6133e_comp[0], af6133e_comp[1], af6133e_comp[2], af6133e_comp[3]);
  }
  
  return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133e_Chipset_Init(void)
{
  /* AF6133E Golden Mode Setting (Temporary) */
  uint8_t databuf[2];
  
  databuf[0] = REG_PCODE;
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x101, I2C_FLAG_READ); 
  
  if(AF6133E_PID != databuf[0])
  {
    MSE_ERR("af6133e PCODE is incorrect: %d\n", databuf[0]);
    return -3;
  } 
  else
  { 
    printk("%s chip id:%#x\n",__func__,databuf[0]);
  }
  
  databuf[0] = REG_AVG;
  databuf[1] = 0x47; 
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

  databuf[0] = REG_SR_MODE;
  databuf[1] = 0x0D; 
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

  databuf[0] = REG_OSR;
  databuf[1] = 0x3D; 
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
  
  databuf[0] = REG_OSC_FREQ;
  databuf[1] = 0x3F; 
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
  
  databuf[0] = REG_XY_WAITING;
  databuf[1] = 0x07; 
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

  databuf[0] = REG_AVG_2ND;
  databuf[1] = 0x50; 
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);

  mdelay(5);
  
  return 0;
}
/*----------------------------------------------------------------------------*/
void af6133e_Para_Init(void)
{
  int16_t i;

  for(i=0;i<3;i++)
  {
    af6133e_offset[i] = 0;
    af6133e_gain[i] = 100;
  }

  for(i=0;i<4;i++)
  {
    af6133e_comp[i] = 0;
  }

#ifdef AF6133E_TEMP_COMP
  af6133e_temp_t0 = 0;
  af6133e_temp_dt = 0;
  af6133e_dt_pre = 0;
  af6133e_t_count = 0;
  af6133e_t_index = 0;
#endif
}
/*----------------------------------------------------------------------------*/
static int af6133e_mag_temp_comp(int16_t *mag_data)
{
  int32_t temp_buf[3];

  if(af6133e_controldata[2] & 0x01)
  {
      printk("%s af6133e_odr:%d\n", __func__, af6133e_odr);
  }

  if(af6133e_odr > 0 && af6133e_t_count++ > af6133e_odr)
  {
    
    //int16_t temp_dt;
    int16_t i, j;
    int16_t buf[TEMP_MF_NUM];
    int16_t delta;
    uint8_t databuf[2];
    
    af6133e_t_count = 0;
    
    databuf[0] = REG_TEMP;
    af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x201, I2C_FLAG_READ);
  
    af6133e_t_buf[af6133e_t_index] = (databuf[1] << 8) | databuf[0];
    af6133e_t_buf[af6133e_t_index] = (int16_t)((float)af6133e_t_buf[af6133e_t_index] * TEMP_RESOLUTION) / 1000;

    if(af6133e_controldata[2] & 0x01)
    {
      printk("%s t_buf[%d]: %d\n", __func__, af6133e_t_index, af6133e_t_buf[af6133e_t_index]);
    }
    
    for(i=0;i<TEMP_MF_NUM;i++)
      buf[i] = af6133e_t_buf[i];

    for(i=0;i<(TEMP_MF_NUM-1);i++)
      for(j=0;j<(TEMP_MF_NUM-1);j++)
        if(buf[j] > buf[j+1])
        {
          int16_t temp = buf[j+1];
          buf[j+1] = buf[j];
          buf[j] = temp;
        }
    
    af6133e_temp_dt = buf[TEMP_MF_IDX] - af6133e_temp_t0;
    delta = af6133e_temp_dt - af6133e_dt_pre;

    if(af6133e_controldata[2] & 0x01)
    {
      printk("%s af6133e_temp_dt:%d = buf[%d]:%d - af6133e_temp_t0:%d \n", __func__, af6133e_temp_dt, TEMP_MF_IDX, buf[TEMP_MF_IDX], af6133e_temp_t0);
      printk("%s delta:%d = af6133e_temp_dt:%d - af6133e_dt_pre:%d \n", __func__, delta, af6133e_temp_dt, af6133e_dt_pre);
    }
    
    if(vtc_fabs(delta) > TEMP_DELTA_THRESHOLD)
      af6133e_dt_pre = af6133e_temp_dt;
    else
      af6133e_temp_dt = af6133e_dt_pre;
      
    if(++af6133e_t_index >= TEMP_MF_NUM)
      af6133e_t_index = 0;
    
  }

  temp_buf[0] = (int32_t)mag_data[0];
  temp_buf[1] = (int32_t)mag_data[1];
  temp_buf[2] = (int32_t)mag_data[2];

  if(af6133e_controldata[2] & 0x01)
  {
    printk("%s af6133e_temp_dt: %d; temp_buf: %d, %d, %d;\n", __func__, af6133e_temp_dt, temp_buf[0], temp_buf[1], temp_buf[2]);
  }

  temp_buf[0] = temp_buf[0] * af6133e_temp_dt * TEMP_SENS_COEFF_X;
  temp_buf[1] = temp_buf[1] * af6133e_temp_dt * TEMP_SENS_COEFF_Y;
  temp_buf[2] = temp_buf[2] * af6133e_temp_dt * TEMP_SENS_COEFF_Z;

  if(af6133e_controldata[2] & 0x01)
  {
    printk("%s temp_buf: %d, %d, %d\n", __func__, temp_buf[0], temp_buf[1], temp_buf[2]);
  }

  mag_data[0] += (int16_t)(temp_buf[0] / 100000);
  mag_data[1] += (int16_t)(temp_buf[1] / 100000);
  mag_data[2] += (int16_t)(temp_buf[2] / 100000);

  return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133e_ReadSensorData(int *buf)
{
  uint8_t databuf[6];
  int16_t output[3];
  int16_t i;

  if(NULL == af6133e_i2c_client)
  {
    buf[0] = 0;
    buf[1] = 0;
    buf[2] = 0;
    return -2;
  }

  /* read sensor data */
  databuf[0] = REG_DATA;
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x601, I2C_FLAG_READ);
  
  output[0] = (databuf[1] << 8) | databuf[0];
  output[1] = (databuf[3] << 8) | databuf[2];
  output[2] = (databuf[5] << 8) | databuf[4];

  if(af6133e_controldata[2] & 0x01)
  {
    printk("%s magdata (raw): %d, %d, %d,\n", __func__, output[0], output[1], output[2]);
  }

  /* reduce sensor offset */  
  if(af6133e_controldata[1] & FUNC_MASK_BIAS)
  {
    for(i=0;i<3;i++)
      output[i] = output[i] - af6133e_offset[i];
  }

  if(af6133e_controldata[2] & 0x01)
  {
    printk("%s magdata (remove offset): %d, %d, %d,\n", __func__, output[0], output[1], output[2]);
  }
  
#ifdef AF6133E_TEMP_COMP
  if(af6133e_controldata[1] & FUNC_MASK_TEMP)
  {
    af6133e_mag_temp_comp(output);
    if(af6133e_controldata[2] & 0x01)
    {
      printk("%s magdata (temperature compensation): %d, %d, %d,\n", __func__, output[0], output[1], output[2]);
    }
  }
#endif

  /* multiple sensor gain */  
  if(af6133e_controldata[1] & FUNC_MASK_GAIN)
  {
    for(i=0;i<3;i++)
      output[i] = (output[i] * af6133e_gain[i]) / 100;

    if(af6133e_controldata[2] & 0x01)
    {
      printk("%s gain: %d, %d, %d, (1/100)\n", __func__, af6133e_gain[0], af6133e_gain[1], af6133e_gain[2]);
      printk("%s magdata (comp gain): %d, %d, %d,\n", __func__, output[0], output[1], output[2]);
    }
  }

  /* sens. comp. */  
  if(af6133e_controldata[1] & FUNC_MASK_COMP)
  {
    buf[0] = output[0] + (output[1] * af6133e_comp[0]) / 100;
    buf[1] = output[1] + (output[0] * af6133e_comp[1]) / 100;
    buf[2] = output[2] + (output[0] * af6133e_comp[2]) / 100
                       + (output[1] * af6133e_comp[3]) / 100;

    if(af6133e_controldata[2] & 0x01)
    {
      printk("%s orthogonal: %d, %d, %d, %d, (1/100)\n", __func__, af6133e_comp[0], af6133e_comp[1], af6133e_comp[2], af6133e_comp[3]);
      printk("%s magdata (comp orthogonal): %d, %d, %d,\n", __func__, buf[0], buf[1], buf[2]);
    }
  }
  else
  {
    buf[0] = output[0];
    buf[1] = output[1];
    buf[2] = output[2];
  }

  if(af6133e_controldata[2] & 0x01)
  {
    printk("%s magdata (output): %d, %d, %d,\n", __func__, buf[0], buf[1], buf[2]);
  }

  /* next measurement */    
  databuf[0] = REG_MEASURE;
  databuf[1] = 0x01;
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x02, I2C_FLAG_WRITE);
  
  return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_daemon_name(struct device_driver *ddri, char *buf)
{
  char strbuf[10];

  sprintf(strbuf, "af6133ed");

  return sprintf(buf, "%s", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
  char databuf[2];

  databuf[0] = REG_PCODE;
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x101, I2C_FLAG_READ); 
  
  if(AF6133E_PID != databuf[0])
  {
    printk("af6133e PCODE is incorrect: %d\n", databuf[0]);
  } 

  return sprintf(buf, "%X\n", databuf[0]);       
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
  int databuf[3];

  af6133e_ReadSensorData(databuf);

  return sprintf(buf, "%d %d %d\n", databuf[0],databuf[1],databuf[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_calidata_value(struct device_driver *ddri, char *buf)
{
  int status;
  int mag[3], tmp[3];
  char strbuf[32];

  af6133e_get_data(&mag[0], &mag[1], &mag[2], &status);

  tmp[0] = mag[0] * CONVERT_M / CONVERT_M_DIV;
  tmp[1] = mag[1] * CONVERT_M / CONVERT_M_DIV;
  tmp[2] = mag[2] * CONVERT_M / CONVERT_M_DIV;
 
  sprintf(strbuf, "%d, %d, %d\n", tmp[0],tmp[1], tmp[2]);

  return sprintf(buf, "%s\n", strbuf);            
}
/*----------------------------------------------------------------------------*/

static ssize_t show_midcontrol_value(struct device_driver *ddri, char *buf)
{
  int tmp[3];
  char strbuf[32];

  tmp[0] = af6133e_controldata[0];
  tmp[1] = af6133e_controldata[1];
  tmp[2] = af6133e_controldata[2];
  sprintf(strbuf, "%d %d %d\n", tmp[0],tmp[1], tmp[2]);

  return sprintf(buf, "%s\n", strbuf);            
}
/*----------------------------------------------------------------------------*/

static ssize_t store_midcontrol_value(struct device_driver *ddri, const char *buf, size_t count)
{   
  int p[CONTROL_DATA_LEN];

  if(CONTROL_DATA_LEN == sscanf(buf, "%d %d %d",&p[0], &p[1], &p[2]))
  {
    memcpy(&af6133e_controldata[0], &p, sizeof(int)*CONTROL_DATA_LEN);           
  }
  else
  {
    MSE_ERR("invalid format\n");     
  }
  
  return strlen(buf);          
}
/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
  struct af6133e_i2c_data *data;

  if(NULL == af6133e_i2c_client)
  {
    MSE_ERR("af6133e_i2c_client IS NULL !\n");
    return -1;
  }

  data = i2c_get_clientdata(af6133e_i2c_client);
  if(NULL == data)
  {
    MSE_ERR("data IS NULL !\n");
    return -1;
  }

  return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
  data->hw->direction,atomic_read(&data->layout),data->cvt.sign[0], data->cvt.sign[1],
  data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);            
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
  struct af6133e_i2c_data *data;
  int layout=0;

  if(NULL == af6133e_i2c_client)
  {
    MSE_ERR("af6133e_i2c_client IS NULL !\n");
    return -1;
  }

    data = i2c_get_clientdata(af6133e_i2c_client);
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
  struct af6133e_i2c_data *data;
  ssize_t len;

  if(NULL == af6133e_i2c_client)
  {
    MSE_ERR("af6133e_i2c_client IS NULL !\n");
    return -1;
  }
    
  data = i2c_get_clientdata(af6133e_i2c_client);
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
  struct af6133e_i2c_data *data;
  ssize_t res;

  if(NULL == af6133e_i2c_client)
  {
    MSE_ERR("af6133e_i2c_client IS NULL !\n");
    return -1;
  }

  data = i2c_get_clientdata(af6133e_i2c_client);
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
  struct af6133e_i2c_data *data;
  int trace = 0;

  if(NULL == af6133e_i2c_client)
  {
    MSE_ERR("af6133e_i2c_client IS NULL !\n");
    return -1;
  }

  data = i2c_get_clientdata(af6133e_i2c_client);
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
int AF6133E_TEST_DATA(const char testno[], const char testname[], const int testdata,
const int lolimit, const int hilimit, int *pf_total)
{
  int pf;/* Pass;1, Fail;-1 */

  if ((testno == NULL) && (strncmp(testname, "START", 5) == 0)) {
    MSE_LOG("--------------------------------------------------------------------\n");
    MSE_LOG(" Test No. Test NameFailTest Data[ LowHigh]\n");
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
    MSE_LOG(" %7s  %-10s %c%9d[%9d%9d]\n",
    testno, testname, ((pf == 1) ? ('.') : ('F')), testdata,
    lolimit, hilimit);
  }

  /* Pass/Fail check */
  if (*pf_total != 0) {
    if ((*pf_total == 1) && (pf == 1))
      *pf_total = 1;/* Pass */
  else
   *pf_total = -1;/* Fail */
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
int FST_AF6133E(void)
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
  af6133e_Chipset_Init();

  /* Read values from WIA. */
  databuf[0] = REG_PCODE;
  if (af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x101, I2C_FLAG_READ) < 0) {
   MSE_ERR("af6133e I2C Error.\n");
   return 0;
  }

  if(AF6133E_PID == databuf[0])
    pid_flag = 1;

  /* TEST 1 */
  AF6133E_TEST_DATA("T1", "AF6133E Product Code is correct", pid_flag, 1, 1, &pf_total);


  /* Check sensor offset */
  if(af6133e_GetSensorOffset(offset))
  {
    MSE_ERR("af6133e_GetSensorOffset return fail\n");
    return 0;
  }

  /* TEST 2 */
  AF6133E_TEST_DATA("T2_1", "AF6133E x-axis offset", offset[0], AF6133E_OFFSET_MIN, AF6133E_OFFSET_MAX, &pf_total);
  AF6133E_TEST_DATA("T2_2", "AF6133E y-axis offset", offset[1], AF6133E_OFFSET_MIN, AF6133E_OFFSET_MAX, &pf_total);
  AF6133E_TEST_DATA("T2_3", "AF6133E z-axis offset", offset[2], AF6133E_OFFSET_MIN, AF6133E_OFFSET_MAX, &pf_total);

  if(af6133e_GetBistData(bistX, bistY, bistZ))
  {
    MSE_ERR("af6133e_GetBistData return fail\n");
    return 0;
  }

  value[0] = (int16_t)(bistX[0]);
  value[1] = (int16_t)(bistY[1]);
  value[2] = (int16_t)(bistZ[2]);

  /* TEST 3 */
  AF6133E_TEST_DATA("T3_1", "AF6133E BIST test 1", value[0], AF6133E_SENS_MIN, AF6133E_SENS_MAX, &pf_total);
  AF6133E_TEST_DATA("T3_2", "AF6133E BIST test 2", value[1], AF6133E_SENS_MIN, AF6133E_SENS_MAX, &pf_total);
  AF6133E_TEST_DATA("T3_3", "AF6133E BIST test 3", value[2], AF6133E_SENS_MIN, AF6133E_SENS_MAX, &pf_total);

  return pf_total;
}
/*----------------------------------------------------------------------------*/
/*!
 Execute "Onboard Function Test" (includes "START" and "END" command).
 @retval 1 The test is passed successfully.
 @retval -1 The test is failed.
 @retval 0 The test is aborted by kind of system error.
 */
int AF6133E_FctShipmntTestProcess_Body(void)
{
  int pf_total = 1;

  /* *********************************************** */
  /* Reset Test Result */
  /* *********************************************** */
  AF6133E_TEST_DATA(NULL, "START", 0, 0, 0, &pf_total);

  /* *********************************************** */
  /* Step 1 to 2 */
  /* *********************************************** */
  pf_total = FST_AF6133E();

  /* *********************************************** */
  /* Judge Test Result */
  /* *********************************************** */
  AF6133E_TEST_DATA(NULL, "END", 0, 0, 0, &pf_total);

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

  res = AF6133E_FctShipmntTestProcess_Body();
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

  tmp[0] = af6133e_offset[0];
  tmp[1] = af6133e_offset[1];
  tmp[2] = af6133e_offset[2];
  tmp[3] = af6133e_gain[0];
  tmp[4] = af6133e_gain[1];
  tmp[5] = af6133e_gain[2];
  tmp[6] = af6133e_comp[0];
  tmp[7] = af6133e_comp[1];
  tmp[8] = af6133e_comp[2];
  tmp[9] = af6133e_comp[3];
 
  sprintf(strbuf, "%d %d %d %d %d %d %d %d %d %d\n", tmp[0],tmp[1], tmp[2],
                      tmp[3],tmp[4], tmp[5], tmp[6],tmp[7], tmp[8], tmp[9]);
  return sprintf(buf, "%s\n", strbuf);            
}
/*----------------------------------------------------------------------------*/
static ssize_t show_bistdata_value(struct device_driver *ddri, char *buf)
{
  int tmp[9];
  char strbuf[128];

  tmp[0] = af6133e_bist_x[0];
  tmp[1] = af6133e_bist_x[1];
  tmp[2] = af6133e_bist_x[2];
  tmp[3] = af6133e_bist_y[0];
  tmp[4] = af6133e_bist_y[1];
  tmp[5] = af6133e_bist_y[2];
  tmp[6] = af6133e_bist_z[0];
  tmp[7] = af6133e_bist_z[1];
  tmp[8] = af6133e_bist_z[2];

  sprintf(strbuf, "%d %d %d %d %d %d %d %d %d \n", tmp[0],tmp[1], tmp[2],
                                          tmp[3],tmp[4], tmp[5], tmp[6],tmp[7], tmp[8]);
  return sprintf(buf, "%s\n", strbuf);            
}
/*----------------------------------------------------------------------------*/
static ssize_t show_temp_value(struct device_driver *ddri, char *buf)
{
  int tmp[5];
  char strbuf[128];

  tmp[0] = af6133e_temp_t0;

  sprintf(strbuf, "%d, %d, %d \n", tmp[0], tmp[1], tmp[2]);
  return sprintf(buf, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_register_value(struct device_driver *ddri, char *buf)
{
  int tmp[6];
  char strbuf[32];
  uint8_t databuf[6];

  databuf[0] = REG_AVG;
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x201, I2C_FLAG_READ);
  tmp[0] = databuf[0];

  databuf[0] = REG_SR_MODE;
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x201, I2C_FLAG_READ);
  tmp[1] = databuf[0];

  databuf[0] = REG_OSR;
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x201, I2C_FLAG_READ);
  tmp[2] = databuf[0];

  databuf[0] = REG_OSC_FREQ;
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x201, I2C_FLAG_READ);
  tmp[3] = databuf[0];

  databuf[0] = REG_XY_WAITING;
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x201, I2C_FLAG_READ);
  tmp[4] = databuf[0];

  databuf[0] = REG_AVG_2ND;
  af6133e_i2c_master_operate(af6133e_i2c_client, databuf, 0x201, I2C_FLAG_READ);
  tmp[5] = databuf[0];
 
  sprintf(strbuf, "REG_AVG=0x%x, REG_SR_MODE=0x%x, REG_OSR=0x%x, REG_OSC_FREQ=0x%x, REG_XY_WAITING=0x%x, REG_AVG_2ND=0x%x\n", tmp[0],tmp[1], tmp[2], tmp[3],tmp[4], tmp[5]);

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
static DRIVER_ATTR(temp,        S_IRUGO, show_temp_value, NULL);
static DRIVER_ATTR(registermode,S_IRUGO, show_register_value, NULL);
static DRIVER_ATTR(midcontrol,  S_IRUGO | S_IWUSR, show_midcontrol_value, store_midcontrol_value );
static DRIVER_ATTR(layout,      S_IRUGO | S_IWUSR, show_layout_value, store_layout_value );
static DRIVER_ATTR(trace,       S_IRUGO | S_IWUSR, show_trace_value, store_trace_value );
static DRIVER_ATTR(shipmenttest,S_IRUGO | S_IWUSR, show_shipment_test, store_shipment_test);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *af6133e_attr_list[] = {
  &driver_attr_daemon,
  &driver_attr_chipinfo,
  &driver_attr_sensordata,
  &driver_attr_calidata,
  &driver_attr_status,
  &driver_attr_caliparas,
  &driver_attr_bistdata,
  &driver_attr_temp,
  &driver_attr_registermode,
  &driver_attr_midcontrol,
  &driver_attr_layout,
  &driver_attr_trace,
  &driver_attr_shipmenttest,
};
/*----------------------------------------------------------------------------*/
static int af6133e_create_attr(struct device_driver *driver) 
{
  int idx, err = 0;
  int num = (int)(sizeof(af6133e_attr_list)/sizeof(af6133e_attr_list[0]));

  if (driver == NULL)
  {
    return -EINVAL;
  }

  for(idx = 0; idx < num; idx++)
  {
    if((err = driver_create_file(driver, af6133e_attr_list[idx])))
    {            
      MSE_ERR("driver_create_file (%s) = %d\n", af6133e_attr_list[idx]->attr.name, err);
      break;
    }
  }    
  return err;
}
/*----------------------------------------------------------------------------*/
static int af6133e_delete_attr(struct device_driver *driver)
{
  int idx ,err = 0;
  int num = (int)(sizeof(af6133e_attr_list)/sizeof(af6133e_attr_list[0]));

  if(driver == NULL)
  {
    return -EINVAL;
  }


  for(idx = 0; idx < num; idx++)
  {
    driver_remove_file(driver, af6133e_attr_list[idx]);
  }


  return err;
}
/*----------------------------------------------------------------------------*/
static int af6133e_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
  int value = 0;
  struct af6133e_i2c_data *data;

  if(NULL == af6133e_i2c_client)
  {
    MSE_ERR("af6133e_i2c_client IS NULL !\n");
    return -1;
  }

  data = i2c_get_clientdata(af6133e_i2c_client);
  if(NULL == data)
  {
    MSE_ERR("data IS NULL !\n");
    return -1;
  }
  
  value = (int)samplingPeriodNs / 1000 / 1000;

  if(value == 0)
    af6133e_odr = 0;
  else if(value > 600)
    af6133e_odr = 1;
  else if(value > 150)
    af6133e_odr = 5; 
  else if(value > 83)
    af6133e_odr = 10;
  else if(value > 58)
    af6133e_odr = 15; 
  else if(value > 45)
    af6133e_odr = 20;
  else if(value > 30)
    af6133e_odr = 25;
  else if(value > 15)
    af6133e_odr = 50;
  else if(value > 7)
    af6133e_odr = 100;
  else 
    af6133e_odr = 200;
  
  MSE_LOG("af6133e mag set delay = (%d) ok.\n", value);
  return 0;
}

/*----------------------------------------------------------------------------*/
static int af6133e_flush(void)
{
  return mag_flush_report();
}
/*----------------------------------------------------------------------------*/
static int af6133e_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
  int err;

  err = af6133e_enable(enabledisable == true ? 1 : 0);
  if (err) {
    MSE_ERR("%s enable sensor failed!\n", __func__);
    return -1;
  }
  err = af6133e_batch(0, sample_periods_ms * 1000000, 0);
  if (err) {
    MSE_ERR("%s enable set batch failed!\n", __func__);
    return -1;
  }
  return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133e_factory_get_data(int32_t data[3], int *status)
{
  /* get raw data */
  return  af6133e_get_data(&data[0], &data[1], &data[2], status);
}
/*----------------------------------------------------------------------------*/
static int af6133e_factory_get_raw_data(int32_t data[3])
{
  MSE_LOG("do not support af6133e_factory_get_raw_data!\n");
  return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133e_factory_enable_calibration(void)
{
  return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133e_factory_clear_cali(void)
{
  return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133e_factory_set_cali(int32_t data[3])
{
  return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133e_factory_get_cali(int32_t data[3])
{
  return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133e_factory_do_self_test(void)
{
  return 0;
}
/*----------------------------------------------------------------------------*/
static struct mag_factory_fops af6133e_factory_fops = {
  .enable_sensor      = af6133e_factory_enable_sensor,
  .get_data           = af6133e_factory_get_data,
  .get_raw_data       = af6133e_factory_get_raw_data,
  .enable_calibration = af6133e_factory_enable_calibration,
  .clear_cali         = af6133e_factory_clear_cali,
  .set_cali           = af6133e_factory_set_cali,
  .get_cali           = af6133e_factory_get_cali,
  .do_self_test       = af6133e_factory_do_self_test,
};
/*----------------------------------------------------------------------------*/
static struct mag_factory_public af6133e_factory_device = {
  .gain = 1,
  .sensitivity = 1,
  .fops = &af6133e_factory_fops,
};
/*----------------------------------------------------------------------------*/
static int af6133e_open_report_data(int en)
{
  return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133e_set_delay(u64 delay)
{
  int value = 0;

  struct af6133e_i2c_data *data;

  if(NULL == af6133e_i2c_client)
  {
    MSE_ERR("af6133e_i2c_client IS NULL !\n");
    return -1;
  }

  data = i2c_get_clientdata(af6133e_i2c_client);
  if(NULL == data)
  {
    MSE_ERR("data IS NULL !\n");
    return -1;
  }

  value = (int)(delay / 1000 / 1000);

  return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133e_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
  strcpy(info->type, AF6133E_DEV_NAME);
  return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133e_enable(int en)
{
  int value = 0;
  int err = 0;
  struct af6133e_i2c_data *data ;

  if(NULL == af6133e_i2c_client)
  {
    MSE_ERR("af6133e_i2c_client IS NULL !\n");
    return -1;
  }

  data = i2c_get_clientdata(af6133e_i2c_client);
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

    err = af6133e_Chipset_Init();
    if(err)
    {
      MSE_ERR("af6133e register initial fail\n");
    }
/*
    err = af6133e_SetSensorOffset();
    if(err)
    {
      MSE_ERR("af6133e sensor offset initial fail\n");
    }
*/
    err = af6133e_SetBistData();
    if(err)
    {
      MSE_ERR("af6133e sensor gain & comp. initial fail\n");
    }
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
static int af6133e_get_data(int *x,int *y, int *z,int *status)
{
  struct af6133e_i2c_data *data ;
  int databuf[3]={0};

  if(NULL == af6133e_i2c_client)
  {
    MSE_ERR("af6133e_i2c_client is NULL !\n");
    return -1;
  }

  data = i2c_get_clientdata(af6133e_i2c_client);
  if(NULL == data)
  {
    MSE_ERR("data IS NULL !\n");
    return -1;
  }

  af6133e_ReadSensorData(databuf);

  *x = databuf[0] * CONVERT_M;
  *y = databuf[1] * CONVERT_M;
  *z = databuf[2] * CONVERT_M;

  if (atomic_read(&data->trace) & VTC_TRC_DEBUG) {
  MSE_LOG("%s get data: %d, %d, %d. divide %d, status %d!", __func__,
           *x, *y, *z, CONVERT_M_DIV, *status);
  }

  return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133e_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
  struct i2c_client *new_client = NULL;
  struct af6133e_i2c_data *data = NULL;
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
     
  if (!(data = kmalloc(sizeof(struct af6133e_i2c_data), GFP_KERNEL)))
  {
    err = -ENOMEM;
    goto exit;
  }
  memset(data, 0, sizeof(struct af6133e_i2c_data));

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

  af6133e_i2c_client = new_client;

  af6133e_Para_Init();

  err = af6133e_Chipset_Init();
  if(err)
  {
    MSE_ERR("af6133e register initial fail\n");
    goto exit_init_failed;
  }

  err = af6133e_SetSensorOffset();
  if(err)
  {
    MSE_ERR("af6133e sensor offset initial fail\n");
    goto exit_init_failed;
  }
/*
  err = af6133e_SetBistData();
  if(err)
  {
    MSE_ERR("af6133e sensor gain & comp. initial fail\n");
    goto exit_init_failed;
  }
*/

  /* Register sysfs attribute */
  err = af6133e_create_attr(&af6133e_init_info.platform_diver_addr->driver);
  if(err)
  {
    MSE_ERR("create attribute err = %d\n", err);
    goto exit_sysfs_create_group_failed;
  }

  err = mag_factory_device_register(&af6133e_factory_device);
  if(err)
  {
    MSE_ERR("factory device register failed, err = %d\n", err);
    goto exit_misc_device_register_failed;
  }  

  ctl_path.is_use_common_factory = false;
  ctl_path.open_report_data = af6133e_open_report_data;
  ctl_path.enable = af6133e_enable;
  ctl_path.set_delay = af6133e_set_delay;

  ctl_path.is_report_input_direct = false;
  ctl_path.is_support_batch = data->hw->is_batch_supported;

  ctl_path.batch = af6133e_batch;
  ctl_path.flush = af6133e_flush;

  strcpy(ctl_path.libinfo.libname, "vtclib");
  ctl_path.libinfo.layout = hw->direction;
  ctl_path.libinfo.deviceid = AF6133E_PID;

  err = mag_register_control_path(&ctl_path);

  if(err < 0)
  {
    MSE_ERR("mag_register_control_path failed!\n");
    goto exit_misc_device_register_failed;
  }

  dat_path.div = CONVERT_M_DIV;

  dat_path.get_data = af6133e_get_data;

  err = mag_register_data_path(&dat_path);
  if(err < 0)
  {
    MSE_ERR("mag_register_control_path failed!\n");
    goto exit_misc_device_register_failed;
  }

  MSE_LOG("%s: probe done\n", __func__);

  af6133e_init_flag=0;

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

  af6133e_init_flag=-1;

  return err;
}
/*----------------------------------------------------------------------------*/
static int af6133e_i2c_remove(struct i2c_client *client)
{
  int err;

  if((err = af6133e_delete_attr(&af6133e_init_info.platform_diver_addr->driver)))
  {
    MSE_ERR("af6133e_delete_attr fail: %d\n", err);
  }

  af6133e_i2c_client = NULL;
  i2c_unregister_device(client);
  mag_factory_device_deregister(&af6133e_factory_device);
  kfree(i2c_get_clientdata(client));   
  return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133e_local_init(void)
{
  printk("af6133e_local_init");
  af6133e_controldata[0] = 100;// Loop delay (ms)
  af6133e_controldata[1] = 6;  // Use sensor offset, gain, comp.
  af6133e_controldata[2] = 0;  // resevered

  atomic_set(&dev_open_count, 0);
  
  if(i2c_add_driver(&af6133e_i2c_driver))
  {
    MSE_ERR("add driver error\n");
    return -1;
  } 
 
  if(-1 == af6133e_init_flag)
  {   
    return -1;
  }

  printk("%s done\n",__func__);
    
  return 0;
}
/*----------------------------------------------------------------------------*/
static int af6133e_remove(void)
{
  MSE_FUN(); 
  af6133e_power(hw, 0);
  atomic_set(&dev_open_count, 0);  
  i2c_del_driver(&af6133e_i2c_driver);
  af6133e_init_flag = -1;
  return 0;
}
/*----------------------------------------------------------------------------*/
static int __init af6133e_init(void)
{
  int err;

  err = mag_driver_add(&af6133e_init_info);

  if(err < 0)
  {
    MSE_ERR("mag_driver_add failed!\n");
  }

  return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit af6133e_exit(void)
{
  MSE_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(af6133e_init);
module_exit(af6133e_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("George Tseng");
MODULE_DESCRIPTION("AF6133E m-Sensor driver");
MODULE_LICENSE("VTC");
MODULE_VERSION(DRIVER_VERSION);
