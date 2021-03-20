/**************************************************************************
*  aw9524_key.c
* 
*  Create Date : 
* 
*  Modify Date : 
*
*  Create by   : AWINIC Technology CO., LTD
*
*  Version     : 0.9, 2016/02/15
**************************************************************************/

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/wakelock.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/leds.h>
#include <linux/mutex.h>
#include <mt-plat/mtk_pwm.h>

#define CONFIG_GREAT_AW9524_LED_PROBE

#ifdef CONFIG_AW9524_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#include "aw9524_key.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define AW9524_I2C_NAME		"AW9524keyboard" 

//reg list
#define P0_INPUT_AW9524		0x00	//P0口引脚当前逻辑状态。0-低电平;1-高电平
#define P1_INPUT_AW9524 	0x01	//P1口引脚当前逻辑状态。0-低电平;1-高电平
#define P0_OUTPUT_AW9524 	0x02	//设置P0口引脚输出值。0-输出低电平;1-输出高电平
#define P1_OUTPUT_AW9524 	0x03	//设置P1口引脚输出值。0-输出低电平;1-输出高电平
#define P0_CONFIG_AW9524	0x04	//P0口输入/输出模式选择。0-输出模式;1-输入模式
#define P1_CONFIG_AW9524 	0x05	//P1口输入/输出模式选择。0-输出模式;1-输入模式
#define P0_INT_AW9524		0x06	//P0口中断使能。0-中断使能;1-中断不使能
#define P1_INT_AW9524		0x07	//P1口中断使能。0-中断使能;1-中断不使能
#define ID_REG_AW9524		0x10	//ID寄存器,只读,读出值为23H
#define CTL_REG_AW9524		0x11	//设置P0口驱动模式。若D[4]=0,P0口为Open-Drain模式;若D[4]=1,P0口为Push-Pull模式
#define P0_LED_MODE_AW9524	0x12	//配置P0_7~P0_0为LED或GPIO模式。 1:GPIO模式 0:LED模式
#define P1_LED_MODE_AW9524	0x13	//配置P1_7~P1_0为LED或GPIO模式。 1:GPIO模式 0:LED模式
#define P1_0_DIM0_AW9524   0x20
#define P1_1_DIM0_AW9524   0x21
#define P1_2_DIM0_AW9524   0x22
#define P1_3_DIM0_AW9524   0x23
#define P0_0_DIM0_AW9524   0x24
#define P0_1_DIM0_AW9524   0x25
#define P0_2_DIM0_AW9524   0x26
#define P0_3_DIM0_AW9524   0x27
#define P0_4_DIM0_AW9524   0x28
#define P0_5_DIM0_AW9524   0x29
#define P0_6_DIM0_AW9524   0x2A
#define P0_7_DIM0_AW9524   0x2B
#define P1_4_DIM0_AW9524   0x2C
#define P1_5_DIM0_AW9524   0x2D
#define P1_6_DIM0_AW9524   0x2E
#define P1_7_DIM0_AW9524   0x2F
#define SW_RSTN_AW9524		0x7F

extern int aeon_gpio_set(const char *name);

static int aw9524_i2c_flag = 0;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static unsigned char aw9524_i2c_write_reg(unsigned char addr, unsigned char reg_data);
static unsigned char aw9524_i2c_read_reg(unsigned char addr);

static ssize_t aw9524_get_reg(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t aw9524_set_reg(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);

static DEVICE_ATTR(aw9524_reg, 0664, aw9524_get_reg,  aw9524_set_reg);

static struct workqueue_struct *aw9524_wq = NULL;
static struct mutex aw9524_lock;

struct aw9524_key_data {
	struct device       *dev_aw9524;
	struct input_dev	*input_dev_aw9524;
	struct work_struct 	eint_work_aw9524;
	struct device_node *irq_node_aw9524;
	struct led_classdev led_dev;
	int irq_aw9524;
	bool is_screen_on_aw9524;
};

struct pinctrl *aw9524_pin;
struct pinctrl_state *aw9524_shdn_high;
struct pinctrl_state *aw9524_shdn_low;

struct aw9524_key_data *aw9524_key;
struct i2c_client *aw9524_i2c_client;

unsigned int gpio_aw9524_eint;

#define LED_MAX_BRIGHTNESS 5
#define DEFAULT_LED_NAME "kbd_backlight"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GPIO Control
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int aw9524_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;
	aw9524_pin = devm_pinctrl_get(&pdev->dev);
    AW9524_LOG("%s : pinctrl init 00000000\n", __func__);
	if (IS_ERR(aw9524_pin)) {
		dev_err(&pdev->dev, "Cannot find aw9524 pinctrl!");
		ret = PTR_ERR(aw9524_pin);
		AW9524_LOG("%s devm_pinctrl_get fail!\n", __func__);
	}
    
	aw9524_shdn_high = pinctrl_lookup_state(aw9524_pin, "aw9524_shdn_high");
	if (IS_ERR(aw9524_shdn_high)) {
		ret = PTR_ERR(aw9524_shdn_high);
		AW9524_LOG("%s : pinctrl err, aw9524_shdn_high\n", __func__);
	}

	aw9524_shdn_low = pinctrl_lookup_state(aw9524_pin, "aw9524_shdn_low");
	if (IS_ERR(aw9524_shdn_low)) {
		ret = PTR_ERR(aw9524_shdn_low);
		AW9524_LOG("%s : pinctrl err, aw9524_shdn_low\n", __func__);
	}
	
	AW9524_LOG("%s : pinctrl init 11111111\n", __func__);
	return ret;
}

static void aw9524_hw_reset(void)
{
	AW9524_LOG("%s enter\n", __func__);
	pinctrl_select_state(aw9524_pin, aw9524_shdn_low);
	msleep(5);
	pinctrl_select_state(aw9524_pin, aw9524_shdn_high);
	msleep(5);
	AW9524_LOG("%s out\n", __func__);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// i2c write and read
//////////////////////////////////////////////////////////////////////////
////////////////////////////////////
static unsigned char aw9524_i2c_write_reg(unsigned char addr, unsigned char reg_data)
{
	char ret;
	u8 wdbuf[512] = {0};

	struct i2c_msg msgs[] = {
		{
			.addr	= aw9524_i2c_client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= wdbuf,
		},
	};

	wdbuf[0] = addr;
	wdbuf[1] = reg_data;

	ret = i2c_transfer(aw9524_i2c_client->adapter, msgs, 1);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

    return ret;
}

static unsigned char aw9524_i2c_read_reg(unsigned char addr)
{
	unsigned char ret;
	u8 rdbuf[512] = {0};

	struct i2c_msg msgs[] = {
		{
			.addr	= aw9524_i2c_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rdbuf,
		},
		{
			.addr	= aw9524_i2c_client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= rdbuf,
		},
	};

	rdbuf[0] = addr;
	
	ret = i2c_transfer(aw9524_i2c_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

    return rdbuf[0];
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void aw9524_init_functioncfg(void)
{
	unsigned char var,var1;

	aw9524_i2c_write_reg(SW_RSTN_AW9524, 0x00);		// Software Reset
	
	aw9524_i2c_write_reg(P0_LED_MODE_AW9524, 0x00);	// P0: 0~7 led
	aw9524_i2c_write_reg(P0_CONFIG_AW9524, 0x00);	// P0: output Mode
	aw9524_i2c_write_reg(CTL_REG_AW9524, 0x02);	// P0: 1/4
	aw9524_i2c_write_reg(P0_0_DIM0_AW9524,0x00); //设置电流等级64
	aw9524_i2c_write_reg(P0_1_DIM0_AW9524,0x00);
	aw9524_i2c_write_reg(P0_2_DIM0_AW9524,0x40);
	aw9524_i2c_write_reg(P0_3_DIM0_AW9524,0x00);
	aw9524_i2c_write_reg(P0_4_DIM0_AW9524,0x00);
	aw9524_i2c_write_reg(P0_5_DIM0_AW9524,0x00);	
	//aw9524_i2c_write_reg(P0_OUTPUT_AW9524,0x00);	// P0: 0000 0000
	
	aw9524_i2c_write_reg(P1_LED_MODE_AW9524, 0x00);	// P1: 0~7 led
	aw9524_i2c_write_reg(P1_CONFIG_AW9524, 0x00);	// P1: output Mode
	aw9524_i2c_write_reg(P1_0_DIM0_AW9524,0x00); //设置电流等级64
	aw9524_i2c_write_reg(P1_1_DIM0_AW9524,0x00);
	aw9524_i2c_write_reg(P1_2_DIM0_AW9524,0x00);
	aw9524_i2c_write_reg(P1_3_DIM0_AW9524,0x00);
	aw9524_i2c_write_reg(P1_4_DIM0_AW9524,0x00);
	aw9524_i2c_write_reg(P1_5_DIM0_AW9524,0x00);	
	//aw9524_i2c_write_reg(P1_OUTPUT_AW9524, 0x00);	// P1: 0000 0000
	
	var = aw9524_i2c_read_reg(P0_OUTPUT_AW9524);
	AW9524_LOG("aw9524_init_functioncfg  P0_OUTPUT_AW9524 = 0x%0X\n", var);	
	var1 = aw9524_i2c_read_reg(P1_OUTPUT_AW9524);
	AW9524_LOG("aw9524_init_functioncfg  P1_OUTPUT_AW9524 = 0x%0X\n", var1);
	

	AW9524_LOG("aw9524_init_functioncfg\n");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Debug
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t aw9524_get_reg(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned char reg_val;
	ssize_t len = 0;
	u8 i;
	for(i=0;i<0x30;i++)
	{
		reg_val = aw9524_i2c_read_reg(i);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg%2X = 0x%2X, ", i,reg_val);
	}
	return len;
}

static ssize_t aw9524_set_reg(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	unsigned int databuf[2];
	if(2 == sscanf(buf,"%x %x",&databuf[0], &databuf[1]))
	{
		aw9524_i2c_write_reg(databuf[0],databuf[1]);
	}
	return len;
}

static int aw9524_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	err = device_create_file(dev, &dev_attr_aw9524_reg);
		
	AW9524_LOG("aw9524_create_sysfs\n");
	return err;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef  CONFIG_GREAT_AW9524_LED_PROBE
extern unsigned int hdmi_det_gpio;
extern unsigned int keyboardlight_flag;

static void gpio_main_pwm_keyboardlight(u32 data1, u32 data2)
{
	struct pwm_spec_config pwm_setting;
	if (data1 == 0x0) {
		keyboardlight_flag = 0;
		if (!gpio_get_value(hdmi_det_gpio)) {
			aeon_gpio_set("sil9022_hdmi_hplg0");//GPIO178
		}
	}
	else
	{
		keyboardlight_flag = 1;
		aeon_gpio_set("sil9022_hdmi_hplg1");//GPIO178
	}
	pwm_setting.pwm_no	= PWM1;
	pwm_setting.mode	= PWM_MODE_FIFO;
	pwm_setting.clk_div = CLK_DIV8;
	pwm_setting.clk_src = PWM_CLK_NEW_MODE_BLOCK;
	pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = false;
	pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = false;
	pwm_setting.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 63;
	pwm_setting.PWM_MODE_FIFO_REGS.HDURATION = 1;
	pwm_setting.PWM_MODE_FIFO_REGS.LDURATION = 1;
	pwm_setting.PWM_MODE_FIFO_REGS.GDURATION = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM  = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 = data1;
	pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA1 = data2;
	pwm_set_spec_config(&pwm_setting);
}

static ssize_t aw9524_led_proc_fops_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
    char str_buf[3] = {0};
	unsigned char reg_val_0,reg_val_1;
	unsigned char brightness=10,brightness_red=8;
    if (copy_from_user(str_buf, buff, count) ){
        pr_warn("copy_from_user---error\n");
        return -EFAULT;
    }   
	reg_val_0 = aw9524_i2c_read_reg(P0_OUTPUT_AW9524);
	AW9524_LOG("start P0_OUTPUT_AW9524 reg_val_0 = 0x%2X\n", reg_val_0);
	reg_val_1 = aw9524_i2c_read_reg(P1_OUTPUT_AW9524);
	AW9524_LOG("start P1_OUTPUT_AW9524 reg_val_1 = 0x%2X\n", reg_val_1);
	AW9524_LOG("start  str_buf = 0x%d\n", str_buf[2]);
	if (str_buf[0]== '1') {
		if (str_buf[1]== '1') {
			aw9524_i2c_write_reg(P0_0_DIM0_AW9524,(((int)str_buf[2]-48)*brightness_red));
		} else if (str_buf[1]== '2') {
			aw9524_i2c_write_reg(P0_1_DIM0_AW9524,(((int)str_buf[2]-48)*brightness));
		} else if (str_buf[1]== '3') {
			aw9524_i2c_write_reg(P0_2_DIM0_AW9524,(((int)str_buf[2]-48)*brightness));
		} 
	} else if (str_buf[0]== '7'){	
		 if (str_buf[1]== '1') {
			aw9524_i2c_write_reg(P0_3_DIM0_AW9524,(((int)str_buf[2]-48)*brightness_red));
		} else if (str_buf[1]== '2') {
			aw9524_i2c_write_reg(P0_4_DIM0_AW9524,(((int)str_buf[2]-48)*brightness));
		} else if (str_buf[1]== '3') {
			aw9524_i2c_write_reg(P0_5_DIM0_AW9524,(((int)str_buf[2]-48)*brightness));
		} 
	} else if (str_buf[0]== '2'){
		if (str_buf[1]== '1') {
			aw9524_i2c_write_reg(P1_0_DIM0_AW9524,(((int)str_buf[2]-48)*brightness_red));
		} else if (str_buf[1]== '2') {
			aw9524_i2c_write_reg(P1_1_DIM0_AW9524,(((int)str_buf[2]-48)*brightness));
		} else if (str_buf[1]== '3') {
			aw9524_i2c_write_reg(P1_2_DIM0_AW9524,(((int)str_buf[2]-48)*brightness));
		} 
	} else if (str_buf[0]== '3'){	
		 if (str_buf[1]== '1') {
			aw9524_i2c_write_reg(P1_3_DIM0_AW9524,(((int)str_buf[2]-48)*brightness_red));
		} else if (str_buf[1]== '2') {
			aw9524_i2c_write_reg(P1_4_DIM0_AW9524,(((int)str_buf[2]-48)*brightness));
		} else if (str_buf[1]== '3') {
			aw9524_i2c_write_reg(P1_5_DIM0_AW9524,(((int)str_buf[2]-48)*brightness));
		} 
	} else if (str_buf[0]== '6'){
		if (str_buf[1]== '1') {
			//aw9524_i2c_write_reg(P0_0_DIM0_AW9524,(((int)str_buf[2]-48)*brightness_red)); 
			aw9524_i2c_write_reg(P0_3_DIM0_AW9524,(((int)str_buf[2]-48)*brightness_red));
			aw9524_i2c_write_reg(P1_0_DIM0_AW9524,(((int)str_buf[2]-48)*brightness_red));
			aw9524_i2c_write_reg(P1_3_DIM0_AW9524,(((int)str_buf[2]-48)*brightness_red));
		} else  if (str_buf[1]== '2'){
			//aw9524_i2c_write_reg(P0_1_DIM0_AW9524,(((int)str_buf[2]-48)*brightness));
			aw9524_i2c_write_reg(P0_4_DIM0_AW9524,(((int)str_buf[2]-48)*brightness));
			aw9524_i2c_write_reg(P1_1_DIM0_AW9524,(((int)str_buf[2]-48)*brightness));
			aw9524_i2c_write_reg(P1_4_DIM0_AW9524,(((int)str_buf[2]-48)*brightness));
		} else if (str_buf[1]== '3') {
			//aw9524_i2c_write_reg(P0_2_DIM0_AW9524,(((int)str_buf[2]-48)*brightness));
			aw9524_i2c_write_reg(P0_5_DIM0_AW9524,(((int)str_buf[2]-48)*brightness));
			aw9524_i2c_write_reg(P1_2_DIM0_AW9524,(((int)str_buf[2]-48)*brightness));
			aw9524_i2c_write_reg(P1_5_DIM0_AW9524,(((int)str_buf[2]-48)*brightness));
		} 
	} else if (str_buf[0]== '4'){  //keyboard light
		if (str_buf[1]== '0') {	
			gpio_main_pwm_keyboardlight(0x00000000, 0x00000000); //0
		} else if (str_buf[1]== '1') {
			gpio_main_pwm_keyboardlight(0x00001fff, 0x00000000); //20
		} else if (str_buf[1]== '2') {
			gpio_main_pwm_keyboardlight(0x07ffffff, 0x00000000); //40
		} else if (str_buf[1]== '3') {
			gpio_main_pwm_keyboardlight(0xffffffff, 0x0000003F); //60
		}else if (str_buf[1]== '4') {
			gpio_main_pwm_keyboardlight(0xffffffff, 0x000fffff); //80
		}else if (str_buf[1]== '5') {
			gpio_main_pwm_keyboardlight(0xffffffff, 0xffffffff); //100
		}
	}else{
		AW9524_LOG("proc end!\n");
	}

	reg_val_0 = aw9524_i2c_read_reg(P0_OUTPUT_AW9524);
	AW9524_LOG("end P0_OUTPUT_AW9524 reg_val_0 = 0x%2X\n", reg_val_0);
	
	reg_val_1 = aw9524_i2c_read_reg(P1_OUTPUT_AW9524);
	AW9524_LOG("end P1_OUTPUT_AW9524 reg_val_1 = 0x%2X\n", reg_val_1);  
    AW9524_LOG("%s \n", str_buf);	
    return count;
}

static void aw9524_brightness_set(struct led_classdev *led_cdev,
								 enum led_brightness val)
{
	switch ((int)val) {
		case 0:
			gpio_main_pwm_keyboardlight(0x00000000, 0x00000000); //0
			break;
		case 1:
			gpio_main_pwm_keyboardlight(0x00001fff, 0x00000000); //20
			break;
		case 2:
			gpio_main_pwm_keyboardlight(0x07ffffff, 0x00000000); //40
			break;
		case 3:
			gpio_main_pwm_keyboardlight(0xffffffff, 0x0000003F); //60
			break;
		case 4:
			gpio_main_pwm_keyboardlight(0xffffffff, 0x000fffff); //80
			break;
		case 5:
			gpio_main_pwm_keyboardlight(0xffffffff, 0xffffffff); //100
			break;
	}
}

static const struct file_operations aw9524_led_proc_fops = { 
	.write = aw9524_led_proc_fops_write
};
#endif 	

static int aw9524_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	unsigned char reg_value = 0;
	int err = 0;
	unsigned char cnt = 5;
	AW9524_LOG("%s start\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}
	aw9524_i2c_client = client;
	i2c_set_clientdata(client, aw9524_key);
	
	aw9524_hw_reset();
#if 1
	// CHIP ID
	while((cnt>0)&&(reg_value != 0x23)){
		reg_value = aw9524_i2c_read_reg(0x10);
		printk("aw9524 chipid=0x%2x\n", reg_value);
    		cnt --;
	    	msleep(10);
		
	}
	if(!cnt){
		aw9524_i2c_flag = 0;
		err = -ENODEV;
		goto exit_create_singlethread;
	}
#endif	
	#ifdef  CONFIG_GREAT_AW9524_LED_PROBE
	proc_create("aw9524_led_proc", 0777, NULL, &aw9524_led_proc_fops);
	AW9524_LOG("aw9524_led_proc\n");
	#endif

	aw9524_key->dev_aw9524 = &client->dev;
	aw9524_key->is_screen_on_aw9524 = 1;
	
	mutex_init(&aw9524_lock);
	
	aw9524_init_functioncfg();

#ifdef CONFIG_AW9524_FB
	aw9524_key->fb_notif_aw9524.notifier_call = aw9524_fb_notifier_callback;
    err = fb_register_client(&aw9524_key->fb_notif_aw9524);
    if (err) {
		pr_err("%s: Unable to register aw9524_key fb_notifier: %d\n", __func__, err);
	}
	else {
		pr_info("%s: Success to register aw9524_key fb_notifier.\n", __func__);
	}
#endif	

	aw9524_create_sysfs(client);
	
	aw9524_i2c_flag =1;
	AW9524_LOG("aw9524 chipid end\n");
	
	return 0;

exit_create_singlethread:
	aw9524_i2c_client = NULL;
exit_check_functionality_failed:
	return err;	
}

#ifdef AW9524_EARLAY_SUSPEND
static void aw9524_i2c_early_suspend(struct i2c_client *client)
{
	struct aw9524_key_data *aw9524_key = i2c_get_clientdata(client);
	disable_irq_nosync(aw9524_key->irq_aw9524);
	AW9524_LOG("%s\n", __func__);
	return ;
}
/*----------------------------------------------------------------------------*/
static void aw9524_i2c_early_resume(struct i2c_client *client)
{      
	struct aw9524_key_data *aw9524_key = i2c_get_clientdata(client);
	aw9524_hw_reset();
	msleep(5);
	aw9524_init_functioncfg();
	msleep(10);
	enable_irq(aw9524_key->irq_aw9524);
	AW9524_LOG("%s\n", __func__);
	return ;
}
#endif

static int aw9524_i2c_remove(struct i2c_client *client)
{
	struct aw9524_key_data *aw9524_key = i2c_get_clientdata(client);

	AW9524_LOG("%s enter\n", __func__);
	
	cancel_work_sync(&aw9524_key->eint_work_aw9524);
	input_unregister_device(aw9524_key->input_dev_aw9524);
	
	kfree(aw9524_key);
	
	aw9524_i2c_client = NULL;
	i2c_set_clientdata(client, NULL);
#ifdef CONFIG_AW9524_FB
		fb_unregister_client(&aw9524_key->fb_notif_aw9524);
#endif

	return 0;
}

static const struct i2c_device_id aw9524_i2c_id[] = {
	{ AW9524_I2C_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static const struct of_device_id aw9524_extgpio_of_match[] = {
	{.compatible = "mediatek,aw9524_key"},
	{},
};
#endif

static struct i2c_driver aw9524_i2c_driver = {
	.driver = {
		.name   = AW9524_I2C_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = aw9524_extgpio_of_match,
#endif
	},
	.probe          = aw9524_i2c_probe,
	.remove         = aw9524_i2c_remove,
	.id_table       = aw9524_i2c_id,
};


static int aw9524_key_remove(struct platform_device *pdev)
{
	AW9524_LOG("aw9524 remove\n");
	i2c_del_driver(&aw9524_i2c_driver);
	return 0;
}
static int aw9524_key_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret = 0;

	AW9524_LOG("%s start!\n", __func__);

	AW9524_LOG("%s: kzalloc\n", __func__);
	aw9524_key = kzalloc(sizeof(*aw9524_key), GFP_KERNEL);
	if (!aw9524_key)	{
		ret = -ENOMEM;
		return ret;
	}

	ret = aw9524_pinctrl_init(pdev);
	if (ret != 0) {
		AW9524_LOG("[%s] failed to init aw9524 pinctrl.\n", __func__);
		return ret;
	} else {
		AW9524_LOG("[%s] Success to init aw9524 pinctrl.\n", __func__);
	}
	
	ret = i2c_add_driver(&aw9524_i2c_driver);
	if (ret != 0) {
		AW9524_LOG("[%s] failed to register aw9524 i2c driver.\n", __func__);
		return ret;
	} else {
		AW9524_LOG("[%s] Success to register aw9524 i2c driver.\n", __func__);
	}

	AW9524_FUN("RegisterLED");
	aw9524_key->led_dev.max_brightness = LED_MAX_BRIGHTNESS;
	aw9524_key->led_dev.brightness_set = aw9524_brightness_set;
	aw9524_key->led_dev.name = DEFAULT_LED_NAME;

	ret = devm_led_classdev_register(dev, &aw9524_key->led_dev);
	if (ret) {
		dev_err(dev, "led register err: %d\n", ret);
		return ret;
	}

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id aw9524plt_of_match[] = {
	{.compatible = "mediatek,aw9524_key"},
	{},
};
#endif

static struct platform_driver aw9524_key_driver = {
		.probe	 = aw9524_key_probe,
		.remove	 = aw9524_key_remove,
        .driver = {
                .name   = "aw9524_key",
#ifdef CONFIG_OF
				.of_match_table = aw9524plt_of_match,
#endif
        }
};

static int __init aw9524_key_init(void) {
	int ret;
	AW9524_LOG("%s start\n", __func__);
	
	ret = platform_driver_register(&aw9524_key_driver);
	if (ret) {
		AW9524_LOG("****[%s] Unable to register driver (%d)\n", __func__, ret);
		return ret;
	}		
	return 0;
}

static void __exit aw9524_key_exit(void) {
	AW9524_LOG("%s exit\n", __func__);
	flush_workqueue(aw9524_wq);
	destroy_workqueue(aw9524_wq);
	free_irq(aw9524_key->irq_aw9524, NULL);
	platform_driver_unregister(&aw9524_key_driver);
}

module_init(aw9524_key_init);
module_exit(aw9524_key_exit);

MODULE_AUTHOR("<liweilei@awinic.com.cn>");
MODULE_DESCRIPTION("AWINIC aw9524 Key Driver");
MODULE_LICENSE("GPL");
