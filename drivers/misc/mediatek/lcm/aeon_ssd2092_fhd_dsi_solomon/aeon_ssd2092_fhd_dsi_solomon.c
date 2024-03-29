#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
/*#include <mach/mt_pm_ldo.h>*/
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#endif
#endif
#ifdef CONFIG_MTK_LEGACY
#include <cust_gpio_usage.h>
#endif
#ifndef CONFIG_FPGA_EARLY_PORTING
#if defined(CONFIG_MTK_LEGACY)
#include <cust_i2c.h>
#endif
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

extern int aeon_gpio_set(const char *name);
extern void lp3101_poweron(void);

static LCM_UTIL_FUNCS lcm_util;

#define LCM_ID_SSD2092                  0x01572098

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define MDELAY(n)       (lcm_util.mdelay(n))
#define UDELAY(n)       (lcm_util.udelay(n))

#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
        lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
        lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
        lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
        lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define set_gpio_lcd_enp(cmd) \
		lcm_util.set_gpio_lcd_enp_bias(cmd)
#if 0		
#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>


#ifndef CONFIG_FPGA_EARLY_PORTING

#define LP_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL    /* for I2C channel 0 */
#define I2C_ID_NAME "lp3101"
#define LP_ADDR 0x3E

#if defined(CONFIG_MTK_LEGACY)
static struct i2c_board_info lp3101_board_info __initdata = {I2C_BOARD_INFO(I2C_ID_NAME, LP_ADDR)};
#endif
#if !defined(CONFIG_MTK_LEGACY)
static const struct of_device_id lcm_of_match[] = {
		{.compatible = "mediatek,I2C_LCD_BIAS"},
		{},
};
#endif

static struct i2c_client *lp3101_i2c_client;

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int lp3101_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int lp3101_remove(struct i2c_client *client);
/*****************************************************************************
 * Data Structure
 *****************************************************************************/
struct lp3101_dev	{	
	struct i2c_client *client;

};

static const struct i2c_device_id lp3101_id[] = {
	{I2C_ID_NAME, 0},
	{}
};

/* #if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)) */
/* static struct i2c_client_address_data addr_data = { .forces = forces,}; */
/* #endif */
static struct i2c_driver lp3101_iic_driver = {
	.id_table	= lp3101_id,
	.probe		= lp3101_probe,
	.remove		= lp3101_remove,
	/* .detect               = mt6605_detect, */
	.driver = {
		.owner = THIS_MODULE,
		.name	= "lp3101",
#if !defined(CONFIG_MTK_LEGACY)
			.of_match_table = lcm_of_match,
#endif
	},
};

/*****************************************************************************
 * Function
 *****************************************************************************/
static int lp3101_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	LCM_LOGI("lp3101_iic_probe ssd2092\n");
	LCM_LOGI("LP: info==>name=%s addr=0x%x  ssd2092\n",client->name,client->addr);
	lp3101_i2c_client  = client;		
	return 0;
}

static int lp3101_remove(struct i2c_client *client)
{
	LCM_LOGI("lp3101_remove ssd2092\n");
	lp3101_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}

static int lp3101_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = lp3101_i2c_client;
	char write_data[2] = { 0 };
	if (client == NULL) {
		LCM_LOGI("i2c_client = NULL, skip lp3101_write_bytes ssd2092\n");
		return 0;
	}
	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		LCM_LOGI("lp3101 write data fail ssd2092!!\n");
	return ret;
}

static int __init lp3101_iic_init(void)
{
	LCM_LOGI( "lp3101_iic_init ssd2092\n");
#if defined(CONFIG_MTK_LEGACY)
	i2c_register_board_info(LP_I2C_BUSNUM, &lp3101_board_info, 1);
#endif
	i2c_add_driver(&lp3101_iic_driver);
	LCM_LOGI( "lp3101_iic_init success ssd2092\n");	
	return 0;
}

static void __exit lp3101_iic_exit(void)
{
  LCM_LOGI( "lp3101_iic_exit ssd2092\n");
  i2c_del_driver(&lp3101_iic_driver);  
}

module_init(lp3101_iic_init);
module_exit(lp3101_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK lp3101 I2C Driver");
MODULE_LICENSE("GPL");
#endif
#endif
#endif
/* static unsigned char lcd_id_pins_value = 0xFF; */
//static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE									0
#define FRAME_WIDTH                                     (1080)
#define FRAME_HEIGHT                                    (2160)

#ifndef CONFIG_FPGA_EARLY_PORTING
#define GPIO_65132_EN GPIO_LCD_BIAS_ENP_PIN
#endif

#define REGFLAG_DELAY       0xFFFC
#define REGFLAG_UDELAY  0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD
#define REGFLAG_RESET_LOW   0xFFFE
#define REGFLAG_RESET_HIGH  0xFFFF

//static LCM_DSI_MODE_SWITCH_CMD lcm_switch_mode_cmd;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28,0,{}},
	{REGFLAG_DELAY, 50, {}},
	{0x10,0,{}},
	{REGFLAG_DELAY, 120, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table init_setting[] = {
	{0x28, 0, {}},
	{0x10, 0, {}},
	{0xB5, 2, {0x00,0x00}},
	{0xB3, 2, {0x03,0x96}},
	{0xB4, 2, {0x21,0xFF}},
	{0xE1, 2, {0x00,0x00}},
	{0xE1, 3, {0x03,0x00,0x10}},
	{0xB0, 2, {0x04,0x01}},
	{0xB0, 2, {0x07,0x14}},
	{0xC6, 2, {0x04,0x2D}},
	{0xB9, 2, {0x0E,0x44}},
	{0xC1, 2, {0x0C,0x26}},
	{0xC1, 2, {0x08,0x99}},
	{0xB9, 2, {0x23,0x00}},
	{0xC2, 2, {0x04,0x0F}},
	{0xB9, 2, {0x11,0x22}},
	{0xB9, 2, {0x12,0x22}},
	{0xB9, 2, {0x13,0x22}},
	{0xB3, 2, {0x19,0x0B}},
	{0xB0, 2, {0x3D,0xC8}},
	{0xB3, 2, {0x0F,0x20}},
	{0x36, 1, {0x02}},
	{0xC7, 2, {0x01,0x09}},
	{0xB3, 2, {0x15,0x80}},
	{0xB3, 2, {0x1E,0x02}},
	{0xB3, 2, {0x14,0xC0}},
	{0xB3, 2, {0x16,0x50}},
	{0xB3, 2, {0x1C,0xE4}},
	{0xB3, 2, {0x28,0x09}},
	{0xB3, 2, {0x24,0x19}},
	{0xB0, 2, {0x12,0x00}},
	{0xB0, 2, {0x13,0x14}},  //0x18
	{0xB0, 2, {0x1A,0x00}},
	{0xB0, 2, {0x1B,0x14}},  //0x18
	{0xC1, 2, {0x0D,0x87}},
	{0xC1, 2, {0x0E,0x87}},
	{0xC3, 22, {0x00,0x10,0x2E,0x3E,0x48,0x5E,0x6B,0x7A,0x83,0x97,0xA3,0xA8,0xA5,0xAC,0xAD,0xB5,0xC1,0xC4,0x4E,0x4F,0x59,0x5A}},
	{0xC3, 22, {0x15,0x10,0x2E,0x3E,0x48,0x5E,0x6B,0x7A,0x83,0x97,0xA3,0xA8,0xA5,0xAC,0xAD,0xB5,0xC1,0xC4,0x4E,0x4F,0x59,0x5A}},
	{0xBB, 2, {0x01,0x8A}},
	{0xBB, 2, {0x02,0x23}},
	{0xBB, 2, {0x03,0xBC}},
	{0xBB, 2, {0x04,0xC0}},
	{0xBB, 2, {0x05,0x19}},
	{0xBB, 2, {0x06,0x37}},
	{0xBB, 2, {0x07,0xEE}},
	{0xBB, 2, {0x08,0xDD}},
	{0xBB, 2, {0x09,0xCC}},
	{0xBB, 2, {0x0A,0x36}},
	{0xBB, 2, {0x0B,0x70}},
	{0xBB, 2, {0x0C,0x00}},
	{0xBB, 2, {0x0D,0x8A}},
	{0xBB, 2, {0x0E,0x23}},
	{0xBB, 2, {0x0F,0xBC}},
	{0xBB, 2, {0x10,0xC0}},
	{0xBB, 2, {0x11,0x18}},
	{0xBB, 2, {0x12,0xA7}},
	{0xBB, 2, {0x13,0xEE}},
	{0xBB, 2, {0x14,0xDD}},
	{0xBB, 2, {0x15,0xCC}},
	{0xBB, 2, {0x16,0x36}},
	{0xBB, 2, {0x17,0x70}},
	{0xBB, 2, {0x18,0x00}},
	{0xBB, 2, {0x31,0x01}},
	{0xBD, 2, {0x00,0xF0}},
	{0xBD, 2, {0x01,0xF0}},
	{0xBD, 2, {0x02,0xF0}},
	{0xBD, 2, {0x03,0xF0}},
	{0xBD, 2, {0x1A,0x02}},
	{0xBD, 2, {0x1B,0x24}},
	{0xBD, 2, {0x1D,0x01}},
	{0xBD, 2, {0x1E,0x01}},
	{0xBD, 2, {0x21,0x00}},
	{0xBD, 2, {0x1F,0x79}},
	{0xBD, 2, {0x20,0x14}},
	{0xBD, 2, {0x22,0x89}},
	{0xBD, 2, {0x23,0x13}},
	{0xBD, 2, {0x24,0x35}},
	{0xBD, 2, {0x26,0x01}},
	{0xBD, 2, {0x27,0x01}},
	{0xBD, 2, {0x2A,0x00}},
	{0xBD, 2, {0x28,0x79}},
	{0xBD, 2, {0x29,0x14}},
	{0xBD, 2, {0x2B,0x89}},
	{0xBD, 2, {0x2C,0x33}},
	{0xBD, 2, {0x2D,0x33}},
	{0xBD, 2, {0x2E,0x11}},
	{0xBD, 2, {0x2F,0x11}},
	{0xBD, 2, {0x31,0x22}},
	{0xBD, 2, {0x32,0x01}},
	{0xBD, 2, {0x33,0x04}},
	{0xBD, 2, {0x34,0x11}},
	{0xBD, 2, {0x35,0x11}},
	{0xBD, 2, {0x37,0x22}},
	{0xBD, 2, {0x38,0x01}},
	{0xBD, 2, {0x39,0x04}},
	{0xBD, 2, {0x3A,0x1E}},
	{0xBD, 2, {0x3B,0x1E}},
	{0xBD, 2, {0x3C,0x1E}},
	{0xBD, 2, {0x3D,0x1E}},
	{0xBD, 2, {0x40,0xAA}},
	{0xBD, 2, {0x42,0x1E}},
	{0xBD, 2, {0x43,0x1E}},
	{0xBD, 2, {0x44,0x1E}},
	{0xBD, 2, {0x45,0x1E}},
	{0xBD, 2, {0x48,0xAA}},
	{0xBA, 2, {0x02,0x05}},
	{0xBA, 2, {0x03,0x01}},
	{0xBC, 2, {0x1E,0x71}},
	{0xBC, 2, {0x24,0x83}},
	{0xB8, 2, {0x03,0xA9}},
	{0xBA, 2, {0x15,0x41}},
	{0xBB, 2, {0x19,0x8A}},
	{0xBB, 2, {0x1A,0x32}},
	{0xBB, 2, {0x1B,0xBC}},
	{0xBB, 2, {0x1C,0xC2}},
	{0xBB, 2, {0x1D,0x39}},
	{0xBB, 2, {0x1E,0x37}},
	{0xBB, 2, {0x1F,0xEE}},
	{0xBB, 2, {0x20,0xDD}},
	{0xBB, 2, {0x21,0xCC}},
	{0xBB, 2, {0x22,0x36}},
	{0xBB, 2, {0x23,0x70}},
	{0xBB, 2, {0x24,0x00}},
	{0xBB, 2, {0x25,0x8A}},
	{0xBB, 2, {0x26,0x32}},
	{0xBB, 2, {0x27,0xBC}},
	{0xBB, 2, {0x28,0xC2}},
	{0xBB, 2, {0x29,0x38}},
	{0xBB, 2, {0x2A,0xA7}},
	{0xBB, 2, {0x2B,0xEE}},
	{0xBB, 2, {0x2C,0xDD}},
	{0xBB, 2, {0x2D,0xCC}},
	{0xBB, 2, {0x2E,0x36}},
	{0xBB, 2, {0x2F,0x70}},
	{0xBB, 2, {0x30,0x00}},
	{0xB8, 2, {0x15,0xA2}},
	{0xB0, 2, {0x39,0x00}},
	{0xC6, 3, {0x00,0x03,0x40}},
	{0xB3, 2, {0x18,0x00}},
	{0xBA, 2, {0x17,0x28}},
	{0xBA, 2, {0x23,0x15}},
	{0xB0, 2, {0x06,0x2C}},
	{0xBA, 2, {0x17,0x28}},
	{0xBA, 2, {0x23,0x15}},
	{0xBA, 2, {0x16,0x00}},
	{0xBA, 3, {0x28,0x02,0x02}},
	{0xBA, 2, {0x0F,0x8C}},
	{0xBA, 2, {0x10,0x0C}},
	{0xBA, 2, {0x11,0x23}},
	{0xBA, 4, {0x12,0x9D,0x9D,0x9D}},
	{0xBA, 2, {0x25,0x22}},
	{0xBA, 3, {0x0D,0x40,0x02}},
	{0xBA, 2, {0x2F,0x8C}},
	{0xC2, 2, {0x04,0x00}},
	{0xC2, 2, {0x05,0xC0}},
	{0xB9, 10, {0x11,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0xBA, 2, {0x15,0x33}},
	{0xB9, 5, {0x00,0x5F,0x5F,0x61,0x0A}},
	{0xBD, 2, {0x08,0x07}},
	{0xBD, 2, {0x09,0x07}},
	{0xBC, 2, {0x03,0x00}},
	{0xBC, 2, {0x04,0x00}},
	{0xBC, 2, {0x05,0x00}},
	{0xBC, 2, {0x07,0x10}},
	{0xBC, 2, {0x08,0x10}},
	{0xBC, 2, {0x09,0x10}},
	{0xB9, 2, {0x11,0x00}},
	{0xB9, 2, {0x12,0x00}},
	{0xB9, 2, {0x13,0x00}},
	{0xB9, 2, {0x1D,0x00}},
	{0xB9, 2, {0x1E,0x00}},
	{0xB9, 2, {0x1F,0x00}},
	{0xC2, 2, {0x00,0x07}},
	{0xC1, 2, {0x0B,0x1F}},
	{0xB1, 2, {0x05,0x2F}},
	{0x35, 1, {0x00}},
	{0xB0, 3, {0x01,0x08,0x70}},
	{0xB9, 2, {0x25,0xA0}},
	{0xBA, 2, {0x2F,0xCC}},
	{0xC1, 2, {0x02,0xB0}},
	{0xB3, 2, {0x1C,0xC4}},
	{0xB3, 2, {0x1D,0x80}},
	{0xBD, 3, {0x11,0x02,0x22}},
    {0x36, 1, {0x09}},  //0x0A
 
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    {0x29, 1,{0x00}},
	{REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(void *cmdq, struct LCM_setting_table *table,
	unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;

		switch (cmd) {

			case REGFLAG_DELAY:
				if (table[i].count <= 10)
					MDELAY(table[i].count);
				else
					MDELAY(table[i].count);
				break;

			case REGFLAG_UDELAY:
				UDELAY(table[i].count);
				break;

			case REGFLAG_END_OF_TABLE:
				break;

			default:
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}


static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

    params->type   = LCM_TYPE_DSI;

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
	params->physical_width = 68;
	params->physical_height = 136;
    // enable tearing-free
    params->dbi.te_mode 			= LCM_DBI_TE_MODE_DISABLED;
    //params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

    #if (LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
    #else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;
    #endif

    // DSI
    /* Command mode setting */
    //1 Three lane or Four lane
    params->dsi.LANE_NUM				= LCM_FOUR_LANE;

    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size=256;
    // Video mode setting
    params->dsi.intermediat_buffer_num = 2;

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.word_count=FRAME_WIDTH*3;	//DSI CMD mode need set these two bellow params, different to 6577
    //params->dsi.vertical_active_line=FRAME_HEIGHT;


	params->dsi.vertical_sync_active				= 1;	//10
	params->dsi.vertical_backporch = 43;
	params->dsi.vertical_frontporch = 76;
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 4;
	params->dsi.horizontal_backporch				= 20;
	params->dsi.horizontal_frontporch				= 26;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	params->dsi.PLL_CLOCK=502;	
	params->dsi.ssc_disable = 1;
	params->dsi.noncont_clock=TRUE;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 0;
	//params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
	//params->dsi.lcm_esd_check_table[0].count        = 1;
	//params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;		
}

#ifdef BUILD_LK
#ifndef CONFIG_FPGA_EARLY_PORTING
#define lp3101_SLAVE_ADDR_WRITE  0x7C  
static struct mt_i2c_t lp3101_i2c;

static int lp3101_write_byte(kal_uint8 addr, kal_uint8 value)
{
	kal_uint32 ret_code = I2C_OK;
	kal_uint8 write_data[2];
	kal_uint16 len;

	write_data[0] = addr;
	write_data[1] = value;

  lp3101_i2c.id = 1; /* I2C1; */
	/* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
  lp3101_i2c.addr = (lp3101_SLAVE_ADDR_WRITE >> 1);
  lp3101_i2c.mode = ST_MODE;
  lp3101_i2c.speed = 400;//100;
	len = 2;

  ret_code = i2c_write(&lp3101_i2c, write_data, len);
	/* printf("%s: i2c_write: ret_code: %d\n", __func__, ret_code); */

	return ret_code;
}
#else
/* extern int mt8193_i2c_write(u16 addr, u32 data); */
/* extern int mt8193_i2c_read(u16 addr, u32 *data); */

/* #define TPS65132_write_byte(add, data)  mt8193_i2c_write(add, data) */
/* #define TPS65132_read_byte(add)  mt8193_i2c_read(add) */

#endif
#endif

static void lcm_init_power(void)
{
	printk("SSD2092 lcm_init_power\n");
	display_bias_enable();
}

static void lcm_suspend_power(void)
{
	printk("SSD2092 lcm_suspend_power\n");
	display_bias_disable();
}

static void lcm_resume_power(void)
{
	printk("SSD2092 lcm_resume_power\n");
	SET_RESET_PIN(0);
	display_bias_enable();
}
 
static void lcm_poweron(void)
{
	printk("SSD2092 lcm_poweron=start====\n");
	SET_RESET_PIN(0);
	//aeon_gpio_set("aeon_lcd_bias_enp1");
	//aeon_gpio_set("aeon_lcd_bias_enn1");


	MDELAY(20);
	//lp3101_poweron();
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(20);	
	printk("SSD2092 lcm_poweron==end====\n");
}

static void lcm_init(void)
{
	printk("SSD2092 lcm_init enter!\n");
	push_table(0,init_setting, sizeof(init_setting) / sizeof(struct LCM_setting_table), 1);
	printk("SSD2092 lcm_init end!\n");
}

static void lcm_suspend(void)
{
	printk("SSD2092 lcm_suspend enter!\n");

	push_table(0, lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(10);
	
	SET_RESET_PIN(0);
	//aeon_gpio_set("aeon_lcd_bias_enn0");
	//aeon_gpio_set("aeon_lcd_bias_enp0");
	printk("SSD2092 lcm_suspend end!\n");
}

static void lcm_resume(void)
{
	lcm_poweron();
	lcm_init();
	printk("SSD2092 lcm_resume\n");
}


static unsigned int lcm_compare_id(void)
{

#ifdef BUILD_LK
	unsigned int id0,id1,id2,id3;
	unsigned int id = 0;	
	unsigned char buffer[4];
        printk("%s,ssd2092.\n", __func__);

	read_reg_v2(0xA1, buffer,4); 

	id0 = buffer[0];
	id1 = buffer[1];
	id2 = buffer[2];
	id3 = buffer[3];
	id = (id0<<24)|(id1<<16)|(id2<<8)|(id3<<0);

	LCM_LOGI("%s,ssd2092 debug: ssd2092 id = 0x%08x\n", __func__, id);

	if (id == LCM_ID_SSD2092)
		return 1;
	else
		return 0;
		
#else
	return 1;
	
#endif
}

/* return TRUE: need recovery */
/* return FALSE: No need recovery */

LCM_DRIVER aeon_ssd2092_fhd_dsi_solomon_lcm_drv = 
{
    .name			= "aeon_ssd2092_fhd_dsi_solomon",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.compare_id = lcm_compare_id,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.resume         = lcm_resume,
	.suspend        = lcm_suspend,
};
