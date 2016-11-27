/**
 *	Revision history
 *
 *
 *	Sep 24 2016
 *
 *	I.nfraR.ed
 *	Use nt35516_lg50_yushun_qhd as a base driver and tune for Lenovo A820/A830
 *	Gamma is still a little bit off
 *
 */

#ifdef BUILD_LK
#else
    #include <linux/string.h>
    #include <linux/kernel.h>
#if defined(BUILD_UBOOT)
    #include <asm/arch/mt_gpio.h>
#else
    #include <mach/mt_gpio.h>
#endif
#endif
#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(540)
#define FRAME_HEIGHT 										(960)
#define LCM_DSI_CMD_MODE

#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0x00   // END OF REGISTERS MARKER

#define GPIO_LCD_RST_EN      								GPIO131
#define LCM_NT35516_ID       								(0x80)

//#define TEST

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};
static struct LCM_setting_table lcm_compare_id_setting[] = {
    // Display off sequence
    {0xF0,  5,      {0x55,0xAA,0x52,0x08,0x01}},
    {REGFLAG_DELAY, 10, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},

    // Sleep Mode On
	{0x10, 1, {0x00}},

	{REGFLAG_DELAY, 50, {}},
	{0x4F, 1, {0x01}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_initialization_setting[] = {

	/*
	Note :

	Data ID will depends on the following rule.

		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag

	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/


	{0xFF,4,{0xAA,0x55,0x25,0x01}},
	{0xF3,7,{0x02,0x03,0x07,0x45,0x88,0xD4,0x0D}},
	{0xF4,5,{0x00,0x48,0x00,0x20,0x40}},
	{0xFA,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x11}},

	{0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
	{0xB0,5,{0x00,0x0C,0x40,0x3C,0x3C}},
	{0xB1,2,{0xEC,0x00}},
	{0xB6,1,{0x08}},
	{0xB7,2,{0x72,0x72}},
	{0xBA,1,{0x05}},
	{0xBC,1,{0x04}},

    //Modified to shorten TE period since it is 5.5 ms before
	{0xBD,5,{0x01,0x41,0x10,0x37,0x01}},
//	{0xBD,5,{0x01,0x41,0x7,0x39,0x01}},
	{0xCC,1,{0x01}},

	{0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},
	{0xB0,3,{0x0A,0x0A,0x0A}},
	{0xB6,3,{0x54,0x54,0x54}},
	{0xB1,3,{0x0A,0x0A,0x0A}},
	{0xB7,3,{0x24,0x24,0x24}},
	{0xB2,3,{0x03,0x03,0x03}},
	{0xB8,3,{0x30,0x30,0x30}},
	{0xB3,3,{0x0D,0x0D,0x0D}},
	{0xB9,3,{0x24,0x24,0x24}},
	{0xB4,3,{0x0A,0x0A,0x0A}},
	{0xBA,3,{0x24,0x24,0x24}},
	{0xB5,3,{0x07,0x07,0x07}},
	{0xBC,3,{0x00,0xA0,0x01}},
	{0xBD,3,{0x00,0xA0,0x01}},
	{0xBE,1,{0x48}},

	{0xD1,16,{0x00,0x00,0x00,0x68,0x00,0xB1,0x00,0xE4,0x00,0xFD,0x01,0x2D,0x01,0x4C,0x01,0x79}},
	{0xD2,16,{0x01,0x9B,0x01,0xD3,0x02,0x00,0x02,0x4A,0x02,0x87,0x02,0x88,0x02,0xC0,0x02,0xF8}},
	{0xD3,16,{0x03,0x1A,0x03,0x48,0x03,0x63,0x03,0x8C,0x03,0xA4,0x03,0xCC,0x03,0xE2,0x03,0xEF}},
	{0xD4,4,{0x03,0xF4,0x03,0xFF}},

	{0xD5,16,{0x00,0x00,0x00,0x68,0x00,0xB1,0x00,0xE4,0x00,0xFD,0x01,0x2D,0x01,0x4C,0x01,0x79}},
	{0xD6,16,{0x01,0x9B,0x01,0xD3,0x02,0x00,0x02,0x4A,0x02,0x87,0x02,0x88,0x02,0xC0,0x02,0xF8}},
	{0xD7,16,{0x03,0x1A,0x03,0x48,0x03,0x63,0x03,0x8C,0x03,0xA4,0x03,0xCC,0x03,0xE2,0x03,0xEF}},
	{0xD8,4,{0x03,0xF4,0x03,0xFF}},

	{0xD9,16,{0x00,0x00,0x00,0x68,0x00,0xB1,0x00,0xE4,0x00,0xFD,0x01,0x2D,0x01,0x4C,0x01,0x79}},
	{0xDD,16,{0x01,0x9B,0x01,0xD3,0x02,0x00,0x02,0x4A,0x02,0x87,0x02,0x88,0x02,0xC0,0x02,0xF8}},
	{0xDE,16,{0x03,0x1A,0x03,0x48,0x03,0x63,0x03,0x8C,0x03,0xA4,0x03,0xCC,0x03,0xE2,0x03,0xEF}},
	{0xDF,4,{0x03,0xF4,0x03,0xFF}},

	{0xE0,16,{0x00,0x00,0x00,0x68,0x00,0xB1,0x00,0xE4,0x00,0xFD,0x01,0x2D,0x01,0x4C,0x01,0x79}},
	{0xE1,16,{0x01,0x9B,0x01,0xD3,0x02,0x00,0x02,0x4A,0x02,0x87,0x02,0x88,0x02,0xC0,0x02,0xF8}},
	{0xE2,16,{0x03,0x1A,0x03,0x48,0x03,0x63,0x03,0x8C,0x03,0xA4,0x03,0xCC,0x03,0xE2,0x03,0xEF}},
	{0xE3,4,{0x03,0xF4,0x03,0xFF}},

	{0xE4,16,{0x00,0x00,0x00,0x68,0x00,0xB1,0x00,0xE4,0x00,0xFD,0x01,0x2D,0x01,0x4C,0x01,0x79}},
	{0xE5,16,{0x01,0x9B,0x01,0xD3,0x02,0x00,0x02,0x4A,0x02,0x87,0x02,0x88,0x02,0xC0,0x02,0xF8}},
	{0xE6,16,{0x03,0x1A,0x03,0x48,0x03,0x63,0x03,0x8C,0x03,0xA4,0x03,0xCC,0x03,0xE2,0x03,0xEF}},
	{0xE7,4,{0x03,0xF4,0x03,0xFF}},

	{0xE8,16,{0x00,0x00,0x00,0x68,0x00,0xB1,0x00,0xE4,0x00,0xFD,0x01,0x2D,0x01,0x4C,0x01,0x79}},
	{0xE9,16,{0x01,0x9B,0x01,0xD3,0x02,0x00,0x02,0x4A,0x02,0x87,0x02,0x88,0x02,0xC0,0x02,0xF8}},
	{0xEA,16,{0x03,0x1A,0x03,0x48,0x03,0x63,0x03,0x8C,0x03,0xA4,0x03,0xCC,0x03,0xE2,0x03,0xEF}},
	{0xEB,4,{0x03,0xF4,0x03,0xFF}},

	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.
/*	{0x3A,1,{0x77}},

	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 150, {}},

	{0x29,1,{0x00}},
	{REGFLAG_DELAY, 40, {}},

	{0x2C,1,{0x00}},
*/
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},

	{0x44,2,{(FRAME_HEIGHT/2)>>8,(FRAME_HEIGHT/2)&0xFF}},
	{0x35,1,{0x00}},

	{0xB1,2,{0xE8,0x00}},

	{0xE0,2,{0x01,0x03}},

	{0x3A,1,{0x77}},

	{0x4C,1,{0x11}}, //vivid color by zxy

	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 120, {}},

	{0x29,1,{0x00}},

	//[15.11.2016] I.nfraR.ed: Modify delay to value in stock kernel
	//{REGFLAG_DELAY, 1, {}},
	{REGFLAG_DELAY, 40, {}},

	{0x2C,1,{0x00}},
	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {

        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {

            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}

    }

}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

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

#if defined(TEST)
	// enable tearing-free
	params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
#endif

#if defined(LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif

		// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_TWO_LANE;
	//The following defined the format for data coming from LCD engine.
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
	params->dsi.PS 						= LCM_PACKED_PS_24BIT_RGB888;

#if defined(TEST)
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;

	//params->dsi.compatibility_for_nvk=1;
	params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
#endif

#if defined(TEST)
	params->dsi.word_count					=540*3;

    params->dsi.vertical_sync_active		= 0;  //---3
    params->dsi.vertical_backporch			= 10; //---14
    params->dsi.vertical_frontporch			= 44;  //----8
    params->dsi.vertical_active_line		= FRAME_HEIGHT;

    params->dsi.horizontal_sync_active		= 0;  //----2
    params->dsi.horizontal_backporch		= 60; //----28
    params->dsi.horizontal_frontporch		= 20; //----50
    params->dsi.horizontal_active_pixel		= FRAME_WIDTH;
#endif

	// Bit rate calculation
	//1 Every lane speed
	//params->dsi.pll_select=1;	//0: MIPI_PLL; 1: LVDS_PLL
	params->dsi.PLL_CLOCK = LCM_DSI_6589_PLL_CLOCK_221;

#if 0
	params->dsi.pll_div1 	= 0x00;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
	params->dsi.pll_div2 	= 0x1;		// div2=0,1,2,3;div1_real=1,2,4,4
	params->dsi.fbk_div 	= 0x12;     // fref=26MHz, fvco=fref*(fbk_div+1)*fbk_sel_real/(div1_real*div2_real)
	params->dsi.fbk_sel 	= 0x1;		// fbk_sel=0,1,2,3;fbk_select_real=1,2,4,4
	params->dsi.rg_bir 		= 0x5;
	params->dsi.rg_bic 		= 0x2;
	params->dsi.rg_bp 		= 0xC;
#else
	params->dsi.pll_div1	= 0x1;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
	params->dsi.pll_div2	= 0x1;		// div2=0,1,2,3;div1_real=1,2,4,4
	params->dsi.fbk_div		= 0x11;      // fref=26MHz, fvco=fref*(fbk_div+1)*fbk_sel_real/(div1_real*div2_real)
	params->dsi.fbk_sel 	= 0x1;		// fbk_sel=0,1,2,3;fbk_select_real=1,2,4,4
	params->dsi.rg_bir 		= 0x5;
	params->dsi.rg_bic 		= 0x2;
	params->dsi.rg_bp 		= 0xC;
#endif
	/* ESD or noise interference recovery For video mode LCM only. */ // Send TE packet to LCM in a period of n frames and check the response.
	//params->dsi.lcm_int_te_monitor = 0;
	//params->dsi.lcm_int_te_period = 1; // Unit : frames

	// Need longer FP for more opportunity to do int. TE monitor applicably.
	//if(params->dsi.lcm_int_te_monitor)
	//	params->dsi.vertical_frontporch *= 2;

	// Monitor external TE (or named VSYNC) from LCM once per 2 sec. (LCM VSYNC must be wired to baseband TE pin.)
	//params->dsi.lcm_ext_te_monitor = 0;
	// Non-continuous clock
	//params->dsi.noncont_clock = 1;
	//params->dsi.noncont_clock_period = 2; // Unit : frames
}

static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char buffer[5];
	unsigned int array[5];
	//Do reset here
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(100);

	//push_table(lcm_compare_id_setting, sizeof(lcm_compare_id_setting) / sizeof(struct LCM_setting_table), 1);

	array[0] = 0x00083700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x04, buffer, 3);
	id = ((buffer[0] << 8) | buffer[1]); //we only need ID
#if defined(BUILD_LK)
	printf("%s, id1 = 0x%x,id2 = 0x%x,id3 = 0x%x\n", __func__, buffer[0],buffer[1],buffer[2]);
#endif
	printk("%s, id1 = 0x%x,id2 = 0x%x,id3 = 0x%x\n", __func__, buffer[0],buffer[1],buffer[2]);
	return (LCM_NT35516_ID == id)?1:0;

}

static void lcm_init(void)
{
	unsigned int data_array[16];
	SET_RESET_PIN(1);
	MDELAY(20);
	SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(100);
	lcm_compare_id();
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

}


static void lcm_suspend(void)
{
	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
#ifndef BUILD_LK
	mt_set_gpio_mode(GPIO130, GPIO_MODE_GPIO);
	mt_set_gpio_mode(GPIO131, GPIO_MODE_GPIO);
	mt_set_gpio_mode(GPIO154, GPIO_MODE_GPIO);

	mt_set_gpio_dir(GPIO130, GPIO_DIR_IN);
	mt_set_gpio_dir(GPIO131, GPIO_DIR_IN);
	mt_set_gpio_dir(GPIO154, GPIO_DIR_IN);

	mt_set_gpio_pull_enable(GPIO130, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_enable(GPIO131, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_enable(GPIO154, GPIO_PULL_ENABLE);

	mt_set_gpio_pull_select(GPIO130, GPIO_PULL_DOWN);
	mt_set_gpio_pull_select(GPIO131, GPIO_PULL_DOWN);
	mt_set_gpio_pull_select(GPIO154, GPIO_PULL_DOWN);
#endif
}



static void lcm_resume(void)
{
#ifndef BUILD_LK
	mt_set_gpio_mode(GPIO131, GPIO_MODE_07);
	mt_set_gpio_dir(GPIO131, GPIO_DIR_OUT);
#endif
	lcm_init();
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(&data_array, 3, 1);

	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(&data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(&data_array, 1, 0);
}

#if (0)
static int __esd_check_count = 0;
static unsigned int lcm_esd_check(void)
{
    unsigned char buffer[5];
    unsigned int id = 0;
#ifndef BUILD_LK
    read_reg_v2(0xc5, buffer, 3);
    id = ((buffer[0] << 8) | buffer[1]);
    if(LCM_NT35516_ID == id)
    {
        __esd_check_count = 0;
        return 0;
    }
    else
    {
    #if 1
    __esd_check_count ++;
    if(__esd_check_count > 6){
        __esd_check_count = 0;
        return 1;
    }
    __esd_check_count = 0;
    return 0;
    #else
        return 1;
    #endif
    }
#endif
}

static unsigned int lcm_esd_recover(void)
{
//    unsigned char para = 0;

	lcm_init();
	lcm_resume();

    return 1;
}
#endif
LCM_DRIVER nt35516_qhd_dsi_cmd_ipsboe_lcm_drv =
{
    .name			= "nt35516_qhd_dsi_ips_boe",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
#if defined(LCM_DSI_CMD_MODE)
	.update         = lcm_update,
#endif
#if (0)
	.esd_check = lcm_esd_check,
	.esd_recover = lcm_esd_recover,
#endif
};
