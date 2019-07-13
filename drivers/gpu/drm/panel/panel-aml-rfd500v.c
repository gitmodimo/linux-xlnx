/*
 * MIPI-DSI based aml_rfd500v AMOLED LCD 5.3 inch panel driver.
 *
 * Copyright (c) 2013 Samsung Electronics Co., Ltd
 *
 * Inki Dae, <inki.dae@samsung.com>
 * Donghwa Lee, <dh09.lee@samsung.com>
 * Joongmock Shin <jmock.shin@samsung.com>
 * Eunchul Kim <chulspro.kim@samsung.com>
 * Tomasz Figa <t.figa@samsung.com>
 * Andrzej Hajda <a.hajda@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/



#define DEBUG 1
#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>



struct aml_rfd500v {
	struct device *dev;
	struct drm_panel panel;

	struct regulator_bulk_data supplies[2];
	struct gpio_desc *reset_gpio;
	u32 power_on_delay;
	u32 reset_delay;
	u32 init_delay;
	//bool flip_horizontal;
	//bool flip_vertical;
	struct videomode vm;
	u32 width_mm;
	u32 height_mm;

	u8 version;
	u8 id;
	//int brightness;

	struct kobject *kobj;
	/* This field is tested by functions directly accessing DSI bus before
	 * transfer, transfer is skipped if it is set. In case of transfer
	 * failure or unexpected response the field is set to error value.
	 * Such construct allows to eliminate many checks in higher level
	 * functions.
	 */
	int error;
};






static inline struct aml_rfd500v *panel_to_aml_rfd500v(struct drm_panel *panel)
{
	return container_of(panel, struct aml_rfd500v, panel);
}

static int aml_rfd500v_clear_error(struct aml_rfd500v *ctx)
{
	int ret = ctx->error;

	ctx->error = 0;
	return ret;
}

static void aml_rfd500v_dcs_write(struct aml_rfd500v *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return;

	ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %zd writing dcs seq: %*ph\n", ret,
			(int)len, data);
		ctx->error = ret;
	}
}





static ssize_t dcs_write_store(struct device *dev,
		 struct device_attribute *attr,
        const char *buff, size_t count)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
    struct aml_rfd500v *ctx = mipi_dsi_get_drvdata(dsi);


    int ptr=buff;
    int val=0;
    int readl=0;
    int readlt=0;
    int rdctr=0;
    int r;
    u8 d[64];
	//printk("DSC write:",val);
    while(readlt<count){
    	r=sscanf(ptr,"%x%n",&val,&readl);
    	if(r<=0)
    		break;
    	d[rdctr++]=val;
    	//printk(KERN_CONT" 0x%02x",val);
    	ptr+=readl;
    	readlt+=readl;
    }

	//printk(KERN_CONT".\n",val);
	aml_rfd500v_dcs_write(ctx,d,rdctr);
    return count;
}

static DEVICE_ATTR_WO(dcs_write);

static struct attribute *attrs[] = {
    &dev_attr_dcs_write.attr,
    NULL,
};

static struct attribute_group attr_group = {
    .attrs = attrs,
};




#define aml_rfd500v_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	aml_rfd500v_dcs_write(ctx, d, ARRAY_SIZE(d));\
})



static void aml_rfd500v_panel_init(struct aml_rfd500v *ctx)
{



	dev_dbg(ctx->dev, "aml_rfd500v_panel_init\n");


	//aml_rfd500v_dcs_write_seq_static(ctx, MIPI_DCS_EXIT_SLEEP_MODE);

	/*aml_rfd500v_apply_level_1_key(ctx);
	aml_rfd500v_apply_level_2_key(ctx);
	msleep(20);

	aml_rfd500v_dcs_write_seq_static(ctx, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(40);

	aml_rfd500v_panel_cond_set(ctx);
	aml_rfd500v_display_condition_set(ctx);
	aml_rfd500v_brightness_set(ctx);
	aml_rfd500v_etc_source_control(ctx);
	aml_rfd500v_etc_pentile_control(ctx);
	aml_rfd500v_elvss_nvm_set(ctx);
	aml_rfd500v_etc_power_control(ctx);
	aml_rfd500v_etc_elvss_control(ctx);*/
	msleep(ctx->init_delay);
}


struct LCM_setting_table {
   unsigned cmd;
   unsigned char count;
   unsigned char para_list[64];
};

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE 0xDD // END OF REGISTERS MARKER


static struct LCM_setting_table lcm_initialization_settingq[] =
{
{0xFF,5,{0xFF,0x98,0x6,0x4,0x1}},
{0x8,1,{0x10}},
{0x21,1,{0x1}},
{0x30,1,{0x1}},
{0x31,1,{0x0}},
{0x40,1,{0x16}},
{0x41,1,{0x33}},
{0x42,1,{0x3}},
{0x43,1,{0x89}},
{0x44,1,{0x6}},
{0x50,1,{0x80}},
{0x51,1,{0x80}},
{0x52,1,{0x0}},
{0x53,1,{0x43}},
{0x60,1,{0x7}},
{0x61,1,{0x0}},
{0x62,1,{0x7}},
{0x63,1,{0x0}},
{0xA0,1,{0x0}},
{0xA1,1,{0x1}},
{0xA2,1,{0x0A}},
{0xA3,1,{0x10}},
{0xA4,1,{0x0B}},
{0xA5,1,{0x1C}},
{0xA6,1,{0x0B}},
{0xA7,1,{0x9}},
{0xA8,1,{0x5}},
{0xA9,1,{0x0B}},
{0xAA,1,{0x7}},
{0xAB,1,{0x6}},
{0xAC,1,{0x0E}},
{0xAD,1,{0x29}},
{0xAE,1,{0x25}},
{0xAF,1,{0x0}},
{0xC0,1,{0x0}},
{0xC1,1,{0x2}},
{0xC2,1,{0x7}},
{0xC3,1,{0x0C}},
{0xC4,1,{0x6}},
{0xC5,1,{0x18}},
{0xC6,1,{0x0B}},
{0xC7,1,{0x0A}},
{0xC8,1,{0x2}},
{0xC9,1,{0x6}},
{0xCA,1,{0x3}},
{0xCB,1,{0x3}},
{0xCC,1,{0x0B}},
{0xCD,1,{0x2A}},
{0xCE,1,{0x25}},
{0xCF,1,{0x0}},
{0xFF,5,{0xFF,0x98,0x6,0x4,0x6}},
{0x0,1,{0x20}},
{0x1,1,{0x0A}},
{0x2,1,{0x0}},
{0x3,1,{0x0}},
{0x4,1,{0x1}},
{0x5,1,{0x1}},
{0x6,1,{0x98}},
{0x7,1,{0x6}},
{0x8,1,{0x1}},
{0x9,1,{0x80}},
{0x0A,1,{0x0}},
{0x0B,1,{0x0}},
{0x0C,1,{0x1}},
{0x0D,1,{0x1}},
{0x0E,1,{0x5}},
{0x0F,1,{0x0}},
{0x10,1,{0xF0}},
{0x11,1,{0xF4}},
{0x12,1,{0x1}},
{0x13,1,{0x0}},
{0x14,1,{0x0}},
{0x15,1,{0xC0}},
{0x16,1,{0x8}},
{0x17,1,{0x0}},
{0x18,1,{0x0}},
{0x19,1,{0x0}},
{0x1A,1,{0x0}},
{0x1B,1,{0x0}},
{0x1C,1,{0x0}},
{0x1D,1,{0x0}},
{0x20,1,{0x1}},
{0x21,1,{0x23}},
{0x22,1,{0x45}},
{0x23,1,{0x67}},
{0x24,1,{0x1}},
{0x25,1,{0x23}},
{0x26,1,{0x45}},
{0x27,1,{0x67}},
{0x30,1,{0x11}},
{0x31,1,{0x11}},
{0x32,1,{0x0}},
{0x33,1,{0xEE}},
{0x34,1,{0xFF}},
{0x35,1,{0xBB}},
{0x36,1,{0xAA}},
{0x37,1,{0xDD}},
{0x38,1,{0xCC}},
{0x39,1,{0x66}},
{0x3A,1,{0x77}},
{0x3B,1,{0x22}},
{0x3C,1,{0x22}},
{0x3D,1,{0x22}},
{0x3E,1,{0x22}},
{0x3F,1,{0x22}},
{0x40,1,{0x22}},
{0xFF,5,{0xFF,0x98,0x6,0x4,0x7}},
{0x17,1,{0x22}},
{0x2,1,{0x77}},
{0xFF,5,{0xFF,0x98,0x6,0x4,0x0}},
{0x11,1,{0x0}},
{REGFLAG_DELAY, 220, {}},
{0x29,1,{0x0}},
{REGFLAG_DELAY, 220, {}},
 // Note
 // Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.
 // Setting ending by predefined flag
{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_initialization_setting[] =
{
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},//page 1
	{0x08,1,{0x10}},//spi sda pin
	{0x21,1,{0x01}},//polarities

	{0x25,1,{0x08}},//VFP
	{0x26,1,{0x14}},//VBP
	{0x27,1,{0x32}},//HBP
	{0x28,1,{0x01}},//HBP MSB

	{0x30,1,{0x01}},//480x854
	{0x31,1,{0x00}},//inversion columnwise

	/*1*/
	{0x40,1,{0x10}},
	{0x41,1,{0x33}},
	{0x42,1,{0x03}},
	{0x43,1,{0x09}},
	{0x44,1,{0x07}},
	{0x50,1,{0x78}},
	{0x51,1,{0x78}},
	{0x52,1,{0x00}},
	{0x53,1,{0x40}},
	{0x60,1,{0x07}},
	{0x61,1,{0x00}},
	{0x62,1,{0x08}},
	{0x63,1,{0x00}},


	/*2*
	{0x60,1,{0x07}},
	{0x61,1,{0x06}},
	{0x62,1,{0x06}},
	{0x63,1,{0x04}},

	{0x40,1,{0x18}},
	{0x41,1,{0x33}},
	{0x42,1,{0x11}},//0x12
	{0x43,1,{0x09}},//0x89
	{0x44,1,{0x0C}},//0x86
	{0x46,1,{0x55}},
	{0x47,1,{0x55}},
	{0x45,1,{0x14}},

	{0x50,1,{0x50}},
	{0x51,1,{0x50}},
	{0x52,1,{0x00}},
	{0x53,1,{0x38}},
	*/

	/*my
	{0x40,1,{0x16}},//DDVDH/DDVDL: VCIx3=8,4 / VCIx-2,5=-7 default 0x15
	{0x41,1,{0x33}},//DDVDH/DDVDL clamp 6 -6 default 0x22 5 -5
	{0x42,1,{0x03}},//VGH VGL 2DDVDH-DDVDL  /  DDVDL-DDVDH  // VGH=23 VGL-15,4
	{0x43,1,{0x09}},//VGH clamp 15V - disabled
	{0x44,1,{0x06}},//VGL clamp -9.5 - disabled --default enabled 0x86
	//45 vgh_reg vgl_reg
	//46 stepup freq
	//47 stepup freq
	{0x50,1,{0x78}},//VREG1OUT positive gamma 4.5V
	{0x51,1,{0x78}},//VREG2OUT neatibe gamma -4.5V
	{0x52,1,{0x00}},//msb dla 53
    {0x53, 1,  {0x4b}},//VCM1 -1.125 --default 6F -1.575
	//54 55 VCM2
	//56 NV mem set VCM1 VCM2 voltage
	//57 lvd detect
	//58 deep standby
	{0x60,1,{0x07}},//source sdt timming SDTI default 0x5
	{0x61,1,{0x04}},//source cr timmint CRTI def 0x5
	{0x62,1,{0x07}},//source eq timming EQTI def 0xe
	{0x63,1,{0x02}},//source pc timming PCTI def 0x5*/
//80- 83synchr timming adjust
 {0xA0,1,{0x00}},//positive gamma
{0xA1,1,{0x1e}},
{0xA2,1,{0x20}},
{0xA3,1,{0x0D}},
{0xA4,1,{0x07}},
{0xA5,1,{0x14}},
{0xA6,1,{0x08}},
{0xA7,1,{0x07}},
{0xA8,1,{0x03}},
{0xA9,1,{0x08}},
{0xAA,1,{0x07}},  //  07
{0xAB,1,{0x07}},  // 07
{0xAC,1,{0x0e}},  // 10
{0xAD,1,{0x2f}},  //  31
{0xAE,1,{0x2f}},
{0xAF,1,{0x00}},



{0xC0,1,{0x00}},//negative gamma
{0xC1,1,{0x0e}},
{0xC2,1,{0x15}},
{0xC3,1,{0x0b}},
{0xC4,1,{0x06}},
{0xC5,1,{0x0f}},
{0xC6,1,{0x08}},
{0xC7,1,{0x08}},
{0xC8,1,{0x05}},
{0xC9,1,{0x0a}},
{0xCA,1,{0x07}},  //08
{0xCB,1,{0x05}},   // 06
{0xCC,1,{0x0a}},   // 0b
{0xCD,1,{0x2a}},  //2c
{0xCE,1,{0x26}},
{0xCF,1,{0x00}},

	{0xFF,5,{0xFF,0x98,0x06,0x04,0x06}},//GIP setting timming?? wtf
	{0x00,1,{0x20}},
	{0x01,1,{0x0A}},
	{0x02,1,{0x00}},
	{0x03,1,{0x04}},
	{0x04,1,{0x01}},
	{0x05,1,{0x01}},
	{0x06,1,{0x98}},
	{0x07,1,{0x08}},
	{0x08,1,{0x02}},
	{0x09,1,{0x00}},
	{0x0A,1,{0x00}},
	{0x0B,1,{0x00}},
	{0x0C,1,{0x01}},
	{0x0D,1,{0x01}},
	{0x0E,1,{0x00}},
	{0x0F,1,{0x00}},
	{0x10,1,{0xFF}},
	{0x11,1,{0xF0}},
	{0x12,1,{0x04}},
	{0x13,1,{0x00}},
	{0x14,1,{0x00}},
	{0x15,1,{0x43}},
	{0x16,1,{0x0B}},
	{0x17,1,{0x00}},
	{0x18,1,{0x00}},
	{0x19,1,{0x00}},
	{0x1A,1,{0x00}},
	{0x1B,1,{0x00}},
	{0x1C,1,{0x00}},
	{0x1D,1,{0x00}},
	{0x20,1,{0x01}},
	{0x21,1,{0x23}},
	{0x22,1,{0x45}},
	{0x23,1,{0x67}},
	{0x24,1,{0x01}},
	{0x25,1,{0x23}},
	{0x26,1,{0x45}},
	{0x27,1,{0x67}},
	{0x30,1,{0x13}},
	{0x31,1,{0x22}},
	{0x32,1,{0x22}},
	{0x33,1,{0x96}},
	{0x34,1,{0xDA}},
	{0x35,1,{0xAB}},
	{0x36,1,{0xBC}},
	{0x37,1,{0xCD}},
	{0x38,1,{0x22}},
	{0x39,1,{0xFE}},
	{0x3A,1,{0xEF}},
	{0x3B,1,{0x22}},
	{0x3C,1,{0x68}},
	{0x3D,1,{0x22}},
	{0x3E,1,{0x22}},
	{0x3F,1,{0x89}},
	{0x40,1,{0x22}},

	/*1*/
	{0x52,1,{0x10}},
	{0x53,1,{0x10}},  //VGLO refer VGL_REG
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x07}},
	//{0x18,1,{0x1d}},
	{0x17,1,{0x22}},
	{0x02,1,{0x77}},
	{0xE1,1,{0x79}},
	{0x18,1,{0x1d}},
	{0x26,1,{0xb2}},
	{0x06,1,{0x13}},

	{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},
	//{0x35, 1, {0X00}},
	{0x11, 1, {0X00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29,1,{0}},
	{REGFLAG_DELAY, 120, {}},
	{0x38,1,{0}},
	{REGFLAG_DELAY, 120, {}},
	{0x21,1,{0}},

/*2*
	{0x52,1,{0x12}},
	{0x53,1,{0x12}},
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x07}},
	{0x17,1,{0x32}},
	{0x02,1,{0x17}},
	{0x18,1,{0x1D}},
	{0xE1,1,{0x79}},

	{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},
	{0x3A,1,{0x70}},//0x50
	{0x11,0,{}},
	{REGFLAG_DELAY,150,{0x00}},
	{0x29,0,{}},
	{REGFLAG_DELAY,150,{0x00}},
*/
	/*my
    {0xFF, 5,  {0xFF,0x98,0x06,0x04,0x07}},
	{0x18,1,{0x1D}},//VREG en
	{0x06,1,{0x00}},//VCL -2.8 default 0x1 -3.0
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},
	{0x11,1,{0x00}},//seep out
    {REGFLAG_DELAY, 150, {}},
	{0x29,1,{0x00}},//disp on
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},
*/
    {REGFLAG_DELAY, 20, {}},
	 // Note
	 // Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.
	 // Setting ending by predefined flag
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
static void push_table(struct aml_rfd500v *ctx,struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	unsigned int i,j;
	unsigned char data[1024];
    for(i = 0; i < count; i++) {

        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {

            case REGFLAG_DELAY :

            	usleep_range(table[i].count*1000, table[i].count*1000+1000);
                //MDELAY(table[i].count);
                break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
            	data[0]=cmd;
            	for(j=0;j<table[i].count;j++)
            		data[j+1]=table[i].para_list[j];

            	dev_dbg(ctx->dev, "Table Write %d\n",i);
            	mipi_dsi_dcs_write_buffer(dsi, data, table[i].count+1);
            	//usleep_range(500000, 500000+1000);
		        //dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }

}
static struct LCM_setting_table set_page_0[] =
{
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},
	{0x0A,1,{0x1}},//my
};

static struct LCM_setting_table lcm_initialization_setting1[] =
{
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},
	{0x0A,1,{0x0}},//my
};


static void aml_rfd500v_set_sequence(struct aml_rfd500v *ctx)
{
	//aml_rfd500v_set_maximum_return_packet_size(ctx, 3);
	//aml_rfd500v_read_mtp_id(ctx);

	dev_dbg(ctx->dev, "aml_rfd500v_set_sequence\n");
	//aml_rfd500v_panel_init(ctx);

	//push_table(ctx,set_page_0, sizeof(set_page_0) / sizeof(struct LCM_setting_table), 1);

	//usleep_range(2200000, 2210000);
	aml_rfd500v_dcs_write_seq_static(ctx, MIPI_DCS_SOFT_RESET);
	usleep_range(1200000, 1210000);
	push_table(ctx,lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

	//push_table(ctx,set_page_0, sizeof(set_page_0) / sizeof(struct LCM_setting_table), 1);
	//usleep_range(2200000, 2210000);
	//aml_rfd500v_dcs_write_seq_static(ctx, MIPI_DCS_EXIT_SLEEP_MODE);
	//usleep_range(1200000, 1210000);
	//aml_rfd500v_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_NORMAL_MODE);
	//usleep_range(2200000, 2210000);
	//aml_rfd500v_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_ON);
	//usleep_range(2200000, 2210000);

	/*aml_rfd500v_dcs_write_seq_static(ctx, 0x51,0x0);
	usleep_range(2200000, 2210000);
	aml_rfd500v_dcs_write_seq_static(ctx, 0x51,0xff);
	usleep_range(2200000, 2210000);*/
	/*aml_rfd500v_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	usleep_range(5200000, 5210000);
	aml_rfd500v_dcs_write_seq_static(ctx, MIPI_DCS_EXIT_SLEEP_MODE);
	usleep_range(5200000, 5210000);
	aml_rfd500v_dcs_write_seq_static(ctx, MIPI_DCS_EXIT_SLEEP_MODE);
	usleep_range(5200000, 5210000);
*/
	//usleep_range(5200000, 5210000);

	/*usleep_range(5200000, 5210000);
	aml_rfd500v_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_ON);
	usleep_range(5200000, 5210000);
	aml_rfd500v_dcs_write_seq_static(ctx, MIPI_DCS_EXIT_IDLE_MODE);
	usleep_range(5200000, 5210000);*/
    /*aml_rfd500v_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_NORMAL_MODE);

	usleep_range(20000, 21000);
    aml_rfd500v_dcs_write_seq_static(ctx, MIPI_DCS_EXIT_SLEEP_MODE);

	usleep_range(120000, 121000);
    aml_rfd500v_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_ON);*/
	//usleep_range(5120000, 5121000);


}


static int aml_rfd500v_power_on(struct aml_rfd500v *ctx)
{
	int ret;

	dev_dbg(ctx->dev, "aml_rfd500v_power_on\n");

	/*ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0)
		return ret;*/

	msleep(ctx->power_on_delay);

	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(120000, 121000);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(120000, 121000);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(120000, 121000);
	//for(ret=0;ret<10;ret++)
	//	push_table(ctx,lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	msleep(ctx->reset_delay);

	return 0;
}

static int aml_rfd500v_power_off(struct aml_rfd500v *ctx)
{
	return 0;//regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
}

static int aml_rfd500v_disable(struct drm_panel *panel)
{
	return 0;
}

static int aml_rfd500v_unprepare(struct drm_panel *panel)
{
	struct aml_rfd500v *ctx = panel_to_aml_rfd500v(panel);

	dev_dbg(ctx->dev, "aml_rfd500v_unprepare\n");
	aml_rfd500v_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	aml_rfd500v_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	msleep(40);

	aml_rfd500v_clear_error(ctx);

	return aml_rfd500v_power_off(ctx);
}

static int aml_rfd500v_prepare(struct drm_panel *panel)
{
	struct aml_rfd500v *ctx = panel_to_aml_rfd500v(panel);
	int ret;

	dev_dbg(ctx->dev, "aml_rfd500v_prepare\n");

	ret = aml_rfd500v_power_on(ctx);
	if (ret < 0)
		return ret;



	return ret;
}

static int aml_rfd500v_enable(struct drm_panel *panel)
{
	struct aml_rfd500v *ctx = panel_to_aml_rfd500v(panel);
	int ret;
	aml_rfd500v_set_sequence(ctx);
	ret = ctx->error;

	if (ret < 0)
		aml_rfd500v_unprepare(panel);


	return 0;
}

static int aml_rfd500v_get_modes(struct drm_panel *panel)
{
	struct drm_connector *connector = panel->connector;
	struct aml_rfd500v *ctx = panel_to_aml_rfd500v(panel);
	struct drm_display_mode *mode;


	dev_dbg(ctx->dev, "aml_rfd500v_get_modes\n");


	mode = drm_mode_create(connector->dev);
	if (!mode) {
		DRM_ERROR("failed to create a new display mode\n");
		return 0;
	}

	drm_display_mode_from_videomode(&ctx->vm, mode);
	mode->width_mm = ctx->width_mm;
	mode->height_mm = ctx->height_mm;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs aml_rfd500v_drm_funcs = {
	.disable = aml_rfd500v_disable,
	.unprepare = aml_rfd500v_unprepare,
	.prepare = aml_rfd500v_prepare,
	.enable = aml_rfd500v_enable,
	.get_modes = aml_rfd500v_get_modes,
};

static int aml_rfd500v_parse_dt(struct aml_rfd500v *ctx)
{
	struct device *dev = ctx->dev;
	struct device_node *np = dev->of_node;
	int ret;

	ret = of_get_videomode(np, &ctx->vm, 0);
	if (ret < 0)
		return ret;

	of_property_read_u32(np, "power-on-delay", &ctx->power_on_delay);
	of_property_read_u32(np, "reset-delay", &ctx->reset_delay);
	of_property_read_u32(np, "init-delay", &ctx->init_delay);
	of_property_read_u32(np, "panel-width-mm", &ctx->width_mm);
	of_property_read_u32(np, "panel-height-mm", &ctx->height_mm);

	//ctx->flip_horizontal = of_property_read_bool(np, "flip-horizontal");
	//ctx->flip_vertical = of_property_read_bool(np, "flip-vertical");

	return 0;
}

static int aml_rfd500v_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct aml_rfd500v *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(struct aml_rfd500v), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;

	dsi->lanes = 2;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST
		| MIPI_DSI_MODE_VIDEO_HFP | MIPI_DSI_MODE_VIDEO_HBP
		| MIPI_DSI_MODE_VIDEO_HSA | MIPI_DSI_MODE_EOT_PACKET
		| MIPI_DSI_MODE_VSYNC_FLUSH | MIPI_DSI_MODE_VIDEO_AUTO_VERT;

	ret = aml_rfd500v_parse_dt(ctx);
	if (ret < 0)
		return ret;

	/*ctx->supplies[0].supply = "vdd3";
	ctx->supplies[1].supply = "vci";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies),
				      ctx->supplies);
	if (ret < 0) {
		dev_err(dev, "failed to get regulators: %d\n", ret);
		return ret;
	}*/

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(dev, "cannot get reset-gpios %ld\n",
			PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}

	//ctx->brightness = GAMMA_LEVEL_NUM - 1;

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &aml_rfd500v_drm_funcs;

	ret = drm_panel_add(&ctx->panel);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_attach(dsi);





	if (ret < 0)
		drm_panel_remove(&ctx->panel);

	//ctx->kobj = kobject_create_and_add("aml_lcd", NULL);
	ret = sysfs_create_group(&dev->kobj, &attr_group);


	return ret;
}

static int aml_rfd500v_remove(struct mipi_dsi_device *dsi)
{
	struct aml_rfd500v *ctx = mipi_dsi_get_drvdata(dsi);


//	kobject_put(ctx->kobj);
	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id aml_rfd500v_of_match[] = {
	{ .compatible = "amelin,rfd500v" },
	{ }
};
MODULE_DEVICE_TABLE(of, aml_rfd500v_of_match);

static struct mipi_dsi_driver aml_rfd500v_driver = {
	.probe = aml_rfd500v_probe,
	.remove = aml_rfd500v_remove,
	.driver = {
		.name = "panel-aml-rfd500v",
		.of_match_table = aml_rfd500v_of_match,
	},
};
module_mipi_dsi_driver(aml_rfd500v_driver);


MODULE_DESCRIPTION("MIPI-DSI based aml_rfd500v LCD Panel Driver");
MODULE_LICENSE("GPL v2");
