/*
 * MIPI-DSI based aml_080wxbi AMOLED LCD 5.3 inch panel driver.
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



struct aml_080wxbi {
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






static inline struct aml_080wxbi *panel_to_aml_080wxbi(struct drm_panel *panel)
{
	return container_of(panel, struct aml_080wxbi, panel);
}

static int aml_080wxbi_clear_error(struct aml_080wxbi *ctx)
{
	int ret = ctx->error;

	ctx->error = 0;
	return ret;
}

static void aml_080wxbi_dcs_write(struct aml_080wxbi *ctx, const void *data, size_t len)
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
    struct aml_080wxbi *ctx = mipi_dsi_get_drvdata(dsi);


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
	aml_080wxbi_dcs_write(ctx,d,rdctr);
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




#define aml_080wxbi_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	aml_080wxbi_dcs_write(ctx, d, ARRAY_SIZE(d));\
})



static void aml_080wxbi_set_sequence(struct aml_080wxbi *ctx)
{
	//aml_080wxbi_set_maximum_return_packet_size(ctx, 3);
	//aml_080wxbi_read_mtp_id(ctx);

	dev_dbg(ctx->dev, "aml_080wxbi_set_sequence\n");
	//aml_080wxbi_panel_init(ctx);

	//push_table(ctx,set_page_0, sizeof(set_page_0) / sizeof(struct LCM_setting_table), 1);

	//usleep_range(2200000, 2210000);
	aml_080wxbi_dcs_write_seq_static(ctx,0xE0,0x00);

	//--- PASSWORD  ----//
	aml_080wxbi_dcs_write_seq_static(ctx,0xE1,0x93);
	aml_080wxbi_dcs_write_seq_static(ctx,0xE2,0x65);
	aml_080wxbi_dcs_write_seq_static(ctx,0xE3,0xF8);


	//Page0
	aml_080wxbi_dcs_write_seq_static(ctx,0xE0,0x00);
	//--- Sequence Ctrl  ----//
	aml_080wxbi_dcs_write_seq_static(ctx,0x70,0x10);	//DC0,DC1
	aml_080wxbi_dcs_write_seq_static(ctx,0x71,0x13);	//DC2,DC3
	aml_080wxbi_dcs_write_seq_static(ctx,0x72,0x06);	//DC7
	aml_080wxbi_dcs_write_seq_static(ctx,0x80,0x02);	//0x03:4-Lane£»0x02:3-Lane
	//--- Page4  ----//
	aml_080wxbi_dcs_write_seq_static(ctx,0xE0,0x04);
	aml_080wxbi_dcs_write_seq_static(ctx,0x2D,0x03);
	//--- Page1  ----//
	aml_080wxbi_dcs_write_seq_static(ctx,0xE0,0x01);

	//Set VCOM
	aml_080wxbi_dcs_write_seq_static(ctx,0x00,0x00);
	aml_080wxbi_dcs_write_seq_static(ctx,0x01,0xA0);
	//Set VCOM_Reverse
	aml_080wxbi_dcs_write_seq_static(ctx,0x03,0x00);
	aml_080wxbi_dcs_write_seq_static(ctx,0x04,0xA0);

	//Set Gamma Power, VGMP,VGMN,VGSP,VGSN
	aml_080wxbi_dcs_write_seq_static(ctx,0x17,0x00);
	aml_080wxbi_dcs_write_seq_static(ctx,0x18,0xB1);
	aml_080wxbi_dcs_write_seq_static(ctx,0x19,0x01);
	aml_080wxbi_dcs_write_seq_static(ctx,0x1A,0x00);
	aml_080wxbi_dcs_write_seq_static(ctx,0x1B,0xB1);  //VGMN=0
	aml_080wxbi_dcs_write_seq_static(ctx,0x1C,0x01);

	//Set Gate Power
	aml_080wxbi_dcs_write_seq_static(ctx,0x1F,0x3E);     //VGH_R  = 15V
	aml_080wxbi_dcs_write_seq_static(ctx,0x20,0x2D);     //VGL_R  = -12V
	aml_080wxbi_dcs_write_seq_static(ctx,0x21,0x2D);     //VGL_R2 = -12V
	aml_080wxbi_dcs_write_seq_static(ctx,0x22,0x0E);     //PA[6]=0, PA[5]=0, PA[4]=0, PA[0]=0

	//SETPANEL
	aml_080wxbi_dcs_write_seq_static(ctx,0x37,0x19);	//SS=1,BGR=1

	//SET RGBCYC
	aml_080wxbi_dcs_write_seq_static(ctx,0x38,0x05);	//JDT=101 zigzag inversion
	aml_080wxbi_dcs_write_seq_static(ctx,0x39,0x08);	//RGB_N_EQ1, modify 20140806
	aml_080wxbi_dcs_write_seq_static(ctx,0x3A,0x12);	//RGB_N_EQ2, modify 20140806
	aml_080wxbi_dcs_write_seq_static(ctx,0x3C,0x78);	//SET EQ3 for TE_H
	aml_080wxbi_dcs_write_seq_static(ctx,0x3E,0x80);	//SET CHGEN_OFF, modify 20140806
	aml_080wxbi_dcs_write_seq_static(ctx,0x3F,0x80);	//SET CHGEN_OFF2, modify 20140806


	//Set TCON
	aml_080wxbi_dcs_write_seq_static(ctx,0x40,0x06);	//RSO=800 RGB
	aml_080wxbi_dcs_write_seq_static(ctx,0x41,0xA0);	//LN=640->1280 line

	//--- power voltage  ----//
	aml_080wxbi_dcs_write_seq_static(ctx,0x55,0x01);	//DCDCM=0001, JD PWR_IC
	aml_080wxbi_dcs_write_seq_static(ctx,0x56,0x01);
	aml_080wxbi_dcs_write_seq_static(ctx,0x57,0x69);
	aml_080wxbi_dcs_write_seq_static(ctx,0x58,0x0A);
	aml_080wxbi_dcs_write_seq_static(ctx,0x59,0x0A);	//VCL = -2.9V
	aml_080wxbi_dcs_write_seq_static(ctx,0x5A,0x28);	//VGH = 19V
	aml_080wxbi_dcs_write_seq_static(ctx,0x5B,0x19);	//VGL = -11V



	//--- Gamma  ----//
	aml_080wxbi_dcs_write_seq_static(ctx,0x5D,0x7C);
	aml_080wxbi_dcs_write_seq_static(ctx,0x5E,0x65);
	aml_080wxbi_dcs_write_seq_static(ctx,0x5F,0x53);
	aml_080wxbi_dcs_write_seq_static(ctx,0x60,0x48);
	aml_080wxbi_dcs_write_seq_static(ctx,0x61,0x43);
	aml_080wxbi_dcs_write_seq_static(ctx,0x62,0x35);
	aml_080wxbi_dcs_write_seq_static(ctx,0x63,0x39);
	aml_080wxbi_dcs_write_seq_static(ctx,0x64,0x23);
	aml_080wxbi_dcs_write_seq_static(ctx,0x65,0x3D);
	aml_080wxbi_dcs_write_seq_static(ctx,0x66,0x3C);
	aml_080wxbi_dcs_write_seq_static(ctx,0x67,0x3D);
	aml_080wxbi_dcs_write_seq_static(ctx,0x68,0x5A);
	aml_080wxbi_dcs_write_seq_static(ctx,0x69,0x46);
	aml_080wxbi_dcs_write_seq_static(ctx,0x6A,0x57);
	aml_080wxbi_dcs_write_seq_static(ctx,0x6B,0x4B);
	aml_080wxbi_dcs_write_seq_static(ctx,0x6C,0x49);
	aml_080wxbi_dcs_write_seq_static(ctx,0x6D,0x2F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x6E,0x03);
	aml_080wxbi_dcs_write_seq_static(ctx,0x6F,0x00);
	aml_080wxbi_dcs_write_seq_static(ctx,0x70,0x7C);
	aml_080wxbi_dcs_write_seq_static(ctx,0x71,0x65);
	aml_080wxbi_dcs_write_seq_static(ctx,0x72,0x53);
	aml_080wxbi_dcs_write_seq_static(ctx,0x73,0x48);
	aml_080wxbi_dcs_write_seq_static(ctx,0x74,0x43);
	aml_080wxbi_dcs_write_seq_static(ctx,0x75,0x35);
	aml_080wxbi_dcs_write_seq_static(ctx,0x76,0x39);
	aml_080wxbi_dcs_write_seq_static(ctx,0x77,0x23);
	aml_080wxbi_dcs_write_seq_static(ctx,0x78,0x3D);
	aml_080wxbi_dcs_write_seq_static(ctx,0x79,0x3C);
	aml_080wxbi_dcs_write_seq_static(ctx,0x7A,0x3D);
	aml_080wxbi_dcs_write_seq_static(ctx,0x7B,0x5A);
	aml_080wxbi_dcs_write_seq_static(ctx,0x7C,0x46);
	aml_080wxbi_dcs_write_seq_static(ctx,0x7D,0x57);
	aml_080wxbi_dcs_write_seq_static(ctx,0x7E,0x4B);
	aml_080wxbi_dcs_write_seq_static(ctx,0x7F,0x49);
	aml_080wxbi_dcs_write_seq_static(ctx,0x80,0x2F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x81,0x03);
	aml_080wxbi_dcs_write_seq_static(ctx,0x82,0x00);


	//Page2, for GIP
	aml_080wxbi_dcs_write_seq_static(ctx,0xE0,0x02);

	//GIP_L Pin mapping
	aml_080wxbi_dcs_write_seq_static(ctx,0x00,0x47);
	aml_080wxbi_dcs_write_seq_static(ctx,0x01,0x47);
	aml_080wxbi_dcs_write_seq_static(ctx,0x02,0x45);
	aml_080wxbi_dcs_write_seq_static(ctx,0x03,0x45);
	aml_080wxbi_dcs_write_seq_static(ctx,0x04,0x4B);
	aml_080wxbi_dcs_write_seq_static(ctx,0x05,0x4B);
	aml_080wxbi_dcs_write_seq_static(ctx,0x06,0x49);
	aml_080wxbi_dcs_write_seq_static(ctx,0x07,0x49);
	aml_080wxbi_dcs_write_seq_static(ctx,0x08,0x41);
	aml_080wxbi_dcs_write_seq_static(ctx,0x09,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x0A,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x0B,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x0C,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x0D,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x0E,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x0F,0x43);
	aml_080wxbi_dcs_write_seq_static(ctx,0x10,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x11,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x12,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x13,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x14,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x15,0x1F);

	//GIP_R Pin mapping
	aml_080wxbi_dcs_write_seq_static(ctx,0x16,0x46);
	aml_080wxbi_dcs_write_seq_static(ctx,0x17,0x46);
	aml_080wxbi_dcs_write_seq_static(ctx,0x18,0x44);
	aml_080wxbi_dcs_write_seq_static(ctx,0x19,0x44);
	aml_080wxbi_dcs_write_seq_static(ctx,0x1A,0x4A);
	aml_080wxbi_dcs_write_seq_static(ctx,0x1B,0x4A);
	aml_080wxbi_dcs_write_seq_static(ctx,0x1C,0x48);
	aml_080wxbi_dcs_write_seq_static(ctx,0x1D,0x48);
	aml_080wxbi_dcs_write_seq_static(ctx,0x1E,0x40);
	aml_080wxbi_dcs_write_seq_static(ctx,0x1F,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x20,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x21,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x22,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x23,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x24,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x25,0x42);
	aml_080wxbi_dcs_write_seq_static(ctx,0x26,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x27,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x28,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x29,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x2A,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x2B,0x1F);

	//GIP_L_GS Pin mapping
	aml_080wxbi_dcs_write_seq_static(ctx,0x2C,0x11);
	aml_080wxbi_dcs_write_seq_static(ctx,0x2D,0x0F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x2E,0x0D);
	aml_080wxbi_dcs_write_seq_static(ctx,0x2F,0x0B);
	aml_080wxbi_dcs_write_seq_static(ctx,0x30,0x09);
	aml_080wxbi_dcs_write_seq_static(ctx,0x31,0x07);
	aml_080wxbi_dcs_write_seq_static(ctx,0x32,0x05);
	aml_080wxbi_dcs_write_seq_static(ctx,0x33,0x18);
	aml_080wxbi_dcs_write_seq_static(ctx,0x34,0x17);
	aml_080wxbi_dcs_write_seq_static(ctx,0x35,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x36,0x01);
	aml_080wxbi_dcs_write_seq_static(ctx,0x37,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x38,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x39,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x3A,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x3B,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x3C,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x3D,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x3E,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x3F,0x13);
	aml_080wxbi_dcs_write_seq_static(ctx,0x40,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x41,0x1F);

	//GIP_R_GS Pin mapping
	aml_080wxbi_dcs_write_seq_static(ctx,0x42,0x10);
	aml_080wxbi_dcs_write_seq_static(ctx,0x43,0x0E);
	aml_080wxbi_dcs_write_seq_static(ctx,0x44,0x0C);
	aml_080wxbi_dcs_write_seq_static(ctx,0x45,0x0A);
	aml_080wxbi_dcs_write_seq_static(ctx,0x46,0x08);
	aml_080wxbi_dcs_write_seq_static(ctx,0x47,0x06);
	aml_080wxbi_dcs_write_seq_static(ctx,0x48,0x04);
	aml_080wxbi_dcs_write_seq_static(ctx,0x49,0x18);
	aml_080wxbi_dcs_write_seq_static(ctx,0x4A,0x17);
	aml_080wxbi_dcs_write_seq_static(ctx,0x4B,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x4C,0x00);
	aml_080wxbi_dcs_write_seq_static(ctx,0x4D,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x4E,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x4F,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x50,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x51,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x52,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x53,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x54,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x55,0x12);
	aml_080wxbi_dcs_write_seq_static(ctx,0x56,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x57,0x1F);

	//GIP Timing
	aml_080wxbi_dcs_write_seq_static(ctx,0x58,0x40);
	aml_080wxbi_dcs_write_seq_static(ctx,0x59,0x00);
	aml_080wxbi_dcs_write_seq_static(ctx,0x5A,0x00);
	aml_080wxbi_dcs_write_seq_static(ctx,0x5B,0x30);
	aml_080wxbi_dcs_write_seq_static(ctx,0x5C,0x03);
	aml_080wxbi_dcs_write_seq_static(ctx,0x5D,0x30);
	aml_080wxbi_dcs_write_seq_static(ctx,0x5E,0x01);
	aml_080wxbi_dcs_write_seq_static(ctx,0x5F,0x02);
	aml_080wxbi_dcs_write_seq_static(ctx,0x60,0x00);
	aml_080wxbi_dcs_write_seq_static(ctx,0x61,0x01);
	aml_080wxbi_dcs_write_seq_static(ctx,0x62,0x02);
	aml_080wxbi_dcs_write_seq_static(ctx,0x63,0x03);
	aml_080wxbi_dcs_write_seq_static(ctx,0x64,0x6B);
	aml_080wxbi_dcs_write_seq_static(ctx,0x65,0x00);
	aml_080wxbi_dcs_write_seq_static(ctx,0x66,0x00);
	aml_080wxbi_dcs_write_seq_static(ctx,0x67,0x73);
	aml_080wxbi_dcs_write_seq_static(ctx,0x68,0x05);
	aml_080wxbi_dcs_write_seq_static(ctx,0x69,0x06);
	aml_080wxbi_dcs_write_seq_static(ctx,0x6A,0x6B);
	aml_080wxbi_dcs_write_seq_static(ctx,0x6B,0x08);
	aml_080wxbi_dcs_write_seq_static(ctx,0x6C,0x00);
	aml_080wxbi_dcs_write_seq_static(ctx,0x6D,0x04);
	aml_080wxbi_dcs_write_seq_static(ctx,0x6E,0x04);
	aml_080wxbi_dcs_write_seq_static(ctx,0x6F,0x88);
	aml_080wxbi_dcs_write_seq_static(ctx,0x70,0x00);
	aml_080wxbi_dcs_write_seq_static(ctx,0x71,0x00);
	aml_080wxbi_dcs_write_seq_static(ctx,0x72,0x06);
	aml_080wxbi_dcs_write_seq_static(ctx,0x73,0x7B);
	aml_080wxbi_dcs_write_seq_static(ctx,0x74,0x00);
	aml_080wxbi_dcs_write_seq_static(ctx,0x75,0x07);
	aml_080wxbi_dcs_write_seq_static(ctx,0x76,0x00);
	aml_080wxbi_dcs_write_seq_static(ctx,0x77,0x5D);
	aml_080wxbi_dcs_write_seq_static(ctx,0x78,0x17);
	aml_080wxbi_dcs_write_seq_static(ctx,0x79,0x1F);
	aml_080wxbi_dcs_write_seq_static(ctx,0x7A,0x00);
	aml_080wxbi_dcs_write_seq_static(ctx,0x7B,0x00);
	aml_080wxbi_dcs_write_seq_static(ctx,0x7C,0x00);
	aml_080wxbi_dcs_write_seq_static(ctx,0x7D,0x03);
	aml_080wxbi_dcs_write_seq_static(ctx,0x7E,0x7B);


	//Page1
	aml_080wxbi_dcs_write_seq_static(ctx,0xE0,0x01);
	aml_080wxbi_dcs_write_seq_static(ctx,0x0E,0x01);	//LEDON output VCSW2


	//Page3
	aml_080wxbi_dcs_write_seq_static(ctx,0xE0,0x03);
	aml_080wxbi_dcs_write_seq_static(ctx,0x98,0x2F);	//From 2E to 2F, LED_VOL

	//Page4
	aml_080wxbi_dcs_write_seq_static(ctx,0xE0,0x04);
	aml_080wxbi_dcs_write_seq_static(ctx,0x09,0x10);
	aml_080wxbi_dcs_write_seq_static(ctx,0x2B,0x2B);
	aml_080wxbi_dcs_write_seq_static(ctx,0x2E,0x44);

	//Page0
	aml_080wxbi_dcs_write_seq_static(ctx,0xE0,0x00);
	aml_080wxbi_dcs_write_seq_static(ctx,0xE6,0x02);
	aml_080wxbi_dcs_write_seq_static(ctx,0xE7,0x02);
	aml_080wxbi_dcs_write_seq_static(ctx,0x35,0x00);




	aml_080wxbi_dcs_write_seq_static(ctx,0x11, 0x00); /* send DCS SLEEP_OUT */
    usleep_range(120000, 121000);
    aml_080wxbi_dcs_write_seq_static(ctx, 0x29, 0x00); /* send DCS DISPLAY_ON */
    usleep_range(120000, 121000);
    aml_080wxbi_dcs_write_seq_static(ctx, 0x38, 0x00); /* send DCS EXIT_IDLE_MODE */
    usleep_range(120000, 121000);

    aml_080wxbi_dcs_write_seq_static(ctx,0x21, 0x00); /* send DCS ENTER_INVERT_MODE */
    usleep_range(120000, 121000);



}


static int aml_080wxbi_power_on(struct aml_080wxbi *ctx)
{
	int ret;

	dev_dbg(ctx->dev, "aml_080wxbi_power_on\n");

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

static int aml_080wxbi_power_off(struct aml_080wxbi *ctx)
{
	return 0;//regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
}

static int aml_080wxbi_disable(struct drm_panel *panel)
{
	return 0;
}

static int aml_080wxbi_unprepare(struct drm_panel *panel)
{
	struct aml_080wxbi *ctx = panel_to_aml_080wxbi(panel);

	dev_dbg(ctx->dev, "aml_080wxbi_unprepare\n");
	aml_080wxbi_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	aml_080wxbi_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	msleep(40);

	aml_080wxbi_clear_error(ctx);

	return aml_080wxbi_power_off(ctx);
}

static int aml_080wxbi_prepare(struct drm_panel *panel)
{
	struct aml_080wxbi *ctx = panel_to_aml_080wxbi(panel);
	int ret;

	dev_dbg(ctx->dev, "aml_080wxbi_prepare\n");

	ret = aml_080wxbi_power_on(ctx);
	if (ret < 0)
		return ret;



	return ret;
}

static int aml_080wxbi_enable(struct drm_panel *panel)
{
	struct aml_080wxbi *ctx = panel_to_aml_080wxbi(panel);
	int ret;
	aml_080wxbi_set_sequence(ctx);
	ret = ctx->error;

	if (ret < 0)
		aml_080wxbi_unprepare(panel);


	return 0;
}

static int aml_080wxbi_get_modes(struct drm_panel *panel)
{
	struct drm_connector *connector = panel->connector;
	struct aml_080wxbi *ctx = panel_to_aml_080wxbi(panel);
	struct drm_display_mode *mode;


	dev_dbg(ctx->dev, "aml_080wxbi_get_modes\n");


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

static const struct drm_panel_funcs aml_080wxbi_drm_funcs = {
	.disable = aml_080wxbi_disable,
	.unprepare = aml_080wxbi_unprepare,
	.prepare = aml_080wxbi_prepare,
	.enable = aml_080wxbi_enable,
	.get_modes = aml_080wxbi_get_modes,
};

static int aml_080wxbi_parse_dt(struct aml_080wxbi *ctx)
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

static int aml_080wxbi_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct aml_080wxbi *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(struct aml_080wxbi), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;

	dsi->lanes = 3;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST
		| MIPI_DSI_MODE_VIDEO_HFP | MIPI_DSI_MODE_VIDEO_HBP
		| MIPI_DSI_MODE_VIDEO_HSA | MIPI_DSI_MODE_EOT_PACKET
		| MIPI_DSI_MODE_VSYNC_FLUSH | MIPI_DSI_MODE_VIDEO_AUTO_VERT;

	ret = aml_080wxbi_parse_dt(ctx);
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
	ctx->panel.funcs = &aml_080wxbi_drm_funcs;

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

static int aml_080wxbi_remove(struct mipi_dsi_device *dsi)
{
	struct aml_080wxbi *ctx = mipi_dsi_get_drvdata(dsi);


//	kobject_put(ctx->kobj);
	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id aml_080wxbi_of_match[] = {
	{ .compatible = "amelin,080wxbi" },
	{ }
};
MODULE_DEVICE_TABLE(of, aml_080wxbi_of_match);

static struct mipi_dsi_driver aml_080wxbi_driver = {
	.probe = aml_080wxbi_probe,
	.remove = aml_080wxbi_remove,
	.driver = {
		.name = "panel-aml-080wxbi",
		.of_match_table = aml_080wxbi_of_match,
	},
};
module_mipi_dsi_driver(aml_080wxbi_driver);


MODULE_DESCRIPTION("MIPI-DSI based aml_080wxbi LCD Panel Driver");
MODULE_LICENSE("GPL v2");
