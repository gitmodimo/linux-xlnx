// SPDX-License-Identifier: GPL-2.0
/*
 * Xilinx FPGA MIPI DSI Tx Controller driver.
 *
 * Copyright (C) 2017 - 2018 Xilinx, Inc.
 *
 * Author : Saurabh Sengar <saurabhs@xilinx.com>
 *        : Siva Rajesh J <siva.rajesh.jarugula@xilinx.com>
 */
#define DEBUG 1
#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/phy/phy.h>
#include <video/mipi_display.h>
#include <video/videomode.h>

#include "xlnx_bridge.h"

/* Register definitions */
#define REG_H_FRONT_PORCH 0x0
#define REG_H_BACK_PORCH  0x4
#define REG_H_ACTIVE      0x8
#define REG_H_TOTAL       0xc
#define REG_V_FRONT_PORCH 0x10
#define REG_V_BACK_PORCH  0x14
#define REG_V_ACTIVE      0x18
#define REG_V_TOTAL       0x1c
#define REG_TIMING_CTL    0x20
#define REG_DSI_TICKDIV   0x24
#define REG_DSI_CTL       0x28
#define REG_TEST_XSIZE    0x2c
#define REG_TEST_YSIZE    0x30
#define REG_TEST_CTL      0x34
#define REG_LP_TX         0x38
#define REG_DSI_GPIO      0x3c
#define REG_DSI_LANE_CTL      0x40

#define DSI_LANE(index, swap, reverse) \
		((swap&3) << ((index) * 2) | ((reverse) ? (1<<((index)+8)) : 0 ))

#define DSI_LANE_INVERT_CLOCK (1<<12)
#define DSI_LANE_CLOCK_POLARITY (1<<13)



/*
 * XDSI_NUM_DATA_T represents number of data types in the
 * enum mipi_dsi_pixel_format in the MIPI DSI part of DRM framework.
 */
#define XDSI_NUM_DATA_T			4
#define XDSI_VIDEO_MODE_SYNC_PULSE	0x0
#define XDSI_VIDEO_MODE_SYNC_EVENT	0x1
#define XDSI_VIDEO_MODE_BURST		0x2

#define XDSI_DPHY_CLK_MIN	197000000000UL
#define XDSI_DPHY_CLK_MAX	203000000000UL
#define XDSI_DPHY_CLK_REQ	200000000000UL

/**
 * struct open_dsi - Core configuration DSI Tx subsystem device structure
 * @encoder: DRM encoder structure
 * @dsi_host: DSI host device
 * @connector: DRM connector structure
 * @panel_node: MIPI DSI device panel node
 * @panel:  DRM panel structure
 * @dev: device structure
 * @iomem: Base address of DSI subsystem
 * @lanes: number of active data lanes supported by DSI controller
 * @mode_flags: DSI operation mode related flags
 * @format: pixel format for video mode of DSI controller
 * @vm: videomode data structure
 * @mul_factor: multiplication factor for HACT timing parameter
 * @eotp_prop: configurable EoTP DSI parameter
 * @bllp_mode_prop: configurable BLLP mode DSI parameter
 * @bllp_type_prop: configurable BLLP type DSI parameter
 * @video_mode_prop: configurable Video mode DSI parameter
 * @bllp_burst_time_prop: Configurable BLLP time for burst mode
 * @cmd_queue_prop: configurable command queue
 * @eotp_prop_val: configurable EoTP DSI parameter value
 * @bllp_mode_prop_val: configurable BLLP mode DSI parameter value
 * @bllp_type_prop_val: configurable BLLP type DSI parameter value
 * @video_mode_prop_val: configurable Video mode DSI parameter value
 * @bllp_burst_time_prop_val: Configurable BLLP time for burst mode value
 * @cmd_queue_prop_val: configurable command queue value
 * @bridge: bridge structure
 * @height_out: configurable bridge output height parameter
 * @height_out_prop_val: configurable bridge output height parameter value
 * @width_out: configurable bridge output width parameter
 * @width_out_prop_val: configurable bridge output width parameter value
 * @in_fmt: configurable bridge input media format
 * @in_fmt_prop_val: configurable media bus format value
 * @out_fmt: configurable bridge output media format
 * @out_fmt_prop_val: configurable media bus format value
 * @video_aclk: Video clock
 * @dphy_clk_200M: 200MHz DPHY clock and AXI Lite clock
 */
struct open_dsi {
	struct drm_encoder encoder;
	struct mipi_dsi_host dsi_host;
	struct drm_connector connector;
	struct device_node *panel_node;
	struct drm_panel *panel;
	struct device *dev;
	void __iomem *iomem;
	u32 lanes;
	u32 lp_divider;
	int dsi_ctl;
	u32 mode_flags;
	enum mipi_dsi_pixel_format format;
	struct videomode vm;
	u32 mul_factor;
	/*struct drm_property *eotp_prop;
	struct drm_property *bllp_mode_prop;
	struct drm_property *bllp_type_prop;
	struct drm_property *video_mode_prop;
	struct drm_property *bllp_burst_time_prop;
	struct drm_property *cmd_queue_prop;*/
	/*bool eotp_prop_val;
	bool bllp_mode_prop_val;
	bool bllp_type_prop_val;
	u32 video_mode_prop_val;
	u32 bllp_burst_time_prop_val;
	u32 cmd_queue_prop_val;*/
	struct xlnx_bridge *bridge;
	/*struct drm_property *height_out;
	u32 height_out_prop_val;
	struct drm_property *width_out;
	u32 width_out_prop_val;
	struct drm_property *in_fmt;
	u32 in_fmt_prop_val;
	struct drm_property *out_fmt;
	u32 out_fmt_prop_val;*/
	struct clk *video_aclk;
	struct clk *dphy_clk_200M;
};

#define host_to_dsi(host) container_of(host, struct open_dsi, dsi_host)
#define connector_to_dsi(c) container_of(c, struct open_dsi, connector)
#define encoder_to_dsi(e) container_of(e, struct open_dsi, encoder)

static inline void open_dsi_writel(void __iomem *base, int offset, u32 val)
{

	printk("open DSI: 0x%02X <= 0x%02X\n",offset,val);
	writel(val, base + offset);
}

static inline u32 open_dsi_readl(void __iomem *base, int offset)
{

	u32 val=readl(base + offset);
	printk("open DSI: 0x%02X => 0x%02X\n",offset,val);
	return val;
}

/**
 * open_dsi_set_config_parameters - Configure DSI Tx registers with parameters
 * given from user application.
 * @dsi: DSI structure having the updated user parameters
 *
 * This function takes the DSI structure having drm_property parameters
 * configured from  user application and writes them into DSI IP registers.
 */
static void open_dsi_set_config_parameters(struct open_dsi *dsi)
{
	u32 reg;
	dev_info(dsi->dev, "TODO: open_dsi_set_config_parameters\n");


	/*reg = XDSI_PCR_EOTPENABLE(dsi->eotp_prop_val);
	reg |= XDSI_PCR_VIDEOMODE(dsi->video_mode_prop_val);
	reg |= XDSI_PCR_BLLPTYPE(dsi->bllp_type_prop_val);
	reg |= XDSI_PCR_BLLPMODE(dsi->bllp_mode_prop_val);

	open_dsi_writel(dsi->iomem, XDSI_PCR, reg);

	// Configure the burst time if video mode is burst.
	// HSA of TIME1 register is ignored in this mode.

	if (dsi->video_mode_prop_val == XDSI_VIDEO_MODE_BURST) {
		reg = XDSI_TIME1_BLLP_BURST(dsi->bllp_burst_time_prop_val);
		open_dsi_writel(dsi->iomem, XDSI_TIME1, reg);
	}

	reg = XDSI_CMD_QUEUE_PACKET(dsi->cmd_queue_prop_val);
	open_dsi_writel(dsi->iomem, XDSI_CMD, reg);

	dev_dbg(dsi->dev, "PCR register value is = %x\n",
		open_dsi_readl(dsi->iomem, XDSI_PCR));*/
}

/**
 * open_dsi_set_display_mode - Configure DSI timing registers
 * @dsi: DSI structure having the updated user parameters
 *
 * This function writes the timing parameters of DSI IP which are
 * retrieved from panel timing values.
 */
static void open_dsi_set_display_mode(struct open_dsi *dsi)
{
	struct videomode *vm = &dsi->vm;

	dev_info(dsi->dev, "open_dsi_set_display_mode\n");



	open_dsi_writel(dsi->iomem, REG_TIMING_CTL, 0); // disable core, force LP mode
	open_dsi_writel(dsi->iomem, REG_DSI_LANE_CTL, dsi->lanes);
	open_dsi_writel(dsi->iomem, REG_DSI_CTL, 0); // disable core
	open_dsi_writel(dsi->iomem, REG_DSI_TICKDIV, dsi->lp_divider);

	dsi->dsi_ctl = (dsi->lanes << 8);
	open_dsi_writel(dsi->iomem, REG_DSI_CTL, dsi->dsi_ctl); /* disable DSI clock, set lane count */

	dev_info(dsi->dev, "dsi->dsi_ctl %x\n",dsi->dsi_ctl);


	open_dsi_writel(dsi->iomem, REG_H_FRONT_PORCH, vm->hfront_porch);
	open_dsi_writel(dsi->iomem, REG_H_BACK_PORCH, vm->vback_porch);
	open_dsi_writel(dsi->iomem, REG_H_ACTIVE, vm->hactive * 3);
	open_dsi_writel(dsi->iomem, REG_H_TOTAL, vm->vsync_len);//panel->frame_gap);
	open_dsi_writel(dsi->iomem, REG_V_BACK_PORCH, vm->vback_porch);
	open_dsi_writel(dsi->iomem, REG_V_FRONT_PORCH, vm->vback_porch + vm->vactive);
	open_dsi_writel(dsi->iomem, REG_V_ACTIVE,
			vm->vback_porch + vm->vactive + vm->vfront_porch);
	open_dsi_writel(dsi->iomem, REG_V_TOTAL,
			vm->vback_porch + vm->vactive + vm->vfront_porch);

	dev_dbg(dsi->dev, "LCD size = %dx%d\n", vm->hactive, vm->vactive);

	/*
	u32 reg, video_mode;

	reg = open_dsi_readl(dsi->iomem, XDSI_PCR);
	video_mode = (reg & XDSI_PCR_VIDEOMODE_MASK) >>
		      XDSI_PCR_VIDEOMODE_SHIFT;

	// configure the HSA value only if non_burst_sync_pluse video mode
	if (!video_mode &&
	    (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)) {
		reg = XDSI_TIME1_HSA(vm->hsync_len);
		open_dsi_writel(dsi->iomem, XDSI_TIME1, reg);
	}

	reg = XDSI_TIME4_VFP(vm->vfront_porch) |
	      XDSI_TIME4_VBP(vm->vback_porch) |
	      XDSI_TIME4_VSA(vm->vsync_len);
	open_dsi_writel(dsi->iomem, XDSI_TIME4, reg);

	reg = XDSI_TIME3_HFP(vm->hfront_porch) |
	      XDSI_TIME3_HBP(vm->hback_porch);
	open_dsi_writel(dsi->iomem, XDSI_TIME3, reg);

	dev_dbg(dsi->dev, "mul factor for parsed datatype is = %d\n",
		(dsi->mul_factor) / 100);
	 *
	 * The HACT parameter received from panel timing values should be
	 * divisible by 4. The reason for this is, the word count given as
	 * input to DSI controller is HACT * mul_factor. The mul_factor is
	 * 3, 2.25, 2.25, 2 respectively for RGB888, RGB666_L, RGB666_P and
	 * RGB565.
	 * e.g. for RGB666_L color format and 1080p, the word count is
	 * 1920*2.25 = 4320 which is divisible by 4 and it is a valid input
	 * to DSI controller. Based on this 2.25 mul factor, we come up with
	 * the division factor of (XDSI_HACT_MULTIPLIER) as 4 for checking
	 *
	if ((vm->hactive & XDSI_HACT_MULTIPLIER) != 0)
		dev_warn(dsi->dev, "Incorrect HACT will be programmed\n");

	reg = XDSI_TIME2_HACT((vm->hactive) * (dsi->mul_factor) / 100) |
	      XDSI_TIME2_VACT(vm->vactive);
	open_dsi_writel(dsi->iomem, XDSI_TIME2, reg);

	dev_dbg(dsi->dev, "LCD size = %dx%d\n", vm->hactive, vm->vactive);*/
}

/**
 * open_dsi_set_display_enable - Enables the DSI Tx IP core enable
 * register bit
 * @dsi: DSI structure having the updated user parameters
 *
 * This function takes the DSI strucure and enables the core enable bit
 * of core configuration register.
 */
static void open_dsi_set_display_enable(struct open_dsi *dsi)
{
	u32 reg;
	int ret;

	dev_info(dsi->dev, "open_dsi_set_display_enable\n");



	if (dsi->panel) {
		ret = drm_panel_prepare(dsi->panel);
		if (ret < 0)
			goto err_put_sync;
	}

	 dsi->dsi_ctl |= 1;
	 open_dsi_writel(dsi->iomem, REG_DSI_CTL, dsi->dsi_ctl); /* enable DSI clock */

	if (dsi->panel) {
		ret = drm_panel_enable(dsi->panel);
		if (ret < 0)
			goto err_display_disable;
	}

	open_dsi_writel(dsi->iomem, REG_TIMING_CTL, 1); /* start display refresh */


	/*reg = open_dsi_readl(dsi->iomem, XDSI_CCR);
	reg |= XDSI_CCR_COREENB;

	open_dsi_writel(dsi->iomem, XDSI_CCR, reg);
	dev_dbg(dsi->dev, "MIPI DSI Tx controller is enabled.\n");*/
err_display_disable:
	drm_panel_unprepare(dsi->panel);

err_put_sync:
	return;
}

/**
 * open_dsi_set_display_disable - Disable the DSI Tx IP core enable
 * register bit
 * @dsi: DSI structure having the updated user parameters
 *
 * This function takes the DSI strucure and disables the core enable bit
 * of core configuration register.
 */
static void open_dsi_set_display_disable(struct open_dsi *dsi)
{
	u32 reg;

	dev_info(dsi->dev, "TODO: open_dsi_set_display_disable\n");
	/*reg = open_dsi_readl(dsi->iomem, XDSI_CCR);
	reg &= ~XDSI_CCR_COREENB;

	open_dsi_writel(dsi->iomem, XDSI_CCR, reg);
	dev_dbg(dsi->dev, "DSI Tx is disabled. reset regs to default values\n");*/
}

/**
 * open_dsi_atomic_set_property - implementation of drm_connector_funcs
 * set_property invoked by IOCTL call to DRM_IOCTL_MODE_OBJ_SETPROPERTY
 *
 * @connector: pointer Xilinx DSI connector
 * @state: DRM connector state
 * @prop: pointer to the drm_property structure
 * @val: DSI parameter value that is configured from user application
 *
 * This function takes a drm_property name and value given from user application
 * and update the DSI structure property varabiles with the values.
 * These values are later used to configure the DSI Rx IP.
 *
 * Return: 0 on success OR -EINVAL if setting property fails
 */
static int open_dsi_atomic_set_property(struct drm_connector *connector,
		struct drm_connector_state *state,
		struct drm_property *prop, u64 val)
{
	struct open_dsi *dsi = connector_to_dsi(connector);

	dev_dbg(dsi->dev, "set property name = %s, value = %lld\n",
			prop->name, val);

	/*if (prop == dsi->eotp_prop)
		dsi->eotp_prop_val = !!val;
	else if (prop == dsi->bllp_mode_prop)
		dsi->bllp_mode_prop_val = !!val;
	else if (prop == dsi->bllp_type_prop)
		dsi->bllp_type_prop_val = !!val;
	else if (prop == dsi->video_mode_prop)
		dsi->video_mode_prop_val = (unsigned int)val;
	else if (prop == dsi->bllp_burst_time_prop)
		dsi->bllp_burst_time_prop_val = (unsigned int)val;
	else if (prop == dsi->cmd_queue_prop)
		dsi->cmd_queue_prop_val = (unsigned int)val;
	else if (prop == dsi->height_out)
		dsi->height_out_prop_val = (u32)val;
	else if (prop == dsi->width_out)
		dsi->width_out_prop_val = (u32)val;
	else if (prop == dsi->in_fmt)
		dsi->in_fmt_prop_val = (u32)val;
	else if (prop == dsi->out_fmt)
		dsi->out_fmt_prop_val = (u32)val;
	else
		return -EINVAL;
	 */
	open_dsi_set_config_parameters(dsi);

	return 0;
}

static int
open_dsi_atomic_get_property(struct drm_connector *connector,
		const struct drm_connector_state *state,
		struct drm_property *prop, uint64_t *val)
{
	struct open_dsi *dsi = connector_to_dsi(connector);

	dev_dbg(dsi->dev, "get property name = %s\n",
			prop->name);
	/*	if (prop == dsi->eotp_prop)
	 *val = dsi->eotp_prop_val;
	else if (prop == dsi->bllp_mode_prop)
	 *val = dsi->bllp_mode_prop_val;
	else if (prop == dsi->bllp_type_prop)
	 *val = dsi->bllp_type_prop_val;
	else if (prop == dsi->video_mode_prop)
	 *val = dsi->video_mode_prop_val;
	else if (prop == dsi->bllp_burst_time_prop)
	 *val = dsi->bllp_burst_time_prop_val;
	else if (prop == dsi->cmd_queue_prop)
	 *val = dsi->cmd_queue_prop_val;
	else if (prop == dsi->height_out)
	 *val = dsi->height_out_prop_val;
	else if (prop == dsi->width_out)
	 *val = dsi->width_out_prop_val;
	else if (prop == dsi->in_fmt)
	 *val = dsi->in_fmt_prop_val;
	else if (prop == dsi->out_fmt)
	 *val = dsi->out_fmt_prop_val;
	else
		return -EINVAL;
	 */
	return 0;
}

static int open_dsi_host_attach(struct mipi_dsi_host *host,
		struct mipi_dsi_device *device)
{
	u32 panel_lanes;
	struct open_dsi *dsi = host_to_dsi(host);

	dev_info(dsi->dev, "open_dsi_host_attach\n");
	panel_lanes = device->lanes;
	dsi->mode_flags = device->mode_flags;
	dsi->panel_node = device->dev.of_node;

	if (panel_lanes != dsi->lanes) {
		dev_err(dsi->dev, "Mismatch of lanes. panel = %d, DSI = %d\n",
				panel_lanes, dsi->lanes);
		return -EINVAL;
	}

	if (dsi->lanes > 4 || dsi->lanes < 1) {
		dev_err(dsi->dev, "%d lanes : invalid xlnx,dsi-num-lanes\n",
				dsi->lanes);
		return -EINVAL;
	}

	if (device->format != dsi->format) {
		dev_err(dsi->dev, "Mismatch of format. panel = %d, DSI = %d\n",
				device->format, dsi->format);
		return -EINVAL;
	}

	if (dsi->connector.dev)
		drm_helper_hpd_irq_event(dsi->connector.dev);

	return 0;
}

static int open_dsi_host_detach(struct mipi_dsi_host *host,
		struct mipi_dsi_device *device)
{
	struct open_dsi *dsi = host_to_dsi(host);

	dev_info(dsi->dev, "open_dsi_host_detach\n");
	dsi->panel = NULL;

	if (dsi->connector.dev)
		drm_helper_hpd_irq_event(dsi->connector.dev);

	return 0;
}


static inline uint8_t parity(uint32_t d)
{
    int i, p = 0;

    for (i = 0; i < 32; i++)
        p ^= d & (1 << i) ? 1 : 0;
    return p;
}

static uint8_t reverse_bits(uint8_t x)
{
    uint8_t r = 0;
    int     i;

    for (i = 0; i < 8; i++)
        if (x & (1 << i)) r |= (1 << (7 - i));
    return r;
}

/* calculates DSI packet header ECC checksum */
static uint8_t dsi_ecc(uint32_t data)
{
    uint8_t ecc = 0;
    int     i;
    static const uint32_t masks[] =
    { 0xf12cb7, 0xf2555b, 0x749a6d, 0xb8e38e, 0xdf03f0, 0xeffc00 };

    for (i = 0; i < 6; i++)
        if (parity(data & masks[i]))
            ecc |= (1 << i);

    return ecc;
}

int dsi_ctl = 0;

/* Sends a single byte to the display in low power mode */
void dsi_lp_write_byte(struct open_dsi *dsi,uint32_t value)
{
    int rv = 0;

    open_dsi_writel(dsi->iomem, REG_DSI_CTL, (dsi->dsi_ctl | 2));

    while (!(open_dsi_readl(dsi->iomem,REG_DSI_CTL)& 2)) ;//TODO: tak czekac?
    open_dsi_writel(dsi->iomem, REG_LP_TX, value | 0x100);

    while (!(open_dsi_readl(dsi->iomem,REG_DSI_CTL) & 2)) ;
}

/* Composes a short packet and sends it in low power mode to the display */
void dsi_send_lp_short(struct open_dsi *dsi,uint8_t ptype, uint8_t w0, uint8_t w1)
{
    uint8_t  pdata[4];
    uint32_t d;

    dsi_lp_write_byte(dsi,0xe1);
    dsi_lp_write_byte(dsi,reverse_bits(ptype));
    dsi_lp_write_byte(dsi,reverse_bits(w0));
    dsi_lp_write_byte(dsi,reverse_bits(w1));
    dsi_lp_write_byte(dsi,reverse_bits(dsi_ecc(ptype |
                                           (((uint32_t)w0) <<
    8) | (((uint32_t)w1) << 16))));
    open_dsi_writel(dsi->iomem, REG_DSI_CTL, dsi->dsi_ctl);
}

uint16_t dsi_crc(const uint8_t *d, int n)
{
    uint16_t poly = 0x8408;

    int byte_counter;
    int bit_counter;
    uint8_t  current_data;
    uint16_t result = 0xffff;

    for (byte_counter = 0; byte_counter < n; byte_counter++) {
        current_data = d[byte_counter];

        for (bit_counter = 0; bit_counter < 8; bit_counter++)
        {
            if (((result & 0x0001) ^ ((current_data) & 0x0001)))
                result = ((result >> 1) & 0x7fff) ^ poly;
            else
                result = ((result >> 1) & 0x7fff);
            current_data = (current_data >> 1); // & 0x7F;
        }
    }
    return result;
}

void dsi_long_write(struct open_dsi *dsi,int is_dcs, const unsigned char *data, int length)
{
    uint8_t w1 = 0;
    uint8_t w0 = length;

    uint8_t ptype = is_dcs ? 0x39 : 0x29;
    //printf("pp_long write: %d bytes ptype %x\n", length, ptype);

    dsi_lp_write_byte(dsi,0xe1);
    dsi_lp_write_byte(dsi,reverse_bits(ptype));
    dsi_lp_write_byte(dsi,reverse_bits(w0));
    dsi_lp_write_byte(dsi,reverse_bits(w1));
    dsi_lp_write_byte(dsi,reverse_bits(dsi_ecc(ptype |
                                           (((uint32_t)w0) <<
    8) | (((uint32_t)w1) << 16))));

    int i;

    for (i = 0; i < length; i++)
        dsi_lp_write_byte(dsi,reverse_bits(data[i]));

    uint16_t crc = dsi_crc(data, length);

    crc = 0x0000;

    dsi_lp_write_byte(dsi,reverse_bits(crc & 0xff));
    dsi_lp_write_byte(dsi,reverse_bits(crc >> 8));
    open_dsi_writel(dsi->iomem, REG_DSI_CTL, dsi->dsi_ctl);
}




//open_dsi_writel(dsi->iomem,

ssize_t open_dsi_host_transfer(struct mipi_dsi_host *host,
		const struct mipi_dsi_msg *msg){
	struct open_dsi *dsi = host_to_dsi(host);
	const u8 *tx=msg->tx_buf;
	int i;
	dev_info(dsi->dev, "open_dsi_host_transfer\n");

	printk("MSG type %02X txlen %d:",msg->type,msg->tx_len);
	for(i=0;i<msg->tx_len;i++)
		printk(KERN_CONT" 0x%02X",tx[i]);
	printk(KERN_CONT"\n");



	switch(msg->type){
	default:
		return 0;
		break;
	case MIPI_DSI_DCS_SHORT_WRITE:
		dsi_send_lp_short(dsi,MIPI_DSI_DCS_SHORT_WRITE,tx[0],0);
		return 1;
		break;
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
		dsi_send_lp_short(dsi,MIPI_DSI_DCS_SHORT_WRITE,tx[0],tx[1]);
		return 2;
		break;
	case MIPI_DSI_DCS_LONG_WRITE:
		dsi_long_write(dsi,1,tx,msg->tx_len);
		return msg->tx_len;
		break;
	};
}










static const struct mipi_dsi_host_ops open_dsi_ops = {
		.attach = open_dsi_host_attach,
		.detach = open_dsi_host_detach,
		.transfer = open_dsi_host_transfer,
};

static int open_dsi_connector_dpms(struct drm_connector *connector, int mode)
{
	struct open_dsi *dsi = connector_to_dsi(connector);
	int ret;

	dev_info(dsi->dev, "open_dsi_connector_dpms\n");
	dev_dbg(dsi->dev, "connector dpms state: %d\n", mode);

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		ret = drm_panel_prepare(dsi->panel);
		if (ret < 0) {
			dev_err(dsi->dev, "DRM panel not found\n");
			return ret;
		}

		ret = drm_panel_enable(dsi->panel);
		if (ret < 0) {
			drm_panel_unprepare(dsi->panel);
			dev_err(dsi->dev, "DRM panel not enabled\n");
			return ret;
		}
		break;
	default:
		drm_panel_disable(dsi->panel);
		drm_panel_unprepare(dsi->panel);
		break;
	}

	return drm_helper_connector_dpms(connector, mode);
}

struct open_dsi * fn_connector_to_dsi(struct drm_connector * connector){
	return connector_to_dsi(connector);
}


static enum drm_connector_status
__attribute__((optimize("O0"))) open_dsi_detect(struct drm_connector *connector, bool force)
{
	struct open_dsi *dsi = fn_connector_to_dsi(connector);

	dev_info(dsi->dev, "open_dsi_detect\n");
	printk("dsi->panel %px",dsi->panel);
	if (!dsi->panel) {
		dsi->panel = of_drm_find_panel(dsi->panel_node);
		printk("dsi->panel %px",dsi->panel);
		if (IS_ERR(dsi->panel)){
			dsi->panel=0;
		}
		else{
			drm_panel_attach(dsi->panel, &dsi->connector);
		}
	} else if (!dsi->panel_node) {
		open_dsi_connector_dpms(connector, DRM_MODE_DPMS_OFF);
		drm_panel_detach(dsi->panel);
		dsi->panel = NULL;
	}

	if (dsi->panel)
		return connector_status_connected;

	return connector_status_disconnected;
}

static void open_dsi_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
	connector->dev = NULL;
}

static const struct drm_connector_funcs open_dsi_connector_funcs = {
		.dpms = open_dsi_connector_dpms,
		.detect = open_dsi_detect,
		.fill_modes = drm_helper_probe_single_connector_modes,
		.destroy = open_dsi_connector_destroy,
		.atomic_set_property = open_dsi_atomic_set_property,
		.atomic_get_property = open_dsi_atomic_get_property,
		.atomic_duplicate_state	= drm_atomic_helper_connector_duplicate_state,
		.atomic_destroy_state	= drm_atomic_helper_connector_destroy_state,
		.reset			= drm_atomic_helper_connector_reset,
};

static int open_dsi_get_modes(struct drm_connector *connector)
{
	struct open_dsi *dsi = connector_to_dsi(connector);

	dev_info(dsi->dev, "open_dsi_get_modes\n");
	if (dsi->panel)
		return dsi->panel->funcs->get_modes(dsi->panel);

	return 0;
}

static struct drm_encoder *
open_dsi_best_encoder(struct drm_connector *connector)
{

	struct open_dsi *dsi = connector_to_dsi(connector);
	dev_info(dsi->dev, "open_dsi_best_encoder\n");
	return &(connector_to_dsi(connector)->encoder);
}

static struct drm_connector_helper_funcs open_dsi_connector_helper_funcs = {
		.get_modes = open_dsi_get_modes,
		.best_encoder = open_dsi_best_encoder,
};

/**
 * open_dsi_connector_create_property -  create DSI connector properties
 *
 * @connector: pointer to Xilinx DSI connector
 *
 * This function takes the xilinx DSI connector component and defines
 * the drm_property variables with their default values.
 */
/*
static void open_dsi_connector_create_property(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct open_dsi *dsi  = connector_to_dsi(connector);

	dsi->eotp_prop = drm_property_create_bool(dev, 0, "eotp");
	dsi->video_mode_prop = drm_property_create_range(dev, 0, "video_mode",
							 0, 2);
	dsi->bllp_mode_prop = drm_property_create_bool(dev, 0, "bllp_mode");
	dsi->bllp_type_prop = drm_property_create_bool(dev, 0, "bllp_type");
	dsi->bllp_burst_time_prop =
		drm_property_create_range(dev, 0, "bllp_burst_time", 0, 0xFFFF);
	dsi->cmd_queue_prop = drm_property_create_range(dev, 0, "cmd_queue", 0,
							0xffffff);
	dsi->height_out = drm_property_create_range(dev, 0, "height_out",
						    2, 4096);
	dsi->width_out = drm_property_create_range(dev, 0, "width_out",
						   2, 4096);
	dsi->in_fmt = drm_property_create_range(dev, 0, "in_fmt", 0, 16384);
	dsi->out_fmt = drm_property_create_range(dev, 0, "out_fmt", 0, 16384);
}
 */
/**
 * open_dsi_connector_attach_property -  attach DSI connector
 * properties
 *
 * @connector: pointer to Xilinx DSI connector
 */
/*
static void open_dsi_connector_attach_property(struct drm_connector *connector)
{
	struct open_dsi *dsi = connector_to_dsi(connector);
	struct drm_mode_object *obj = &connector->base;

	if (dsi->eotp_prop)
		drm_object_attach_property(obj, dsi->eotp_prop, 1);

	if (dsi->video_mode_prop)
		drm_object_attach_property(obj, dsi->video_mode_prop, 0);

	if (dsi->bllp_burst_time_prop)
		drm_object_attach_property(&connector->base,
					   dsi->bllp_burst_time_prop, 0);

	if (dsi->bllp_mode_prop)
		drm_object_attach_property(&connector->base,
					   dsi->bllp_mode_prop, 0);

	if (dsi->bllp_type_prop)
		drm_object_attach_property(&connector->base,
					   dsi->bllp_type_prop, 0);

	if (dsi->cmd_queue_prop)
		drm_object_attach_property(&connector->base,
					   dsi->cmd_queue_prop, 0);

	if (dsi->height_out)
		drm_object_attach_property(obj, dsi->height_out, 0);

	if (dsi->width_out)
		drm_object_attach_property(obj, dsi->width_out, 0);

	if (dsi->in_fmt)
		drm_object_attach_property(obj, dsi->in_fmt, 0);

	if (dsi->out_fmt)
		drm_object_attach_property(obj, dsi->out_fmt, 0);
}
 */

static int open_dsi_create_connector(struct drm_encoder *encoder)
{
	struct open_dsi *dsi = encoder_to_dsi(encoder);
	struct drm_connector *connector = &dsi->connector;
	int ret;

	connector->polled = DRM_CONNECTOR_POLL_HPD;

	ret = drm_connector_init(encoder->dev, connector,
			&open_dsi_connector_funcs,
			DRM_MODE_CONNECTOR_DSI);
	if (ret) {
		dev_err(dsi->dev, "Failed to initialize connector with drm\n");
		return ret;
	}

	drm_connector_helper_add(connector, &open_dsi_connector_helper_funcs);
	drm_connector_register(connector);
	drm_connector_attach_encoder(connector, encoder);
	//open_dsi_connector_create_property(connector);
	//open_dsi_connector_attach_property(connector);

	return 0;
}

/**
 * open_dsi_atomic_mode_set -  derive the DSI timing parameters
 *
 * @encoder: pointer to Xilinx DRM encoder
 * @crtc_state: Pointer to drm core crtc state
 * @connector_state: DSI connector drm state
 *
 * This function derives the DSI IP timing parameters from the timing
 * values given in the attached panel driver.
 */
static void
open_dsi_atomic_mode_set(struct drm_encoder *encoder,
		struct drm_crtc_state *crtc_state,
		struct drm_connector_state *connector_state)
{
	struct open_dsi *dsi = encoder_to_dsi(encoder);
	struct videomode *vm = &dsi->vm;
	struct drm_display_mode *m = &crtc_state->adjusted_mode;

	dev_info(dsi->dev, "open_dsi_atomic_mode_set\n");
	/* Set bridge input and output parameters */


	dev_info(dsi->dev, "TODO: odniesienia do xlnx_bridge\n");

	/*xlnx_bridge_set_input(dsi->bridge, m->hdisplay, m->vdisplay,
			      dsi->in_fmt_prop_val);
	xlnx_bridge_set_output(dsi->bridge, dsi->width_out_prop_val,
			       dsi->height_out_prop_val,
			       dsi->out_fmt_prop_val);
	xlnx_bridge_enable(dsi->bridge);
	 */

	vm->hactive = m->hdisplay;
	vm->vactive = m->vdisplay;
	vm->vfront_porch = m->vsync_start - m->vdisplay;
	vm->vback_porch = m->vtotal - m->vsync_end;
	vm->vsync_len = m->vsync_end - m->vsync_start;
	vm->hfront_porch = m->hsync_start - m->hdisplay;
	vm->hback_porch = m->htotal - m->hsync_end;
	vm->hsync_len = m->hsync_end - m->hsync_start;
	open_dsi_set_display_mode(dsi);
}

static void open_dsi_disable(struct drm_encoder *encoder)
{
	struct open_dsi *dsi = encoder_to_dsi(encoder);

	dev_info(dsi->dev, "open_dsi_disable\n");
	if (dsi->bridge)
		xlnx_bridge_disable(dsi->bridge);

	open_dsi_set_display_disable(dsi);
}

static void open_dsi_enable(struct drm_encoder *encoder)
{
	struct open_dsi *dsi = encoder_to_dsi(encoder);
	dev_info(dsi->dev, "open_dsi_enable\n");
	open_dsi_set_display_enable(dsi);
}

static const struct drm_encoder_helper_funcs open_dsi_encoder_helper_funcs = {
		.atomic_mode_set = open_dsi_atomic_mode_set,
		.enable = open_dsi_enable,
		.disable = open_dsi_disable,
};

static const struct drm_encoder_funcs open_dsi_encoder_funcs = {
		.destroy = drm_encoder_cleanup,
};

static int open_dsi_parse_dt(struct open_dsi *dsi)
{
	struct device *dev = dsi->dev;
	struct device_node *node = dev->of_node;
	int ret;
	u32 datatype;
	static const int xdsi_mul_fact[XDSI_NUM_DATA_T] = {300, 225, 225, 200};

	/*dsi->dphy_clk_200M = devm_clk_get(dev, "dphy_clk_200M");
	if (IS_ERR(dsi->dphy_clk_200M)) {
		ret = PTR_ERR(dsi->dphy_clk_200M);
		dev_err(dev, "failed to get dphy_clk_200M %d\n", ret);
		return ret;
	}

	dsi->video_aclk = devm_clk_get(dev, "s_axis_aclk");
	if (IS_ERR(dsi->video_aclk)) {
		ret = PTR_ERR(dsi->video_aclk);
		dev_err(dev, "failed to get video_clk %d\n", ret);
		return ret;
	}*/

	/*
	 * Used as a multiplication factor for HACT based on used
	 * DSI data type.
	 *
	 * e.g. for RGB666_L datatype and 1920x1080 resolution,
	 * the Hact (WC) would be as follows -
	 * 1920 pixels * 18 bits per pixel / 8 bits per byte
	 * = 1920 pixels * 2.25 bytes per pixel = 4320 bytes.
	 *
	 * Data Type - Multiplication factor
	 * RGB888    - 3
	 * RGB666_L  - 2.25
-	 * RGB666_P  - 2.25
	 * RGB565    - 2
	 *
	 * Since the multiplication factor maybe a floating number,
	 * a 100x multiplication factor is used.
	 */
	ret = of_property_read_u32(node, "xlnx,lp_divider", &dsi->lp_divider);
	if (ret < 0) {
		dev_err(dsi->dev, "missing xlnx,lp_divider property\n");
		return ret;
	}





	ret = of_property_read_u32(node, "xlnx,dsi-num-lanes", &dsi->lanes);
	if (ret < 0) {
		dev_err(dsi->dev, "missing xlnx,dsi-num-lanes property\n");
		return ret;
	}
	if (dsi->lanes > 4 || dsi->lanes < 1) {
		dev_err(dsi->dev, "%d lanes : invalid lanes\n", dsi->lanes);
		return -EINVAL;
	}
	ret = of_property_read_u32(node, "xlnx,dsi-data-type", &datatype);
	if (ret < 0) {
		dev_err(dsi->dev, "missing xlnx,dsi-data-type property\n");
		return ret;
	}
	dsi->format = datatype;
	if (datatype > MIPI_DSI_FMT_RGB565) {
		dev_err(dsi->dev, "Invalid xlnx,dsi-data-type string\n");
		return -EINVAL;
	}
	dsi->mul_factor = xdsi_mul_fact[datatype];
	dev_dbg(dsi->dev, "DSI controller num lanes = %d", dsi->lanes);
	dev_dbg(dsi->dev, "DSI controller datatype = %d\n", datatype);

	return 0;
}

static int open_dsi_bind(struct device *dev, struct device *master,
		void *data)
{
	struct open_dsi *dsi = dev_get_drvdata(dev);
	struct drm_encoder *encoder = &dsi->encoder;
	struct drm_device *drm_dev = data;
	int ret;

	/*
	 * TODO: The possible CRTCs are 1 now as per current implementation of
	 * DSI tx drivers. DRM framework can support more than one CRTCs and
	 * DSI driver can be enhanced for that.
	 */
	encoder->possible_crtcs = 1;
	drm_encoder_init(drm_dev, encoder, &open_dsi_encoder_funcs,
			DRM_MODE_ENCODER_DSI, NULL);
	drm_encoder_helper_add(encoder, &open_dsi_encoder_helper_funcs);
	ret = open_dsi_create_connector(encoder);
	if (ret) {
		dev_err(dsi->dev, "fail creating connector, ret = %d\n", ret);
		drm_encoder_cleanup(encoder);
		return ret;
	}
	ret = mipi_dsi_host_register(&dsi->dsi_host);
	if (ret) {
		open_dsi_connector_destroy(&dsi->connector);
		drm_encoder_cleanup(encoder);
		return ret;
	}
	return 0;
}

static void open_dsi_unbind(struct device *dev, struct device *master,
		void *data)
{
	struct open_dsi *dsi = dev_get_drvdata(dev);

	open_dsi_disable(&dsi->encoder);
	mipi_dsi_host_unregister(&dsi->dsi_host);
	xlnx_bridge_disable(dsi->bridge);
}

static const struct component_ops open_dsi_component_ops = {
		.bind	= open_dsi_bind,
		.unbind	= open_dsi_unbind,
};

static int open_dsi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct open_dsi *dsi;
	struct device_node *vpss_node;
	int ret;
	unsigned long rate;

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	dsi->dsi_host.ops = &open_dsi_ops;
	dsi->dsi_host.dev = dev;
	dsi->dev = dev;

	ret = open_dsi_parse_dt(dsi);
	if (ret)
		return ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dsi->iomem = devm_ioremap_resource(dev, res);
	if (IS_ERR(dsi->iomem))
		return PTR_ERR(dsi->iomem);

	platform_set_drvdata(pdev, dsi);

	/* Bridge support */
	vpss_node = of_parse_phandle(dsi->dev->of_node, "xlnx,vpss", 0);
	if (vpss_node) {
		dsi->bridge = of_xlnx_bridge_get(vpss_node);
		if (!dsi->bridge) {
			dev_info(dsi->dev, "Didn't get bridge instance\n");
			return -EPROBE_DEFER;
		}
	}
	/*
	ret = clk_set_rate(dsi->dphy_clk_200M, XDSI_DPHY_CLK_REQ);
	if (ret) {
		dev_err(dev, "failed to set dphy clk rate %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(dsi->dphy_clk_200M);
	if (ret) {
		dev_err(dev, "failed to enable dphy clk %d\n", ret);
		return ret;
	}

	rate = clk_get_rate(dsi->dphy_clk_200M);
	if (rate < XDSI_DPHY_CLK_MIN && rate > XDSI_DPHY_CLK_MAX) {
		dev_err(dev, "Error DPHY clock = %lu\n", rate);
		ret = -EINVAL;
		goto err_disable_dphy_clk;
	}

	ret = clk_prepare_enable(dsi->video_aclk);
	if (ret) {
		dev_err(dev, "failed to enable video clk %d\n", ret);
		goto err_disable_dphy_clk;
	}*/

	ret = component_add(dev, &open_dsi_component_ops);
	if (ret < 0)
		goto err_disable_video_clk;
	dev_info(dsi->dev, "Probed\n");
	return ret;

	err_disable_video_clk:
	//clk_disable_unprepare(dsi->video_aclk);
	err_disable_dphy_clk:
	//clk_disable_unprepare(dsi->dphy_clk_200M);
	return ret;
}

static int open_dsi_remove(struct platform_device *pdev)
{
	struct open_dsi *dsi = platform_get_drvdata(pdev);

	component_del(&pdev->dev, &open_dsi_component_ops);
	clk_disable_unprepare(dsi->video_aclk);
	//clk_disable_unprepare(dsi->dphy_clk_200M);

	return 0;
}

static const struct of_device_id open_dsi_of_match[] = {
		{ .compatible = "open,dsi"},
		{ }
};
MODULE_DEVICE_TABLE(of, open_dsi_of_match);

static struct platform_driver dsi_driver = {
		.probe = open_dsi_probe,
		.remove = open_dsi_remove,
		.driver = {
				.name = "open-dsi",
				.of_match_table = open_dsi_of_match,
		},
};

module_platform_driver(dsi_driver);

MODULE_AUTHOR("Siva Rajesh <sivaraj@xilinx.com>");
MODULE_DESCRIPTION("Xilinx FPGA MIPI DSI Tx Driver");
MODULE_LICENSE("GPL v2");
