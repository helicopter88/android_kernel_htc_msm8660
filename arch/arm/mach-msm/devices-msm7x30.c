/*
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
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
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <linux/dma-mapping.h>
#include <linux/clkdev.h>
#include <mach/irqs.h>
#include <mach/msm_iomap.h>
#include <mach/dma.h>
#include <mach/board.h>

#include "devices.h"
#include "smd_private.h"

#include <asm/mach/flash.h>

#include "clock-pcom.h"
#include "clock-7x30.h"

#include <mach/mmc.h>

static struct resource resources_uart2[] = {
	{
		.start	= INT_UART2,
		.end	= INT_UART2,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART2_PHYS,
		.end	= MSM_UART2_PHYS + MSM_UART2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
		.name  = "uart_resource"
	},
};

struct platform_device msm_device_uart2 = {
	.name	= "msm_serial",
	.id	= 1,
	.num_resources	= ARRAY_SIZE(resources_uart2),
	.resource	= resources_uart2,
};

struct platform_device msm_device_smd = {
	.name   = "msm_smd",
	.id     = -1,
};

static struct resource resources_otg[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + MSM_HSUSB_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_USB_HS,
		.end	= INT_USB_HS,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_otg = {
	.name		= "msm_otg",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_otg),
	.resource	= resources_otg,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

static struct resource resources_hsusb[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + MSM_HSUSB_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_USB_HS,
		.end	= INT_USB_HS,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_hsusb = {
	.name		= "msm_hsusb",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_hsusb),
	.resource	= resources_hsusb,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

static u64 dma_mask = 0xffffffffULL;
static struct resource resources_hsusb_host[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + MSM_HSUSB_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_USB_HS,
		.end	= INT_USB_HS,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_hsusb_host = {
	.name		= "msm_hsusb_host",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_hsusb_host),
	.resource	= resources_hsusb_host,
	.dev		= {
		.dma_mask               = &dma_mask,
		.coherent_dma_mask      = 0xffffffffULL,
	},
};

struct clk_lookup msm_clocks_7x30[] = {
	CLK_PCOM("adm_clk",	ADM_CLK,	NULL, 0),
	CLK_PCOM("adsp_clk",	ADSP_CLK,	NULL, 0),
	CLK_PCOM("cam_m_clk",	CAM_M_CLK,	NULL, 0),
	CLK_PCOM("camif_pad_pclk",	CAMIF_PAD_P_CLK,	NULL, OFF),
	CLK_PCOM("ce_clk",	CE_CLK,	NULL, 0),
	CLK_PCOM("codec_ssbi_clk",	CODEC_SSBI_CLK,	NULL, 0),
	CLK_PCOM("ebi1_clk",	EBI1_CLK,	NULL, CLK_MIN),
	CLK_PCOM("ecodec_clk",	ECODEC_CLK,	NULL, 0),
	CLK_PCOM("emdh_clk",	EMDH_CLK,	NULL, OFF | CLK_MINMAX),
	CLK_PCOM("emdh_pclk",	EMDH_P_CLK,	NULL, OFF),
	CLK_PCOM("gp_clk",	GP_CLK,		NULL, 0),
	CLK_PCOM("grp_2d_clk",	GRP_2D_CLK,	NULL, 0),
	CLK_PCOM("grp_2d_pclk",	GRP_2D_P_CLK,	NULL, 0),
	CLK_PCOM("grp_clk",	GRP_3D_CLK,	NULL, 0),
	CLK_PCOM("grp_pclk",	GRP_3D_P_CLK,	NULL, 0),
	CLK_7X30S("grp_src_clk", GRP_3D_SRC_CLK, GRP_3D_CLK,	NULL, 0),
	CLK_PCOM("hdmi_clk",	HDMI_CLK,	NULL, 0),
	CLK_PCOM("imem_clk",	IMEM_CLK,	NULL, OFF),
	CLK_PCOM("jpeg_clk",	JPEG_CLK,	NULL, OFF),
	CLK_PCOM("jpeg_pclk",	JPEG_P_CLK,	NULL, OFF),
	CLK_PCOM("lpa_codec_clk",	LPA_CODEC_CLK,		NULL, 0),
	CLK_PCOM("lpa_core_clk",	LPA_CORE_CLK,		NULL, 0),
	CLK_PCOM("lpa_pclk",		LPA_P_CLK,		NULL, 0),
	CLK_PCOM("mdc_clk",	MDC_CLK,	NULL, 0),
	CLK_PCOM("mddi_clk",	PMDH_CLK,	NULL, OFF | CLK_MINMAX),
	CLK_PCOM("mddi_pclk",	PMDH_P_CLK,	NULL, 0),
	CLK_PCOM("mdp_clk",	MDP_CLK,	NULL, OFF),
	CLK_PCOM("mdp_pclk",	MDP_P_CLK,	NULL, 0),
	CLK_PCOM("mdp_lcdc_pclk_clk", MDP_LCDC_PCLK_CLK, NULL, 0),
	CLK_PCOM("mdp_lcdc_pad_pclk_clk", MDP_LCDC_PAD_PCLK_CLK, NULL, 0),
	CLK_PCOM("mdp_vsync_clk",	MDP_VSYNC_CLK,  NULL, 0),
	CLK_PCOM("mfc_clk",		MFC_CLK,		NULL, 0),
	CLK_PCOM("mfc_div2_clk",	MFC_DIV2_CLK,		NULL, 0),
	CLK_PCOM("mfc_pclk",		MFC_P_CLK,		NULL, 0),
	CLK_PCOM("mi2s_m_clk",		MI2S_M_CLK,  		NULL, 0),
	CLK_PCOM("mi2s_s_clk",		MI2S_S_CLK,  		NULL, 0),
	CLK_PCOM("mi2s_codec_rx_m_clk",	MI2S_CODEC_RX_M_CLK,  NULL, 0),
	CLK_PCOM("mi2s_codec_rx_s_clk",	MI2S_CODEC_RX_S_CLK,  NULL, 0),
	CLK_PCOM("mi2s_codec_tx_m_clk",	MI2S_CODEC_TX_M_CLK,  NULL, 0),
	CLK_PCOM("mi2s_codec_tx_s_clk",	MI2S_CODEC_TX_S_CLK,  NULL, 0),
	CLK_PCOM("pbus_clk",	PBUS_CLK,	NULL, CLK_MIN),
	CLK_PCOM("pcm_clk",	PCM_CLK,	NULL, 0),
	CLK_PCOM("rotator_clk",	AXI_ROTATOR_CLK,		NULL, 0),
	CLK_PCOM("rotator_imem_clk",	ROTATOR_IMEM_CLK,	NULL, OFF),
	CLK_PCOM("rotator_pclk",	ROTATOR_P_CLK,		NULL, OFF),
	CLK_PCOM("sdac_clk",	SDAC_CLK,	NULL, OFF),
	CLK_PCOM("spi_clk",	SPI_CLK,	NULL, 0),
	CLK_PCOM("spi_pclk",	SPI_P_CLK,	NULL, 0),
	CLK_7X30S("tv_src_clk",	TV_CLK, 	TV_ENC_CLK,	NULL, 0),
	CLK_PCOM("tv_dac_clk",	TV_DAC_CLK,	NULL, 0),
	CLK_PCOM("tv_enc_clk",	TV_ENC_CLK,	NULL, 0),
	CLK_PCOM("uart_clk",	UART2_CLK,	"msm_serial.1", 0),
	CLK_PCOM("usb_phy_clk",	USB_PHY_CLK,	NULL, 0),
	CLK_PCOM("usb_hs_clk",		USB_HS_CLK,		NULL, OFF),
	CLK_PCOM("usb_hs_pclk",		USB_HS_P_CLK,		NULL, OFF),
	CLK_PCOM("usb_hs_core_clk",	USB_HS_CORE_CLK,	NULL, OFF),
	CLK_PCOM("usb_hs2_clk",		USB_HS2_CLK,		NULL, OFF),
	CLK_PCOM("usb_hs2_pclk",	USB_HS2_P_CLK,		NULL, OFF),
	CLK_PCOM("usb_hs2_core_clk",	USB_HS2_CORE_CLK,	NULL, OFF),
	CLK_PCOM("usb_hs3_clk",		USB_HS3_CLK,		NULL, OFF),
	CLK_PCOM("usb_hs3_pclk",	USB_HS3_P_CLK,		NULL, OFF),
	CLK_PCOM("usb_hs3_core_clk",	USB_HS3_CORE_CLK,	NULL, OFF),
	CLK_PCOM("vdc_clk",	VDC_CLK,	NULL, OFF | CLK_MIN),
	CLK_PCOM("vfe_camif_clk",	VFE_CAMIF_CLK, 	NULL, 0),
	CLK_PCOM("vfe_clk",	VFE_CLK,	NULL, 0),
	CLK_PCOM("vfe_mdc_clk",	VFE_MDC_CLK,	NULL, 0),
	CLK_PCOM("vfe_pclk",	VFE_P_CLK,	NULL, OFF),
	CLK_PCOM("vpe_clk",	VPE_CLK,	NULL, 0),

	/* 7x30 v2 hardware only. */
	CLK_PCOM("csi_clk",	CSI0_CLK,	NULL, 0),
	CLK_PCOM("csi_pclk",	CSI0_P_CLK,	NULL, 0),
	CLK_PCOM("csi_vfe_clk",	CSI0_VFE_CLK,	NULL, 0),
};

unsigned msm_num_clocks_7x30 = ARRAY_SIZE(msm_clocks_7x30);

#ifdef CONFIG_MSM_ROTATOR
static struct resource resources_msm_rotator[] = {
	{
		.start	= 0xA3E00000,
		.end	= 0xA3F00000 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_ROTATOR,
		.end	= INT_ROTATOR,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct msm_rot_clocks rotator_clocks[] = {
	{
		.clk_name = "core_clk",
		.clk_type = ROTATOR_CORE_CLK,
		.clk_rate = 0,
	},
	{
		.clk_name = "iface_clk",
		.clk_type = ROTATOR_PCLK,
		.clk_rate = 0,
	},
	{
		.clk_name = "mem_clk",
		.clk_type = ROTATOR_IMEM_CLK,
		.clk_rate = 0,
	},
};

static struct msm_rotator_platform_data rotator_pdata = {
	.number_of_clocks = ARRAY_SIZE(rotator_clocks),
	.hardware_version_number = 0x1000303,
	.rotator_clks = rotator_clocks,
	.regulator_name = "fs_rot",
};

struct platform_device msm_rotator_device = {
	.name		= "msm_rotator",
	.id		= 0,
	.num_resources  = ARRAY_SIZE(resources_msm_rotator),
	.resource       = resources_msm_rotator,
	.dev = {
		.platform_data = &rotator_pdata,
	},
};
#endif

static void __init msm_register_device(struct platform_device *pdev, void *data)
{
	int ret;

	pdev->dev.platform_data = data;

	ret = platform_device_register(pdev);
	if (ret)
		dev_err(&pdev->dev,
			  "%s: platform_device_register() failed = %d\n",
			  __func__, ret);
}

void __init msm_fb_register_device(char *name, void *data)
{
	if (!strncmp(name, "mdp", 3))
		msm_register_device(&msm_mdp_device, data);
	else if (!strncmp(name, "pmdh", 4))
		msm_register_device(&msm_mddi_device, data);
	else if (!strncmp(name, "emdh", 4))
		msm_register_device(&msm_mddi_ext_device, data);
	else if (!strncmp(name, "ebi2", 4))
		msm_register_device(&msm_ebi2_lcd_device, data);
	else if (!strncmp(name, "tvenc", 5))
		msm_register_device(&msm_tvenc_device, data);
	else if (!strncmp(name, "lcdc", 4))
		msm_register_device(&msm_lcdc_device, data);
	else if (!strncmp(name, "dtv", 3))
		msm_register_device(&msm_dtv_device, data);
#ifdef CONFIG_FB_MSM_TVOUT
	else if (!strncmp(name, "tvout_device", 12))
		msm_register_device(&tvout_msm_device, data);
#endif
	else
		printk(KERN_ERR "%s: unknown device! %s\n", __func__, name);
}

static struct platform_device msm_camera_device = {
	.name	= "msm_camera",
	.id	= 0,
};

void __init msm_camera_register_device(void *res, uint32_t num,
	void *data)
{
	msm_camera_device.num_resources = num;
	msm_camera_device.resource = res;

	msm_register_device(&msm_camera_device, data);
}

struct resource kgsl_3d0_resources[] = {
	{
		.name  = KGSL_3D0_REG_MEMORY,
		.start = 0xA3500000, /* 3D GRP address */
		.end = 0xA351ffff,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = KGSL_3D0_IRQ,
		.start = INT_GRP_3D,
		.end = INT_GRP_3D,
		.flags = IORESOURCE_IRQ,
	},
};

static struct kgsl_device_platform_data kgsl_3d0_pdata = {
	.pwr_data = {
		.pwrlevel = {
			{
				.gpu_freq = 245760000,
				.bus_freq = 192000000,
			},
			{
				.gpu_freq = 192000000,
				.bus_freq = 152000000,
			},
			{
				.gpu_freq = 192000000,
				.bus_freq = 0,
			},
		},
		.init_level = 0,
		.num_levels = 3,
		.set_grp_async = set_grp3d_async,
		.idle_timeout = HZ/20,
		.nap_allowed = true,
	},
	.clk = {
		.name = {
			.clk = "core_clk",
			.pclk = "iface_clk",
		},
	},
	.imem_clk_name = {
		.clk = "mem_clk",
		.pclk = NULL,
	},
};

struct platform_device msm_kgsl_3d0 = {
	.name = "kgsl-3d0",
	.id = 0,
	.num_resources = ARRAY_SIZE(kgsl_3d0_resources),
	.resource = kgsl_3d0_resources,
	.dev = {
		.platform_data = &kgsl_3d0_pdata,
	},
};

static struct resource kgsl_2d0_resources[] = {
	{
		.name = KGSL_2D0_REG_MEMORY,
		.start = 0xA3900000, /* Z180 base address */
		.end = 0xA3900FFF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = KGSL_2D0_IRQ,
		.start = INT_GRP_2D,
		.end = INT_GRP_2D,
		.flags = IORESOURCE_IRQ,
	},
};

static struct kgsl_device_platform_data kgsl_2d0_pdata = {
	.pwr_data = {
		.pwrlevel = {
			{
				.gpu_freq = 0,
				.bus_freq = 192000000,
			},
		},
		.init_level = 0,
		.num_levels = 1,
		/* HW workaround, run Z180 SYNC @ 192 MHZ */
		.set_grp_async = NULL,
		.idle_timeout = HZ/10,
		.nap_allowed = true,
	},
	.clk = {
		.name = {
			.clk = "core_clk",
			.pclk = "iface_clk",
		},
	},
};

struct platform_device msm_kgsl_2d0 = {
	.name = "kgsl-2d0",
	.id = 0,
	.num_resources = ARRAY_SIZE(kgsl_2d0_resources),
	.resource = kgsl_2d0_resources,
	.dev = {
		.platform_data = &kgsl_2d0_pdata,
	},
};

struct platform_device *msm_footswitch_devices[] = {
	FS_PCOM(FS_GFX2D0, "fs_gfx2d0"),
	FS_PCOM(FS_GFX3D,  "fs_gfx3d"),
	FS_PCOM(FS_MDP,    "fs_mdp"),
	FS_PCOM(FS_MFC,    "fs_mfc"),
	FS_PCOM(FS_ROT,    "fs_rot"),
	FS_PCOM(FS_VFE,    "fs_vfe"),
	FS_PCOM(FS_VPE,    "fs_vpe"),
};
unsigned msm_num_footswitch_devices = ARRAY_SIZE(msm_footswitch_devices);
