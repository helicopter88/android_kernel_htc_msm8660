/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "../../../../drivers/video/msm/msm_fb.h"
#include "../../../../drivers/video/msm/mipi_dsi.h"
#include "../../../../drivers/video/msm/mdp4.h"

#include <mach/gpio.h>
#include <mach/panel_id.h>
#include <mach/msm_bus_board.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/leds.h>
#include <linux/mfd/pmic8058.h>
#include <linux/pwm.h>
#include <linux/pmic8058-pwm.h>
#include <mach/debug_display.h>
#include <mach/board-msm8660.h>

#include "../devices.h"
#include "../board-shooter.h"

static int mipi_dsi_panel_power(const int on);
static int __devinit shooter_lcd_probe(struct platform_device *pdev);
static void shooter_lcd_shutdown(struct platform_device *pdev);
static int shooter_lcd_on(struct platform_device *pdev);
static int shooter_lcd_off(struct platform_device *pdev);
static void shooter_set_backlight(struct msm_fb_data_type *mfd);
static void shooter_display_on(struct msm_fb_data_type *mfd);
static int mipi_shooter_device_register(const char* dev_name, struct msm_panel_info *pinfo, u32 channel, u32 panel);
static int wled_trigger_initialized;

static struct dsi_buf panel_tx_buf;
static struct dsi_buf panel_rx_buf;
static struct msm_panel_info pinfo;
static int cur_bl_level = 0;
static int mipi_lcd_on = 1;
struct dcs_cmd_req cmdreq;
static u32 manu_id;

static char sw_reset[2] = {0x01, 0x00};
static char enter_sleep[2] = {0x10, 0x00};
static char exit_sleep[2] = {0x11, 0x00};
static char display_off[2] = {0x28, 0x00};
static char display_on[2] = {0x29, 0x00};
static char enable_te[2] = {0x35, 0x00};
static char test_reg[3] = {0x44, 0x01, 0x3f};
static char max_pktsize[2] = {MIPI_DSI_MRPS, 0x00};

void mdp4_dsi_color_enhancement(const struct mdp_reg *reg_seq, int size);

static struct pm_gpio pwm_gpio_config = {
		.direction	= PM_GPIO_DIR_OUT,
		.output_value	= 0,
		.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
		.pull		= PM_GPIO_PULL_NO,
		.out_strength	= PM_GPIO_STRENGTH_HIGH,
		.function	= PM_GPIO_FUNC_NORMAL,
		.vin_sel	= PM8058_GPIO_VIN_L5,
		.inv_int_pol	= 0,
};

static struct pm_gpio clk_gpio_config_on = {
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 1,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_2,
				.vin_sel	= PM8058_GPIO_VIN_L5,
				.inv_int_pol	= 0,
};

static struct pm_gpio clk_gpio_config_off = {
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 0,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= PM8058_GPIO_VIN_L5,
				.inv_int_pol	= 0,
};

enum MODE_3D {
	BARRIER_OFF       = 0,
	BARRIER_LANDSCAPE = 1,
	BARRIER_PORTRAIT  = 2,
	BARRIER_END
};

atomic_t g_3D_mode = ATOMIC_INIT(BARRIER_OFF);
static struct pwm_device *pwm_3d = NULL;
struct kobject *kobj, *uevent_kobj;
struct kset *uevent_kset;

static struct platform_driver this_driver = {
	.probe  = shooter_lcd_probe,
	.shutdown = shooter_lcd_shutdown,
};

struct msm_fb_panel_data shooter_panel_data = {
	.on		= shooter_lcd_on,
	.off		= shooter_lcd_off,
	.set_backlight  = shooter_set_backlight,
	.display_on 	= shooter_display_on,
};

static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.vsync_gpio 	= GPIO_LCD_TE,
	.dsi_power_save = mipi_dsi_panel_power,
};

static char set_width[5] = {0x2A, 0x00, 0x00, 0x02, 0x1B};
static char set_height[5] = {0x2B, 0x00, 0x00, 0x03, 0xBF};
static char set_num_of_lanes[2] = {0xae, 0x03};
static char rgb_888[2] = {0x3A, 0x77};

static char novatek_pwm_f3[2] = {0xF3, 0xAA };
static char novatek_pwm_00[2] = {0x00, 0x01 };
static char novatek_pwm_21[2] = {0x21, 0x20 };
static char novatek_pwm_22[2] = {0x22, 0x03 };
static char novatek_pwm_7d[2] = {0x7D, 0x01 };
static char novatek_pwm_7f[2] = {0x7F, 0xAA };

static char novatek_pwm_cp[2] = {0x09, 0x34 };
static char novatek_pwm_cp2[2] = {0xc9, 0x01 };
static char novatek_pwm_cp3[2] = {0xff, 0xaa };

static struct dsi_cmd_desc shr_sharp_cmd_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 10,
		sizeof(sw_reset), sw_reset},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_f3), novatek_pwm_f3},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_00), novatek_pwm_00},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_21), novatek_pwm_21},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_22), novatek_pwm_22},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_7d), novatek_pwm_7d},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_7f), novatek_pwm_7f},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_f3), novatek_pwm_f3},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_cp), novatek_pwm_cp},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_cp2), novatek_pwm_cp2},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_cp3), novatek_pwm_cp3},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(enable_te), enable_te},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(test_reg), test_reg},
	{DTYPE_MAX_PKTSIZE, 1, 0, 0, 0,
		sizeof(max_pktsize), max_pktsize},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(set_num_of_lanes), set_num_of_lanes},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(set_width), set_width},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(set_height), set_height},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(rgb_888), rgb_888},
};

static struct dsi_cmd_desc shr_sharp_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 0,
		sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 110,
		sizeof(enter_sleep), enter_sleep}
};

static struct dsi_cmd_desc shr_sharp_display_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 0,
		sizeof(display_on), display_on},
};

static char manufacture_id[2] = {0x04, 0x00}; 

static struct dsi_cmd_desc shooter_manufacture_id_cmd = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(manufacture_id), manufacture_id};

static void mipi_shooter_manufature_cb(u32 data)
{
	manu_id = data;
	pr_info("%s: manufature_id=%x\n", __func__, manu_id);
}

static uint32 mipi_shooter_manufacture_id(struct msm_fb_data_type *mfd)
{
	cmdreq.cmds = &shooter_manufacture_id_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = 3;
	cmdreq.cb = mipi_shooter_manufature_cb;
	mipi_dsi_cmdlist_put(&cmdreq);

	return manu_id;
}

#define BRI_SETTING_MIN		30
#define BRI_SETTING_DEF		143
#define BRI_SETTING_MAX		255

#define PWM_MIN				8
#define PWM_DEFAULT			91
#define PWM_MAX				232

unsigned char shrink_br = BRI_SETTING_MAX;
unsigned char last_br_2d = BRI_SETTING_MAX;

static unsigned char shooter_shrink_pwm(int val)
{
	if (val <= 0) {
		shrink_br = 0;
	} else if (val > 0 && (val < BRI_SETTING_MIN)) {
		shrink_br = PWM_MIN;
	} else if ((val >= BRI_SETTING_MIN) && (val <= BRI_SETTING_DEF)) {
		shrink_br = (val - BRI_SETTING_MIN) * (PWM_DEFAULT - PWM_MIN) /
			(BRI_SETTING_DEF - BRI_SETTING_MIN) + PWM_MIN;
	} else if (val > BRI_SETTING_DEF && val <= BRI_SETTING_MAX) {
		shrink_br = (val - BRI_SETTING_DEF) * (PWM_MAX - PWM_DEFAULT) /
			(BRI_SETTING_MAX - BRI_SETTING_DEF) + PWM_DEFAULT;
	} else if (val > BRI_SETTING_MAX)
		shrink_br = PWM_MAX;

	if (atomic_read(&g_3D_mode) != BARRIER_OFF && shrink_br != 0)
		shrink_br = 255;
	else
		last_br_2d = val;

	PR_DISP_DEBUG("brightness orig=%d, transformed=%d\n", val, shrink_br);

	return shrink_br;
}

static struct mipi_dsi_phy_ctrl dsi_cmd_mode_phy_db = {
	{0x03, 0x01, 0x01, 0x00},		
	{0xB4, 0x8D, 0x1D, 0x00, 0x20, 0x94, 0x20,
	0x8F, 0x20, 0x03, 0x04},
	{0x7f, 0x00, 0x00, 0x00},	
	{0xee, 0x02, 0x86, 0x00},		
	{0x40, 0xf9, 0xb0, 0xda, 0x00, 0x50, 0x48, 0x63, 0x30, 0x07, 0x03,
	0x05, 0x14, 0x03, 0x0, 0x0, 0x54, 0x06, 0x10, 0x04, 0x0},
};

static int __init shooter_shooter_blue_qhd_pt_init(void)
{
	int ret;

	pinfo.xres = 540;
	pinfo.yres = 960;
	pinfo.type = MIPI_CMD_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;
	pinfo.lcdc.h_back_porch = 50;
	pinfo.lcdc.h_front_porch = 50;
	pinfo.lcdc.h_pulse_width = 20;
	pinfo.lcdc.v_back_porch = 11;
	pinfo.lcdc.v_front_porch = 10;
	pinfo.lcdc.v_pulse_width = 5;
	pinfo.lcdc.border_clr = 0;	
	pinfo.lcdc.underflow_clr = 0xff;	
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = 255;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;
	pinfo.clk_rate = 454000000;
	pinfo.is_3d_panel = FB_TYPE_3D_PANEL;
	pinfo.lcd.vsync_enable = TRUE;
	pinfo.lcd.hw_vsync_mode = TRUE;
	pinfo.lcd.refx100 = 6000; 
	pinfo.lcd.v_back_porch = 11;
	pinfo.lcd.v_front_porch = 10;
	pinfo.lcd.v_pulse_width = 5;

	pinfo.mipi.mode = DSI_CMD_MODE;
	pinfo.mipi.dst_format = DSI_CMD_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.esc_byte_ratio = 4;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.t_clk_post = 0x22;
	pinfo.mipi.t_clk_pre = 0x3f;
	pinfo.mipi.stream = 0;	
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_NONE;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.te_sel = 1; 
	pinfo.mipi.interleave_max = 1;
	pinfo.mipi.insert_dcs_cmd = TRUE;
	pinfo.mipi.wr_mem_continue = 0x3c;
	pinfo.mipi.wr_mem_start = 0x2c;
	pinfo.mipi.dsi_phy_db = &dsi_cmd_mode_phy_db;

	ret = mipi_shooter_device_register("mipi_novatek", &pinfo, MIPI_DSI_PRIM,
						MIPI_DSI_PANEL_QHD_PT);
	if (ret)
		pr_err("%s: failed to register device!\n", __func__);

	this_driver.driver.name = "mipi_novatek";

	return ret;
}

DEFINE_LED_TRIGGER(bkl_led_trigger);

static char led_pwm1[2] = {0x51, 0xF0};

static struct dsi_cmd_desc backlight_cmd[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(led_pwm1), led_pwm1},
};

static inline void shooter_mipi_dsi_set_backlight(struct msm_fb_data_type *mfd)
{
	struct mipi_panel_info *mipi;

	mipi  = &mfd->panel_info.mipi;

	if (panel_type == PANEL_ID_SHR_SHARP_NT)
		shooter_shrink_pwm(mfd->bl_level);

	if (panel_type == PANEL_ID_SHR_SHARP_NT) {
		cmdreq.cmds = (struct dsi_cmd_desc*)&backlight_cmd;
		cmdreq.cmds_cnt = 1;
		cmdreq.flags = CMD_REQ_COMMIT;
		cmdreq.rlen = 0;
		cmdreq.cb = NULL;
	}

	return;
}

static int shooter_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mipi  = &mfd->panel_info.mipi;

	if (mipi->mode == DSI_VIDEO_MODE) {		
		PR_DISP_ERR("%s: not support DSI_VIDEO_MODE!(%d)", __func__, mipi->mode);
	} else {
		if (!mipi_lcd_on) {
			mipi_dsi_cmd_bta_sw_trigger(); 
			if (panel_type == PANEL_ID_SHR_SHARP_NT) {
				cmdreq.cmds = shr_sharp_cmd_on_cmds;
				cmdreq.cmds_cnt = ARRAY_SIZE(shr_sharp_cmd_on_cmds);
				cmdreq.flags = CMD_REQ_COMMIT;
				cmdreq.rlen = 0;
				cmdreq.cb = NULL;
				mipi_dsi_cmdlist_put(&cmdreq);
			} else {
				PR_DISP_ERR("%s: panel_type is not supported!(%d)", __func__, panel_type);
			}
		}

		mipi_dsi_cmd_bta_sw_trigger(); 

		mipi_shooter_manufacture_id(mfd);
	}

	mipi_lcd_on = 1;

	return 0;
}

static void shooter_display_on(struct msm_fb_data_type *mfd)
{
	if (panel_type == PANEL_ID_SHR_SHARP_NT) {
		cmdreq.cmds = shr_sharp_display_on_cmds;
		cmdreq.cmds_cnt = ARRAY_SIZE(shr_sharp_display_on_cmds);
		cmdreq.flags = CMD_REQ_COMMIT;
		cmdreq.rlen = 0;
		cmdreq.cb = NULL;
		mipi_dsi_cmdlist_put(&cmdreq);
	}

	cur_bl_level = 0;
}

static int shooter_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;
	if (!mipi_lcd_on)
		return 0;

	if (panel_type == PANEL_ID_SHR_SHARP_NT) {
		cmdreq.cmds = shr_sharp_display_off_cmds;
		cmdreq.cmds_cnt = ARRAY_SIZE(shr_sharp_display_off_cmds);
		cmdreq.flags = CMD_REQ_COMMIT;
		cmdreq.rlen = 0;
		cmdreq.cb = NULL;
		mipi_dsi_cmdlist_put(&cmdreq);
	}
	mipi_lcd_on = 0;
	return 0;
}

static void shooter_set_backlight(struct msm_fb_data_type *mfd)
{
	if (!mfd->panel_power_on || cur_bl_level == mfd->bl_level) {
		return;
	}

	shooter_mipi_dsi_set_backlight(mfd);

	cur_bl_level = mfd->bl_level;
	PR_DISP_DEBUG("%s- bl_level=%d\n", __func__, mfd->bl_level);
}

static int __devinit shooter_lcd_probe(struct platform_device *pdev)
{
	msm_fb_add_device(pdev);

	return 0;
}

static void shooter_lcd_shutdown(struct platform_device *pdev)
{
	mipi_dsi_panel_power(0);
}

static int mipi_shooter_device_register(const char* dev_name, struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	static int ch_used[3] = {0, 0, 0};
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc(dev_name, (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	shooter_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &shooter_panel_data,
		sizeof(shooter_panel_data));
	if (ret) {
		PR_DISP_ERR("%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		PR_DISP_ERR("%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}
	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int mipi_dsi_panel_power(const int on)
{
	static bool dsi_power_on = false;
	static struct regulator *v_lcm, *v_lcmio;
	static bool bPanelPowerOn = false;
	int rc;

	const char * const lcm_str 	  = "8058_l12";
	const char * const lcmio_str  = "8901_lvs1";

	PR_DISP_INFO("%s: state : %d\n", __func__, on);

	if (!dsi_power_on) {

		v_lcm = regulator_get(NULL,
				lcm_str);
		if (IS_ERR(v_lcm)) {
			PR_DISP_ERR("could not get %s, rc = %ld\n",
				lcm_str, PTR_ERR(v_lcm));
			return -ENODEV;
		}

		v_lcmio = regulator_get(NULL,
				lcmio_str);
		if (IS_ERR(v_lcmio)) {
			PR_DISP_ERR("could not get %s, rc = %ld\n",
				lcmio_str, PTR_ERR(v_lcmio));
			return -ENODEV;
		}

		if (panel_type == PANEL_ID_SHR_SHARP_NT) {
			rc = regulator_set_voltage(v_lcm, 3000000, 3000000);
			if (rc) {
				PR_DISP_ERR("%s#%d: set_voltage %s failed, rc=%d\n", __func__, __LINE__, lcm_str, rc);
				return -EINVAL;
			}
		}

		rc = gpio_request(GPIO_LCM_RST_N, "LCM_RST_N");
		if (rc) {
			PR_DISP_ERR("%s:LCM gpio %d request failed, rc=%d\n", __func__,  GPIO_LCM_RST_N, rc);
			return -EINVAL;
		}
		dsi_power_on = true;
	}

	if (on) {
		PR_DISP_INFO("%s: on\n", __func__);
		rc = regulator_set_optimum_mode(v_lcm, 100000);
		if (rc < 0) {
			PR_DISP_ERR("set_optimum_mode %s failed, rc=%d\n", lcm_str, rc);
			return -EINVAL;
		}

		hr_msleep(1);
		rc = regulator_enable(v_lcmio);
		if (rc) {
			PR_DISP_ERR("enable regulator %s failed, rc=%d\n", lcmio_str, rc);
			return -ENODEV;
		}

		rc = regulator_enable(v_lcm);
		if (rc) {
			PR_DISP_ERR("enable regulator %s failed, rc=%d\n", lcm_str, rc);
			return -ENODEV;
		}

		if (!mipi_lcd_on) {
			hr_msleep(10);
			gpio_set_value(GPIO_LCM_RST_N, 1);
			hr_msleep(1);
			gpio_set_value(GPIO_LCM_RST_N, 0);
			hr_msleep(35);
			gpio_set_value(GPIO_LCM_RST_N, 1);
		}
		hr_msleep(60);
		bPanelPowerOn = true;
	} else {
		PR_DISP_INFO("%s: off\n", __func__);
		if (!bPanelPowerOn) return 0;
		hr_msleep(100);
		gpio_set_value(GPIO_LCM_RST_N, 0);
		hr_msleep(10);

		if (regulator_disable(v_lcm)) {
			PR_DISP_ERR("%s: Unable to enable the regulator: %s\n", __func__, lcm_str);
			return -EINVAL;
		}
		hr_msleep(5);
		if (regulator_disable(v_lcmio)) {
			PR_DISP_ERR("%s: Unable to enable the regulator: %s\n", __func__, lcmio_str);
			return -EINVAL;
		}

		bPanelPowerOn = false;
	}
	return 0;
}

static int __init shooter_panel_init(void)
{
	if(panel_type != PANEL_ID_NONE)
		msm_fb_register_device("mipi_dsi", &mipi_dsi_pdata);
	else
		printk(KERN_INFO "[DISP]panel ID= NONE\n");

	return 0;
}

static int __init shooter_panel_late_init(void)
{
	led_trigger_register_simple("bkl_trigger", &bkl_led_trigger);
	pr_info("%s: SUCCESS (WLED TRIGGER)\n", __func__);
	wled_trigger_initialized = 1;

	mipi_dsi_buf_alloc(&panel_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&panel_rx_buf, DSI_BUF_SIZE);

	if (panel_type == PANEL_ID_SHR_SHARP_NT) {
		PR_DISP_INFO("%s: panel ID = PANEL_ID_PYD_SHARP\n", __func__);
		shooter_shooter_blue_qhd_pt_init();
	}

	return platform_driver_register(&this_driver);
}

module_init(shooter_panel_init);
late_initcall(shooter_panel_late_init);

struct mdp_reg shooter_color_v11[] = {
	{0x93400, 0x0222, 0x0},
	{0x93404, 0xFFE4, 0x0},
	{0x93408, 0xFFFD, 0x0},
	{0x9340C, 0xFFF1, 0x0},
	{0x93410, 0x0212, 0x0},
	{0x93414, 0xFFF9, 0x0},
	{0x93418, 0xFFF1, 0x0},
	{0x9341C, 0xFFE6, 0x0},
	{0x93420, 0x022D, 0x0},
	{0x93600, 0x0000, 0x0},
	{0x93604, 0x00FF, 0x0},
	{0x93608, 0x0000, 0x0},
	{0x9360C, 0x00FF, 0x0},
	{0x93610, 0x0000, 0x0},
	{0x93614, 0x00FF, 0x0},
	{0x93680, 0x0000, 0x0},
	{0x93684, 0x00FF, 0x0},
	{0x93688, 0x0000, 0x0},
	{0x9368C, 0x00FF, 0x0},
	{0x93690, 0x0000, 0x0},
	{0x93694, 0x00FF, 0x0},
	{0x90070, 0xCD298008, 0x0},
};

struct mdp_reg mdp_sharp_barrier_on[] = {
	{0x94800, 0x000000, 0x0},
	{0x94804, 0x020202, 0x0},
	{0x94808, 0x040404, 0x0},
	{0x9480C, 0x060606, 0x0},
	{0x94810, 0x080808, 0x0},
	{0x94814, 0x090909, 0x0},
	{0x94818, 0x0B0B0B, 0x0},
	{0x9481C, 0x0C0C0C, 0x0},
	{0x94820, 0x0E0E0E, 0x0},
	{0x94824, 0x0F0F0F, 0x0},
	{0x94828, 0x111111, 0x0},
	{0x9482C, 0x121212, 0x0},
	{0x94830, 0x141414, 0x0},
	{0x94834, 0x151515, 0x0},
	{0x94838, 0x161616, 0x0},
	{0x9483C, 0x181818, 0x0},
	{0x94840, 0x191919, 0x0},
	{0x94844, 0x1A1A1A, 0x0},
	{0x94848, 0x1C1C1C, 0x0},
	{0x9484C, 0x1D1D1D, 0x0},
	{0x94850, 0x1E1E1E, 0x0},
	{0x94854, 0x1F1F1F, 0x0},
	{0x94858, 0x212121, 0x0},
	{0x9485C, 0x222222, 0x0},
	{0x94860, 0x232323, 0x0},
	{0x94864, 0x242424, 0x0},
	{0x94868, 0x252525, 0x0},
	{0x9486C, 0x272727, 0x0},
	{0x94870, 0x282828, 0x0},
	{0x94874, 0x292929, 0x0},
	{0x94878, 0x2A2A2A, 0x0},
	{0x9487C, 0x2B2B2B, 0x0},
	{0x94880, 0x2D2D2D, 0x0},
	{0x94884, 0x2E2E2E, 0x0},
	{0x94888, 0x2F2F2F, 0x0},
	{0x9488C, 0x303030, 0x0},
	{0x94890, 0x313131, 0x0},
	{0x94894, 0x323232, 0x0},
	{0x94898, 0x343434, 0x0},
	{0x9489C, 0x353535, 0x0},
	{0x948A0, 0x363636, 0x0},
	{0x948A4, 0x373737, 0x0},
	{0x948A8, 0x383838, 0x0},
	{0x948AC, 0x393939, 0x0},
	{0x948B0, 0x3A3A3A, 0x0},
	{0x948B4, 0x3B3B3B, 0x0},
	{0x948B8, 0x3D3D3D, 0x0},
	{0x948BC, 0x3E3E3E, 0x0},
	{0x948C0, 0x3F3F3F, 0x0},
	{0x948C4, 0x404040, 0x0},
	{0x948C8, 0x414141, 0x0},
	{0x948CC, 0x424242, 0x0},
	{0x948D0, 0x434343, 0x0},
	{0x948D4, 0x444444, 0x0},
	{0x948D8, 0x454545, 0x0},
	{0x948DC, 0x464646, 0x0},
	{0x948E0, 0x474747, 0x0},
	{0x948E4, 0x484848, 0x0},
	{0x948E8, 0x4A4A4A, 0x0},
	{0x948EC, 0x4B4B4B, 0x0},
	{0x948F0, 0x4C4C4C, 0x0},
	{0x948F4, 0x4D4D4D, 0x0},
	{0x948F8, 0x4E4E4E, 0x0},
	{0x948FC, 0x4F4F4F, 0x0},
	{0x94900, 0x505050, 0x0},
	{0x94904, 0x515151, 0x0},
	{0x94908, 0x525252, 0x0},
	{0x9490C, 0x535353, 0x0},
	{0x94910, 0x545454, 0x0},
	{0x94914, 0x555555, 0x0},
	{0x94918, 0x565656, 0x0},
	{0x9491C, 0x575757, 0x0},
	{0x94920, 0x585858, 0x0},
	{0x94924, 0x595959, 0x0},
	{0x94928, 0x5A5A5A, 0x0},
	{0x9492C, 0x5B5B5B, 0x0},
	{0x94930, 0x5C5C5C, 0x0},
	{0x94934, 0x5D5D5D, 0x0},
	{0x94938, 0x5E5E5E, 0x0},
	{0x9493C, 0x5F5F5F, 0x0},
	{0x94940, 0x606060, 0x0},
	{0x94944, 0x616161, 0x0},
	{0x94948, 0x626262, 0x0},
	{0x9494C, 0x636363, 0x0},
	{0x94950, 0x646464, 0x0},
	{0x94954, 0x656565, 0x0},
	{0x94958, 0x666666, 0x0},
	{0x9495C, 0x676767, 0x0},
	{0x94960, 0x686868, 0x0},
	{0x94964, 0x696969, 0x0},
	{0x94968, 0x6A6A6A, 0x0},
	{0x9496C, 0x6B6B6B, 0x0},
	{0x94970, 0x6C6C6C, 0x0},
	{0x94974, 0x6D6D6D, 0x0},
	{0x94978, 0x6E6E6E, 0x0},
	{0x9497C, 0x6F6F6F, 0x0},
	{0x94980, 0x707070, 0x0},
	{0x94984, 0x717171, 0x0},
	{0x94988, 0x727272, 0x0},
	{0x9498C, 0x737373, 0x0},
	{0x94990, 0x747474, 0x0},
	{0x94994, 0x757575, 0x0},
	{0x94998, 0x767676, 0x0},
	{0x9499C, 0x777777, 0x0},
	{0x949A0, 0x787878, 0x0},
	{0x949A4, 0x797979, 0x0},
	{0x949A8, 0x7A7A7A, 0x0},
	{0x949AC, 0x7B7B7B, 0x0},
	{0x949B0, 0x7C7C7C, 0x0},
	{0x949B4, 0x7D7D7D, 0x0},
	{0x949B8, 0x7E7E7E, 0x0},
	{0x949BC, 0x7F7F7F, 0x0},
	{0x949C0, 0x808080, 0x0},
	{0x949C4, 0x818181, 0x0},
	{0x949C8, 0x828282, 0x0},
	{0x949CC, 0x838383, 0x0},
	{0x949D0, 0x848484, 0x0},
	{0x949D4, 0x858585, 0x0},
	{0x949D8, 0x858585, 0x0},
	{0x949DC, 0x868686, 0x0},
	{0x949E0, 0x878787, 0x0},
	{0x949E4, 0x888888, 0x0},
	{0x949E8, 0x898989, 0x0},
	{0x949EC, 0x8A8A8A, 0x0},
	{0x949F0, 0x8B8B8B, 0x0},
	{0x949F4, 0x8C8C8C, 0x0},
	{0x949F8, 0x8D8D8D, 0x0},
	{0x949FC, 0x8E8E8E, 0x0},
	{0x94A00, 0x8F8F8F, 0x0},
	{0x94A04, 0x909090, 0x0},
	{0x94A08, 0x919191, 0x0},
	{0x94A0C, 0x929292, 0x0},
	{0x94A10, 0x939393, 0x0},
	{0x94A14, 0x949494, 0x0},
	{0x94A18, 0x959595, 0x0},
	{0x94A1C, 0x959595, 0x0},
	{0x94A20, 0x969696, 0x0},
	{0x94A24, 0x979797, 0x0},
	{0x94A28, 0x989898, 0x0},
	{0x94A2C, 0x999999, 0x0},
	{0x94A30, 0x9A9A9A, 0x0},
	{0x94A34, 0x9B9B9B, 0x0},
	{0x94A38, 0x9C9C9C, 0x0},
	{0x94A3C, 0x9D9D9D, 0x0},
	{0x94A40, 0x9E9E9E, 0x0},
	{0x94A44, 0x9F9F9F, 0x0},
	{0x94A48, 0xA0A0A0, 0x0},
	{0x94A4C, 0xA1A1A1, 0x0},
	{0x94A50, 0xA1A1A1, 0x0},
	{0x94A54, 0xA2A2A2, 0x0},
	{0x94A58, 0xA3A3A3, 0x0},
	{0x94A5C, 0xA4A4A4, 0x0},
	{0x94A60, 0xA5A5A5, 0x0},
	{0x94A64, 0xA6A6A6, 0x0},
	{0x94A68, 0xA7A7A7, 0x0},
	{0x94A6C, 0xA8A8A8, 0x0},
	{0x94A70, 0xA9A9A9, 0x0},
	{0x94A74, 0xAAAAAA, 0x0},
	{0x94A78, 0xABABAB, 0x0},
	{0x94A7C, 0xABABAB, 0x0},
	{0x94A80, 0xACACAC, 0x0},
	{0x94A84, 0xADADAD, 0x0},
	{0x94A88, 0xAEAEAE, 0x0},
	{0x94A8C, 0xAFAFAF, 0x0},
	{0x94A90, 0xB0B0B0, 0x0},
	{0x94A94, 0xB1B1B1, 0x0},
	{0x94A98, 0xB2B2B2, 0x0},
	{0x94A9C, 0xB3B3B3, 0x0},
	{0x94AA0, 0xB4B4B4, 0x0},
	{0x94AA4, 0xB4B4B4, 0x0},
	{0x94AA8, 0xB5B5B5, 0x0},
	{0x94AAC, 0xB6B6B6, 0x0},
	{0x94AB0, 0xB7B7B7, 0x0},
	{0x94AB4, 0xB8B8B8, 0x0},
	{0x94AB8, 0xB9B9B9, 0x0},
	{0x94ABC, 0xBABABA, 0x0},
	{0x94AC0, 0xBBBBBB, 0x0},
	{0x94AC4, 0xBCBCBC, 0x0},
	{0x94AC8, 0xBDBDBD, 0x0},
	{0x94ACC, 0xBDBDBD, 0x0},
	{0x94AD0, 0xBEBEBE, 0x0},
	{0x94AD4, 0xBFBFBF, 0x0},
	{0x94AD8, 0xC0C0C0, 0x0},
	{0x94ADC, 0xC1C1C1, 0x0},
	{0x94AE0, 0xC2C2C2, 0x0},
	{0x94AE4, 0xC3C3C3, 0x0},
	{0x94AE8, 0xC4C4C4, 0x0},
	{0x94AEC, 0xC5C5C5, 0x0},
	{0x94AF0, 0xC5C5C5, 0x0},
	{0x94AF4, 0xC6C6C6, 0x0},
	{0x94AF8, 0xC7C7C7, 0x0},
	{0x94AFC, 0xC8C8C8, 0x0},
	{0x94B00, 0xC9C9C9, 0x0},
	{0x94B04, 0xCACACA, 0x0},
	{0x94B08, 0xCBCBCB, 0x0},
	{0x94B0C, 0xCCCCCC, 0x0},
	{0x94B10, 0xCCCCCC, 0x0},
	{0x94B14, 0xCDCDCD, 0x0},
	{0x94B18, 0xCECECE, 0x0},
	{0x94B1C, 0xCFCFCF, 0x0},
	{0x94B20, 0xD0D0D0, 0x0},
	{0x94B24, 0xD1D1D1, 0x0},
	{0x94B28, 0xD2D2D2, 0x0},
	{0x94B2C, 0xD3D3D3, 0x0},
	{0x94B30, 0xD3D3D3, 0x0},
	{0x94B34, 0xD4D4D4, 0x0},
	{0x94B38, 0xD5D5D5, 0x0},
	{0x94B3C, 0xD6D6D6, 0x0},
	{0x94B40, 0xD7D7D7, 0x0},
	{0x94B44, 0xD8D8D8, 0x0},
	{0x94B48, 0xD9D9D9, 0x0},
	{0x94B4C, 0xD9D9D9, 0x0},
	{0x94B50, 0xDADADA, 0x0},
	{0x94B54, 0xDBDBDB, 0x0},
	{0x94B58, 0xDCDCDC, 0x0},
	{0x94B5C, 0xDDDDDD, 0x0},
	{0x94B60, 0xDEDEDE, 0x0},
	{0x94B64, 0xDFDFDF, 0x0},
	{0x94B68, 0xE0E0E0, 0x0},
	{0x94B6C, 0xE0E0E0, 0x0},
	{0x94B70, 0xE1E1E1, 0x0},
	{0x94B74, 0xE2E2E2, 0x0},
	{0x94B78, 0xE3E3E3, 0x0},
	{0x94B7C, 0xE4E4E4, 0x0},
	{0x94B80, 0xE5E5E5, 0x0},
	{0x94B84, 0xE6E6E6, 0x0},
	{0x94B88, 0xE6E6E6, 0x0},
	{0x94B8C, 0xE7E7E7, 0x0},
	{0x94B90, 0xE8E8E8, 0x0},
	{0x94B94, 0xE9E9E9, 0x0},
	{0x94B98, 0xEAEAEA, 0x0},
	{0x94B9C, 0xEBEBEB, 0x0},
	{0x94BA0, 0xECECEC, 0x0},
	{0x94BA4, 0xECECEC, 0x0},
	{0x94BA8, 0xEDEDED, 0x0},
	{0x94BAC, 0xEEEEEE, 0x0},
	{0x94BB0, 0xEFEFEF, 0x0},
	{0x94BB4, 0xF0F0F0, 0x0},
	{0x94BB8, 0xF1F1F1, 0x0},
	{0x94BBC, 0xF1F1F1, 0x0},
	{0x94BC0, 0xF2F2F2, 0x0},
	{0x94BC4, 0xF3F3F3, 0x0},
	{0x94BC8, 0xF4F4F4, 0x0},
	{0x94BCC, 0xF5F5F5, 0x0},
	{0x94BD0, 0xF6F6F6, 0x0},
	{0x94BD4, 0xF7F7F7, 0x0},
	{0x94BD8, 0xF7F7F7, 0x0},
	{0x94BDC, 0xF8F8F8, 0x0},
	{0x94BE0, 0xF9F9F9, 0x0},
	{0x94BE4, 0xFAFAFA, 0x0},
	{0x94BE8, 0xFBFBFB, 0x0},
	{0x94BEC, 0xFCFCFC, 0x0},
	{0x94BF0, 0xFCFCFC, 0x0},
	{0x94BF4, 0xFDFDFD, 0x0},
	{0x94BF8, 0xFEFEFE, 0x0},
	{0x94BFC, 0xFFFFFF, 0x0},
	{0x90070, 0x17    , 0x17},
};

struct mdp_reg mdp_sharp_barrier_off[] = {
	{0x94800, 0x000000, 0x0},
	{0x94804, 0x010101, 0x0},
	{0x94808, 0x020202, 0x0},
	{0x9480C, 0x030303, 0x0},
	{0x94810, 0x040404, 0x0},
	{0x94814, 0x050505, 0x0},
	{0x94818, 0x060606, 0x0},
	{0x9481C, 0x070707, 0x0},
	{0x94820, 0x080808, 0x0},
	{0x94824, 0x090909, 0x0},
	{0x94828, 0x0A0A0A, 0x0},
	{0x9482C, 0x0B0B0B, 0x0},
	{0x94830, 0x0C0C0C, 0x0},
	{0x94834, 0x0D0D0D, 0x0},
	{0x94838, 0x0E0E0E, 0x0},
	{0x9483C, 0x0F0F0F, 0x0},
	{0x94840, 0x101010, 0x0},
	{0x94844, 0x111111, 0x0},
	{0x94848, 0x121212, 0x0},
	{0x9484C, 0x131313, 0x0},
	{0x94850, 0x141414, 0x0},
	{0x94854, 0x151515, 0x0},
	{0x94858, 0x161616, 0x0},
	{0x9485C, 0x171717, 0x0},
	{0x94860, 0x181818, 0x0},
	{0x94864, 0x191919, 0x0},
	{0x94868, 0x1A1A1A, 0x0},
	{0x9486C, 0x1B1B1B, 0x0},
	{0x94870, 0x1C1C1C, 0x0},
	{0x94874, 0x1D1D1D, 0x0},
	{0x94878, 0x1E1E1E, 0x0},
	{0x9487C, 0x1F1F1F, 0x0},
	{0x94880, 0x202020, 0x0},
	{0x94884, 0x212121, 0x0},
	{0x94888, 0x222222, 0x0},
	{0x9488C, 0x232323, 0x0},
	{0x94890, 0x242424, 0x0},
	{0x94894, 0x252525, 0x0},
	{0x94898, 0x262626, 0x0},
	{0x9489C, 0x272727, 0x0},
	{0x948A0, 0x282828, 0x0},
	{0x948A4, 0x292929, 0x0},
	{0x948A8, 0x2A2A2A, 0x0},
	{0x948AC, 0x2B2B2B, 0x0},
	{0x948B0, 0x2C2C2C, 0x0},
	{0x948B4, 0x2D2D2D, 0x0},
	{0x948B8, 0x2E2E2E, 0x0},
	{0x948BC, 0x2F2F2F, 0x0},
	{0x948C0, 0x303030, 0x0},
	{0x948C4, 0x313131, 0x0},
	{0x948C8, 0x323232, 0x0},
	{0x948CC, 0x333333, 0x0},
	{0x948D0, 0x343434, 0x0},
	{0x948D4, 0x353535, 0x0},
	{0x948D8, 0x363636, 0x0},
	{0x948DC, 0x373737, 0x0},
	{0x948E0, 0x383838, 0x0},
	{0x948E4, 0x393939, 0x0},
	{0x948E8, 0x3A3A3A, 0x0},
	{0x948EC, 0x3B3B3B, 0x0},
	{0x948F0, 0x3C3C3C, 0x0},
	{0x948F4, 0x3D3D3D, 0x0},
	{0x948F8, 0x3E3E3E, 0x0},
	{0x948FC, 0x3F3F3F, 0x0},
	{0x94900, 0x404040, 0x0},
	{0x94904, 0x414141, 0x0},
	{0x94908, 0x424242, 0x0},
	{0x9490C, 0x434343, 0x0},
	{0x94910, 0x444444, 0x0},
	{0x94914, 0x454545, 0x0},
	{0x94918, 0x464646, 0x0},
	{0x9491C, 0x474747, 0x0},
	{0x94920, 0x484848, 0x0},
	{0x94924, 0x494949, 0x0},
	{0x94928, 0x4A4A4A, 0x0},
	{0x9492C, 0x4B4B4B, 0x0},
	{0x94930, 0x4C4C4C, 0x0},
	{0x94934, 0x4D4D4D, 0x0},
	{0x94938, 0x4E4E4E, 0x0},
	{0x9493C, 0x4F4F4F, 0x0},
	{0x94940, 0x505050, 0x0},
	{0x94944, 0x515151, 0x0},
	{0x94948, 0x525252, 0x0},
	{0x9494C, 0x535353, 0x0},
	{0x94950, 0x545454, 0x0},
	{0x94954, 0x555555, 0x0},
	{0x94958, 0x565656, 0x0},
	{0x9495C, 0x575757, 0x0},
	{0x94960, 0x585858, 0x0},
	{0x94964, 0x595959, 0x0},
	{0x94968, 0x5A5A5A, 0x0},
	{0x9496C, 0x5B5B5B, 0x0},
	{0x94970, 0x5C5C5C, 0x0},
	{0x94974, 0x5D5D5D, 0x0},
	{0x94978, 0x5E5E5E, 0x0},
	{0x9497C, 0x5F5F5F, 0x0},
	{0x94980, 0x606060, 0x0},
	{0x94984, 0x616161, 0x0},
	{0x94988, 0x626262, 0x0},
	{0x9498C, 0x636363, 0x0},
	{0x94990, 0x646464, 0x0},
	{0x94994, 0x656565, 0x0},
	{0x94998, 0x666666, 0x0},
	{0x9499C, 0x676767, 0x0},
	{0x949A0, 0x686868, 0x0},
	{0x949A4, 0x696969, 0x0},
	{0x949A8, 0x6A6A6A, 0x0},
	{0x949AC, 0x6B6B6B, 0x0},
	{0x949B0, 0x6C6C6C, 0x0},
	{0x949B4, 0x6D6D6D, 0x0},
	{0x949B8, 0x6E6E6E, 0x0},
	{0x949BC, 0x6F6F6F, 0x0},
	{0x949C0, 0x707070, 0x0},
	{0x949C4, 0x717171, 0x0},
	{0x949C8, 0x727272, 0x0},
	{0x949CC, 0x737373, 0x0},
	{0x949D0, 0x747474, 0x0},
	{0x949D4, 0x757575, 0x0},
	{0x949D8, 0x767676, 0x0},
	{0x949DC, 0x777777, 0x0},
	{0x949E0, 0x787878, 0x0},
	{0x949E4, 0x797979, 0x0},
	{0x949E8, 0x7A7A7A, 0x0},
	{0x949EC, 0x7B7B7B, 0x0},
	{0x949F0, 0x7C7C7C, 0x0},
	{0x949F4, 0x7D7D7D, 0x0},
	{0x949F8, 0x7E7E7E, 0x0},
	{0x949FC, 0x7F7F7F, 0x0},
	{0x94A00, 0x808080, 0x0},
	{0x94A04, 0x818181, 0x0},
	{0x94A08, 0x828282, 0x0},
	{0x94A0C, 0x838383, 0x0},
	{0x94A10, 0x848484, 0x0},
	{0x94A14, 0x858585, 0x0},
	{0x94A18, 0x868686, 0x0},
	{0x94A1C, 0x878787, 0x0},
	{0x94A20, 0x888888, 0x0},
	{0x94A24, 0x898989, 0x0},
	{0x94A28, 0x8A8A8A, 0x0},
	{0x94A2C, 0x8B8B8B, 0x0},
	{0x94A30, 0x8C8C8C, 0x0},
	{0x94A34, 0x8D8D8D, 0x0},
	{0x94A38, 0x8E8E8E, 0x0},
	{0x94A3C, 0x8F8F8F, 0x0},
	{0x94A40, 0x909090, 0x0},
	{0x94A44, 0x919191, 0x0},
	{0x94A48, 0x929292, 0x0},
	{0x94A4C, 0x939393, 0x0},
	{0x94A50, 0x949494, 0x0},
	{0x94A54, 0x959595, 0x0},
	{0x94A58, 0x969696, 0x0},
	{0x94A5C, 0x979797, 0x0},
	{0x94A60, 0x989898, 0x0},
	{0x94A64, 0x999999, 0x0},
	{0x94A68, 0x9A9A9A, 0x0},
	{0x94A6C, 0x9B9B9B, 0x0},
	{0x94A70, 0x9C9C9C, 0x0},
	{0x94A74, 0x9D9D9D, 0x0},
	{0x94A78, 0x9E9E9E, 0x0},
	{0x94A7C, 0x9F9F9F, 0x0},
	{0x94A80, 0xA0A0A0, 0x0},
	{0x94A84, 0xA1A1A1, 0x0},
	{0x94A88, 0xA2A2A2, 0x0},
	{0x94A8C, 0xA3A3A3, 0x0},
	{0x94A90, 0xA4A4A4, 0x0},
	{0x94A94, 0xA5A5A5, 0x0},
	{0x94A98, 0xA6A6A6, 0x0},
	{0x94A9C, 0xA7A7A7, 0x0},
	{0x94AA0, 0xA8A8A8, 0x0},
	{0x94AA4, 0xA9A9A9, 0x0},
	{0x94AA8, 0xAAAAAA, 0x0},
	{0x94AAC, 0xABABAB, 0x0},
	{0x94AB0, 0xACACAC, 0x0},
	{0x94AB4, 0xADADAD, 0x0},
	{0x94AB8, 0xAEAEAE, 0x0},
	{0x94ABC, 0xAFAFAF, 0x0},
	{0x94AC0, 0xB0B0B0, 0x0},
	{0x94AC4, 0xB1B1B1, 0x0},
	{0x94AC8, 0xB2B2B2, 0x0},
	{0x94ACC, 0xB3B3B3, 0x0},
	{0x94AD0, 0xB4B4B4, 0x0},
	{0x94AD4, 0xB5B5B5, 0x0},
	{0x94AD8, 0xB6B6B6, 0x0},
	{0x94ADC, 0xB7B7B7, 0x0},
	{0x94AE0, 0xB8B8B8, 0x0},
	{0x94AE4, 0xB9B9B9, 0x0},
	{0x94AE8, 0xBABABA, 0x0},
	{0x94AEC, 0xBBBBBB, 0x0},
	{0x94AF0, 0xBCBCBC, 0x0},
	{0x94AF4, 0xBDBDBD, 0x0},
	{0x94AF8, 0xBEBEBE, 0x0},
	{0x94AFC, 0xBFBFBF, 0x0},
	{0x94B00, 0xC0C0C0, 0x0},
	{0x94B04, 0xC1C1C1, 0x0},
	{0x94B08, 0xC2C2C2, 0x0},
	{0x94B0C, 0xC3C3C3, 0x0},
	{0x94B10, 0xC4C4C4, 0x0},
	{0x94B14, 0xC5C5C5, 0x0},
	{0x94B18, 0xC6C6C6, 0x0},
	{0x94B1C, 0xC7C7C7, 0x0},
	{0x94B20, 0xC8C8C8, 0x0},
	{0x94B24, 0xC9C9C9, 0x0},
	{0x94B28, 0xCACACA, 0x0},
	{0x94B2C, 0xCBCBCB, 0x0},
	{0x94B30, 0xCCCCCC, 0x0},
	{0x94B34, 0xCDCDCD, 0x0},
	{0x94B38, 0xCECECE, 0x0},
	{0x94B3C, 0xCFCFCF, 0x0},
	{0x94B40, 0xD0D0D0, 0x0},
	{0x94B44, 0xD1D1D1, 0x0},
	{0x94B48, 0xD2D2D2, 0x0},
	{0x94B4C, 0xD3D3D3, 0x0},
	{0x94B50, 0xD4D4D4, 0x0},
	{0x94B54, 0xD5D5D5, 0x0},
	{0x94B58, 0xD6D6D6, 0x0},
	{0x94B5C, 0xD7D7D7, 0x0},
	{0x94B60, 0xD8D8D8, 0x0},
	{0x94B64, 0xD9D9D9, 0x0},
	{0x94B68, 0xDADADA, 0x0},
	{0x94B6C, 0xDBDBDB, 0x0},
	{0x94B70, 0xDCDCDC, 0x0},
	{0x94B74, 0xDDDDDD, 0x0},
	{0x94B78, 0xDEDEDE, 0x0},
	{0x94B7C, 0xDFDFDF, 0x0},
	{0x94B80, 0xE0E0E0, 0x0},
	{0x94B84, 0xE1E1E1, 0x0},
	{0x94B88, 0xE2E2E2, 0x0},
	{0x94B8C, 0xE3E3E3, 0x0},
	{0x94B90, 0xE4E4E4, 0x0},
	{0x94B94, 0xE5E5E5, 0x0},
	{0x94B98, 0xE6E6E6, 0x0},
	{0x94B9C, 0xE7E7E7, 0x0},
	{0x94BA0, 0xE8E8E8, 0x0},
	{0x94BA4, 0xE9E9E9, 0x0},
	{0x94BA8, 0xEAEAEA, 0x0},
	{0x94BAC, 0xEBEBEB, 0x0},
	{0x94BB0, 0xECECEC, 0x0},
	{0x94BB4, 0xEDEDED, 0x0},
	{0x94BB8, 0xEEEEEE, 0x0},
	{0x94BBC, 0xEFEFEF, 0x0},
	{0x94BC0, 0xF0F0F0, 0x0},
	{0x94BC4, 0xF1F1F1, 0x0},
	{0x94BC8, 0xF2F2F2, 0x0},
	{0x94BCC, 0xF3F3F3, 0x0},
	{0x94BD0, 0xF4F4F4, 0x0},
	{0x94BD4, 0xF5F5F5, 0x0},
	{0x94BD8, 0xF6F6F6, 0x0},
	{0x94BDC, 0xF7F7F7, 0x0},
	{0x94BE0, 0xF8F8F8, 0x0},
	{0x94BE4, 0xF9F9F9, 0x0},
	{0x94BE8, 0xFAFAFA, 0x0},
	{0x94BEC, 0xFBFBFB, 0x0},
	{0x94BF0, 0xFCFCFC, 0x0},
	{0x94BF4, 0xFDFDFD, 0x0},
	{0x94BF8, 0xFEFEFE, 0x0},
	{0x94BFC, 0xFFFFFF, 0x0},
	{0x90070, 0x17	  , 0x17},
};

int shooter_mdp_gamma(void)
{
	PR_DISP_INFO("%s\n", __func__);
	mdp4_dsi_color_enhancement(mdp_sharp_barrier_off, ARRAY_SIZE(mdp_sharp_barrier_off));

	return 0;
}

/* 3D Panel */
static void shooter_3Dpanel_on(bool bLandscape)
{
	struct pm8058_pwm_period pwm_conf;
	int rc;

	led_brightness_value_set("lcd-backlight", 255);

	if (system_rev >= 1) {
		pwm_gpio_config.output_value = 1;
		rc = pm8xxx_gpio_config(PM8058_GPIO_PM_TO_SYS(SHOOTER_3DLCM_PD), &pwm_gpio_config);
		if (rc < 0)
			pr_err("%s pmic gpio config gpio %d failed\n", __func__, PM8058_GPIO_PM_TO_SYS(SHOOTER_3DLCM_PD));
	}

	rc = pm8xxx_gpio_config(PM8058_GPIO_PM_TO_SYS(SHOOTER_3DCLK), &clk_gpio_config_on);
	if (rc < 0)
		pr_err("%s pmic gpio config gpio %d failed\n", __func__, SHOOTER_3DCLK);

	pwm_disable(pwm_3d);
	pwm_conf.pwm_size = 9;
	pwm_conf.clk = PM_PWM_CLK_19P2MHZ;
	pwm_conf.pre_div = PM_PWM_PDIV_3;
	pwm_conf.pre_div_exp = 6;
	rc = pwm_configure(pwm_3d, &pwm_conf, 1, 255);
	if (rc < 0)
		pr_err("%s pmic configure %d\n", __func__, rc);
	pwm_enable(pwm_3d);

	if (bLandscape) {
		mdp4_dsi_color_enhancement(mdp_sharp_barrier_on, ARRAY_SIZE(mdp_sharp_barrier_on));
		gpio_set_value(SHOOTER_CTL_3D_1, 1);
		gpio_set_value(SHOOTER_CTL_3D_2, 1);
		gpio_set_value(SHOOTER_CTL_3D_3, 1);
		gpio_set_value(SHOOTER_CTL_3D_4, 0);
	} else {
		mdp4_dsi_color_enhancement(mdp_sharp_barrier_on, ARRAY_SIZE(mdp_sharp_barrier_on));
		gpio_set_value(SHOOTER_CTL_3D_1, 1);
		gpio_set_value(SHOOTER_CTL_3D_2, 1);
		gpio_set_value(SHOOTER_CTL_3D_3, 0);
		gpio_set_value(SHOOTER_CTL_3D_4, 1);
	}
}

static void shooter_3Dpanel_off(void)
{
	int rc;
	if (system_rev >= 1) {
		pwm_gpio_config.output_value = 0;
		rc = pm8xxx_gpio_config(PM8058_GPIO_PM_TO_SYS(SHOOTER_3DLCM_PD), &pwm_gpio_config);
		if (rc < 0)
			pr_err("%s pmic gpio config gpio %d failed\n", __func__, SHOOTER_3DLCM_PD);
	}
	mdp4_dsi_color_enhancement(mdp_sharp_barrier_off, ARRAY_SIZE(mdp_sharp_barrier_off));
	pwm_disable(pwm_3d);

	rc = pm8xxx_gpio_config(PM8058_GPIO_PM_TO_SYS(SHOOTER_3DCLK), &clk_gpio_config_off);
	if (rc < 0)
		pr_err("%s pmic gpio config gpio %d failed\n", __func__, SHOOTER_3DCLK);
	gpio_set_value(SHOOTER_CTL_3D_1, 0);
	gpio_set_value(SHOOTER_CTL_3D_2, 0);
	gpio_set_value(SHOOTER_CTL_3D_3, 0);
	gpio_set_value(SHOOTER_CTL_3D_4, 0);
	led_brightness_value_set("lcd-backlight", last_br_2d);
}

static ssize_t show_3D_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	len += scnprintf(buf+len, PAGE_SIZE-1, "%d\n", atomic_read(&g_3D_mode));
	return len;
}

static ssize_t store_3D_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	enum MODE_3D val = buf[0] - '0';

	if (val < 0 || val >= BARRIER_END) {
		pr_err("%s: unsupport value %c\n", __func__, val);
		return -EINVAL;
	}

	if (val == atomic_read(&g_3D_mode)) {
		printk(KERN_NOTICE "%s: status is same(%d)\n", __func__, val);
		return count;
	}

	atomic_set(&g_3D_mode, val);
	PR_DISP_INFO("%s mode = %d\n", __func__, val);
	switch (val) {
	case BARRIER_OFF:
		shooter_3Dpanel_off();
		break;
	case BARRIER_LANDSCAPE:
		shooter_3Dpanel_on(true);
		break;
	case BARRIER_PORTRAIT:
		shooter_3Dpanel_on(false);
		break;
	default:
		break;
	}

	return count;
}

static DEVICE_ATTR(3D_mode, 0666, show_3D_mode, store_3D_mode);

static int shooter_3Dpanel_probe(struct platform_device *pdev)
{
	int err = 0;
	printk(KERN_INFO "%s\n", __func__);

	if (pwm_3d == NULL)
		pwm_3d = pwm_request(2, "3DCLK");
	err = device_create_file(&pdev->dev, &dev_attr_3D_mode);
	if (err != 0)
		printk(KERN_WARNING "attr_3D_mode failed\n");

	return 0;
}

static int shooter_3Dpanel_remove(struct platform_device *pdev)
{
	printk(KERN_INFO "%s\n", __func__);
	if (pwm_3d) {
		pwm_free(pwm_3d);
		pwm_3d = NULL;
	}
	device_remove_file(&pdev->dev, &dev_attr_3D_mode);
	kobject_del(uevent_kobj);
	kobject_del(kobj);
	return 0;
}

struct platform_driver shooter_3Dpanel_driver = {
	.probe	= shooter_3Dpanel_probe,
	.remove	= shooter_3Dpanel_remove,
	.driver	= {
		.name = "panel_3d",
	},
};

static int __init shooter_3Dpanel_init(void)
{
	pr_info("%s(%d)\n", __func__, __LINE__);
	return platform_driver_register(&shooter_3Dpanel_driver);
}

static void __exit shooter_3Dpanel_exit(void)
{
	platform_driver_unregister(&shooter_3Dpanel_driver);
}

module_init(shooter_3Dpanel_init);
module_exit(shooter_3Dpanel_exit);
