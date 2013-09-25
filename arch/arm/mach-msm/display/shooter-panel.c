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
static void shooter_set_backlight(struct msm_fb_data_type *mfd);
static int mipi_shooter_device_register(const char* dev_name, struct msm_panel_info *pinfo, u32 channel, u32 panel);
static int wled_trigger_initialized;

static struct dsi_buf panel_tx_buf;
static struct dsi_buf panel_rx_buf;
static struct msm_panel_info pinfo;
static int cur_bl_level = 0;
static int mipi_lcd_on = 1;
struct dcs_cmd_req cmdreq_shooter;
static u32 manu_id;

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

static struct msm_fb_panel_data shooter_panel_data;

static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.vsync_gpio 	= GPIO_LCD_TE,
	.dsi_power_save = mipi_dsi_panel_power,
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
	cmdreq_shooter.cmds = &shooter_manufacture_id_cmd;
	cmdreq_shooter.cmds_cnt = 1;
	cmdreq_shooter.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq_shooter.rlen = 3;
	cmdreq_shooter.cb = mipi_shooter_manufature_cb;
	mipi_dsi_cmdlist_put(&cmdreq_shooter);

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
/* DSI_BIT_CLK at 482MHz, 2 lane, RGB888 */
	{0x03, 0x01, 0x01, 0x00},	/* regulator */
	/* timing   */
	{0x96, 0x1E, 0x1E, 0x00, 0x3C, 0x3C, 0x1E, 0x28,
	0x0b, 0x13, 0x04},
	{0x7f, 0x00, 0x00, 0x00},	/* phy ctrl */
	{0xee, 0x02, 0x86, 0x00},	/* strength */
	/* pll control */
	{0x41, 0x9c, 0xb9, 0xd6, 0x00, 0x50, 0x48, 0x63,
	0x01, 0x0f, 0x07,
	0x05, 0x14, 0x03, 0x03, 0x03, 0x54, 0x06, 0x10, 0x04, 0x03 },
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
	pinfo.lcdc.h_back_porch = 64;
	pinfo.lcdc.h_front_porch = 96;
	pinfo.lcdc.h_pulse_width = 32;
	pinfo.lcdc.v_back_porch = 16;
	pinfo.lcdc.v_front_porch = 16;
	pinfo.lcdc.v_pulse_width = 4;
	pinfo.lcdc.border_clr = 0;	
	pinfo.lcdc.underflow_clr = 0xff;	
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = 255;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;
	pinfo.clk_rate = 482000000;
	pinfo.is_3d_panel = FB_TYPE_3D_PANEL;
	pinfo.lcd.vsync_enable = TRUE;
	pinfo.lcd.hw_vsync_mode = TRUE;
	pinfo.lcd.refx100 = 6096; 
	pinfo.lcd.v_back_porch = 11;
	pinfo.lcd.v_front_porch = 10;
	pinfo.lcd.v_pulse_width = 5;

	pinfo.mipi.mode = DSI_CMD_MODE;
	pinfo.mipi.dst_format = DSI_CMD_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.esc_byte_ratio = 4;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.t_clk_post = 0x0a;
	pinfo.mipi.t_clk_pre = 0x1e;
	pinfo.mipi.stream = 0;	
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.te_sel = 1; 
	pinfo.mipi.interleave_max = 1;
	pinfo.mipi.insert_dcs_cmd = TRUE;
	pinfo.mipi.wr_mem_continue = 0x3c;
	pinfo.mipi.wr_mem_start = 0x2c;
	pinfo.mipi.dsi_phy_db = &dsi_cmd_mode_phy_db;
	
	ret = mipi_shooter_device_register("mipi_video_novatek_qhd", &pinfo, MIPI_DSI_PRIM,
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

	shooter_shrink_pwm(mfd->bl_level);
	cmdreq_shooter.cmds = (struct dsi_cmd_desc*)&backlight_cmd;
	cmdreq_shooter.cmds_cnt = 1;
	cmdreq_shooter.flags = CMD_REQ_COMMIT;
	cmdreq_shooter.rlen = 0;
	cmdreq_shooter.cb = NULL;

	return;
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
	if(panel_type != PANEL_ID_NONE) {
		msm_fb_register_device("mipi_dsi", &mipi_dsi_pdata);
	} else
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
		PR_DISP_INFO("%s: panel ID = PANEL_ID_SHR_SHARP_NT\n", __func__);
		shooter_shooter_blue_qhd_pt_init();
	}

	return platform_driver_register(&this_driver);
}

module_init(shooter_panel_init);
late_initcall(shooter_panel_late_init);
int shooter_mdp_gamma(void)
{
	PR_DISP_INFO("%s\n", __func__);
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
		gpio_set_value(SHOOTER_CTL_3D_1, 1);
		gpio_set_value(SHOOTER_CTL_3D_2, 1);
		gpio_set_value(SHOOTER_CTL_3D_3, 1);
		gpio_set_value(SHOOTER_CTL_3D_4, 0);
	} else {
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
