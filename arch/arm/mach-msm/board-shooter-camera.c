/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
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

#include <asm/mach-types.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <mach/board.h>
#include <mach/msm_bus_board.h>
#include <mach/gpiomux.h>
#include <asm/setup.h>
#include "devices-msm8x60.h"
#include "devices.h"
#include "board-shooter.h"

#include <linux/spi/spi.h>
#include <mach/rpm-regulator.h>

#ifdef CONFIG_MSM_CAMERA_FLASH
#include <linux/htc_flashlight.h>
#include <linux/leds.h>
#endif

extern unsigned int engineerid;
static int camera_sensor_power_enable(char *power, unsigned volt, struct regulator **sensor_power);
static int camera_sensor_power_disable(struct regulator *sensor_power);
static struct platform_device msm_camera_server = {
	.name = "msm_cam_server",
	.id = 0,
};

#ifdef CONFIG_MSM_CAMERA
static struct msm_bus_vectors cam_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_zsl_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 566231040,
		.ib  = 905969664,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 706199040,
		.ib  = 1129918464,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 320864256,
		.ib  = 513382810,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 320864256,
		.ib  = 513382810,
	},
};

static struct msm_bus_vectors cam_stereo_video_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 212336640,
		.ib  = 339738624,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 25090560,
		.ib  = 40144896,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 239708160,
		.ib  = 383533056,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 79902720,
		.ib  = 127844352,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_stereo_snapshot_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 300902400,
		.ib  = 481443840,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 230307840,
		.ib  = 368492544,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 245113344,
		.ib  = 392181351,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 106536960,
		.ib  = 170459136,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 106536960,
		.ib  = 170459136,
	},
};

static struct msm_bus_paths cam_bus_client_config[] = {
	{
		ARRAY_SIZE(cam_init_vectors),
		cam_init_vectors,
	},
	{
		ARRAY_SIZE(cam_zsl_vectors),
		cam_zsl_vectors,
	},
	{
		ARRAY_SIZE(cam_zsl_vectors),
		cam_zsl_vectors,
	},
	{
		ARRAY_SIZE(cam_zsl_vectors),
		cam_zsl_vectors,
	},
	{
		ARRAY_SIZE(cam_zsl_vectors),
		cam_zsl_vectors,
	},
	{
		ARRAY_SIZE(cam_stereo_video_vectors),
		cam_stereo_video_vectors,
	},
	{
		ARRAY_SIZE(cam_stereo_snapshot_vectors),
		cam_stereo_snapshot_vectors,
	},
};

static struct msm_bus_scale_pdata cam_bus_client_pdata = {
		cam_bus_client_config,
		ARRAY_SIZE(cam_bus_client_config),
		.name = "msm_camera",
};

struct msm_camera_device_platform_data msm_camera_csi_device_data[] = {
	{
		.ioext.csiphy = 0x04800000,
		.ioext.csisz  = 0x00000400,
		.ioext.csiirq = CSI_0_IRQ,
		.ioclk.mclk_clk_rate = 24000000,
		.ioclk.vfe_clk_rate  = 228570000,
		.csid_core = 0,
		.is_csic = 1,
		.is_vpe = 1,
		.cam_bus_scale_table = &cam_bus_client_pdata,
	},
	{
		.ioext.csiphy = 0x04900000,
		.ioext.csisz  = 0x00000400,
		.ioext.csiirq = CSI_1_IRQ,
		.ioclk.mclk_clk_rate = 24000000,
		.ioclk.vfe_clk_rate  = 228570000,
		.csid_core = 1,
		.is_csic = 1,
		.is_vpe = 1,
		.cam_bus_scale_table = &cam_bus_client_pdata,
	},
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("[CAM] %s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static uint32_t camera_off_gpio_table_sp3d[] = {
	GPIO_CFG(SHOOTER_CAM_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),/*i2c*/
	GPIO_CFG(SHOOTER_CAM_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),/*i2c*/
	GPIO_CFG(SHOOTER_SP3D_MCLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),/*MCLK*/
	GPIO_CFG(106, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/*sharp INT*/
	GPIO_CFG(SHOOTER_SP3D_SPI_DO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_DI, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CS, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_GATE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_CORE_GATE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SYS_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_PDX, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_WEBCAM_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),/*camera reset*/
	GPIO_CFG(SHOOTER_WEBCAM_STB, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),/*camera standby*/
	GPIO_CFG(SHOOTER_CAM_SEL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),/*camera switch*/
};

static uint32_t camera_on_gpio_table_sp3d[] = {
	GPIO_CFG(SHOOTER_CAM_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_CAM_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_SP3D_MCLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),/*MCLK*/
	GPIO_CFG(106, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_DO,  1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_DI,  1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CS,  1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_GATE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_CORE_GATE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SYS_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_PDX, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_WEBCAM_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_WEBCAM_STB, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_CAM_SEL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t sp3d_spi_gpio[] = {
	/* or this? the i/o direction and up/down are much more correct */
	GPIO_CFG(SHOOTER_SP3D_SPI_DO,  1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_DI,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CS,  1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
};

static uint32_t camera_off_gpio_table_liteon[] = {
	GPIO_CFG(SHOOTER_CAM_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),/*i2c*/
	GPIO_CFG(SHOOTER_CAM_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),/*i2c*/
	GPIO_CFG(SHOOTER_SP3D_MCLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),/*MCLK*/
	GPIO_CFG(106, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/*sharp INT*/
	GPIO_CFG(SHOOTER_SP3D_SPI_DO, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_DI, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CS, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CLK, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_S5K4E1_VCM_PD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_S5K4E1_INTB, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_S5K4E1_PD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_WEBCAM_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),/*camera reset*/
	GPIO_CFG(SHOOTER_WEBCAM_STB, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),/*camera standby*/
	GPIO_CFG(SHOOTER_CAM_SEL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),/*camera switch*/
};

static uint32_t camera_on_gpio_table_liteon[] = {
	GPIO_CFG(SHOOTER_CAM_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_CAM_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_SP3D_MCLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),/*MCLK*/
	GPIO_CFG(106, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_DO,  0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_DI,  0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CS,  0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CLK, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_S5K4E1_VCM_PD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_S5K4E1_INTB, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_S5K4E1_PD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_WEBCAM_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_WEBCAM_STB, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_CAM_SEL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static struct regulator *shooter_reg_8058_l3 = NULL;
static struct regulator *shooter_reg_8058_l8 = NULL;
static struct regulator *shooter_reg_8058_l9 = NULL;
static struct regulator *shooter_reg_8058_l10 = NULL;
static struct regulator *shooter_reg_8058_l15 = NULL;

static struct regulator *shooter_reg_8901_lvs2 = NULL;
static struct regulator *shooter_reg_8901_lvs3 = NULL;

static int camera_sensor_power_enable(char *power, unsigned volt, struct regulator **sensor_power)
{
	int rc;

	if (power == NULL)
		return -ENODEV;

	*sensor_power = regulator_get(NULL, power);
	if (IS_ERR(sensor_power)) {
		pr_err("[CAM] %s: Unable to get %s\n", __func__, power);
		return -ENODEV;
	}

	if (system_rev != 2 && engineerid < 3) {/*VERSION A*/
		rc = regulator_set_voltage(*sensor_power, volt, volt);
		if (rc) {
			pr_err("[CAM] %s: unable to set %s voltage to %d rc:%d\n",
				__func__, power, volt, rc);
			regulator_put(*sensor_power);
			*sensor_power = NULL;
			return -ENODEV;
		}
	}

	rc = regulator_enable(*sensor_power);
	if (rc) {
		pr_err("[CAM] %s: Enable regulator %s failed\n", __func__, power);
		regulator_put(*sensor_power);
		*sensor_power = NULL;
		return -ENODEV;
	}

	return rc;
}

static int camera_sensor_power_disable(struct regulator *sensor_power)
{
	int rc;

	if (sensor_power == NULL)
		return -ENODEV;

	if (IS_ERR(sensor_power)) {
		pr_err("[CAM] %s: Invalid regulator ptr\n", __func__);
		return -ENODEV;
	}
	rc = regulator_disable(sensor_power);
	if (rc) {
		pr_err("[CAM] %s: Disable regulator failed\n", __func__);
		regulator_put(sensor_power);
		sensor_power = NULL;
		return -ENODEV;
	}
	regulator_put(sensor_power);
	sensor_power = NULL;
	return rc;
}

static void shooter_set_gpio(int gpio, int state)
{
	gpio_set_value(gpio, state);
}

static int shooter_config_camera_on_gpios(void)
{
	if (engineerid == 7) {
		config_gpio_table(camera_on_gpio_table_liteon,
			ARRAY_SIZE(camera_on_gpio_table_liteon));
	} else {
		config_gpio_table(camera_on_gpio_table_sp3d,
			ARRAY_SIZE(camera_on_gpio_table_sp3d));
	}

	return 0;
}

static void shooter_config_camera_off_gpios(void)
{
	if (engineerid == 7) {
		config_gpio_table(camera_off_gpio_table_liteon,
			ARRAY_SIZE(camera_off_gpio_table_liteon));
	} else {
		config_gpio_table(camera_off_gpio_table_sp3d,
			ARRAY_SIZE(camera_off_gpio_table_sp3d));
	}
	shooter_set_gpio(SHOOTER_SP3D_SPI_DO, 0);
	shooter_set_gpio(SHOOTER_SP3D_SPI_CS, 0);
	shooter_set_gpio(SHOOTER_SP3D_SPI_CLK, 0);
	shooter_set_gpio(SHOOTER_SP3D_MCLK, 0);
	shooter_set_gpio(SHOOTER_CAM_SEL, 0);
}

#ifdef CONFIG_SP3D
static int Shooter_sp3d_vreg_on(void)
{
	int rc = 0;
	pr_info("[CAM] %s\n", __func__);
	/* VDDIO*/
	if (system_rev == 2 && engineerid >= 3) {/*VERSION A*/
		rc = camera_sensor_power_enable("8901_lvs2", 0, &shooter_reg_8901_lvs2);
		pr_info("[CAM] sensor_power_enable(\"8901_lvs2\", 1800) == %d\n", rc);
		udelay(26);
		/*DVDD18 */
		rc = camera_sensor_power_enable("8901_lvs3", 0, &shooter_reg_8901_lvs3);
		pr_info("[CAM] sensor_power_enable(\"8901_lvs3\", 1800) == %d\n", rc);
	} else {
		rc = camera_sensor_power_enable("8058_l8", 1800000, &shooter_reg_8058_l8);
		pr_info("[CAM] sensor_power_enable(\"8058_l8\", 1800) == %d\n", rc);
		udelay(26);
		/*DVDD18 */
		rc = camera_sensor_power_enable("8058_l9", 1800000, &shooter_reg_8058_l9);
		pr_info("[CAM] sensor_power_enable(\"8058_l9\", 1800) == %d\n", rc);
	}

	shooter_config_camera_on_gpios();

	shooter_set_gpio(SHOOTER_SP3D_CORE_GATE, 1); /* CORE GATE */
	shooter_set_gpio(SHOOTER_SP3D_SYS_RST, 1); /* RST */
	shooter_set_gpio(SHOOTER_SP3D_PDX, 1); /* PDX */
	shooter_set_gpio(SHOOTER_SP3D_GATE, 1); /* GATE */
	/* main camera AVDD */
	rc = camera_sensor_power_enable("8058_l15", 2800000, &shooter_reg_8058_l15);
	pr_info("[CAM] sensor_power_enable(\"8058_l15\", 2800) == %d\n", rc);
	/* main camera MVDD */
	rc = camera_sensor_power_enable("8058_l10", 2800000, &shooter_reg_8058_l10);
	pr_info("[CAM] sensor_power_enable(\"8058_l10\", 2800) == %d\n", rc);
	gpio_tlmm_config(sp3d_spi_gpio[0], GPIO_CFG_ENABLE);
	gpio_tlmm_config(sp3d_spi_gpio[1], GPIO_CFG_ENABLE);
	gpio_tlmm_config(sp3d_spi_gpio[2], GPIO_CFG_ENABLE);
	gpio_tlmm_config(sp3d_spi_gpio[3], GPIO_CFG_ENABLE);
	return rc;
}

static int Shooter_sp3d_vreg_off(void)
{
	int rc;
	pr_info("[CAM] %s\n", __func__);
	shooter_set_gpio(SHOOTER_SP3D_PDX, 0); /* PDX */
	/* main camera MVDD */
	rc = camera_sensor_power_disable(shooter_reg_8058_l10);
	udelay(10);

	if (!(engineerid == 7)) {	/*according to logic Jason Kao, only tutn off l15 when sharp */
	/* main camera AVDD */
		rc = camera_sensor_power_disable(shooter_reg_8058_l15);
		udelay(10);
	}
	if (system_rev == 2 && engineerid >= 3) {
		/* main camera DVDD18 */
		rc = camera_sensor_power_disable(shooter_reg_8901_lvs3);
		shooter_set_gpio(SHOOTER_SP3D_SYS_RST, 0); /* RST */
		shooter_set_gpio(SHOOTER_SP3D_CORE_GATE, 0); /* CORE GATE */
		shooter_set_gpio(SHOOTER_SP3D_GATE, 0); /* GATE */
		/*VDDIO*/
		rc = camera_sensor_power_disable(shooter_reg_8901_lvs2);
	} else {
		/* main camera DVDD18 */
		rc = camera_sensor_power_disable(shooter_reg_8058_l9);
		shooter_set_gpio(SHOOTER_SP3D_SYS_RST, 0); /* RST */
		shooter_set_gpio(SHOOTER_SP3D_CORE_GATE, 0); /* CORE GATE */
		shooter_set_gpio(SHOOTER_SP3D_GATE, 0); /* GATE */
		/*VDDIO*/
		rc = camera_sensor_power_disable(shooter_reg_8058_l8);
	}

	shooter_config_camera_off_gpios();

	return rc;
}
#endif

#ifdef CONFIG_QS_S5K4E1
static int Shooter_qs_s5k4e1_vreg_on(void)
{
	int rc;
	pr_info("[CAM] %s\n", __func__);
	mdelay(50);

	rc = camera_sensor_power_enable("8058_l15", 2800000, &shooter_reg_8058_l15);
	pr_info("[CAM] sensor_power_enable(\"8058_l15\", 1800) == %d\n", rc);
	udelay(50);

	if (system_rev >= 2 && engineerid >= 3) {/*VERSION A*/
		/*IO*//*This is switch power*/
		rc = camera_sensor_power_enable("8901_lvs3", 0, &shooter_reg_8901_lvs3);
		pr_info("[CAM] sensor_power_enable(\"8901_lvs3\", 1800) == %d\n", rc);
		mdelay(1);

		rc = camera_sensor_power_enable("8901_lvs2", 0, &shooter_reg_8901_lvs2);
		pr_info("[CAM] sensor_power_enable(\"8901_lvs2\", 1800) == %d\n", rc);
	} else {
		rc = camera_sensor_power_enable("8058_l9", 1800000, &shooter_reg_8058_l9);
		pr_info("[CAM] sensor_power_enable(\"8058_l9\", 1800) == %d\n", rc);
		/* VDDIO*/
		rc = camera_sensor_power_enable("8058_l8", 1800000, &shooter_reg_8058_l8);
		pr_info("[CAM] sensor_power_enable(\"8058_l8\", 1800) == %d\n", rc);
	}
	udelay(50);

	/* main camera AVDD */
	rc = camera_sensor_power_enable("8058_l10", 2800000, &shooter_reg_8058_l10);
	pr_info("[CAM] sensor_power_enable(\"8058_l10\", 2800) == %d\n", rc);
	udelay(50);

	shooter_config_camera_on_gpios();

	shooter_set_gpio(SHOOTER_S5K4E1_INTB, 1);
	shooter_set_gpio(SHOOTER_S5K4E1_PD, 1);
	shooter_set_gpio(SHOOTER_S5K4E1_VCM_PD, 1);

	return rc;
}

static int Shooter_qs_s5k4e1_vreg_off(void)
{
	int rc;
	pr_info("[CAM] %s\n", __func__);
	shooter_set_gpio(SHOOTER_S5K4E1_INTB, 0); /* interrupt */
	shooter_set_gpio(SHOOTER_S5K4E1_VCM_PD, 0); /* PDX */
	shooter_set_gpio(SHOOTER_S5K4E1_PD, 0); /* RST */

	/* main camera AVDD */
	rc = camera_sensor_power_disable(shooter_reg_8058_l10);
	udelay(50);

	/*VDDIO*/
	if (system_rev >= 2 && engineerid >= 3) {/*VERSION A*/
		rc = camera_sensor_power_disable(shooter_reg_8901_lvs2);
		/*This is swich power*/
		rc = camera_sensor_power_disable(shooter_reg_8901_lvs3);
		pr_info("[CAM] sensor_power_enable(\"8901_lvs3\", 1800) == %d\n", rc);
	} else {
		/*This is swich power*/
		rc = camera_sensor_power_disable(shooter_reg_8058_l9);
		pr_info("[CAM] sensor_power_disable(\"8058_l9\") == %d\n", rc);
		rc = camera_sensor_power_disable(shooter_reg_8058_l8);
		pr_info("[CAM] sensor_power_disable(\"8058_l8\") == %d\n", rc);
	}

	/* according to logic Jason Kao, do not turn off l15 to avoid current leakage */
	if (!(engineerid == 7)) {
		rc = camera_sensor_power_disable(shooter_reg_8058_l15);
		udelay(50);
	}

	shooter_config_camera_off_gpios();

	return rc;
}
#endif

static int Shooter_s5k6aafx_vreg_on(void)
{
	int rc;
	pr_info("[CAM] %s\n", __func__);
	/* main / 2nd camera analog power */
	rc = camera_sensor_power_enable("8058_l3", 2850000, &shooter_reg_8058_l3);
	pr_info("[CAM] sensor_power_enable(\"8058_l3\", 2850) == %d\n", rc);
	mdelay(5);
	/* main / 2nd camera digital power */
	if (system_rev == 2 && engineerid >= 3) {
		rc = camera_sensor_power_enable("8901_lvs2", 0, &shooter_reg_8901_lvs2);
		pr_info("[CAM] sensor_power_enable(\"8901_lvs2\", 1800) == %d\n", rc);
		mdelay(5);
		/*IO*/
		rc = camera_sensor_power_enable("8901_lvs3", 0, &shooter_reg_8901_lvs3);
		pr_info("[CAM] sensor_power_enable(\"8901_lvs3\", 1800) == %d\n", rc);
		mdelay(1);
	} else {
		rc = camera_sensor_power_enable("8058_l8", 1800000, &shooter_reg_8058_l8);
		pr_info("[CAM] sensor_power_enable(\"8058_l8\", 1800) == %d\n", rc);
		mdelay(5);
		/*IO*/
		rc = camera_sensor_power_enable("8058_l9", 1800000, &shooter_reg_8058_l9);
		pr_info("[CAM]s ensor_power_enable(\"8058_l9\", 1800) == %d\n", rc);
		mdelay(1);
	}

	shooter_config_camera_on_gpios();

	return rc;
}

static int Shooter_s5k6aafx_vreg_off(void)
{
	int rc;
	pr_info("[CAM] %s\n", __func__);
	/* IO power off */
	if (system_rev == 2 && engineerid >= 3) {
		rc = camera_sensor_power_disable(shooter_reg_8901_lvs3);
		pr_info("[CAM] sensor_power_disable(\"8901_lvs3\") == %d\n", rc);
		mdelay(1);

		/* main / 2nd camera digital power */
		rc = camera_sensor_power_disable(shooter_reg_8901_lvs2);
		pr_info("[CAM] sensor_power_disable(\"8901_lvs2\") == %d\n", rc);
	} else {
		rc = camera_sensor_power_disable(shooter_reg_8058_l9);
		pr_info("[CAM] sensor_power_disable(\"8058_l9\") == %d\n", rc);
		mdelay(1);

		/* main / 2nd camera digital power */
		rc = camera_sensor_power_disable(shooter_reg_8058_l8);
		pr_info("[CAM] sensor_power_disable(\"8058_l8\") == %d\n", rc);
	}
	mdelay(1);
	/* main / 2nd camera analog power */
	rc = camera_sensor_power_disable(shooter_reg_8058_l3);
	pr_info("[CAM] sensor_power_disable(\"8058_l3\") == %d\n", rc);

	shooter_config_camera_off_gpios();

	return rc;
}

static void Shooter_maincam_clk_switch(void)
{
	pr_info("[CAM] Doing clk switch (Main Cam)\n");
	gpio_set_value(SHOOTER_CAM_SEL, 0);
}

static void Shooter_seccam_clk_switch(void)
{
	pr_info("[CAM] Doing clk switch (2nd Cam)\n");
	gpio_set_value(SHOOTER_CAM_SEL, 1);
}

struct resource msm_camera_resources[] = {
	{
		.start	= 0x04500000,
		.end	= 0x04500000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= VFE_IRQ,
		.end	= VFE_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct spi_board_info sp3d_spi_board_info[] __initdata = {
	{
		.modalias	= "sp3d_spi",
		.mode		= SPI_MODE_3,
		.bus_num	= 1,
		.chip_select	= 0,
		.max_speed_hz	= 15060000,
	}
};

#ifdef CONFIG_MSM_CAMERA_FLASH
int flashlight_control(int mode)
{
#if CONFIG_FLASHLIGHT_AAT1277
	return aat1277_flashlight_control(mode);
#else
	return 0;
#endif
}

static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type		= MSM_CAMERA_FLASH_SRC_CURRENT_DRIVER,
	.camera_flash		= flashlight_control,
};
#endif

static struct camera_flash_cfg msm_camera_sensor_flash_cfg = {
	.low_temp_limit		= 10,
	.low_cap_limit		= 15,
};

#ifdef CONFIG_SP3D
static struct msm_camera_sensor_flash_data flash_sp3d = {
	.flash_type		= MSM_CAMERA_FLASH_LED,
#ifdef CONFIG_MSM_CAMERA_FLASH
	.flash_src		= &msm_flash_src,
#endif
};

static struct msm_camera_sensor_info msm_camera_sensor_sp3d_data = {
	.sensor_name		= "sp3d",
	.vcm_enable		= 0,
	.camera_power_on 	= Shooter_sp3d_vreg_on,
	.camera_power_off	= Shooter_sp3d_vreg_off,
	.camera_clk_switch	= Shooter_maincam_clk_switch,
	.pdata			= &msm_camera_csi_device_data[0],
	.resource		= msm_camera_resources,
	.num_resources		= ARRAY_SIZE(msm_camera_resources),
	.flash_data		= &flash_sp3d,
	.flash_cfg		= &msm_camera_sensor_flash_cfg,
	//.stereo_low_cap_limit = 15,
	.mirror_mode		= 0,
	.csi_if			= 1,
	.dev_node		= 0,
};

struct platform_device msm_camera_sensor_sp3d = {
	.name	= "msm_camera_sp3d",
	.dev	= {
		.platform_data = &msm_camera_sensor_sp3d_data,
	},
};
#endif

#ifdef CONFIG_QS_S5K4E1
static char eeprom_data[864];
static struct msm_camera_sensor_flash_data flash_qs_s5k4e1 = {
	.flash_type		= MSM_CAMERA_FLASH_LED,
#ifdef CONFIG_MSM_CAMERA_FLASH
	.flash_src		= &msm_flash_src,
#endif
};

static struct msm_camera_sensor_info msm_camera_sensor_qs_s5k4e1_data = {
	.sensor_name		= "qs_s5k4e1",
	.sensor_reset		= SHOOTER_S5K4E1_PD,
	.vcm_enable		= 0,
	.camera_power_on 	= Shooter_qs_s5k4e1_vreg_on,
	.camera_power_off 	= Shooter_qs_s5k4e1_vreg_off,
	.camera_clk_switch 	= Shooter_maincam_clk_switch,
	.pdata			= &msm_camera_csi_device_data[0],
	.resource		= msm_camera_resources,
	.num_resources		= ARRAY_SIZE(msm_camera_resources),
	.flash_data		= &flash_qs_s5k4e1,
	.flash_cfg 		= &msm_camera_sensor_flash_cfg,
	//.stereo_low_cap_limit = 15,
	.csi_if			= 1,
	.dev_node		= 0,
	.eeprom_data		= eeprom_data,
};

struct platform_device msm_camera_sensor_qs_s5k4e1 = {
	.name	= "msm_camera_qs_s5k4e1",
	.dev	= {
		.platform_data = &msm_camera_sensor_qs_s5k4e1_data,
	},
};
#endif

static struct msm_camera_sensor_flash_data flash_s5k6aafx = {
	.flash_type		= MSM_CAMERA_FLASH_NONE,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k6aafx_data = {
	.sensor_name		= "s5k6aafx",
	.sensor_reset		= SHOOTER_WEBCAM_RST,/*2nd Cam RST*/
	.sensor_pwd		= SHOOTER_WEBCAM_STB,/*2nd Cam PWD*/
	.vcm_enable		= 0,
	.camera_power_on 	= Shooter_s5k6aafx_vreg_on,
	.camera_power_off 	= Shooter_s5k6aafx_vreg_off,
	.camera_clk_switch 	= Shooter_seccam_clk_switch,
	.pdata			= &msm_camera_csi_device_data[1],
	.resource		= msm_camera_resources,
	.num_resources		= ARRAY_SIZE(msm_camera_resources),
	.flash_data             = &flash_s5k6aafx,
	.mirror_mode		= 0,
	.csi_if			= 1,
	.dev_node		= 1,
};

struct platform_device msm_camera_sensor_s5k6aafx = {
	.name	= "msm_camera_qs_s5k4e1",
	.dev	= {
		.platform_data = &msm_camera_sensor_s5k6aafx_data,
	},
};

static struct i2c_board_info msm_camera_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("s5k6aafx", 0x78 >> 1),
	},
	{
		I2C_BOARD_INFO("s5k6aafx", 0x5a >> 1), /* COB type */
	},
#ifdef CONFIG_QS_S5K4E1
	{
		I2C_BOARD_INFO("qs_s5k4e1", 0x20),
	},
#endif
};

void __init msm8x60_init_cam(void)
{
	pr_info("%s", __func__);

	i2c_register_board_info(MSM_GSBI4_QUP_I2C_BUS_ID,
		msm_camera_boardinfo,
		ARRAY_SIZE(msm_camera_boardinfo));

#ifdef CONFIG_SP3D
	spi_register_board_info(sp3d_spi_board_info,
			ARRAY_SIZE(sp3d_spi_board_info));
	platform_device_register(&msm_camera_sensor_sp3d);
#endif
#ifdef CONFIG_QS_S5K4E1
	platform_device_register(&msm_camera_sensor_qs_s5k4e1);
#endif
	platform_device_register(&msm_camera_sensor_s5k6aafx);

	platform_device_register(&msm_camera_server);

	platform_device_register(&msm_device_csic0);
	platform_device_register(&msm_device_csic1);
	platform_device_register(&msm_device_vfe);
	platform_device_register(&msm_device_vpe);
}

#endif	
