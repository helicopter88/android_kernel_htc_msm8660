GCC_VERSION      := $(shell $(CONFIG_SHELL) $(PWD)/scripts/gcc-version.sh $(CROSS_COMPILE)gcc)

obj-$(CONFIG_MSM_CAMERA) += msm_camera.o sensors/

ifeq ($(CONFIG_CAMERA_3D),y)
obj-$(CONFIG_MSM_CAMERA) += msm_camera_liteon.o sensors/
endif

obj-$(CONFIG_MSM_CAMERA) += msm_axi_qos.o

ifeq ($(CONFIG_MSM_CAMERA_V4L2),y)
	obj-$(CONFIG_MSM_CAMERA) += gemini/
	obj-$(CONFIG_MSM_CAMERA_FLASH) += flash_v4l2.o
else
	obj-$(CONFIG_MSM_CAMERA) += gemini/
	obj-$(CONFIG_MSM_CAMERA_FLASH) += flash.o
endif

ifeq ($(CONFIG_MSM_CAMERA_V4L2),y)
  obj-$(CONFIG_ARCH_MSM8X60) += msm_io_8x60_v4l2.o msm_vfe31_v4l2.o msm_vpe_8x60_v4l2.o
else
ifdef CONFIG_CAMERA_ZSL
	obj-$(CONFIG_ARCH_MSM8X60) += msm_io_8x60.o msm_vfe_8x60_ZSL.o msm_vpe1_8x60.o
else
	ifdef CONFIG_CAMERA_3D
		obj-$(CONFIG_ARCH_MSM8X60) += msm_io_8x60.o msm_vfe_8x60.o msm_vpe1_8x60.o
		obj-$(CONFIG_ARCH_MSM8X60) += msm_vfe31_liteon.o msm_vpe1_liteon.o
	else
		obj-$(CONFIG_ARCH_MSM8X60) += msm_io_8x60.o msm_vfe_8x60.o msm_vpe1_8x60.o
	endif
endif
endif
