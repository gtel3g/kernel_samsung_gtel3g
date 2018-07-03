# SC88XX Platform Support
ifeq ($(CONFIG_ARCH_SCX15),y)
DEFS = -DVERSION1=1
ccflags-y += $(DEFS)
else ifeq ($(CONFIG_ARCH_SC8825),y)
DEFS = -DVERSION2=1
ccflags-y += $(DEFS)
else ifeq ($(CONFIG_ARCH_SC8810),y)
DEFS = -DVERSION2T=1
ccflags-y += $(DEFS)
else ifeq ($(CONFIG_ARCH_SCX30G),y)
DEFS = -DVERSION3T=1
ccflags-y += $(DEFS)
else ifeq ($(CONFIG_ARCH_SCX35L),y)
DEFS = -DVERSION3L=1
ccflags-y += $(DEFS)
endif
obj-y += sprd_sensor.o
ccflags-y += -Idrivers/media/sprd_dcam/common

common-file := csi2/csi_access.o csi2/csi_driver.o csi2/csi_log.o csi2/csi_system.o
ifeq ($(CONFIG_ARCH_SCX35),y)
sprd_sensor-objs := csi2/csi_api.o
endif
ifeq ($(CONFIG_ARCH_SCX15),y)
sprd_sensor-objs := csi2/csi_api_null.o
endif
sprd_sensor-objs += sensor_drv_k.o $(common-file)
obj-y += power/
