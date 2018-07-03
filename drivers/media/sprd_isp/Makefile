# SC88XX Platform Support

ifeq ($(CONFIG_ARCH_SCX35),y)
ifeq ($(CONFIG_ARCH_SCX15),y)
sprd_isp-objs := isp_drv_null.o
obj-y += sprd_isp.o
else
sprd_isp-objs := isp_drv_kernel.o
obj-y += sprd_isp.o
endif
endif

ifeq ($(CONFIG_ARCH_SCX15),y)
DEFS = -DVERSION1=1
ccflags-y += $(DEFS)
else ifeq ($(CONFIG_ARCH_SC8825),y)
DEFS = -DVERSION2=1
ccflags-y += $(DEFS)
else ifeq ($(CONFIG_ARCH_SCX30G),y)
DEFS = -DVERSION3T=1
ccflags-y += $(DEFS)
else ifeq ($(CONFIG_ARCH_SCX35L),y)
DEFS = -DVERSION3L=1
ccflags-y += $(DEFS)
endif

ccflags-y += -Idrivers/media/sprd_dcam/common

ifeq ($(CONFIG_ARCH_SC8825),y)
sprd_isp-objs := isp_drv_kernel.o
obj-y += sprd_isp.o
endif
