/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <asm/io.h>
#include <linux/file.h>
#include <mach/dma.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/wakelock.h>
#include <mach/hardware.h>
#include <mach/board.h>
#include <linux/regulator/consumer.h>
#include <mach/regulator.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_i2c.h>
#include <mach/arch_misc.h>
<<<<<<< HEAD
#include <mach/pinmap.h> // Get pinmap name
#include <mach/sci.h> // Pinmap write codes

#if defined(CONFIG_MACH_CORE3_W)|| defined(CONFIG_MACH_GRANDNEOVE3G) || defined(CONFIG_MACH_VIVALTO5MVE3G)\
	|| defined(CONFIG_MACH_VIVALTO3MVE3G_LTN) || defined(CONFIG_MACH_J13G) || defined(CONFIG_MACH_VIVALTO3MVEML3G)	\
	|| defined(CONFIG_MACH_YOUNG33G)
#include <linux/leds.h>
#include <linux/mfd/sm5701_core.h>
#endif

#if defined(CONFIG_MACH_VIVALTO3MVE3G_LTN) || defined(CONFIG_MACH_VIVALTO3MVEML3G)
#undef CONFIG_MACH_VIVALTO5MVE3G
#endif

#if defined(CONFIG_MACH_GOYAVE3G_SWA)
#undef CONFIG_MACH_GOYAVE3G
#endif

#if defined (CONFIG_ARCH_SC8825)
#include <mach/i2c-sprd.h>
#define SENSOR_I2C_ID	1
#elif defined (CONFIG_ARCH_SC8810)
#include <mach/i2c-sc8810.h>
#elif defined (CONFIG_ARCH_SCX35)
=======
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
#include <mach/i2c-sprd.h>
#include <mach/adi.h>
#include <video/sensor_drv_k.h>
#include "sensor_drv_sprd.h"
#include "csi2/csi_api.h"
#include "parse_hwinfo.h"
<<<<<<< HEAD
#include "../sprd_dcam/flash/flash.h"

/* FIXME: Move to camera device platform data later */

#if defined(CONFIG_ARCH_SCX35)

#define REGU_NAME_CAMVIO     "vddcamio"

#if defined(CONFIG_MACH_KANAS_W) || defined(CONFIG_MACH_KANAS_TD) || defined (CONFIG_MACH_TSHARKWSAMSUNG)

#define REGU_NAME_SUB_CAMDVDD	"vddcamd"
#define REGU_NAME_CAMMOT			"vddcammot"
#define REGU_NAME_CAMAVDD		"vddcama"
#define REGU_NAME_CAMDVDD		"vddcamd"

#elif defined(CONFIG_MACH_CORE3_W) || defined(CONFIG_MACH_GRANDNEOVE3G)

#define REGU_NAME_SUB_CAMDVDD	"vddcamd"
#define REGU_NAME_CAMAVDD		"vddcama"
#define REGU_NAME_CAMDVDD		"vddcamd"
#define REGU_NAME_CAMMOT			"vddcammot"

#elif defined(CONFIG_MACH_J13G)

#define REGU_NAME_CAMAVDD	"vddcama"
#define REGU_NAME_CAMVIO		"vddcamio"
#define REGU_NAME_CAMMOT			"vddcammot"
#define REGU_NAME_SUB_CAMDVDD		"vddcamd"
#define REGU_NAME_CAMDVDD		"vddcamd"

#elif defined(CONFIG_MACH_GOYAVE3G_SWA) || defined(CONFIG_MACH_GTEL3G) || defined(CONFIG_MACH_GTELWIFI)
#define REGU_NAME_SUB_CAMDVDD	"vddcamd"
#define REGU_NAME_CAMAVDD		"vddcama"
#define REGU_NAME_CAMDVDD		"vddcamd"
#define REGU_NAME_CAMMOT		"vddcammot"
#define REGU_NAME_CAMVIO     	"vddcamio"

#elif defined(CONFIG_MACH_YOUNG23GDTV)|| defined(CONFIG_MACH_VIVALTO3MVE3G_LTN) 

#ifdef REGU_NAME_CAMVIO
#undef REGU_NAME_CAMVIO
#define REGU_NAME_CAMVIO	"vddcamd"
#endif

#define REGU_NAME_SUB_CAMDVDD	"vddcamd"
#define REGU_NAME_CAMAVDD		"vddcama"
#define REGU_NAME_CAMDVDD		"vddcamio"
#define REGU_NAME_CAMMOT			"vddcama"  

#elif defined(CONFIG_MACH_VIVALTO3MVEML3G)

#ifdef REGU_NAME_CAMVIO
#undef REGU_NAME_CAMVIO
#define REGU_NAME_CAMVIO	"vddcamd"
#endif

#define REGU_NAME_SUB_CAMDVDD	"vddcamd"
#define REGU_NAME_CAMAVDD		"vddcama"
#define REGU_NAME_CAMDVDD		"vddcamio"
#define REGU_NAME_CAMMOT			"vddcama"  

#elif defined(CONFIG_MACH_YOUNG33G)
#define REGU_NAME_SUB_CAMDVDD	"vddcamd"
#define REGU_NAME_CAMMOT		"vddcama"
#define REGU_NAME_CAMVIO		"vddcamd"

#define REGU_NAME_CAMAVDD		"vddcama"
#define REGU_NAME_CAMDVDD		"vddcamio"

#elif defined(CONFIG_MACH_VIVALTO5MVE3G)

#define REGU_NAME_SUB_CAMDVDD	"vddwifipa"
#define REGU_NAME_CAMAVDD		"vddcama"
#define REGU_NAME_CAMDVDD		"vddcamio"
#define REGU_NAME_CAMMOT			"vddcammot"

#elif defined(CONFIG_MACH_GOYAVE3G_SEA) || defined(CONFIG_MACH_GOYAVEWIFI_SEA_XTC)
#define REGU_NAME_SUB_CAMDVDD	"vddcamio"
#define REGU_NAME_CAMAVDD		"vddcama"
#define REGU_NAME_CAMDVDD		"vddcamd"
#define REGU_NAME_CAMMOT		"vddcammot"

#else // defined(CONFIG_MACH_KANAS_W) || defined(CONFIG_MACH_KANAS_TD) || defined (CONFIG_MACH_TSHARKWSAMSUNG)

#define REGU_NAME_SUB_CAMDVDD	"vddcamd"
#define GPIO_SUB_SENSOR_RESET		GPIO_SENSOR_RESET
#define REGU_NAME_CAMAVDD		"vddcama"
#define REGU_NAME_CAMDVDD		"vddcamd"
#define REGU_NAME_CAMMOT			"vddcammot"

#endif // defined(CONFIG_MACH_KANAS_W) || defined(CONFIG_MACH_KANAS_TD) || defined (CONFIG_MACH_TSHARKWSAMSUNG)

#define SENSOR_CLK	"clk_sensor"

#else // defined(CONFIG_ARCH_SCX35)

#define REGU_NAME_CAMAVDD	"vddcama"
#define REGU_NAME_CAMVIO		"vddcamio"
#define REGU_NAME_CAMDVDD	"vddcamcore"
#define REGU_NAME_CAMMOT		"vddcammot"
#define SENSOR_CLK				"ccir_mclk"

#endif // defined(CONFIG_ARCH_SCX35)

=======
#include "power/sensor_power.h"

#define SENSOR_CLK                        "clk_sensor"
#define SENSOR_DEVICE_NAME                "sprd_sensor"

>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
#define DEBUG_SENSOR_DRV
#ifdef  DEBUG_SENSOR_DRV
#define SENSOR_PRINT	pr_debug
#else
#define SENSOR_PRINT(...)
#endif
<<<<<<< HEAD

#define SENSOR_PRINT_ERR	printk
#define SENSOR_PRINT_HIGH	printk

#define SENSOR_K_SUCCESS	0
#define SENSOR_K_FAIL		(-1)
#define SENSOR_K_FALSE		0
#define SENSOR_K_TRUE		1
=======
#define SENSOR_PRINT_ERR                  printk
#define SENSOR_PRINT_HIGH                 printk
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g

#define LOCAL	static

<<<<<<< HEAD
#define PNULL	((void *)0)

#define NUMBER_MAX		0x7FFFFFF
=======
#define LOCAL                             static
#define PNULL                             ((void *)0)
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g

#define SENSOR_MINOR	MISC_DYNAMIC_MINOR
#define SLEEP_MS(ms)	msleep(ms)

<<<<<<< HEAD
#define SENSOR_I2C_OP_TRY_NUM		4
#define SENSOR_CMD_BITS_8			1
#define SENSOR_CMD_BITS_16			2
#define SENSOR_I2C_VAL_8BIT		0x00
#define SENSOR_I2C_VAL_16BIT		0x01
#define SENSOR_I2C_REG_8BIT		(0x00 << 1)
#define SENSOR_I2C_REG_16BIT		(0x01 << 1)
#define SENSOR_I2C_CUSTOM			(0x01 << 2)
#define SENSOR_LOW_EIGHT_BIT		0xff
#define SENSOR_WRITE_DELAY		0xffff

#define DEBUG_STR	"Error L %d, %s \n"
#define DEBUG_ARGS	__LINE__,__FUNCTION__

#define SENSOR_MCLK_SRC_NUM	4
#define SENSOR_MCLK_DIV_MAX	4
#define ABS(a)	((a) > 0 ? (a) : -(a))
#define SENSOR_LOWEST_ADDR	0x800
#define SENSOR_ADDR_INVALID(addr)	((uint32_t)(addr) < SENSOR_LOWEST_ADDR)
=======
#define SENSOR_MINOR                      MISC_DYNAMIC_MINOR
#define SLEEP_MS(ms)                      msleep(ms)
#define PREALLOC_SIZE                     16000

#define SENSOR_I2C_ID                     0
#define SENSOR_I2C_OP_TRY_NUM             4
#define SENSOR_CMD_BITS_8                 1
#define SENSOR_CMD_BITS_16                2
#define SENSOR_I2C_VAL_8BIT               0x00
#define SENSOR_I2C_VAL_16BIT              0x01
#define SENSOR_I2C_REG_8BIT               (0x00 << 1)
#define SENSOR_I2C_REG_16BIT              (0x01 << 1)
#define SENSOR_I2C_CUSTOM                 (0x01 << 2)
#define SENSOR_LOW_EIGHT_BIT              0xff

#define SENSOR_WRITE_DELAY                0xffff
#define DEBUG_STR                         "Error L %d, %s \n"
#define DEBUG_ARGS                        __LINE__,__FUNCTION__
#define SENSOR_MCLK_SRC_NUM               4
#define SENSOR_MCLK_DIV_MAX               4
#define SENSOR_ABS(a)                     ((a) > 0 ? (a) : -(a))
#define SENSOR_LOWEST_ADDR                0x800
#define SENSOR_ADDR_INVALID(addr)         ((uint32_t)(addr) < SENSOR_LOWEST_ADDR)
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g

#define SENSOR_CHECK_ZERO(a)                                   \
	do {                                                       \
		if (SENSOR_ADDR_INVALID(a)) {                          \
			printk("SENSOR, zero pointer \n");                 \
			printk(DEBUG_STR, DEBUG_ARGS);                     \
			return -EFAULT;                                    \
		}                                                      \
	} while(0)

#define SENSOR_CHECK_ZERO_VOID(a)                               \
	do {                                                        \
		if (SENSOR_ADDR_INVALID(a)) {                           \
			printk("SENSOR, zero pointer \n");                  \
			printk(DEBUG_STR, DEBUG_ARGS);                      \
			return;                                             \
		}                                                       \
	} while(0)

struct sensor_mclk_tag {
	uint32_t                        clock;
	char                            *src_name;
};

struct sensor_mem_tag {
	void                            *buf_ptr;
	size_t                          size;
};

<<<<<<< HEAD
uint32_t flash_status = 0;
=======
struct sensor_module_tag {
	struct mutex                    sync_lock;
	atomic_t                        users;
	struct i2c_client               *cur_i2c_client;
};
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g

struct sensor_module_tab_tag {
	atomic_t                        total_users;
	uint32_t                        padding;
	struct mutex                    sensor_id_lock;
	struct device_node              *of_node;
	struct clk                      *sensor_mm_in_clk;
	struct wake_lock                wakelock;
	struct sensor_module_tag        sensor_dev_tab[SENSOR_DEV_MAX];
};

struct sensor_gpio_tag {
	int                             pwn;
	int                             reset;
};

struct sensor_file_tag {
	struct sensor_module_tab_tag    *module_data;
	uint32_t                        sensor_id;
	uint32_t                        sensor_mclk;
	uint32_t                        iopower_on_count;
	uint32_t                        avddpower_on_count;
	uint32_t                        dvddpower_on_count;
	uint32_t                        motpower_on_count;
	uint32_t                        mipi_on;
<<<<<<< HEAD
	struct mutex                sensor_lock;
	struct clk                     *ccir_clk;
	struct clk                     *mipi_clk;
	struct i2c_client            *cur_i2c_client;
	struct regulator            *camvio_regulator;
	struct regulator            *camavdd_regulator;
	struct regulator            *camdvdd_regulator;
	struct regulator            *cammot_regulator;
	struct i2c_driver           sensor_i2c_driver;
	struct sensor_mem_tag sensor_mem;
	unsigned                        pin_main_reset;
	unsigned                        pin_sub_reset;
	unsigned                        pin_main_pd;
	unsigned                        pin_sub_pd;
	unsigned                        pin_main_camdvdd_en;
	unsigned						pin_main_camavdd_en;
	atomic_t                        open_count;
};

typedef struct {
	uint32_t reg;
	uint32_t val;
} camera_pinmap_t;

camera_pinmap_t camera_stby_rst_pinmap[] = {
	{REG_PIN_CCIRRST, BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_Z}, // Main Camera RST
	{REG_PIN_CCIRPD1, BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_Z}, // Front Camera STBY
	{REG_PIN_CCIRPD0, BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_Z}, // Main Camera STBY
	{REG_PIN_TRACEDAT6, BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_Z}, // Front Camera RST
=======
	uint32_t                        padding;
	uint32_t                        phy_id;
	uint32_t                        if_type;
	struct sensor_gpio_tag          gpio_tab;
	struct clk                      *ccir_clk;
	struct clk                      *ccir_enable_clk;
	struct clk                      *mipi_clk;
	struct regulator                *camvio_regulator;
	struct regulator                *camavdd_regulator;
	struct regulator                *camdvdd_regulator;
	struct regulator                *cammot_regulator;
	struct sensor_mem_tag           sensor_mem;
	void                            *csi_handle;
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
};

LOCAL const struct sensor_mclk_tag c_sensor_mclk_tab[SENSOR_MCLK_SRC_NUM] = {
						{96, "clk_96m"},
						{77, "clk_76m8"},
						{48, "clk_48m"},
						{26, "ext_26m"}
};

<<<<<<< HEAD
LOCAL const unsigned short c_sensor_main_default_addr_list[] = {SENSOR_MAIN_I2C_ADDR, SENSOR_SUB_I2C_ADDR, I2C_CLIENT_END};

LOCAL const struct i2c_device_id c_sensor_device_id[] = {
	{SENSOR_MAIN_I2C_NAME, 0},
	{SENSOR_SUB_I2C_NAME, 1},
	{}
};

LOCAL struct sensor_module * s_p_sensor_mod = PNULL;
SENSOR_PROJECT_FUNC_T s_sensor_project_func = {PNULL};
uint32_t flash_torch_status=0;

// Camera Anti-Banding (enum is also defined at SprdCameraHardwareInterface.h)
typedef enum{
	CAM_BANDFILTER_50HZ_AUTO	= 0,
	CAM_BANDFILTER_50HZ_FIXED	= 1,
	CAM_BANDFILTER_60HZ_AUTO	= 2,
	CAM_BANDFILTER_60HZ_FIXED	= 3,
	CAM_BANDFILTER_LIMIT			= 0x7fffffff,
}CAM_BandFilterMode;

int camera_antibanding_val = CAM_BANDFILTER_60HZ_AUTO; // Default
uint16_t VENDOR_ID = 0xFFFF;

LOCAL void* _sensor_k_kmalloc(size_t size)
{
	if (SENSOR_ADDR_INVALID(s_p_sensor_mod))
	{
		printk("_sensor_k_kmalloc : NULL pointer\n");
=======
LOCAL void* _sensor_k_malloc(struct sensor_file_tag *fd_handle, size_t size)
{
	if (SENSOR_ADDR_INVALID(fd_handle)) {
		printk("SENSOR, zero pointer \n");
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
		printk(DEBUG_STR, DEBUG_ARGS);
		return PNULL;
	}

<<<<<<< HEAD
	if(PNULL == s_p_sensor_mod->sensor_mem.buf_ptr)
	{
		s_p_sensor_mod->sensor_mem.buf_ptr = vzalloc(size);
		if(PNULL != s_p_sensor_mod->sensor_mem.buf_ptr)
		{
			s_p_sensor_mod->sensor_mem.size = size;
		}
		return s_p_sensor_mod->sensor_mem.buf_ptr;
	}
	else if (size <= s_p_sensor_mod->sensor_mem.size)
	{
		return s_p_sensor_mod->sensor_mem.buf_ptr;
	}
	else
	{
		// Realloc memory
		vfree(s_p_sensor_mod->sensor_mem.buf_ptr);
		s_p_sensor_mod->sensor_mem.buf_ptr = PNULL;
		s_p_sensor_mod->sensor_mem.size = 0;
		s_p_sensor_mod->sensor_mem.buf_ptr = vzalloc(size);
		if (PNULL != s_p_sensor_mod->sensor_mem.buf_ptr)
		{
			s_p_sensor_mod->sensor_mem.size = size;
		}
		return s_p_sensor_mod->sensor_mem.buf_ptr;
=======
	if(PNULL == fd_handle->sensor_mem.buf_ptr) {
		fd_handle->sensor_mem.buf_ptr = vzalloc(size);
		if(PNULL != fd_handle->sensor_mem.buf_ptr) {
			fd_handle->sensor_mem.size = size;
		} else {
			printk("cam_err: malloc fd_handle->sensor_mem.buf_ptr\n");
		}

	} else if (size > fd_handle->sensor_mem.size) {
		//realloc memory
		printk("size = %d fd_handle->sensor_mem.size= %d\n", size, fd_handle->sensor_mem.size);
		vfree(fd_handle->sensor_mem.buf_ptr);
		fd_handle->sensor_mem.buf_ptr = PNULL;
		fd_handle->sensor_mem.size = 0;
		fd_handle->sensor_mem.buf_ptr = vzalloc(size);
		if (PNULL != fd_handle->sensor_mem.buf_ptr) {
			fd_handle->sensor_mem.size = size;
		} else {
			printk("cam_err: malloc fd_handle->sensor_mem.buf_ptr\n");
		}
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
	}

<<<<<<< HEAD
LOCAL void* _sensor_k_kzalloc(size_t size)
{
	SENSOR_PRINT_HIGH("_sensor_k_kzalloc : E\n");
	void *ptr = _sensor_k_kmalloc(size);
	return ptr;
=======
	return fd_handle->sensor_mem.buf_ptr;
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
}

LOCAL void _sensor_k_free(struct sensor_file_tag *fd_handle, void *p)
{
<<<<<<< HEAD
	/* Memory will NOT be free */
	SENSOR_PRINT_HIGH("_sensor_k_kfree : E");
=======
	/* memory will not be free */
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
	return;
}

#if 0
LOCAL struct platform_device*  _sensor_k_get_platform_device(void)
{
<<<<<<< HEAD
	int res = 0;
	SENSOR_CHECK_ZERO(s_p_sensor_mod);

	SENSOR_PRINT_HIGH(KERN_INFO "sensor_probe E\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		SENSOR_PRINT_HIGH(KERN_INFO "%s : func check failed\n", __FUNCTION__);
		res = -ENODEV;
		goto out;
=======
	struct device *dev;

	dev = bus_find_device_by_name(&platform_bus_type, NULL, SENSOR_DEVICE_NAME);
	if (!dev) {
		printk("%s error: find device\n", __func__);
		return NULL;
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
	}

<<<<<<< HEAD
	SENSOR_PRINT_HIGH(KERN_INFO "sensor_probe : addr 0x%x\n", s_p_sensor_mod->cur_i2c_client->addr);
	return 0;
out:
	return res;
}

LOCAL int sensor_remove(struct i2c_client *client)
{
	return 0;
=======
	return to_platform_device(dev);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
}
#endif

int sensor_k_set_pd_level(uint32_t *fd_handle, uint8_t power_level)
{
<<<<<<< HEAD
	SENSOR_PRINT_HIGH("sensor_detect : E\n");
	strcpy(info->type, client->name);
	return 0;
}

int sensor_k_set_pd_level(BOOLEAN power_level)
{
	switch (_sensor_K_get_curId())
	{
		case SENSOR_MAIN:
		{
			SENSOR_PRINT_HIGH("SENSOR : sensor_k_set_pd_level : Main STBY power level is %d, pin_main = %d\n", power_level, s_p_sensor_mod->pin_main_pd);
			if (0 == power_level)
			{
				gpio_direction_output(s_p_sensor_mod->pin_main_pd, 0);
			}
			else
			{
				gpio_direction_output(s_p_sensor_mod->pin_main_pd, 1);
			}
			SENSOR_PRINT_HIGH("SENSOR : sensor_k_set_pd_level : Main STBY = %d\n", gpio_get_value(s_p_sensor_mod->pin_main_pd));
			break;
		}

		case SENSOR_SUB:
		{
			SENSOR_PRINT_HIGH("SENSOR : sensor_k_set_pd_level : Sub STBY power level is %d, pin_sub = %d\n", power_level, s_p_sensor_mod->pin_sub_pd);
			if (0 == power_level)
			{
				gpio_direction_output(s_p_sensor_mod->pin_sub_pd, 0);
			}
			else
			{
				gpio_direction_output(s_p_sensor_mod->pin_sub_pd, 1);
			}
			SENSOR_PRINT_HIGH("SENSOR : sensor_k_set_pd_level : Sub STBY = %d\n", gpio_get_value(s_p_sensor_mod->pin_sub_pd));
			break;
		}
		default:
			break;
=======
	struct sensor_module_tab_tag    *p_mod;
	struct sensor_file_tag          *fd = (struct sensor_file_tag *)fd_handle;

	SENSOR_CHECK_ZERO(fd);
	p_mod = fd->module_data;
	SENSOR_CHECK_ZERO(p_mod);

	get_gpio_id(p_mod->of_node, &fd->gpio_tab.pwn, &fd->gpio_tab.reset, fd->sensor_id);
	SENSOR_PRINT_HIGH("SENSOR: pwn %d \n", fd->gpio_tab.pwn);
	if (0 == power_level) {
		gpio_direction_output(fd->gpio_tab.pwn, 0);
	} else {
		gpio_direction_output(fd->gpio_tab.pwn, 1);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
	}

	return SENSOR_K_SUCCESS;
}

LOCAL void _sensor_regulator_disable(struct sensor_file_tag *fd_handle, uint32_t *power_on_count, struct regulator * ptr_cam_regulator)
{
	SENSOR_CHECK_ZERO_VOID(fd_handle);

	while (*power_on_count > 0)
	{
		regulator_disable(ptr_cam_regulator);
		(*power_on_count)--;
	}
<<<<<<< HEAD

	SENSOR_PRINT("_sensor_regulator_disable : Sensor power off done : cnt = 0x%x, IO = %x, AVDD = %x, DVDD = %x, AF = %x\n", *power_on_count,
		s_p_sensor_mod->iopower_on_count, s_p_sensor_mod->avddpower_on_count, s_p_sensor_mod->dvddpower_on_count, s_p_sensor_mod->motpower_on_count);
=======
	SENSOR_PRINT("sensor pwr off done: cnt=0x%x, io=%x, av=%x, dv=%x, mo=%x \n", *power_on_count,
		fd_handle->iopower_on_count,
		fd_handle->avddpower_on_count,
		fd_handle->dvddpower_on_count,
		fd_handle->motpower_on_count);

>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
}

LOCAL int _sensor_regulator_enable(struct sensor_file_tag *fd_handle, uint32_t *power_on_count, struct regulator * ptr_cam_regulator)
{
	int err;
	SENSOR_CHECK_ZERO(fd_handle);

	err = regulator_enable(ptr_cam_regulator);
	(*power_on_count)++;

	SENSOR_PRINT("sensor pwr on done: cnt=0x%x, io=%x, av=%x, dv=%x, mo=%x \n", *power_on_count,
		fd_handle->iopower_on_count,
		fd_handle->avddpower_on_count,
		fd_handle->dvddpower_on_count,
		fd_handle->motpower_on_count);

	return err;
}

<<<<<<< HEAD
// AF power control
int sensor_k_set_voltage_cammot(uint32_t cammot_val)
{
	int err = 0;
	uint32_t volt_value = 0;
	SENSOR_CHECK_ZERO(s_p_sensor_mod);
=======
LOCAL int _sensor_k_get_voltage_value(uint32_t *val)
{
	uint32_t             volt_value = 0;
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g

	SENSOR_CHECK_ZERO(val);

<<<<<<< HEAD
	if (NULL == s_p_sensor_mod->cammot_regulator)
	{
		s_p_sensor_mod->cammot_regulator = regulator_get(NULL, REGU_NAME_CAMMOT);
		if (IS_ERR(s_p_sensor_mod->cammot_regulator))
		{
			SENSOR_PRINT_ERR("sensor_k_set_voltage_cammot : Get cammot fail\n");
			return SENSOR_K_FAIL;
		}
	}

	switch (cammot_val)
	{
		case SENSOR_VDD_2800MV:
			err = regulator_set_voltage(s_p_sensor_mod->cammot_regulator, SENSOER_VDD_2800MV, SENSOER_VDD_2800MV);
			volt_value = SENSOER_VDD_2800MV;
			if (err)
				SENSOR_PRINT_ERR("sensor_k_set_voltage_cammot : Set cammot 2.8 fail\n");
			break;

		case SENSOR_VDD_3000MV:
			err = regulator_set_voltage(s_p_sensor_mod->cammot_regulator, SENSOER_VDD_3000MV, SENSOER_VDD_3000MV);
			volt_value = SENSOER_VDD_3000MV;
			if (err)
				SENSOR_PRINT_ERR("sensor_k_set_voltage_cammot : Set cammot 3.0 fail\n");
			break;

#if defined (CONFIG_ARCH_SCX35)
		case SENSOR_VDD_3300MV:
			err = regulator_set_voltage(s_p_sensor_mod->cammot_regulator, SENSOER_VDD_3300MV, SENSOER_VDD_3300MV);
			volt_value = SENSOER_VDD_3300MV;
			if (err)
				SENSOR_PRINT_ERR("sensor_k_set_voltage_cammot : Set cammot 3.3 fail\n");
			break;
#else
		case SENSOR_VDD_2500MV:
			err = regulator_set_voltage(s_p_sensor_mod->cammot_regulator, SENSOER_VDD_2500MV, SENSOER_VDD_2500MV);
			volt_value = SENSOER_VDD_2500MV;
			if (err)
				SENSOR_PRINT_ERR("sensor_k_set_voltage_cammot : Set cammot 2.5 fail\n");
			break;
#endif

		case SENSOR_VDD_1800MV:
			err = regulator_set_voltage(s_p_sensor_mod->cammot_regulator, SENSOER_VDD_1800MV, SENSOER_VDD_1800MV);
			volt_value = SENSOER_VDD_1800MV;
			if (err)
				SENSOR_PRINT_ERR("sensor_k_set_voltage_cammot : Set cammot 1.8 fail\n");
			break;

		case SENSOR_VDD_CLOSED:
		case SENSOR_VDD_UNUSED:
		default:
			volt_value = 0;
			break;
	}

	if (err)
	{
		SENSOR_PRINT_ERR("sensor_k_set_voltage_cammot : Set cammot error(%d)\n", err);
		return SENSOR_K_FAIL;
	}

	if (0 != volt_value)
	{
		err = _sensor_regulator_enable(&s_p_sensor_mod->motpower_on_count, s_p_sensor_mod->cammot_regulator);
		if (err)
		{
			regulator_put(s_p_sensor_mod->cammot_regulator);
			s_p_sensor_mod->cammot_regulator = NULL;
			SENSOR_PRINT_ERR("sensor_k_set_voltage_cammot : Can't enable cammot\n");
			return SENSOR_K_FAIL;
		}
	}
	else
	{
		_sensor_regulator_disable(&s_p_sensor_mod->motpower_on_count, s_p_sensor_mod->cammot_regulator);
		regulator_put(s_p_sensor_mod->cammot_regulator);
		s_p_sensor_mod->cammot_regulator = NULL;
		SENSOR_PRINT("sensor_k_set_voltage_cammot : Disable cammot\n");
	}
=======
	switch (*val) {
	case SENSOR_VDD_3800MV:
		volt_value = SENSOER_VDD_3800MV;
		break;
	case SENSOR_VDD_3300MV:
		volt_value = SENSOER_VDD_3300MV;
		break;
	case SENSOR_VDD_3000MV:
		volt_value = SENSOER_VDD_3000MV;
		break;
	case SENSOR_VDD_2800MV:
		volt_value = SENSOER_VDD_2800MV;
		break;
	case SENSOR_VDD_2500MV:
		volt_value = SENSOER_VDD_2500MV;
		break;
	case SENSOR_VDD_1800MV:
		volt_value = SENSOER_VDD_1800MV;
		break;
	case SENSOR_VDD_1500MV:
		volt_value = SENSOER_VDD_1500MV;
		break;
	case SENSOR_VDD_1300MV:
		volt_value = SENSOER_VDD_1300MV;
		break;
	case SENSOR_VDD_1200MV:
		volt_value = SENSOER_VDD_1200MV;
		break;
	case SENSOR_VDD_CLOSED:
	case SENSOR_VDD_UNUSED:
	default:
		volt_value = 0;
		break;
	}

	*val = volt_value;

	SENSOR_PRINT("sensor get voltage val: %d \n", *val);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g

	return SENSOR_K_SUCCESS;
}

LOCAL int _sensor_k_set_voltage(struct sensor_file_tag *fd_handle, uint32_t val, uint32_t type)
{
<<<<<<< HEAD
	int err = 0;
	uint32_t volt_value = 0;
	SENSOR_CHECK_ZERO(s_p_sensor_mod);

	SENSOR_PRINT_HIGH("sensor set AVDD val %d\n", avdd_val);

#if defined(CONFIG_MACH_YOUNG33G)
	SENSOR_PRINT_HIGH("sensor_k_set_voltage_dvdd : sensor set AVDD gpio %d\n", s_p_sensor_mod->pin_main_camavdd_en);
	if (SENSOR_VDD_CLOSED == avdd_val)
	{
		gpio_direction_output(s_p_sensor_mod->pin_main_camavdd_en, 1);
		gpio_set_value(s_p_sensor_mod->pin_main_camavdd_en, 0);
	}
	else
	{
		gpio_direction_output(s_p_sensor_mod->pin_main_camavdd_en, 1);
		gpio_set_value(s_p_sensor_mod->pin_main_camavdd_en, 1);
	}
	return SENSOR_K_SUCCESS;
#endif

	if (NULL == s_p_sensor_mod->camavdd_regulator)
	{
		s_p_sensor_mod->camavdd_regulator = regulator_get(NULL, REGU_NAME_CAMAVDD);
		if (IS_ERR(s_p_sensor_mod->camavdd_regulator))
		{
			SENSOR_PRINT_ERR("sensor_k_set_voltage_avdd : Get avdd fail\n");
			return SENSOR_K_FAIL;
		}
	}

	switch (avdd_val)
	{
		case SENSOR_VDD_2800MV:
			err = regulator_set_voltage(s_p_sensor_mod->camavdd_regulator, SENSOER_VDD_2800MV, SENSOER_VDD_2800MV);
			volt_value = SENSOER_VDD_2800MV;
			if (err)
				SENSOR_PRINT_ERR("sensor_k_set_voltage_avdd : Set avdd to 2.8 fail\n");
			break;

		case SENSOR_VDD_3000MV:
			err = regulator_set_voltage(s_p_sensor_mod->camavdd_regulator, SENSOER_VDD_3000MV, SENSOER_VDD_3000MV);
			volt_value = SENSOER_VDD_3000MV;
			if (err)
				SENSOR_PRINT_ERR("sensor_k_set_voltage_avdd : Set avdd to 3.0 fail\n");
			break;

		case SENSOR_VDD_2500MV:
			err = regulator_set_voltage(s_p_sensor_mod->camavdd_regulator, SENSOER_VDD_2500MV, SENSOER_VDD_2500MV);
			volt_value = SENSOER_VDD_2500MV;
			if (err)
				SENSOR_PRINT_ERR("sensor_k_set_voltage_avdd : Set avdd to 2.5 fail\n");
			break;

		case SENSOR_VDD_1800MV:
			err = regulator_set_voltage(s_p_sensor_mod->camavdd_regulator, SENSOER_VDD_1800MV, SENSOER_VDD_1800MV);
			volt_value = SENSOER_VDD_1800MV;
			if (err)
				SENSOR_PRINT_ERR("sensor_k_set_voltage_avdd : Set avdd to 1.8 fail\n");
			break;

		case SENSOR_VDD_CLOSED:
		case SENSOR_VDD_UNUSED:
		default:
			volt_value = 0;
			break;
	}

	if (err)
	{
		SENSOR_PRINT_ERR("sensor_k_set_voltage_avdd : Set avdd error\n");
		return SENSOR_K_FAIL;
	}

	if (0 != volt_value)
	{
		err = _sensor_regulator_enable(&s_p_sensor_mod->avddpower_on_count, s_p_sensor_mod->camavdd_regulator);
		if (err)
		{
			regulator_put(s_p_sensor_mod->camavdd_regulator);
			s_p_sensor_mod->camavdd_regulator = NULL;
			SENSOR_PRINT_ERR("sensor_k_set_voltage_avdd : Can't enable avdd\n");
			return SENSOR_K_FAIL;
		}
	}
	else
	{
		_sensor_regulator_disable(&s_p_sensor_mod->avddpower_on_count, s_p_sensor_mod->camavdd_regulator);
		regulator_put(s_p_sensor_mod->camavdd_regulator);
		s_p_sensor_mod->camavdd_regulator = NULL;
		SENSOR_PRINT("sensor_k_set_voltage_avdd : Disable avdd\n");
	}

	return SENSOR_K_SUCCESS;
}

int sensor_k_set_voltage_dvdd(uint32_t dvdd_val)
{
	int err = 0;
	uint32_t volt_value = 0;
	SENSOR_CHECK_ZERO(s_p_sensor_mod);
	SENSOR_PRINT_HIGH("sensor_k_set_voltage_dvdd : Sensor set DVDD val %d\n", dvdd_val);

#if  defined(CONFIG_MACH_CORE3_W) || defined(CONFIG_MACH_J13G) || defined(CONFIG_MACH_GRANDNEOVE3G)	\
	|| defined(CONFIG_MACH_VIVALTO5MVE3G) || defined(CONFIG_MACH_GOYAVE3G_SWA)
	if (SENSOR_MAIN == _sensor_K_get_curId())
	{
		SENSOR_PRINT_HIGH("sensor_k_set_voltage_dvdd : sensor set DVDD gpio %d\n", s_p_sensor_mod->pin_main_camdvdd_en);
		if (SENSOR_VDD_CLOSED == dvdd_val)
		{
			gpio_direction_output(s_p_sensor_mod->pin_main_camdvdd_en, 1);
			gpio_set_value(s_p_sensor_mod->pin_main_camdvdd_en, 0);
		}
		else
		{
			gpio_direction_output(s_p_sensor_mod->pin_main_camdvdd_en, 1);
			gpio_set_value(s_p_sensor_mod->pin_main_camdvdd_en, 1);
		}
		return SENSOR_K_SUCCESS;
	}
#endif

#if defined(CONFIG_MACH_GTEL3G) || defined(CONFIG_MACH_GTELWIFI) || defined(CONFIG_MACH_YOUNG33G)
	SENSOR_PRINT_HIGH("sensor_k_set_voltage_dvdd : sensor set DVDD gpio %d\n", s_p_sensor_mod->pin_main_camdvdd_en);
	if (SENSOR_VDD_CLOSED == dvdd_val)
	{
		gpio_direction_output(s_p_sensor_mod->pin_main_camdvdd_en, 1);
		gpio_set_value(s_p_sensor_mod->pin_main_camdvdd_en, 0);
	}
	else
	{
		gpio_direction_output(s_p_sensor_mod->pin_main_camdvdd_en, 1);
		gpio_set_value(s_p_sensor_mod->pin_main_camdvdd_en, 1);
	}
	return SENSOR_K_SUCCESS;
#endif

	if (!s_p_sensor_mod->camdvdd_regulator)
	{
		switch (_sensor_K_get_curId())
		{
			case SENSOR_MAIN:
			{
				s_p_sensor_mod->camdvdd_regulator = regulator_get(NULL, REGU_NAME_CAMDVDD);
				break;
			}
			case SENSOR_SUB:
			{
				SENSOR_PRINT("_sensor_k_setvoltage_dvdd : dvdd_val = %d, this is sub camera\n", dvdd_val);
				s_p_sensor_mod->camdvdd_regulator = regulator_get(NULL, REGU_NAME_SUB_CAMDVDD);
				break;
			}
			default:
				s_p_sensor_mod->camdvdd_regulator = regulator_get(NULL, REGU_NAME_CAMDVDD);
				break;
		}

		if (IS_ERR(s_p_sensor_mod->camdvdd_regulator))
		{
			SENSOR_PRINT_ERR("_sensor_k_setvoltage_dvdd : Get DVDD fail\n");
			return SENSOR_K_FAIL;
		}
	}

	switch (dvdd_val)
	{

#if defined (CONFIG_ARCH_SCX35)
	case SENSOR_VDD_1200MV:
		err = regulator_set_voltage(s_p_sensor_mod->camdvdd_regulator, SENSOER_VDD_1200MV, SENSOER_VDD_1200MV);
		volt_value = SENSOER_VDD_1200MV;
		if (err)
			SENSOR_PRINT_ERR("_sensor_k_setvoltage_dvdd : Set DVDD to 1.2 fail\n");
		break;
#else
	case SENSOR_VDD_2800MV:
		err = regulator_set_voltage(s_p_sensor_mod->camdvdd_regulator, SENSOER_VDD_2800MV, SENSOER_VDD_2800MV);
		volt_value = SENSOER_VDD_2800MV;
		if (err)
			SENSOR_PRINT_ERR("_sensor_k_setvoltage_dvdd : Set DVDD to 2.8 fail\n");
		break;
#endif

	case SENSOR_VDD_1800MV:
		err = regulator_set_voltage(s_p_sensor_mod->camdvdd_regulator, SENSOER_VDD_1800MV, SENSOER_VDD_1800MV);
		volt_value = SENSOER_VDD_1800MV;
		if (err)
			SENSOR_PRINT_ERR("_sensor_k_setvoltage_dvdd : Set dvdd to 1.8 fail\n");
		break;

	case SENSOR_VDD_1500MV:
		err = regulator_set_voltage(s_p_sensor_mod->camdvdd_regulator, SENSOER_VDD_1500MV, SENSOER_VDD_1500MV);
		volt_value = SENSOER_VDD_1500MV;
		if (err)
			SENSOR_PRINT_ERR("_sensor_k_setvoltage_dvdd : Set dvdd to 1.5 fail\n");
		break;

	case SENSOR_VDD_1300MV:
		err = regulator_set_voltage(s_p_sensor_mod->camdvdd_regulator, SENSOER_VDD_1300MV, SENSOER_VDD_1300MV);
		volt_value = SENSOER_VDD_1300MV;
		if (err)
			SENSOR_PRINT_ERR("_sensor_k_setvoltage_dvdd : Set dvdd to 1.3 fail\n");
		break;

	case SENSOR_VDD_CLOSED:
	case SENSOR_VDD_UNUSED:
	default:
		volt_value = 0;
		break;
	}

	if (err)
	{
		SENSOR_PRINT_ERR("_sensor_k_setvoltage_dvdd : Set DVDD err %d!\n", err);
		return SENSOR_K_FAIL;
	}

	if (0 != volt_value)
	{
		err = _sensor_regulator_enable(&s_p_sensor_mod->dvddpower_on_count,  s_p_sensor_mod->camdvdd_regulator);
		if (err)
		{
			regulator_put(s_p_sensor_mod->camdvdd_regulator);
			s_p_sensor_mod->camdvdd_regulator = NULL;
			SENSOR_PRINT_ERR("_sensor_k_setvoltage_dvdd : Can't enable DVDD\n");
			return SENSOR_K_FAIL;
		}
	}
	else
	{
		_sensor_regulator_disable(&s_p_sensor_mod->dvddpower_on_count,  s_p_sensor_mod->camdvdd_regulator);
		regulator_put(s_p_sensor_mod->camdvdd_regulator);
		s_p_sensor_mod->camdvdd_regulator = NULL;
		SENSOR_PRINT("_sensor_k_setvoltage_dvdd : Disable DVDD\n");
	}
	return SENSOR_K_SUCCESS;
}

int sensor_k_set_voltage_iovdd(uint32_t iodd_val)
{
	int err = 0;
	uint32_t volt_value = 0;
	SENSOR_CHECK_ZERO(s_p_sensor_mod);
=======
	int                              ret = SENSOR_K_SUCCESS;
	uint32_t                         volt_value = 0;
	uint32_t                         *poweron_count = NULL;
	char                             *regu_name;
	struct sensor_module_tab_tag     *p_mod;
	struct regulator                 **p_regulator = NULL;

	SENSOR_CHECK_ZERO(fd_handle);
	p_mod = fd_handle->module_data;
	SENSOR_CHECK_ZERO(p_mod);

	SENSOR_PRINT("sensor set voltage val: %d \n", val);

	switch (type) {
	case REGU_CAMAVDD:
		p_regulator = &fd_handle->camavdd_regulator;
		poweron_count = &fd_handle->avddpower_on_count;
		break;

	case REGU_CAMDVDD:
		p_regulator = &fd_handle->camdvdd_regulator;
		poweron_count = &fd_handle->dvddpower_on_count;
		break;

	case REGU_CAMIOVDD:
		p_regulator = &fd_handle->camvio_regulator;
		poweron_count = &fd_handle->iopower_on_count;
		break;

	case REGU_CAMMOT:
		p_regulator = &fd_handle->cammot_regulator;
		poweron_count = &fd_handle->motpower_on_count;
		break;

	default:
		SENSOR_PRINT_ERR("error type = %d\n",type);
		break;
	}

	if ((NULL == poweron_count) || (NULL == p_regulator)) {
		SENSOR_PRINT_ERR("error param");
		return SENSOR_K_FAIL;
	}

	get_regulator_name(p_mod->of_node, type, fd_handle->sensor_id, &regu_name);
	if (NULL == *p_regulator) {
		*p_regulator = regulator_get(NULL, regu_name);
		if (IS_ERR(*p_regulator)) {
			SENSOR_PRINT_ERR("SENSOR:get regu.fail %s %d\n", regu_name, type);
			return SENSOR_K_FAIL;
		}
	}
	volt_value = val;
	_sensor_k_get_voltage_value(&volt_value);

	if (0 != volt_value) {
		ret = regulator_set_voltage(*p_regulator, volt_value, volt_value);
	}
	if (ret) {
		SENSOR_PRINT_ERR("SENSOR:set vol err %d %s %d!.\n", ret, regu_name, type);
		return SENSOR_K_FAIL;
	}

	if (0 != volt_value) {
		ret = _sensor_regulator_enable(fd_handle, poweron_count, *p_regulator);
		if (ret) {
			regulator_put(*p_regulator);
			*p_regulator = NULL;
			SENSOR_PRINT_ERR("SENSOR:can't en %d %s %d!.\n", ret, regu_name, type);
			return SENSOR_K_FAIL;
		}
	} else {
		_sensor_regulator_disable(fd_handle, poweron_count, *p_regulator);
		regulator_put(*p_regulator);
		*p_regulator = NULL;
		SENSOR_PRINT("SENSOR:dis regu.\n");
	}
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g

	SENSOR_PRINT_HIGH("SENSOR:set vol %d %s %d!.\n", volt_value, regu_name, type);

<<<<<<< HEAD
	if(NULL == s_p_sensor_mod->camvio_regulator)
	{
		s_p_sensor_mod->camvio_regulator = regulator_get(NULL, REGU_NAME_CAMVIO);
		if (IS_ERR(s_p_sensor_mod->camvio_regulator))
		{
			SENSOR_PRINT_ERR("sensor_k_set_voltage_iovdd : Get camvio fail\n");
			return SENSOR_K_FAIL;
		}
	}

	switch (iodd_val)
	{
		case SENSOR_VDD_2800MV:
			err = regulator_set_voltage(s_p_sensor_mod->camvio_regulator, SENSOER_VDD_2800MV, SENSOER_VDD_2800MV);
			volt_value = SENSOER_VDD_2800MV;
			if (err)
				SENSOR_PRINT_ERR("sensor_k_set_voltage_iovdd : Set camvio to 2.8 fail\n");
			break;

#if defined (CONFIG_ARCH_SCX35)
		case SENSOR_VDD_2500MV:
			err = regulator_set_voltage(s_p_sensor_mod->camvio_regulator, SENSOER_VDD_2500MV, SENSOER_VDD_2500MV);
			volt_value = SENSOER_VDD_2500MV;
			if (err)
				SENSOR_PRINT_ERR("sensor_k_set_voltage_iovdd : Set camvio to 2.5 fail\n");
			break;

		case SENSOR_VDD_1500MV:
			err = regulator_set_voltage(s_p_sensor_mod->camvio_regulator, SENSOER_VDD_1500MV, SENSOER_VDD_1500MV);
			volt_value = SENSOER_VDD_1500MV;
			if (err)
				SENSOR_PRINT_ERR("sensor_k_set_voltage_iovdd : Set camvio to 1.5 fail\n");
			break;
#else
		case SENSOR_VDD_3800MV:
			err = regulator_set_voltage(s_p_sensor_mod->camvio_regulator, SENSOER_VDD_3800MV, SENSOER_VDD_3800MV);
			volt_value = SENSOER_VDD_3800MV;
			if (err)
				SENSOR_PRINT_ERR("sensor_k_set_voltage_iovdd : Set camvio to 3.8 fail\n");
			break;

		case SENSOR_VDD_1200MV:
			err = regulator_set_voltage(s_p_sensor_mod->camvio_regulator, SENSOER_VDD_1200MV, SENSOER_VDD_1200MV);
			volt_value = SENSOER_VDD_1200MV;
			if (err)
				SENSOR_PRINT_ERR("sensor_k_set_voltage_iovdd : Set camvio to 1.2 fail\n");
			break;
#endif

		case SENSOR_VDD_1800MV:
			err = regulator_set_voltage(s_p_sensor_mod->camvio_regulator, SENSOER_VDD_1800MV, SENSOER_VDD_1800MV);
			volt_value = SENSOER_VDD_1800MV;
			if (err)
				SENSOR_PRINT_ERR("sensor_k_set_voltage_iovdd : Set camvio to 1.8 fail\n");
			break;

		case SENSOR_VDD_CLOSED:
		case SENSOR_VDD_UNUSED:
		default:
			volt_value = 0;
			break;
	}

	if (err)
	{
		SENSOR_PRINT_ERR("sensor_k_set_voltage_iovdd : Set camvio err\n");
		return SENSOR_K_FAIL;
	}

	if (0 != volt_value)
	{
		err = _sensor_regulator_enable(&s_p_sensor_mod->iopower_on_count, s_p_sensor_mod->camvio_regulator);
		if (err)
		{
			regulator_put(s_p_sensor_mod->camvio_regulator);
			s_p_sensor_mod->camvio_regulator = NULL;
			SENSOR_PRINT_ERR("sensor_k_set_voltage_iovdd : Can't enable camvio\n");
			return SENSOR_K_FAIL;
		}
	}
	else
	{
		_sensor_regulator_disable(&s_p_sensor_mod->iopower_on_count, s_p_sensor_mod->camvio_regulator);
		regulator_put(s_p_sensor_mod->camvio_regulator);
		s_p_sensor_mod->camvio_regulator = NULL;
		SENSOR_PRINT("sensor_k_set_voltage_iovdd : Disable camvio\n");
=======
	return ret;
}

int sensor_k_set_voltage_cammot(uint32_t *fd_handle, uint32_t cammot_val)
{
	return _sensor_k_set_voltage((struct sensor_file_tag *)fd_handle, cammot_val, REGU_CAMMOT);
}

int sensor_k_set_voltage_avdd(uint32_t *fd_handle, uint32_t avdd_val)
{
	return _sensor_k_set_voltage((struct sensor_file_tag *)fd_handle, avdd_val, REGU_CAMAVDD);
}

int sensor_k_set_voltage_dvdd(uint32_t *fd_handle, uint32_t dvdd_val)
{
	int                              gpio_id = 0;
	struct sensor_module_tab_tag     *p_mod;
	struct sensor_file_tag           *fd = (struct sensor_file_tag *)fd_handle;

	SENSOR_CHECK_ZERO(fd);
	p_mod = fd->module_data;
	SENSOR_CHECK_ZERO(p_mod);

	get_gpio_id_ex(p_mod->of_node, GPIO_CAMDVDD, &gpio_id, fd->sensor_id);
	if (SENSOR_DEV_0 == fd->sensor_id && 0 != gpio_id) {
		SENSOR_PRINT_HIGH("sensor set DVDD gpio %d\n", gpio_id);
		if (SENSOR_VDD_CLOSED == dvdd_val) {
			gpio_direction_output(gpio_id, 1);
			gpio_set_value(gpio_id, 0);
		} else {
			gpio_direction_output(gpio_id, 1);
			gpio_set_value(gpio_id, 1);
		}
		return SENSOR_K_SUCCESS;
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
	}

	return _sensor_k_set_voltage((struct sensor_file_tag *)fd_handle, dvdd_val, REGU_CAMDVDD);
}

int sensor_k_set_voltage_iovdd(uint32_t *fd_handle, uint32_t iodd_val)
{
	return _sensor_k_set_voltage((struct sensor_file_tag *)fd_handle, iodd_val, REGU_CAMIOVDD);
}

LOCAL int _select_sensor_mclk(struct sensor_file_tag *fd_handle, uint8_t clk_set, char **clk_src_name,
			uint8_t * clk_div)
{
<<<<<<< HEAD
	uint8_t i = 0;
	uint8_t j = 0;
	uint8_t mark_src = 0;
	uint8_t mark_div = 0;
	uint8_t mark_src_tmp = 0;
	int clk_tmp = NUMBER_MAX;
	int src_delta = NUMBER_MAX;
	int src_delta_min = NUMBER_MAX;
	int div_delta_min = NUMBER_MAX;
	SENSOR_CHECK_ZERO(s_p_sensor_mod);

	SENSOR_PRINT("_select_sensor_mclk : Sel mclk %d\n", clk_set);
=======
	uint8_t               i = 0;
	uint8_t               j = 0;
	uint8_t               mark_src = 0;
	uint8_t               mark_div = 0;
	uint8_t               mark_src_tmp = 0;
	int                   clk_tmp = NUMBER_MAX;
	int                   src_delta = NUMBER_MAX;
	int                   src_delta_min = NUMBER_MAX;
	int                   div_delta_min = NUMBER_MAX;
	SENSOR_CHECK_ZERO(fd_handle);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g

	SENSOR_PRINT("SENSOR sel mclk %d.\n", clk_set);
	if (clk_set > 96 || !clk_src_name || !clk_div) {
		return SENSOR_K_FAIL;
	}

	for (i = 0; i < SENSOR_MCLK_DIV_MAX; i++)
	{
		clk_tmp = (int)(clk_set * (i + 1));
		src_delta_min = NUMBER_MAX;
<<<<<<< HEAD
		for (j = 0; j < SENSOR_MCLK_SRC_NUM; j++)
		{
			src_delta = ABS(c_sensor_mclk_tab[j].clock - clk_tmp);
			if (src_delta < src_delta_min)
			{
=======
		for (j = 0; j < SENSOR_MCLK_SRC_NUM; j++) {
			src_delta = c_sensor_mclk_tab[j].clock - clk_tmp;
			src_delta = SENSOR_ABS(src_delta);
			if (src_delta < src_delta_min) {
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
				src_delta_min = src_delta;
				mark_src_tmp = j;
			}
		}

		if (src_delta_min < div_delta_min)
		{
			div_delta_min = src_delta_min;
			mark_src = mark_src_tmp;
			mark_div = i;
		}
	}
<<<<<<< HEAD

	SENSOR_PRINT("_select_sensor_mclk : src %d, div = %d\n", mark_src, mark_div);
=======
	SENSOR_PRINT("src %d, div=%d .\n", mark_src,
		mark_div);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g

	*clk_src_name = c_sensor_mclk_tab[mark_src].src_name;
	*clk_div = mark_div + 1;

	return SENSOR_K_SUCCESS;
}

int32_t _sensor_k_mipi_clk_en(struct sensor_file_tag *fd_handle, struct device_node *dn)
{
	int ret = 0;

	SENSOR_CHECK_ZERO(fd_handle);

<<<<<<< HEAD
	if (NULL == s_p_sensor_mod->mipi_clk)
	{
		s_p_sensor_mod->mipi_clk = parse_clk(dn,"clk_dcam_mipi");
	}

	if (IS_ERR(s_p_sensor_mod->mipi_clk))
	{
		printk("_sensor_k_mipi_clk_en : Get dcam mipi clk error \n");
		return -1;
	}
	else
	{
		ret = clk_enable(s_p_sensor_mod->mipi_clk);
		if (ret)
		{
			printk("_select_sensor_mclk : Enable dcam mipi clk error %d \n", ret);
=======
	if (NULL == fd_handle->mipi_clk) {
		fd_handle->mipi_clk = parse_clk(dn,"clk_dcam_mipi");
	}

	if (IS_ERR(fd_handle->mipi_clk)) {
		printk("SENSOR: get dcam mipi clk error \n");
		return -1;
	} else {
		ret = clk_enable(fd_handle->mipi_clk);
		if (ret) {
			printk("SENSOR: enable dcam mipi clk error %d \n", ret);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
			return -1;
		}
	}
	return ret;
}

int32_t _sensor_k_mipi_clk_dis(struct sensor_file_tag *fd_handle)
{
	SENSOR_CHECK_ZERO(fd_handle);

<<<<<<< HEAD
	if (s_p_sensor_mod->mipi_clk)
	{
		clk_disable(s_p_sensor_mod->mipi_clk);
		clk_put(s_p_sensor_mod->mipi_clk);
		s_p_sensor_mod->mipi_clk = NULL;
=======
	if (fd_handle->mipi_clk) {
		clk_disable(fd_handle->mipi_clk);
		clk_put(fd_handle->mipi_clk);
		fd_handle->mipi_clk = NULL;
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
	}
	return 0;
}

LOCAL int _sensor_k_set_mclk(struct sensor_file_tag *fd_handle, struct device_node *dn, uint32_t mclk)
{
<<<<<<< HEAD
	struct clk *clk_parent = NULL;
	int ret;
	char *clk_src_name = NULL;
	uint8_t clk_div;
	SENSOR_CHECK_ZERO(s_p_sensor_mod);

	SENSOR_PRINT_HIGH("sensor_k_set_mclk : Set mclk org = %d, clk = %d\n", s_p_sensor_mod->sensor_mclk, mclk);

	if ((0 != mclk) && (s_p_sensor_mod->sensor_mclk != mclk))
	{
		if (mclk > SENSOR_MAX_MCLK)
		{
			mclk = SENSOR_MAX_MCLK;
		}

		if (s_p_sensor_mod->sensor_mclk)
		{
			clk_disable(s_p_sensor_mod->ccir_clk);
		}

		if (SENSOR_K_SUCCESS != _select_sensor_mclk((uint8_t) mclk, &clk_src_name, &clk_div))
		{
			SENSOR_PRINT_ERR("sensor_k_set_mclk : Sensor_SetMCLK select clock source fail\n");
=======
	struct clk                *clk_parent = NULL;
	int                       ret;
	char                      *clk_src_name = NULL;
	uint8_t                   clk_div;
	SENSOR_CHECK_ZERO(fd_handle);

	SENSOR_PRINT_HIGH("SENSOR: set mclk org = %d, clk = %d\n",
				fd_handle->sensor_mclk, mclk);

	if ((0 != mclk) && (fd_handle->sensor_mclk != mclk)) {
		if (fd_handle->ccir_clk) {
			clk_disable(fd_handle->ccir_clk);
			SENSOR_PRINT("###sensor ccir clk off ok.\n");
		} else {
			fd_handle->ccir_clk = parse_clk(dn, SENSOR_CLK);
			if (IS_ERR(fd_handle->ccir_clk)) {
				SENSOR_PRINT_ERR("###: Failed: Can't get clock [ccir_mclk]!\n");
				SENSOR_PRINT_ERR("###: s_sensor_clk = %p.\n",fd_handle->ccir_clk);
			} else {
				SENSOR_PRINT("###sensor ccir clk get ok.\n");
			}
		}
		if (mclk > SENSOR_MAX_MCLK) {
			mclk = SENSOR_MAX_MCLK;
		}
		if (SENSOR_K_SUCCESS != _select_sensor_mclk(fd_handle, (uint8_t) mclk, &clk_src_name, &clk_div)) {
			SENSOR_PRINT_ERR("SENSOR:Sensor_SetMCLK select clock source fail.\n");
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
			return -EINVAL;
		}
		SENSOR_PRINT("clk_src_name=%s, clk_div=%d \n", clk_src_name, clk_div);

		clk_parent = clk_get(NULL, clk_src_name);
<<<<<<< HEAD
		if (!clk_parent)
		{
			SENSOR_PRINT_ERR("sensor_k_set_mclk : clock : failed to get clock [%s] by clk_get()!\n", clk_src_name);
=======
		if (!clk_parent) {
			SENSOR_PRINT_ERR("###:clock: failed to get clock [%s] by clk_get()!\n", clk_src_name);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
			return -EINVAL;
		}
		SENSOR_PRINT("clk_get clk_src_name=%s done\n", clk_src_name);

<<<<<<< HEAD
		SENSOR_PRINT("sensor_k_set_mclk : clk_get clk_src_name = %s done\n", clk_src_name);

		ret = clk_set_parent(s_p_sensor_mod->ccir_clk, clk_parent);
		if (ret)
		{
			SENSOR_PRINT_ERR("sensor_k_set_mclk : clock : clk_set_parent() failed!parent \n");
=======
		ret = clk_set_parent(fd_handle->ccir_clk, clk_parent);
		if (ret) {
			SENSOR_PRINT_ERR("###:clock: clk_set_parent() failed!parent \n");
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
			return -EINVAL;
		}
		SENSOR_PRINT("clk_set_parent s_ccir_clk=%s done\n", (char *)(fd_handle->ccir_clk));

<<<<<<< HEAD
		SENSOR_PRINT("sensor_k_set_mclk : clk_set_parent s_ccir_clk = %s done\n", (char *)(s_p_sensor_mod->ccir_clk));

		ret = clk_set_rate(s_p_sensor_mod->ccir_clk, (mclk * SENOR_CLK_M_VALUE));
		if (ret)
		{
			SENSOR_PRINT_ERR("sensor_k_set_mclk : clock : clk_set_rate failed!\n");
=======
		ret = clk_set_rate(fd_handle->ccir_clk, (mclk * SENOR_CLK_M_VALUE));
		if (ret) {
			SENSOR_PRINT_ERR("###:clock: clk_set_rate failed!\n");
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
			return -EINVAL;
		}
		SENSOR_PRINT("clk_set_rate s_ccir_clk=%s done\n", (char *)(fd_handle->ccir_clk));

		ret = clk_enable(fd_handle->ccir_clk);
		if (ret) {
			SENSOR_PRINT_ERR("###:clock: clk_enable() failed!\n");
		} else {
			SENSOR_PRINT("######ccir enable clk ok\n");
		}

<<<<<<< HEAD
		ret = clk_enable(s_p_sensor_mod->ccir_clk);
		if (ret)
		{
			SENSOR_PRINT_ERR("sensor_k_set_mclk : clock : clk_enable() failed!\n");
		}
		else
		{
			SENSOR_PRINT("sensor_k_set_mclk : ccir enable clk ok\n");
		}

		s_p_sensor_mod->sensor_mclk = mclk;
		SENSOR_PRINT("sensor_k_set_mclk : Set mclk %d Hz\n", s_p_sensor_mod->sensor_mclk);
	}
	else if (0 == mclk)
	{
		if (s_p_sensor_mod->sensor_mclk)
		{
			if (s_p_sensor_mod->ccir_clk)
			{
				clk_disable(s_p_sensor_mod->ccir_clk);
				SENSOR_PRINT("sensor_k_set_mclk : sensor clk disable ok\n");
			}
			s_p_sensor_mod->sensor_mclk = 0;
		}
		SENSOR_PRINT("sensor_k_set_mclk : Disable MCLK !!!");
	}
	else
	{
		SENSOR_PRINT("sensor_k_set_mclk : Do nothing !!");
	}
	SENSOR_PRINT_HIGH("sensor_k_set_mclk : X\n");

=======
		if (NULL == fd_handle->ccir_enable_clk) {
			fd_handle->ccir_enable_clk	= parse_clk(dn,"clk_ccir");
			if (IS_ERR(fd_handle->ccir_enable_clk)) {
				SENSOR_PRINT_ERR("###: Failed: Can't get clock [clk_ccir]!\n");
				SENSOR_PRINT_ERR("###: ccir_enable_clk = %p.\n", fd_handle->ccir_enable_clk);
				return -EINVAL;
			} else {
				SENSOR_PRINT("###sensor ccir_enable_clk clk_get ok.\n");
			}
			ret = clk_enable(fd_handle->ccir_enable_clk);
			if (ret) {
				SENSOR_PRINT_ERR("###:clock: clk_enable() failed!\n");
			} else {
				SENSOR_PRINT("###ccir enable clk ok\n");
			}
		}

		fd_handle->sensor_mclk = mclk;
		SENSOR_PRINT("SENSOR: set mclk %d Hz.\n",
			fd_handle->sensor_mclk);
	} else if (0 == mclk) {
		if (fd_handle->ccir_clk) {
			clk_disable(fd_handle->ccir_clk);
			SENSOR_PRINT("###sensor clk disable ok.\n");
			clk_put(fd_handle->ccir_clk);
			SENSOR_PRINT("###sensor clk put ok.\n");
			fd_handle->ccir_clk = NULL;
		}

		if (fd_handle->ccir_enable_clk) {
			clk_disable(fd_handle->ccir_enable_clk);
			SENSOR_PRINT("###sensor clk disable ok.\n");
			clk_put(fd_handle->ccir_enable_clk);
			SENSOR_PRINT("###sensor clk put ok.\n");
			fd_handle->ccir_enable_clk = NULL;
		}
		fd_handle->sensor_mclk = 0;
		SENSOR_PRINT("SENSOR: Disable MCLK !!!");
	} else {
		SENSOR_PRINT("SENSOR: Do nothing !! ");
	}
	SENSOR_PRINT_HIGH("SENSOR: set mclk X\n");
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
	return 0;
}

int sensor_k_set_mclk(uint32_t *fd_handle, uint32_t mclk)
{
<<<<<<< HEAD
	switch (_sensor_K_get_curId())
	{
		case SENSOR_MAIN:
		{
			SENSOR_PRINT_HIGH("_sensor_k_reset : MAIN RST reset_val = %d\n", level);
			gpio_direction_output(s_p_sensor_mod->pin_main_reset, level);
			gpio_set_value(s_p_sensor_mod->pin_main_reset, level);
			SLEEP_MS(width);
			gpio_set_value(s_p_sensor_mod->pin_main_reset, !level);
			mdelay(1);
			break;
		}
		case SENSOR_SUB:
		{
			SENSOR_PRINT_HIGH("_sensor_k_reset : Sub RST reset_val = %d\n", level);
			gpio_direction_output(s_p_sensor_mod->pin_sub_reset, level);
			gpio_set_value(s_p_sensor_mod->pin_sub_reset, level);
			SLEEP_MS(width);
			gpio_set_value(s_p_sensor_mod->pin_sub_reset, !level);
			mdelay(1);
			break;
		}
		default:
			break;
	}
	return SENSOR_K_SUCCESS;
=======
	struct sensor_module_tab_tag    *p_mod;
	struct sensor_file_tag          *fd = (struct sensor_file_tag *)fd_handle;

	SENSOR_CHECK_ZERO(fd);
	p_mod = fd->module_data;
	SENSOR_CHECK_ZERO(p_mod);

	return _sensor_k_set_mclk(fd, p_mod->of_node, mclk);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
}

LOCAL int _sensor_k_reset(struct sensor_file_tag *fd_handle, uint32_t level, uint32_t width)
{
	struct sensor_module_tab_tag    *p_mod;

<<<<<<< HEAD
=======
	SENSOR_CHECK_ZERO(fd_handle);
	p_mod = fd_handle->module_data;
	SENSOR_CHECK_ZERO(p_mod);

	get_gpio_id(p_mod->of_node, &fd_handle->gpio_tab.pwn, &fd_handle->gpio_tab.reset, fd_handle->sensor_id);
	SENSOR_PRINT_HIGH("SENSOR: reset val %d id %d reset %d\n",level, fd_handle->sensor_id, fd_handle->gpio_tab.reset);

	gpio_direction_output(fd_handle->gpio_tab.reset, level);
	gpio_set_value(fd_handle->gpio_tab.reset, level);
	SLEEP_MS(width);
	gpio_set_value(fd_handle->gpio_tab.reset, !level);

>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
	return SENSOR_K_SUCCESS;
}

int sensor_k_sensor_sel(uint32_t *fd_handle, uint32_t sensor_id)
{
	struct sensor_file_tag          *fd = (struct sensor_file_tag *)fd_handle;
	SENSOR_CHECK_ZERO(fd);
	fd->sensor_id = sensor_id;

	return SENSOR_K_SUCCESS;
}

int sensor_k_sensor_desel(struct sensor_file_tag *fd_handle, uint32_t sensor_id)
{
<<<<<<< HEAD
	switch (_sensor_K_get_curId())
	{
		case SENSOR_MAIN:
		{
			SENSOR_PRINT_HIGH("sensor_k_set_rst_level : Main RST level is %d, rst pin = %d \n", plus_level, s_p_sensor_mod->pin_main_reset);
			gpio_direction_output(s_p_sensor_mod->pin_main_reset, plus_level);
			gpio_set_value(s_p_sensor_mod->pin_main_reset, plus_level);
			break;
		}
		case SENSOR_SUB:
		{
			SENSOR_PRINT_HIGH("sensor_k_set_rst_level : Sub RST level is %d, rst pin = %d \n", plus_level, s_p_sensor_mod->pin_sub_reset);
			gpio_direction_output(s_p_sensor_mod->pin_sub_reset, plus_level);
			gpio_set_value(s_p_sensor_mod->pin_sub_reset, plus_level);
			break;
		}
		default:
		break;
	}
=======
	SENSOR_CHECK_ZERO(fd_handle);
	//fd_handle->sensor_id = SENSOR_ID_MAX;

	SENSOR_PRINT_HIGH("sensor desel %d OK.\n", sensor_id);

>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
	return SENSOR_K_SUCCESS;
}

int sensor_k_set_rst_level(uint32_t *fd_handle, uint32_t plus_level)
{
<<<<<<< HEAD
	uint8_t cmd[2] = { 0 };
	uint16_t w_cmd_num = 0;
	uint16_t r_cmd_num = 0;
	uint8_t buf_r[2] = { 0 };
	int32_t ret = SENSOR_K_SUCCESS;
	struct i2c_msg msg_r[2];
	uint16_t reg_addr;
	int i;
	SENSOR_CHECK_ZERO(s_p_sensor_mod);
=======
	struct sensor_module_tab_tag    *p_mod;
	struct sensor_file_tag          *fd = (struct sensor_file_tag *)fd_handle;

	SENSOR_CHECK_ZERO(fd);
	p_mod = fd->module_data;
	SENSOR_CHECK_ZERO(p_mod);

	get_gpio_id(p_mod->of_node, &fd->gpio_tab.pwn, &fd->gpio_tab.reset, fd->sensor_id);
	SENSOR_PRINT("SENSOR: set rst lvl: lvl %d, rst pin %d \n", plus_level, fd->gpio_tab.reset);

	gpio_direction_output(fd->gpio_tab.reset, plus_level);
	gpio_set_value(fd->gpio_tab.reset, plus_level);

	return SENSOR_K_SUCCESS;
}

LOCAL int _Sensor_K_ReadReg(struct sensor_file_tag *fd_handle, struct sensor_reg_bits_tag *pReg)
{
	uint8_t                       cmd[2] = { 0 };
	uint16_t                      w_cmd_num = 0;
	uint16_t                      r_cmd_num = 0;
	uint8_t                       buf_r[2] = { 0 };
	int32_t                       ret = SENSOR_K_SUCCESS;
	struct i2c_msg                msg_r[2];
	uint16_t                      reg_addr;
	int                           i;
	struct sensor_module_tab_tag  *p_mod;
	SENSOR_CHECK_ZERO(fd_handle);

	p_mod = fd_handle->module_data;
	SENSOR_CHECK_ZERO(p_mod);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g

	reg_addr = pReg->reg_addr;

	if (SENSOR_I2C_REG_16BIT ==(pReg->reg_bits & SENSOR_I2C_REG_16BIT))
	{
		cmd[w_cmd_num++] = (uint8_t) ((reg_addr >> 8) & SENSOR_LOW_EIGHT_BIT);
		cmd[w_cmd_num++] = (uint8_t) (reg_addr & SENSOR_LOW_EIGHT_BIT);
	}
	else
	{
		cmd[w_cmd_num++] = (uint8_t) reg_addr;
	}

	if (SENSOR_I2C_VAL_16BIT == (pReg->reg_bits & SENSOR_I2C_VAL_16BIT))
	{
		r_cmd_num = SENSOR_CMD_BITS_16;
	}
	else
	{
		r_cmd_num = SENSOR_CMD_BITS_8;
	}

<<<<<<< HEAD
	for (i = 0; i < SENSOR_I2C_OP_TRY_NUM; i++)
	{
		msg_r[0].addr = s_p_sensor_mod->cur_i2c_client->addr;
=======
	for (i = 0; i < SENSOR_I2C_OP_TRY_NUM; i++) {
		msg_r[0].addr = p_mod->sensor_dev_tab[fd_handle->sensor_id].cur_i2c_client->addr;
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
		msg_r[0].flags = 0;
		msg_r[0].buf = cmd;
		msg_r[0].len = w_cmd_num;
		msg_r[1].addr = p_mod->sensor_dev_tab[fd_handle->sensor_id].cur_i2c_client->addr;
		msg_r[1].flags = I2C_M_RD;
		msg_r[1].buf = buf_r;
		msg_r[1].len = r_cmd_num;
<<<<<<< HEAD
		ret = i2c_transfer(s_p_sensor_mod->cur_i2c_client->adapter, msg_r, 2);

		if (ret != 2)
		{
			SENSOR_PRINT_ERR("_Sensor_K_ReadReg : Read reg fail, ret %d, addr 0x%x, reg_addr 0x%x\n", ret, s_p_sensor_mod->cur_i2c_client->addr, reg_addr);
=======
		ret = i2c_transfer(p_mod->sensor_dev_tab[fd_handle->sensor_id].cur_i2c_client->adapter, msg_r, 2);
		if (ret != 2) {
			SENSOR_PRINT_ERR("SENSOR:read reg fail, ret %d, addr 0x%x, reg_addr 0x%x \n",
					ret, p_mod->sensor_dev_tab[fd_handle->sensor_id].cur_i2c_client->addr,reg_addr);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
			SLEEP_MS(20);
			ret = SENSOR_K_FAIL;
		} else {
			pReg->reg_value = (r_cmd_num == 1) ? (uint16_t) buf_r[0] : (uint16_t) ((buf_r[0] << 8) + buf_r[1]);
			ret = SENSOR_K_SUCCESS;
			break;
		}
	}

	return ret;
}

LOCAL int _Sensor_K_WriteReg(struct sensor_file_tag *fd_handle, struct sensor_reg_bits_tag *pReg)
{
<<<<<<< HEAD
	uint8_t cmd[4] = { 0 };
	uint32_t index = 0;
	uint32_t cmd_num = 0;
	struct i2c_msg msg_w;
	int32_t ret = SENSOR_K_SUCCESS;
	uint16_t subaddr;
	uint16_t data;
	int i;
	uint16_t delay_value = 0x0000;	
	SENSOR_CHECK_ZERO(s_p_sensor_mod);
=======
	uint8_t                       cmd[4] = { 0 };
	uint32_t                      index = 0;
	uint32_t                      cmd_num = 0;
	struct i2c_msg                msg_w;
	int32_t                       ret = SENSOR_K_SUCCESS;
	uint16_t                      subaddr;
	uint16_t                      data;
	int                           i;
	struct sensor_module_tab_tag  *p_mod;
	SENSOR_CHECK_ZERO(fd_handle);

	p_mod = fd_handle->module_data;
	SENSOR_CHECK_ZERO(p_mod);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g

	subaddr = pReg->reg_addr;
	data = pReg->reg_value;

	if (SENSOR_I2C_REG_16BIT ==(pReg->reg_bits & SENSOR_I2C_REG_16BIT))
	{
		cmd[cmd_num++] = (uint8_t) ((subaddr >> 8) & SENSOR_LOW_EIGHT_BIT);
		index++;
		cmd[cmd_num++] =  (uint8_t) (subaddr & SENSOR_LOW_EIGHT_BIT);
		index++;
	}
	else
	{
		cmd[cmd_num++] = (uint8_t) subaddr;
		index++;
	}

	if (SENSOR_I2C_VAL_16BIT == (pReg->reg_bits & SENSOR_I2C_VAL_16BIT))
	{
		cmd[cmd_num++] = (uint8_t) ((data >> 8) & SENSOR_LOW_EIGHT_BIT);
		index++;
		cmd[cmd_num++] = (uint8_t) (data & SENSOR_LOW_EIGHT_BIT);
		index++;
	}
	else
	{
		cmd[cmd_num++] = (uint8_t) data;
		index++;
	}

<<<<<<< HEAD
	if(s_p_sensor_mod->cur_i2c_client->addr == 0x0028)//For only SR541, need check each sensor
			delay_value = 0xdddd;
		else
		#if defined(CONFIG_MACH_GTEL3G) || defined(CONFIG_MACH_GTELWIFI)
			if (SENSOR_MAIN == _sensor_K_get_curId())
				delay_value = 0xffff;
			else
				delay_value = 0xff;
		#else
			delay_value = 0xffff;
		#endif
		
	#if defined(CONFIG_MACH_GTEL3G) || defined(CONFIG_MACH_GTELWIFI)
	if ((delay_value != subaddr) || (delay_value==data))
	{
	#else
	if ((delay_value != subaddr))
	{
	#endif	
	
		for (i = 0; i < SENSOR_I2C_OP_TRY_NUM; i++)
		{
			msg_w.addr = s_p_sensor_mod->cur_i2c_client->addr;
			msg_w.flags = 0;
			msg_w.buf = cmd;
			msg_w.len = index;
			ret = i2c_transfer(s_p_sensor_mod->cur_i2c_client->adapter, &msg_w, 1);
			if (ret != 1)
			{
				SENSOR_PRINT_ERR("_Sensor_K_WriteReg : Failed : I2C Addr = %x, addr = %x, value = %x, bit = %d\n",
					s_p_sensor_mod->cur_i2c_client->addr, pReg->reg_addr, pReg->reg_value, pReg->reg_bits);
=======
	if (SENSOR_WRITE_DELAY != subaddr) {
		for (i = 0; i < SENSOR_I2C_OP_TRY_NUM; i++) {
			msg_w.addr = p_mod->sensor_dev_tab[fd_handle->sensor_id].cur_i2c_client->addr;
			msg_w.flags = 0;
			msg_w.buf = cmd;
			msg_w.len = index;
			ret = i2c_transfer(p_mod->sensor_dev_tab[fd_handle->sensor_id].cur_i2c_client->adapter, &msg_w, 1);
			if (ret != 1) {
				SENSOR_PRINT_ERR("_Sensor_K_WriteReg failed:i2cAddr=%x, addr=%x, value=%x, bit=%d \n",
						p_mod->sensor_dev_tab[fd_handle->sensor_id].cur_i2c_client->addr, pReg->reg_addr, pReg->reg_value, pReg->reg_bits);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
				ret = SENSOR_K_FAIL;
				continue;
			} else {
				ret = SENSOR_K_SUCCESS;
				break;
			}
		}
<<<<<<< HEAD
	}
	else
	{
		#if defined(CONFIG_MACH_GTEL3G) || defined(CONFIG_MACH_GTELWIFI)
			if (SENSOR_MAIN == _sensor_K_get_curId())
				SLEEP_MS(data);
			else
				SLEEP_MS(data*10);
		#else
=======
	} else {
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
		SLEEP_MS(data);
		#endif
	}
<<<<<<< HEAD
	return ret;
}

enum cmr_flash_status {
	FLASH_CLOSE	= 0x0,
	FLASH_OPEN		= 0x1,
	FLASH_TORCH	= 0x2, /* User only set flash to close/open/torch state */
	FLASH_AUTO	= 0x3,
	FLASH_CLOSE_AFTER_OPEN	= 0x10, /* Following is set to sensor */
	FLASH_HIGH_LIGHT			= 0x11,
	FLASH_OPEN_ON_RECORDING	= 0x22,
	FLASH_CLOSE_AFTER_AUTOFOCUS	= 0x30,
	FLASH_STATUS_MAX
};

LOCAL int _sensor_k_set_flash(uint32_t flash_mode)
{
	printk("_sensor_k_set_flash : flash_mode 0x%x\n", flash_mode);

	if(flash_torch_status==1)
		return 0;

	if(!flash_status)
	{
		switch (flash_mode)
		{
			case FLASH_OPEN: /* Flash on */
			case FLASH_TORCH: /* For torch */
				sprd_flash_on();
				break;

			case FLASH_HIGH_LIGHT:
				sprd_flash_high_light();
				break;

			case FLASH_CLOSE_AFTER_OPEN: /* Close flash */
			case FLASH_CLOSE_AFTER_AUTOFOCUS:
			case FLASH_CLOSE:
				sprd_flash_close();
				break;

			default:
				printk("_sensor_k_set_flash : Unknow mode : Flash_mode 0x%x\n", flash_mode);
				break;
		}
	}
	return 0;
=======

	return ret;
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
}

LOCAL int _sensor_k_get_flash_level(struct sensor_file_tag *fd_handle, struct sensor_flash_level *level)
{
	level->low_light  = SPRD_FLASH_LOW_CUR;
	level->high_light = SPRD_FLASH_HIGH_CUR;
<<<<<<< HEAD
	SENSOR_PRINT("_sensor_k_get_flash_level : Sensor get flash level : low %d, high %d\n", level->low_light, level->high_light);
=======

	SENSOR_PRINT("Sensor get flash lvl: low %d, high %d \n", level->low_light, level->high_light);

>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
	return SENSOR_K_SUCCESS;
}

int _sensor_burst_write_init(struct sensor_file_tag *fd_handle, struct sensor_reg_tag *p_reg_table, uint32_t init_table_size);

LOCAL int _sensor_k_wr_regtab(struct sensor_file_tag *fd_handle, struct sensor_reg_tab_tag *pRegTab)
{
<<<<<<< HEAD
	char *pBuff = PNULL;
	uint32_t cnt = pRegTab->reg_count;
	int ret = SENSOR_K_SUCCESS;
	uint32_t size;
	SENSOR_REG_T_PTR sensor_reg_ptr;
	SENSOR_REG_BITS_T reg_bit;
	uint32_t i;
	int rettmp;
	struct timeval time1, time2;

	do_gettimeofday(&time1);

	size = cnt * sizeof(SENSOR_REG_T);
	pBuff = _sensor_k_kmalloc(size);
	if (PNULL == pBuff)
	{
		ret = SENSOR_K_FAIL;
		SENSOR_PRINT_ERR("_sensor_k_wr_regtab : Alloc fail, cnt %d, size %d\n", cnt, size);
		goto _Sensor_K_WriteRegTab_return;
	}
	else
	{
		SENSOR_PRINT("_sensor_k_wr_regtab : Alloc success, cnt %d, size %d \n",cnt, size);
=======
	char                   *pBuff = PNULL;
	uint32_t               cnt = pRegTab->reg_count;
	int                    ret = SENSOR_K_SUCCESS;
	uint32_t               size;
	struct sensor_reg_tag  *sensor_reg_ptr;
	struct sensor_reg_bits_tag reg_bit;
	uint32_t               i;
	int                    rettmp;
	struct timeval         time1, time2;

	do_gettimeofday(&time1);

	size = cnt*sizeof(struct sensor_reg_tag);
	pBuff = _sensor_k_malloc(fd_handle, size);
	if (PNULL == pBuff) {
		ret = SENSOR_K_FAIL;
		SENSOR_PRINT_ERR("sensor W RegTab err:alloc fail, cnt %d, size %d\n", cnt, size);
		goto _Sensor_K_WriteRegTab_return;
	} else {
		SENSOR_PRINT("sensor W RegTab: alloc success, cnt %d, size %d \n",cnt, size);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
	}

	if (copy_from_user(pBuff, pRegTab->sensor_reg_tab_ptr, size))
	{
		ret = SENSOR_K_FAIL;
<<<<<<< HEAD
		SENSOR_PRINT_ERR("_sensor_k_wr_regtab : Copy user fail, size %d\n", size);
=======
		SENSOR_PRINT_ERR("sensor w err:copy user fail, size %d \n", size);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
		goto _Sensor_K_WriteRegTab_return;
	}

	sensor_reg_ptr = (struct sensor_reg_tag *)pBuff;

	if (0 == pRegTab->burst_mode)
	{
		for ( i = 0 ; i < cnt ; i++)
		{
			reg_bit.reg_addr  = sensor_reg_ptr[i].reg_addr;
			reg_bit.reg_value = sensor_reg_ptr[i].reg_value;
			reg_bit.reg_bits  = pRegTab->reg_bits;

			rettmp = _Sensor_K_WriteReg(fd_handle, &reg_bit);
			if(SENSOR_K_FAIL == rettmp)
				ret = SENSOR_K_FAIL;
		}
	} else if (SENSOR_I2C_BUST_NB == pRegTab->burst_mode) {
		printk("CAM %s, Line %d, burst_mode=%d, cnt=%d, start \n", __FUNCTION__, __LINE__, pRegTab->burst_mode, cnt);
		ret = _sensor_burst_write_init(fd_handle, sensor_reg_ptr, pRegTab->reg_count);
		printk("CAM %s, Line %d, burst_mode=%d, cnt=%d end\n", __FUNCTION__, __LINE__, pRegTab->burst_mode, cnt);
	}


_Sensor_K_WriteRegTab_return:
	if (PNULL != pBuff)
		_sensor_k_free(fd_handle, pBuff);

	do_gettimeofday(&time2);
	SENSOR_PRINT("sensor w RegTab: done, ret %d, cnt %d, time %d us \n", ret, cnt,
		(uint32_t)((time2.tv_sec - time1.tv_sec)*1000000+(time2.tv_usec - time1.tv_usec)));
	return ret;
}

LOCAL int _sensor_k_set_i2c_clk(struct sensor_file_tag *fd_handle, uint32_t clock)
{
	sprd_i2c_ctl_chg_clk(SENSOR_I2C_ID, clock);
<<<<<<< HEAD
	SENSOR_PRINT("_sensor_k_set_i2c_clk : Sensor set i2c clk %d\n", clock);
=======
	SENSOR_PRINT("sensor set i2c clk %d  \n", clock);

>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
	return SENSOR_K_SUCCESS;
}

LOCAL int _sensor_k_wr_i2c(struct sensor_file_tag *fd_handle, struct sensor_i2c_tag *pI2cTab)
{
<<<<<<< HEAD
	char *pBuff = PNULL;
	struct i2c_msg msg_w;
	uint32_t cnt = pI2cTab->i2c_count;
	int ret = SENSOR_K_FAIL;
	SENSOR_CHECK_ZERO(s_p_sensor_mod);

	pBuff = _sensor_k_kmalloc(cnt);
	if (PNULL == pBuff)
	{
		SENSOR_PRINT_ERR("_sensor_k_wr_i2c : Sensor W I2C ERR : Alloc fail, size %d\n", cnt);
=======
	char                          *pBuff = PNULL;
	struct i2c_msg                msg_w;
	uint32_t                      cnt = pI2cTab->i2c_count;
	int                           ret = SENSOR_K_FAIL;
	struct sensor_module_tab_tag  *p_mod;
	SENSOR_CHECK_ZERO(fd_handle);

	p_mod = fd_handle->module_data;
	SENSOR_CHECK_ZERO(p_mod);

	pBuff = _sensor_k_malloc(fd_handle, cnt);
	if (PNULL == pBuff) {
		SENSOR_PRINT_ERR("sensor W I2C ERR: alloc fail, size %d\n", cnt);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
		goto sensor_k_writei2c_return;
	} else {
		SENSOR_PRINT("sensor W I2C: alloc success, size %d\n", cnt);
	}

	if (copy_from_user(pBuff, pI2cTab->i2c_data, cnt)) {
		SENSOR_PRINT_ERR("sensor W I2C ERR: copy user fail, size %d \n", cnt);
		goto sensor_k_writei2c_return;
	}

	msg_w.addr = pI2cTab->slave_addr;
	msg_w.flags = 0;
	msg_w.buf = pBuff;
	msg_w.len = cnt;

<<<<<<< HEAD
	ret = i2c_transfer(s_p_sensor_mod->cur_i2c_client->adapter, &msg_w, 1);
	if (ret != 1)
	{
		SENSOR_PRINT_ERR("_sensor_k_wr_i2c : Write reg fail, ret : %d, addr : 0x%x\n", ret, msg_w.addr);
	}
	else
	{
=======
	ret = i2c_transfer(p_mod->sensor_dev_tab[fd_handle->sensor_id].cur_i2c_client->adapter, &msg_w, 1);
	if (ret != 1) {
		SENSOR_PRINT_ERR("SENSOR: w reg fail, ret: %d, addr: 0x%x\n",
		ret, msg_w.addr);
	} else {
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
		ret = SENSOR_K_SUCCESS;
	}

sensor_k_writei2c_return:
	if(PNULL != pBuff)
		_sensor_k_free(fd_handle, pBuff);

	SENSOR_PRINT("sensor w done, ret %d \n", ret);
	return ret;
}

LOCAL int _sensor_k_rd_i2c(struct sensor_file_tag *fd_handle, struct sensor_i2c_tag *pI2cTab)
{
<<<<<<< HEAD
	struct i2c_msg msg_r[2];
	int i;
	char *pBuff = PNULL;
	uint32_t cnt = pI2cTab->i2c_count;
	int ret = SENSOR_K_FAIL;
	uint16_t read_num = cnt;
=======
	struct i2c_msg	   msg_r[2];
	int                i;
	char               *pBuff = PNULL;
	uint32_t           cnt = pI2cTab->i2c_count;
	int                ret = SENSOR_K_FAIL;
	uint16_t           read_num = cnt;
	struct sensor_module_tab_tag  *p_mod;
	SENSOR_CHECK_ZERO(fd_handle);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g

	p_mod = fd_handle->module_data;
	SENSOR_CHECK_ZERO(p_mod);

	/*alloc buffer */
	pBuff = _sensor_k_malloc(fd_handle, cnt);
	if (PNULL == pBuff) {
		ret = SENSOR_K_FAIL;
		SENSOR_PRINT_ERR("sensor rd I2C ERR: alloc fail, size %d\n", cnt);
		goto sensor_k_readi2c_return;
	} else {
		SENSOR_PRINT("sensor rd I2C: alloc success, size %d\n", cnt);
	}

	if (copy_from_user(pBuff, pI2cTab->i2c_data, cnt)) {
		ret = SENSOR_K_FAIL;
		SENSOR_PRINT_ERR("sensor W I2C ERR: copy user fail, size %d \n", cnt);
		goto sensor_k_readi2c_return;
	}


	for (i = 0; i < SENSOR_I2C_OP_TRY_NUM; i++)
	{
		msg_r[0].addr =  pI2cTab->slave_addr;
		msg_r[0].flags = 0;
		msg_r[0].buf = pBuff;
		msg_r[0].len = cnt;
		msg_r[1].addr =  pI2cTab->slave_addr;
		msg_r[1].flags = I2C_M_RD;
		msg_r[1].buf = pBuff;
		msg_r[1].len = read_num;
<<<<<<< HEAD
		ret = i2c_transfer(s_p_sensor_mod->cur_i2c_client->adapter, msg_r, 2);
		if (ret != 2)
		{
			SENSOR_PRINT_ERR("_sensor_k_rd_i2c : Read reg fail, ret %d, addr 0x%x\n", ret, s_p_sensor_mod->cur_i2c_client->addr);
=======
		ret = i2c_transfer(p_mod->sensor_dev_tab[fd_handle->sensor_id].cur_i2c_client->adapter, msg_r, 2);
		if (ret != 2) {
			SENSOR_PRINT_ERR("SENSOR:read reg fail, ret %d, addr 0x%x \n",
					ret, p_mod->sensor_dev_tab[fd_handle->sensor_id].cur_i2c_client->addr);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
			SLEEP_MS(20);
			ret = SENSOR_K_FAIL;
		}
		else
		{
			ret = SENSOR_K_SUCCESS;
			if (copy_to_user(pI2cTab->i2c_data, pBuff, read_num))
			{
				ret = SENSOR_K_FAIL;
<<<<<<< HEAD
				SENSOR_PRINT_ERR("_sensor_k_rd_i2c : sensor W I2C ERR: copy user fail, size %d \n", cnt);
=======
				SENSOR_PRINT_ERR("sensor W I2C ERR: copy user fail, size %d \n", cnt);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
				goto sensor_k_readi2c_return;
			}
			break;
		}
	}

sensor_k_readi2c_return:
	if(PNULL != pBuff)
		_sensor_k_free(fd_handle, pBuff);

	SENSOR_PRINT("_Sensor_K_ReadI2C, ret %d \n", ret);
	return ret;
}

LOCAL int _sensor_csi2_error(uint32_t err_id, uint32_t err_status, void* u_data)
{
<<<<<<< HEAD
	int ret = 0;
	printk("_sensor_csi2_error : V4L2 : csi2_error %d, 0x%x\n", err_id, err_status);
=======
	int                      ret = 0;

	printk("V4L2: csi2_error, %d 0x%x \n", err_id, err_status);

>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
	return ret;

}

int sensor_k_open(struct inode *node, struct file *file)
{
<<<<<<< HEAD
	int ret = 0;
=======
	int                            ret = 0;
	struct sensor_file_tag         *p_file;
	struct sensor_module_tab_tag   *p_mod = NULL;// platform_get_drvdata(_sensor_k_get_platform_device());
	struct miscdevice *md = (struct miscdevice *)file->private_data ;

	if (!md) {
		ret = -EFAULT;
		printk("rot_k_open fail miscdevice NULL \n");
		return -1;
	}
	p_mod = (struct sensor_module_tab_tag*)md->this_device->platform_data;
	SENSOR_CHECK_ZERO(p_mod);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g

	p_file = (struct sensor_file_tag *)vzalloc(sizeof(struct sensor_file_tag));
	SENSOR_CHECK_ZERO(p_file);
	file->private_data = p_file;
	p_file->module_data = p_mod;

	p_file->sensor_mem.buf_ptr = (void *)vzalloc(PREALLOC_SIZE);
	if (PNULL != p_file->sensor_mem.buf_ptr) {
		p_file->sensor_mem.size = PREALLOC_SIZE;
	} else {
		printk("vzalloc p_file->sensor_mem.buf_ptr fail \n");
	}

	ret = csi_api_malloc(&p_file->csi_handle);
	if (ret) {
		vfree(p_file);
		p_file = NULL;
		return -1;
	}

	if (atomic_inc_return(&p_mod->total_users) == 1) {
		struct device_node *dn = p_mod->of_node;
		ret = clk_mm_i_eb(dn,1);
		wake_lock(&p_mod->wakelock);
	}
	printk("sensor open %d\n", ret);
	return ret;
}

int _sensor_k_close_mipi(struct file *file)
{
	int                            ret = 0;
	struct sensor_file_tag         *p_file = file->private_data;
	struct csi_context             *handle = NULL;
	SENSOR_CHECK_ZERO(p_file);
	handle = p_file->csi_handle;
	if (NULL == handle) {
		printk("handle null\n");
		return -1;
	}
	if (INTERFACE_MIPI == p_file->if_type) {
		if (1 == p_file->mipi_on) {
			csi_api_close(handle, p_file->phy_id);
			_sensor_k_mipi_clk_dis(p_file);
			p_file->mipi_on = 0;
			printk("MIPI off \n");
		} else {
			printk("MIPI already off \n");
		}

	}
	return ret;
}

int sensor_k_release(struct inode *node, struct file *file)
{
<<<<<<< HEAD
	SENSOR_PRINT_HIGH("sensor_k_release : E\n");

	int ret = 0;

	struct miscdevice *md = file->private_data;
	struct device_node *dn = md->this_device->of_node;

	if(atomic_dec_return(&s_p_sensor_mod->open_count) == 0)
	{
		sensor_k_set_voltage_cammot(SENSOR_VDD_CLOSED);
		sensor_k_set_voltage_avdd(SENSOR_VDD_CLOSED);
		sensor_k_set_voltage_dvdd(SENSOR_VDD_CLOSED);
		sensor_k_set_voltage_iovdd(SENSOR_VDD_CLOSED);
		sensor_k_set_mclk(0);
		clk_put(s_p_sensor_mod->ccir_clk);
		s_p_sensor_mod->ccir_clk = NULL;
		ret = clk_mm_i_eb(dn, 0);
=======
	int                            ret = 0;
	struct sensor_file_tag         *p_file = file->private_data;
	struct sensor_module_tab_tag   *p_mod;

	SENSOR_CHECK_ZERO(p_file);
	p_mod = p_file->module_data;
	SENSOR_CHECK_ZERO(p_mod);

	printk("sensor: release \n");
	if (atomic_dec_return(&p_mod->total_users) == 0) {
		struct device_node *dn = p_mod->of_node;
		sensor_k_set_voltage_cammot((uint32_t *)p_file, SENSOR_VDD_CLOSED);
		sensor_k_set_voltage_avdd((uint32_t *)p_file, SENSOR_VDD_CLOSED);
		sensor_k_set_voltage_dvdd((uint32_t *)p_file, SENSOR_VDD_CLOSED);
		sensor_k_set_voltage_iovdd((uint32_t *)p_file, SENSOR_VDD_CLOSED);
		_sensor_k_set_mclk(p_file, dn, 0);
		_sensor_k_close_mipi(file);
		ret = clk_mm_i_eb(dn,0);

		wake_unlock(&p_mod->wakelock);
	}
	if (SENSOR_ADDR_INVALID(p_file)) {
		printk("SENSOR: Invalid addr, 0x%x", (uint32_t)p_mod);
	} else {
		csi_api_free(p_file->csi_handle);
		if (PNULL != p_file->sensor_mem.buf_ptr) {
			vfree(p_file->sensor_mem.buf_ptr);
			p_file->sensor_mem.buf_ptr = PNULL;
			p_file->sensor_mem.size = 0;
		}
		if (PNULL != p_file) {
			vfree(p_file);
			p_file = PNULL;
		}
		file->private_data = PNULL;
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
	}
	printk("sensor: release %d \n", ret);
	return ret;
}

LOCAL ssize_t sensor_k_read(struct file *filp, char __user *ubuf, size_t cnt, loff_t *gpos)
{
	return 0;
}

LOCAL ssize_t sensor_k_write(struct file *filp, const char __user *ubuf, size_t cnt, loff_t *gpos)
{
<<<<<<< HEAD
	char buf[64];
	char *pBuff = PNULL;
	struct i2c_msg msg_w;
	int ret = SENSOR_K_FAIL;
	int need_alloc = 1;
	SENSOR_CHECK_ZERO(s_p_sensor_mod);

	SENSOR_PRINT("sensor_k_write : Write count %d, buf %d\n", cnt, sizeof(buf));
=======
	char                          buf[64];
	char                          *pBuff = PNULL;
	struct                        i2c_msg msg_w;
	int                           ret = SENSOR_K_FAIL;
	int                           need_alloc = 1;
	struct i2c_client             *i2c_client = PNULL;
	struct sensor_file_tag        *p_file = filp->private_data;
	struct sensor_module_tab_tag  *p_mod;
	SENSOR_CHECK_ZERO(p_file);

	p_mod = p_file->module_data;
	SENSOR_CHECK_ZERO(p_mod);

	i2c_client = p_mod->sensor_dev_tab[p_file->sensor_id].cur_i2c_client;

	SENSOR_PRINT("sensor w cnt %d, buf %d\n", cnt, sizeof(buf));
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g

	if (cnt < sizeof(buf))
	{
		pBuff = buf;
		need_alloc = 0;
<<<<<<< HEAD
	}
	else
	{
		pBuff = _sensor_k_kmalloc(cnt);
		if (PNULL == pBuff)
		{
			SENSOR_PRINT_ERR("sensor_k_write : Write error : Alloc fail, size %d \n", cnt);
			goto sensor_k_write_return;
		}
		else
		{
			SENSOR_PRINT("sensor_k_write : Alloc success, size %d \n", cnt);
		}
	}

	if (copy_from_user(pBuff, ubuf, cnt))
	{
		SENSOR_PRINT_ERR("sensor_k_write : Write error : Copy user fail, size %d\n", cnt);
		goto sensor_k_write_return;
	}

	printk("sensor_k_write : Client addr 0x%x\n", s_p_sensor_mod->cur_i2c_client->addr);
	msg_w.addr = s_p_sensor_mod->cur_i2c_client->addr;
=======
	}  else {
		pBuff = _sensor_k_malloc(p_file, cnt);
		if (PNULL == pBuff) {
			SENSOR_PRINT_ERR("sensor w ERR: alloc fail, size %d \n", cnt);
			goto sensor_k_write_return;
		} else {
			SENSOR_PRINT("sensor w: alloc success, size %d \n", cnt);
		}
	}

	if (copy_from_user(pBuff, ubuf, cnt)) {
		SENSOR_PRINT_ERR("sensor w ERR: copy user fail, size %d\n", cnt);
		goto sensor_k_write_return;
	}
	printk("sensor clnt addr 0x%x.\n", i2c_client->addr);
	msg_w.addr = i2c_client->addr;
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
	msg_w.flags = 0;
	msg_w.buf = pBuff;
	msg_w.len = cnt;

<<<<<<< HEAD
	ret = i2c_transfer(s_p_sensor_mod->cur_i2c_client->adapter, &msg_w, 1);
	if (ret != 1)
	{
		SENSOR_PRINT_ERR("sensor_k_write : Write reg fail, ret %d, Write addr : 0x%x\n", ret, s_p_sensor_mod->cur_i2c_client->addr);
	}
	else
	{
=======
	ret = i2c_transfer(i2c_client->adapter, &msg_w, 1);
	if (ret != 1) {
		SENSOR_PRINT_ERR("SENSOR: w reg fail, ret %d, w addr: 0x%x,\n",
				ret, i2c_client->addr);
	} else {
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
		ret = SENSOR_K_SUCCESS;
	}

sensor_k_write_return:
	if ((PNULL != pBuff) && need_alloc)
<<<<<<< HEAD
		_sensor_k_kfree(pBuff);

	SENSOR_PRINT("sensor_k_write : Write done, ret %d\n", ret);
=======
		_sensor_k_free(p_file, pBuff);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g

	SENSOR_PRINT("sensor w done, ret %d \n", ret);
	return ret;
}

<<<<<<< HEAD
#if defined(CONFIG_MACH_YOUNG23GDTV) || defined(CONFIG_MACH_J13G) || defined(CONFIG_MACH_VIVALTO3MVE3G_LTN) || defined(CONFIG_MACH_VIVALTO3MVEML3G)

#define BURST_MODE_BUFFER_MAX_SIZE 255
#define BURST_REG 0x0e
#define DELAY_REG 0xff

int _sensor_burst_write_init(SENSOR_REG_T_PTR p_reg_table, uint32_t init_table_size)
{
	int rtn = 0;
	int ret = 0;
	int idx = 0;
	uint32_t i = 0;
	uint8_t burstmode_data[BURST_MODE_BUFFER_MAX_SIZE]={0};
	struct i2c_msg msg;
	struct i2c_client *i2c_client = PNULL;
	unsigned char buf[2] = { 0 };
	unsigned short subaddr = 0;
	unsigned short value = 0;
	int burst_flag = 0;
	int burst_cnt = 0;

	SENSOR_CHECK_ZERO(s_p_sensor_mod);

	i2c_client = s_p_sensor_mod->cur_i2c_client;
	if (0 == i2c_client)
	{
		SENSOR_PRINT_ERR("_sensor_burst_write_init : Burst write Init err, i2c_clnt NULL\n");
		return -1;
	}

	msg.addr = i2c_client->addr;
	msg.flags = 0;

	for( i = 0 ; i < init_table_size ; i++)
	{
		if( idx > BURST_MODE_BUFFER_MAX_SIZE - 10 )
		{
			printk("_sensor_burst_write_init : Burst mode buffer overflow! Burst Count %d, idx = %d\n",  burst_cnt, idx);
		}
		subaddr = p_reg_table[i].reg_addr;
		value = p_reg_table[i].reg_value;

		if(burst_flag == 0)
		{
			switch(subaddr)
			{
				case BURST_REG:
					if(value!=0x00)
					{
						burst_flag = 1;
						burst_cnt++;
					}
					break;

				case DELAY_REG:
					if (value != DELAY_REG) msleep(value*10);
					break;

				default:
					idx = 0;
					buf[0] = subaddr;
					buf[1] = value;
					msg.buf = buf;
					msg.len = 2;
					ret = i2c_transfer(i2c_client->adapter, &msg, 1);
					break;
			}
		}
		else if(burst_flag == 1)
		{
			if(subaddr == BURST_REG && value == 0x00)
			{
				msg.len = idx;
				msg.buf = burstmode_data;
				ret = i2c_transfer(i2c_client->adapter, &msg, 1);

				if(ret != 1)
				{
					rtn = 1;
					break;
				}
				burst_flag = 0;
				idx = 0;
			}
			else
			{
				if(idx == 0)
				{
					burstmode_data[idx++] = subaddr;
				}
				burstmode_data[idx++] = value;
			}
		}
	}
	return rtn;
}

#else // defined(CONFIG_MACH_YOUNG23GDTV) || defined(CONFIG_MACH_J13G)

int _sensor_burst_write_init(SENSOR_REG_T_PTR p_reg_table, uint32_t init_table_size)
{
	uint32_t rtn = 0;
	int ret = 0;
	uint32_t i = 0;
	uint32_t written_num = 0;
	uint16_t wr_reg = 0;
	uint16_t wr_val = 0;
	uint32_t wr_num_once = 0;
	uint8_t *p_reg_val_tmp = 0;
	struct i2c_msg msg_w;
	struct i2c_client *i2c_client = PNULL;

	SENSOR_CHECK_ZERO(s_p_sensor_mod);

	i2c_client = s_p_sensor_mod->cur_i2c_client;

	printk("_sensor_burst_write_init : E\n");
	if (0 == i2c_client)
	{
		SENSOR_PRINT_ERR("_sensor_burst_write_init : Burst write Init error, I2C client is NULL\n");
=======
int _sensor_burst_write_init(struct sensor_file_tag *fd_handle, struct sensor_reg_tag *p_reg_table, uint32_t init_table_size)
{
	uint32_t                      rtn = 0;
	int                           ret = 0;
	uint32_t                      i = 0;
	uint32_t                      written_num = 0;
	uint16_t                      wr_reg = 0;
	uint16_t                      wr_val = 0;
	uint32_t                      wr_num_once = 0;
	uint8_t                       *p_reg_val_tmp = 0;
	struct i2c_msg                msg_w;
	struct i2c_client             *i2c_client = PNULL;
	struct sensor_file_tag        *p_file = fd_handle;
	struct sensor_module_tab_tag  *p_mod;
	SENSOR_CHECK_ZERO(p_file);

	p_mod = p_file->module_data;
	SENSOR_CHECK_ZERO(p_mod);

	i2c_client = p_mod->sensor_dev_tab[p_file->sensor_id].cur_i2c_client;

	printk("SENSOR: burst w Init\n");
	if (0 == i2c_client) {
		SENSOR_PRINT_ERR("SENSOR: burst w Init err, i2c_clnt NULL!.\n");
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
		return -1;
	}
	p_reg_val_tmp = (uint8_t*)_sensor_k_malloc(fd_handle, init_table_size*sizeof(uint16_t) + 16);

<<<<<<< HEAD
	if(PNULL == p_reg_val_tmp)
	{
		SENSOR_PRINT_ERR("_sensor_burst_write_init ERROR : Alloc is fail, size = %d\n", init_table_size * sizeof(uint16_t) + 16);
		return -1;
	}
	else
	{
		SENSOR_PRINT_HIGH("_sensor_burst_write_init : Alloc success, size = %d \n", init_table_size * sizeof(uint16_t) + 16);
=======
	if(PNULL == p_reg_val_tmp){
		SENSOR_PRINT_ERR("_sensor_burst_write_init ERROR: alloc is fail, size = %d \n", init_table_size*sizeof(uint16_t) + 16);
		return -1;
	}
	else{
		SENSOR_PRINT_HIGH("_sensor_burst_write_init: alloc success, size = %d \n", init_table_size*sizeof(uint16_t) + 16);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
	}

	while (written_num < init_table_size)
	{
		wr_num_once = 2;
		wr_reg = p_reg_table[written_num].reg_addr;
		wr_val = p_reg_table[written_num].reg_value;

		if (SENSOR_WRITE_DELAY == wr_reg)
		{
			if (wr_val >= 10)
			{
				msleep(wr_val);
			}
			else
			{
				mdelay(wr_val);
			}
		}
		else
		{
			p_reg_val_tmp[0] = (uint8_t)(wr_reg);
			p_reg_val_tmp[1] = (uint8_t)(wr_val);

			if ((0x0e == wr_reg) && (0x01 == wr_val))
			{
				for (i = written_num + 1; i < init_table_size; i++)
				{
					if ((0x0e == wr_reg) && (0x00 == wr_val))
					{
						break;
					}
					else
					{
						wr_val = p_reg_table[i].reg_value;
						p_reg_val_tmp[wr_num_once+1] = (uint8_t)(wr_val);
						wr_num_once ++;
					}
				}
			}

			msg_w.addr = i2c_client->addr;
			msg_w.flags = 0;
			msg_w.buf = p_reg_val_tmp;
			msg_w.len = (uint32_t)(wr_num_once);

			ret = i2c_transfer(i2c_client->adapter, &msg_w, 1);
<<<<<<< HEAD
			if (ret!=1)
			{
				SENSOR_PRINT("_sensor_burst_write_init : s err, val {0x%x 0x%x} {0x%x 0x%x} {0x%x 0x%x} {0x%x 0x%x} {0x%x 0x%x} {0x%x 0x%x}\n",
					p_reg_val_tmp[0],p_reg_val_tmp[1],p_reg_val_tmp[2],p_reg_val_tmp[3],
					p_reg_val_tmp[4],p_reg_val_tmp[5],p_reg_val_tmp[6],p_reg_val_tmp[7],
					p_reg_val_tmp[8],p_reg_val_tmp[9],p_reg_val_tmp[10],p_reg_val_tmp[11]);
				SENSOR_PRINT("_sensor_burst_write_init : I2C write once err\n");
=======
			if (ret!=1) {
				SENSOR_PRINT("SENSOR: s err, val {0x%x 0x%x} {0x%x 0x%x} {0x%x 0x%x} {0x%x 0x%x} {0x%x 0x%x} {0x%x 0x%x}.\n",
					p_reg_val_tmp[0],p_reg_val_tmp[1],p_reg_val_tmp[2],p_reg_val_tmp[3],
					p_reg_val_tmp[4],p_reg_val_tmp[5],p_reg_val_tmp[6],p_reg_val_tmp[7],
					p_reg_val_tmp[8],p_reg_val_tmp[9],p_reg_val_tmp[10],p_reg_val_tmp[11]);
					SENSOR_PRINT("SENSOR: i2c w once err\n");
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
				rtn = 1;
				break;
			}
		}
		written_num += wr_num_once - 1;
	}
<<<<<<< HEAD
	SENSOR_PRINT("_sensor_burst_write_init : Burst write Init OK\n");
	_sensor_k_kfree(p_reg_val_tmp);

	return rtn;
}
#endif // defined(CONFIG_MACH_YOUNG23GDTV) || defined(CONFIG_MACH_J13G)
=======
	SENSOR_PRINT("SENSOR: burst w Init OK\n");
	_sensor_k_free(fd_handle, p_reg_val_tmp);
	return rtn;
}
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g

LOCAL long sensor_k_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int                            ret = 0;
	struct sensor_module_tab_tag    *p_mod;
	struct sensor_file_tag         *p_file = file->private_data;

	SENSOR_CHECK_ZERO(p_file);
	p_mod = p_file->module_data;
	SENSOR_CHECK_ZERO(p_mod);

	SENSOR_PRINT("SENSOR: ioctl cmd %d id %d \n", cmd, p_file->sensor_id);

	if (SENSOR_IO_SET_ID == cmd)
	{
		mutex_lock(&p_mod->sensor_id_lock);
		ret = copy_from_user(&p_file->sensor_id, (uint32_t *) arg, sizeof(uint32_t));
		mutex_unlock(&p_mod->sensor_id_lock);
	}

<<<<<<< HEAD
	switch (cmd)
	{
		case SENSOR_IO_PD:
=======
	mutex_lock(&p_mod->sensor_dev_tab[p_file->sensor_id].sync_lock);
	switch (cmd) {
	case SENSOR_IO_PD:
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
		{
			uint8_t power_level;
			SENSOR_PRINT("SENSOR: ioctl SENSOR_IO_PD \n");
			ret = copy_from_user(&power_level, (uint8_t *) arg, sizeof(uint8_t));

			if (0 == ret)
				ret = sensor_k_set_pd_level((uint32_t *)p_file, power_level);
		}
		break;

		case SENSOR_IO_SET_CAMMOT:
		{
			uint32_t vdd_val;
			SENSOR_PRINT("SENSOR: ioctl SENSOR_IO_SET_CAMMOT \n");
			ret = copy_from_user(&vdd_val, (uint32_t *) arg, sizeof(uint32_t));
			if (0 == ret)
				ret = sensor_k_set_voltage_cammot((uint32_t *)p_file, vdd_val);
		}
		break;

		case SENSOR_IO_SET_AVDD:
		{
			uint32_t vdd_val;
			SENSOR_PRINT("SENSOR: ioctl SENSOR_IO_SET_AVDD \n");
			ret = copy_from_user(&vdd_val, (uint32_t *) arg, sizeof(uint32_t));
			if (0 == ret)
				ret = sensor_k_set_voltage_avdd((uint32_t *)p_file, vdd_val);
		}
		break;

		case SENSOR_IO_SET_DVDD:
		{
			uint32_t vdd_val;
			SENSOR_PRINT("SENSOR: ioctl SENSOR_IO_SET_DVDD \n");
			ret = copy_from_user(&vdd_val, (uint32_t *) arg, sizeof(uint32_t));
			if (0 == ret)
				ret = sensor_k_set_voltage_dvdd((uint32_t *)p_file, vdd_val);
		}
		break;

		case SENSOR_IO_SET_IOVDD:
		{
			uint32_t vdd_val;
			SENSOR_PRINT("SENSOR: ioctl SENSOR_IO_SET_IOVDD \n");
			ret = copy_from_user(&vdd_val, (uint32_t *) arg, sizeof(uint32_t));
			if (0 == ret)
				ret = sensor_k_set_voltage_iovdd((uint32_t *)p_file, vdd_val);
		}
		break;

		case SENSOR_IO_SET_MCLK:
		{
			uint32_t mclk;
			SENSOR_PRINT("SENSOR: ioctl SENSOR_IO_SET_MCLK \n");
			ret = copy_from_user(&mclk, (uint32_t *) arg, sizeof(uint32_t));
			if (0 == ret)
				ret = _sensor_k_set_mclk(p_file, p_mod->of_node, mclk);
		}
		break;

		case SENSOR_IO_RST:
		{
			uint32_t rst_val[2];
			SENSOR_PRINT("SENSOR: ioctl SENSOR_IO_RST \n");
			ret = copy_from_user(rst_val, (uint32_t *) arg, 2*sizeof(uint32_t));
			if (0 == ret)
				ret = _sensor_k_reset(p_file, rst_val[0], rst_val[1]);
		}
		break;

		case SENSOR_IO_I2C_INIT:
		{
			uint32_t sensor_id;
			SENSOR_PRINT("SENSOR: ioctl SENSOR_IO_I2C_INIT \n");
			ret = copy_from_user(&sensor_id, (uint32_t *) arg, sizeof(uint32_t));
			if (0 == ret)
				ret = sensor_k_sensor_sel((uint32_t *)p_file, sensor_id);
		}
		break;

		case SENSOR_IO_I2C_DEINIT:
		{
			uint32_t sensor_id;
			SENSOR_PRINT("SENSOR: ioctl SENSOR_IO_I2C_DEINIT \n");
			ret = copy_from_user(&sensor_id, (uint32_t *) arg, sizeof(uint32_t));
			if (0 == ret)
				ret = sensor_k_sensor_desel(p_file, sensor_id);
		}
		break;

		case SENSOR_IO_SET_ID:
		{
			SENSOR_PRINT("SENSOR: ioctl SENSOR_IO_SET_ID \n");
			ret = copy_from_user(&p_file->sensor_id, (uint32_t *) arg, sizeof(uint32_t));
			get_gpio_id(p_mod->of_node, &p_file->gpio_tab.pwn, &p_file->gpio_tab.reset, p_file->sensor_id);
		}
		break;

		case SENSOR_IO_RST_LEVEL:
		{
			uint32_t level;
			SENSOR_PRINT("SENSOR: ioctl SENSOR_IO_RST_LEVEL \n");
			ret = copy_from_user(&level, (uint32_t *) arg, sizeof(uint32_t));
			if (0 == ret)
				ret = sensor_k_set_rst_level((uint32_t *)p_file, level);
		}
		break;

		case SENSOR_IO_I2C_ADDR:
		{
			uint16_t i2c_addr;
			printk("SENSOR: ioctl SENSOR_IO_I2C_ADDR \n");
			ret = copy_from_user(&i2c_addr, (uint16_t *) arg, sizeof(uint16_t));
<<<<<<< HEAD
			if (0 == ret)
			{
				s_p_sensor_mod->cur_i2c_client->addr = (s_p_sensor_mod->cur_i2c_client->addr & (~0xFF)) |i2c_addr;
				printk("SENSOR_IO_I2C_ADDR : addr = %x, %x \n", i2c_addr, s_p_sensor_mod->cur_i2c_client->addr);
=======
			if (0 == ret) {
				p_mod->sensor_dev_tab[p_file->sensor_id].cur_i2c_client->addr = (p_mod->sensor_dev_tab[p_file->sensor_id].cur_i2c_client->addr & (~0xFF)) |i2c_addr;
				printk("SENSOR_IO_I2C_ADDR: addr = %x, %x \n", i2c_addr, p_mod->sensor_dev_tab[p_file->sensor_id].cur_i2c_client->addr);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
			}
		}
		break;

		case SENSOR_IO_I2C_READ:
		{
			struct sensor_reg_bits_tag reg;
			ret = copy_from_user(&reg, (struct sensor_reg_bits_tag *) arg, sizeof(struct sensor_reg_bits_tag));

<<<<<<< HEAD
			if (0 == ret)
			{
				ret = _Sensor_K_ReadReg(&reg);
				if(SENSOR_K_FAIL != ret)
				{
					ret = copy_to_user((SENSOR_REG_BITS_T *)arg, &reg, sizeof(SENSOR_REG_BITS_T));
=======
			if (0 == ret) {
				ret = _Sensor_K_ReadReg(p_file, &reg);
				if(SENSOR_K_FAIL != ret){
					ret = copy_to_user((struct sensor_reg_bits_tag *)arg, &reg, sizeof(struct sensor_reg_bits_tag));
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
				}
			}
		}
		break;

		case SENSOR_IO_I2C_WRITE:
		{
			struct sensor_reg_bits_tag reg;
			ret = copy_from_user(&reg, (struct sensor_reg_bits_tag *) arg, sizeof(struct sensor_reg_bits_tag));

<<<<<<< HEAD
			if (0 == ret)
			{
				ret = _Sensor_K_WriteReg(&reg);
=======
			if (0 == ret) {
				ret = _Sensor_K_WriteReg(p_file, &reg);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
			}
		}
		break;

		case SENSOR_IO_I2C_WRITE_REGS:
		{
			struct sensor_reg_tab_tag regTab;
			ret = copy_from_user(&regTab, (struct sensor_reg_tab_tag *) arg, sizeof(struct sensor_reg_tab_tag));
			if (0 == ret)
				ret = _sensor_k_wr_regtab(p_file, &regTab);
		}
		break;

		case SENSOR_IO_SET_I2CCLOCK:
		{
			uint32_t clock;
			SENSOR_PRINT("SENSOR: ioctl SENSOR_IO_SET_I2CCLOCK \n");
			ret = copy_from_user(&clock, (uint32_t *) arg, sizeof(uint32_t));
<<<<<<< HEAD
			if(0 == ret)
			{
				_sensor_k_set_i2c_clk(clock);
=======
			if(0 == ret){
				_sensor_k_set_i2c_clk(p_file, clock);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
			}
		}
		break;

		case SENSOR_IO_I2C_WRITE_EXT:
		{
			struct sensor_i2c_tag i2cTab;
			SENSOR_PRINT("SENSOR: ioctl SENSOR_IO_I2C_WRITE_EXT \n");
			ret = copy_from_user(&i2cTab, (struct sensor_i2c_tag*)arg, sizeof(struct sensor_i2c_tag));
			if (0 == ret)
				ret = _sensor_k_wr_i2c(p_file, &i2cTab);
		}
		break;

		case SENSOR_IO_I2C_READ_EXT:
		{
			struct sensor_i2c_tag i2cTab;
			ret = copy_from_user(&i2cTab, (struct sensor_i2c_tag*) arg, sizeof(struct sensor_i2c_tag));
			if (0 == ret)
				ret = _sensor_k_rd_i2c(p_file, &i2cTab);
		}
		break;

		case SENSOR_IO_GET_FLASH_LEVEL:
		{
<<<<<<< HEAD
			SENSOR_FLASH_LEVEL_T flash_level;
			ret = copy_from_user(&flash_level, (SENSOR_FLASH_LEVEL_T *) arg, sizeof(SENSOR_FLASH_LEVEL_T));
			if (0 == ret)
			{
				ret = _sensor_k_get_flash_level(&flash_level);
				if(SENSOR_K_FAIL != ret)
				{
					ret = copy_to_user((SENSOR_FLASH_LEVEL_T *)arg, &flash_level, sizeof(SENSOR_FLASH_LEVEL_T));
=======
			struct sensor_flash_level flash_level;
			SENSOR_PRINT("SENSOR: ioctl SENSOR_IO_GET_FLASH_LEVEL \n");
			ret = copy_from_user(&flash_level, (struct sensor_flash_level *) arg, sizeof(struct sensor_flash_level));
			if (0 == ret) {
				ret = _sensor_k_get_flash_level(p_file, &flash_level);
				if(SENSOR_K_FAIL != ret){
					ret = copy_to_user((struct sensor_flash_level *)arg, &flash_level, sizeof(struct sensor_flash_level));
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
				}
			}
		}
		break;

		case SENSOR_IO_GET_SOCID:
		{
			struct sensor_socid_tag	Id  ;
			SENSOR_PRINT("SENSOR: ioctl SENSOR_IO_GET_SOCID \n");
			Id.d_die=sci_get_chip_id();
			Id.a_die=sci_get_ana_chip_id()|sci_get_ana_chip_ver();
			SENSOR_PRINT("cpu id 0x%x,0x%x  \n", Id.d_die,Id.a_die);
			ret = copy_to_user((struct sensor_socid_tag *)arg, &Id, sizeof(struct sensor_socid_tag));
		}
		break;

		case SENSOR_IO_IF_CFG:
		{
<<<<<<< HEAD
			SENSOR_IF_CFG_T if_cfg;
			ret = copy_from_user((void*)&if_cfg, (SENSOR_IF_CFG_T *)arg, sizeof(SENSOR_IF_CFG_T));
			if (0 == ret)
			{
				if (INTERFACE_OPEN == if_cfg.is_open)
				{
					if (INTERFACE_MIPI == if_cfg.if_type)
					{
						if (0 == s_p_sensor_mod->mipi_on)
						{
							struct miscdevice *md = file->private_data ;
							_sensor_k_mipi_clk_en(md->this_device->of_node);
=======
			struct sensor_if_cfg_tag if_cfg;
			struct csi_context *csi_handle;
			csi_handle = p_file->csi_handle;
			if (NULL == csi_handle) {
				printk("handle null\n");
				return -1;
			}
			SENSOR_PRINT("SENSOR: ioctl SENSOR_IO_IF_CFG \n");
			ret = copy_from_user((void*)&if_cfg, (struct sensor_if_cfg_tag *)arg, sizeof(struct sensor_if_cfg_tag));
			if (0 == ret) {
				if (INTERFACE_OPEN == if_cfg.is_open) {
					if (INTERFACE_MIPI == if_cfg.if_type) {
						if (0 == p_file->mipi_on) {
							_sensor_k_mipi_clk_en(p_file, p_mod->of_node);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
							udelay(1);
							csi_api_init(if_cfg.bps_per_lane, if_cfg.phy_id);
							csi_api_start(csi_handle);
							csi_reg_isr(csi_handle,_sensor_csi2_error, (void*)p_file);
							csi_set_on_lanes(if_cfg.lane_num);
<<<<<<< HEAD
							s_p_sensor_mod->mipi_on = 1;
							printk("SENSOR_IO_IF_CFG : MIPI on, lane %d, bps %d, wait 10us \n", if_cfg.lane_num, if_cfg.bps_per_lane);
						}
						else
						{
							printk("SENSOR_IO_IF_CFG : MIPI already on \n");
						}
					}
				}
				else
				{
					if (INTERFACE_MIPI == if_cfg.if_type)
					{
						if (1 == s_p_sensor_mod->mipi_on)
						{
							csi_api_close(if_cfg.phy_id);
							_sensor_k_mipi_clk_dis();
							s_p_sensor_mod->mipi_on = 0;
							printk("SENSOR_IO_IF_CFG : MIPI off \n");
						}
						else
						{
							printk("SENSOR_IO_IF_CFG : MIPI already off \n");
=======
							p_file->mipi_on = 1;
							p_file->phy_id = if_cfg.phy_id;
							p_file->if_type = INTERFACE_MIPI;
							printk("MIPI on, lane %d, bps_per_lane %d, wait 10us \n", if_cfg.lane_num, if_cfg.bps_per_lane);
						} else {
							printk("MIPI already on \n");
						}
					}
				} else {
					if (INTERFACE_MIPI == if_cfg.if_type) {
						if (1 == p_file->mipi_on) {
							csi_api_close(csi_handle, if_cfg.phy_id);
							_sensor_k_mipi_clk_dis(p_file);
							p_file->mipi_on = 0;
							printk("MIPI off \n");
						} else {
							printk("MIPI already off \n");
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
						}
					}
				}
			}
		}
		break;
<<<<<<< HEAD

		case SENSOR_IO_POWER_CFG:
		{
			SENSOR_POWER_CFG_T pwr_cfg;

			ret = copy_from_user(&pwr_cfg, (SENSOR_POWER_CFG_T*) arg, sizeof(SENSOR_POWER_CFG_T));
			if (0 == ret)
			{
				if (pwr_cfg.is_on)
				{
					ret = sensor_power_on((uint8_t)pwr_cfg.op_sensor_id, &pwr_cfg.main_sensor, &pwr_cfg.sub_sensor);
				}
				else
				{
					ret = sensor_power_off((uint8_t)pwr_cfg.op_sensor_id, &pwr_cfg.main_sensor, &pwr_cfg.sub_sensor);
=======

		case SENSOR_IO_POWER_CFG:
		{
			struct sensor_power_info_tag pwr_cfg;
			uint32_t sensor_id_temp = p_file->sensor_id;
			ret = copy_from_user(&pwr_cfg, (struct sensor_power_info_tag*) arg, sizeof(struct sensor_power_info_tag));
			if (0 == ret) {
				if (pwr_cfg.is_on) {
					ret = sensor_power_on((uint32_t *)p_file, pwr_cfg.op_sensor_id, &pwr_cfg.dev0, &pwr_cfg.dev1, &pwr_cfg.dev2);
				} else {
					ret = sensor_power_off((uint32_t *)p_file, pwr_cfg.op_sensor_id, &pwr_cfg.dev0, &pwr_cfg.dev1, &pwr_cfg.dev2);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
				}
			}
			p_file->sensor_id = sensor_id_temp;
		}
		break;
<<<<<<< HEAD

		default:
			SENSOR_PRINT("sensor_k_ioctl : Invalid command %x\n", cmd);
			break;
	}
	mutex_unlock(&s_p_sensor_mod->sensor_lock);

	return ret;
=======
	default:
		printk("sensor_k_ioctl: inv cmd %x  \n", cmd);
		break;
	}

	mutex_unlock(&p_mod->sensor_dev_tab[p_file->sensor_id].sync_lock);
	return (long)ret;
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
}

LOCAL struct file_operations sensor_fops = {
	.owner = THIS_MODULE,
	.open = sensor_k_open,
	.read = sensor_k_read,
	.write = sensor_k_write,
	.unlocked_ioctl = sensor_k_ioctl,
	.release = sensor_k_release,
};

LOCAL struct miscdevice sensor_dev = {
	.minor = SENSOR_MINOR,
	.name = SENSOR_DEVICE_NAME,
	.fops = &sensor_fops,
};

LOCAL int sensor_k_register_subdevs(struct platform_device *pdev)
{
<<<<<<< HEAD
	int ret = 0;
	uint32_t tmp = 0;
	SENSOR_CHECK_ZERO(s_p_sensor_mod);

	printk(KERN_ALERT "sensor_k_probe : Sensor probe called\n");

	ret = misc_register(&sensor_dev);
	if (ret)
	{
		printk(KERN_ERR "sensor_k_probe : can't reg miscdev on minor=%d (%d)\n", SENSOR_MINOR, ret);
		return ret;
	}

#ifdef CONFIG_OF

	sensor_dev.this_device->of_node = pdev->dev.of_node;
	s_p_sensor_mod->pin_main_reset = of_get_gpio(sensor_dev.this_device->of_node,0);
	s_p_sensor_mod->pin_main_pd = of_get_gpio(sensor_dev.this_device->of_node,1);
	s_p_sensor_mod->pin_sub_reset = of_get_gpio(sensor_dev.this_device->of_node,2);
	s_p_sensor_mod->pin_sub_pd = of_get_gpio(sensor_dev.this_device->of_node,3);

#if  defined(CONFIG_MACH_CORE3_W) || defined(CONFIG_MACH_GRANDNEOVE3G) || defined(CONFIG_MACH_J13G)	\
	|| defined(CONFIG_MACH_VIVALTO5MVE3G) || defined(CONFIG_MACH_GOYAVE3G_SWA)	\
	|| defined(CONFIG_MACH_GTEL3G) || defined(CONFIG_MACH_GTELWIFI) || defined(CONFIG_MACH_YOUNG33G)
	s_p_sensor_mod->pin_main_camdvdd_en = of_get_gpio(sensor_dev.this_device->of_node,4);
#endif

#if defined(CONFIG_MACH_YOUNG33G)
	s_p_sensor_mod->pin_main_camavdd_en = of_get_gpio(sensor_dev.this_device->of_node,5);
#endif

#else // CONFIG_OF

	s_p_sensor_mod->pin_main_reset = GPIO_SENSOR_RESET;
	s_p_sensor_mod->pin_main_pd= GPIO_MAIN_SENSOR_PWN;
	s_p_sensor_mod->pin_sub_reset = GPIO_SUB_SENSOR_RESET;
	s_p_sensor_mod->pin_sub_pd= GPIO_SUB_SENSOR_PWN;

#endif // CONFIG_OF

	printk("sensor_k_probe : Sensor pin_main_reset = %d\n", s_p_sensor_mod->pin_main_reset);
	printk("sensor_k_probe : Sensor pin_main_pd = %d\n", s_p_sensor_mod->pin_main_pd);
	printk("sensor_k_probe : Sensor pin_sub_reset = %d\n", s_p_sensor_mod->pin_sub_reset);
	printk("sensor_k_probe : Sensor pin_sub_pd = %d\n", s_p_sensor_mod->pin_sub_pd);

#if  defined(CONFIG_MACH_CORE3_W) || defined(CONFIG_MACH_GRANDNEOVE3G) || defined(CONFIG_MACH_J13G)	\
	|| defined(CONFIG_MACH_VIVALTO5MVE3G) || defined(CONFIG_MACH_GOYAVE3G_SWA)	\
	|| defined(CONFIG_MACH_GTEL3G) || defined(CONFIG_MACH_GTELWIFI) || defined(CONFIG_MACH_YOUNG33G)
	printk("sensor_k_probe : Sensor pin_main_camdvdd_en = %d\n", s_p_sensor_mod->pin_main_camdvdd_en);
#endif
	
#if defined(CONFIG_MACH_YOUNG33G)
	printk("sensor_k_probe : Sensor pin_main_camavdd_en = %d\n", s_p_sensor_mod->pin_main_camavdd_en);
#endif

	// Get main STBY GPIO
	if(s_p_sensor_mod->pin_main_pd >= 0)
		ret = gpio_request(s_p_sensor_mod->pin_main_pd, "main camera");
=======
	struct device_node             *adapter, *child;
	struct sensor_module_tab_tag   *p_mod = platform_get_drvdata(pdev);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g

	SENSOR_CHECK_ZERO(p_mod);

	printk("sensor register sub device E\n");
	for_each_compatible_node(adapter, NULL, "sprd,i2c") {
		if (!of_find_device_by_node(adapter)) {
			of_node_put(adapter);
			printk("sensor find device fail\n");
			return -EPROBE_DEFER;
		}

<<<<<<< HEAD
#if defined (CONFIG_MACH_CORE3_W) || defined(CONFIG_MACH_J13G) || defined (CONFIG_MACH_GRANDNEOVE3G)\
    || defined(CONFIG_MACH_VIVALTO5MVE3G)|| defined(CONFIG_MACH_GOYAVE3G) || defined(CONFIG_MACH_GOYAVEWIFI)	\
    || defined(CONFIG_MACH_GOYAVE3G_SEA) || defined(CONFIG_MACH_GOYAVEWIFI_SEA_XTC) || defined(CONFIG_MACH_YOUNG33G) || defined(CONFIG_MACH_VIVALTO3MVEML3G_SEA)

	// Get sub RST GPIO
	if(s_p_sensor_mod->pin_sub_reset >= 0)
		ret = gpio_request(s_p_sensor_mod->pin_sub_reset, "sub camera rst");
=======
		for_each_available_child_of_node(adapter, child) {
			struct i2c_client *client;
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g

			client = of_find_i2c_device_by_node(child);
			if (!client) {
				printk("sensor find i2c device fail\n");
				goto e_retry;
			}
			if (0 == strcmp(client->name, SENSOR_DEV0_I2C_NAME)) {
				printk("sensor dev0 i2c device 0x%x %s\n", client->addr, client->name);
				p_mod->sensor_dev_tab[0].cur_i2c_client = client;
			}
			if (0 == strcmp(client->name, SENSOR_DEV1_I2C_NAME)) {
				printk("sensor dev1 i2c device 0x%x %s\n", client->addr, client->name);
				p_mod->sensor_dev_tab[1].cur_i2c_client = client;
			}
			if (0 == strcmp(client->name, SENSOR_DEV2_I2C_NAME)) {
				printk("sensor dev2 i2c device 0x%x %s\n", client->addr, client->name);
				p_mod->sensor_dev_tab[2].cur_i2c_client = client;
			}
		}
	}
	printk("sensor register sub device success\n");
	return 0;

<<<<<<< HEAD
	// Get sub STBY GPIO
	if(s_p_sensor_mod->pin_sub_pd >= 0)
		ret = gpio_request(s_p_sensor_mod->pin_sub_pd, "sub camera stby");

	if(ret)
	{
		tmp = s_p_sensor_mod->pin_sub_pd;
		goto gpio_err_exit;
	}
#endif

#if defined(CONFIG_MACH_CORE3_W) || defined(CONFIG_MACH_J13G) || defined(CONFIG_MACH_GRANDNEOVE3G)	\
	|| defined(CONFIG_MACH_VIVALTO5MVE3G) || defined(CONFIG_MACH_GOYAVE3G_SWA) 	\
	|| defined(CONFIG_MACH_GTEL3G) || defined(CONFIG_MACH_GTELWIFI) || defined(CONFIG_MACH_YOUNG33G)

	ret = gpio_request(s_p_sensor_mod->pin_main_camdvdd_en, "main camera dvdd");
	if (ret)
	{
		tmp = s_p_sensor_mod->pin_main_camdvdd_en;
		goto gpio_err_exit;
	}
	else
	{
		gpio_direction_output(s_p_sensor_mod->pin_main_camdvdd_en, 0);
	}

#endif

#if defined(CONFIG_MACH_YOUNG33G)

	ret = gpio_request(s_p_sensor_mod->pin_main_camavdd_en, "main camera avdd");
	if (ret)
	{
		tmp = s_p_sensor_mod->pin_main_camavdd_en;
		goto gpio_err_exit;
	}
	else
	{
		gpio_direction_output(s_p_sensor_mod->pin_main_camavdd_en, 0);
	}

#endif

	s_p_sensor_mod->sensor_i2c_driver.driver.owner = THIS_MODULE;
	s_p_sensor_mod->sensor_i2c_driver.probe  = sensor_probe;
	s_p_sensor_mod->sensor_i2c_driver.remove = sensor_remove;
	s_p_sensor_mod->sensor_i2c_driver.detect = sensor_detect;
	s_p_sensor_mod->sensor_i2c_driver.driver.name = SENSOR_MAIN_I2C_NAME;
	s_p_sensor_mod->sensor_i2c_driver.id_table = c_sensor_device_id;
	s_p_sensor_mod->sensor_i2c_driver.address_list = &c_sensor_main_default_addr_list[0];

	ret = i2c_add_driver(&s_p_sensor_mod->sensor_i2c_driver);
	if (ret)
	{
		SENSOR_PRINT_ERR("sensor_k_probe : +I2C err %d\n", ret);
		return SENSOR_K_FAIL;
	}
	else
	{
		SENSOR_PRINT_HIGH("sensor_k_probe : +I2C OK\n");
=======
e_retry:
	of_node_put(child);
	return -EPROBE_DEFER;
}

int sensor_k_probe(struct platform_device *pdev)
{
	int                          ret = 0;
	uint32_t                     tmp = 0;
	int                          i;
	struct sensor_module_tab_tag *p_mod;
	struct sensor_gpio_tag       gpio_tab;
	int                          gpio_id = 0;

	printk(KERN_ALERT "sensor probe called\n");
	p_mod = (struct sensor_module_tab_tag *)vzalloc(sizeof(struct sensor_module_tab_tag));
	SENSOR_CHECK_ZERO(p_mod);

	for (i = 0; i < SENSOR_DEV_MAX; i++) {
		mutex_init(&p_mod->sensor_dev_tab[i].sync_lock);
		atomic_set(&p_mod->sensor_dev_tab[i].users, 0);
	}
	mutex_init(&p_mod->sensor_id_lock);
	wake_lock_init(&p_mod->wakelock, WAKE_LOCK_SUSPEND,
                   "pm_message_wakelock_sensor_k");
	platform_set_drvdata(pdev, p_mod);
	atomic_set(&p_mod->total_users, 0);

	ret = misc_register(&sensor_dev);
	if (ret) {
		printk(KERN_ERR "can't reg miscdev on minor=%d (%d)\n",
			SENSOR_MINOR, ret);
		goto misc_register_error;
	}
	p_mod->of_node = pdev->dev.of_node;
	sensor_dev.this_device->platform_data = (void *)p_mod;
	for (i = 0; i < SENSOR_DEV_MAX; i++) {
		get_gpio_id(p_mod->of_node, &gpio_tab.pwn, &gpio_tab.reset, i);
		ret = gpio_request(gpio_tab.pwn, NULL);
		if (ret) {
			tmp = 1;
			printk("sensor: gpio already request pwn %d %d.\n", gpio_tab.pwn, i);
		}
		ret = gpio_request(gpio_tab.reset, NULL);
		if (ret) {
			tmp = 1;
			printk("sensor:  gpio already request reset %d %d.\n", gpio_tab.reset, i);
		}
	}

	get_gpio_id_ex(p_mod->of_node, GPIO_CAMDVDD, &gpio_id, 0);
	ret = gpio_request(gpio_id, NULL);
	if (ret) {
		tmp = 1;
		printk("sensor: gpio already request GPIO_CAMDVDD %d.\n", gpio_id);
	}

	ret = sensor_k_register_subdevs(pdev);
	if (ret) {
		printk(KERN_ERR "can't reg sub dev=%d (%d)\n",
			SENSOR_MINOR, ret);
		goto misc_register_error;
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
	}
	goto exit;

misc_register_error:
	misc_deregister(&sensor_dev);

	if (SENSOR_ADDR_INVALID(p_mod)) {
		printk("SENSOR: Invalid addr, 0x%x", (uint32_t)p_mod);
	} else {
		vfree(p_mod);
		p_mod = NULL;
		platform_set_drvdata(pdev, NULL);
	}
exit:
	if (ret) {
		printk(KERN_ERR "sensor prb fail req gpio %d err %d\n",
			tmp, ret);
	} else {
		printk(KERN_ALERT " sensor prb Success\n");
	}

	return ret;
}

LOCAL int sensor_k_remove(struct platform_device *dev)
{
<<<<<<< HEAD
	printk(KERN_INFO "sensor_k_remove : Sensor remove called\n");

	if (s_p_sensor_mod->pin_sub_reset != s_p_sensor_mod->pin_main_reset)
	{
		gpio_free(s_p_sensor_mod->pin_sub_reset);
	}

	if (s_p_sensor_mod->pin_sub_pd != s_p_sensor_mod->pin_main_pd)
	{
		gpio_free(s_p_sensor_mod->pin_sub_pd);
	}

	gpio_free(s_p_sensor_mod->pin_main_reset);
	gpio_free(s_p_sensor_mod->pin_main_pd);

#if defined(CONFIG_MACH_CORE3_W) || defined(CONFIG_MACH_J13G) || defined(CONFIG_MACH_GRANDNEOVE3G)	\
	|| defined(CONFIG_MACH_VIVALTO5MVE3G) || defined(CONFIG_MACH_GOYAVE3G_SWA)	\
	|| defined(CONFIG_MACH_GTEL3G) || defined(CONFIG_MACH_GTELWIFI) || defined(CONFIG_MACH_YOUNG33G)
	gpio_free(s_p_sensor_mod->pin_main_camdvdd_en);
#endif

#if defined(CONFIG_MACH_YOUNG33G)
	gpio_free(s_p_sensor_mod->pin_main_camavdd_en);
#endif
=======
	struct sensor_module_tab_tag *p_mod = platform_get_drvdata(dev);
	struct sensor_gpio_tag       gpio_tab;
	int                          i;
	int                          gpio_id = 0;

	SENSOR_CHECK_ZERO(p_mod);

	printk(KERN_INFO "sensor remove called !\n");

	for (i = 0; i < SENSOR_DEV_MAX; i++) {
		get_gpio_id(p_mod->of_node, &gpio_tab.pwn, &gpio_tab.reset, i);
		gpio_free(gpio_tab.pwn);
		gpio_free(gpio_tab.reset);
	}

	get_gpio_id_ex(p_mod->of_node, GPIO_CAMDVDD, &gpio_id, 0);
	gpio_free(gpio_id);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g

	misc_deregister(&sensor_dev);
	wake_lock_destroy(&p_mod->wakelock);

	if (SENSOR_ADDR_INVALID(p_mod)) {
		printk("SENSOR: Invalid addr, 0x%x", (uint32_t)p_mod);
	} else {
		vfree(p_mod);
		p_mod = NULL;
		platform_set_drvdata(dev, NULL);
	}
	printk(KERN_INFO "sensor remove Success !\n");
	return 0;
}

LOCAL const struct of_device_id of_match_table_sensor[] = {
	{ .compatible = "sprd,sprd_sensor", },
	{ },
};

static struct platform_driver sensor_dev_driver = {
	.probe = sensor_k_probe,
	.remove =sensor_k_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name  = SENSOR_DEVICE_NAME,
		.of_match_table = of_match_ptr(of_match_table_sensor),
	},
};

<<<<<<< HEAD
struct class *camera_class;

#if defined(CONFIG_MACH_YOUNG23GDTV) || defined(CONFIG_MACH_VIVALTO3MVE3G_LTN) || defined(CONFIG_MACH_VIVALTO3MVEML3G)

#define REAR_SENSOR_NAME	"SR352 N\n"

#if defined(CONFIG_MACH_VIVALTO3MVEML3G_SEA)
#define FRONT_SENSOR_NAME	"SR030PC50M N\n"
#endif

#elif defined(CONFIG_MACH_VIVALTO5MVE3G) || defined(CONFIG_MACH_YOUNG33G)

#define REAR_SENSOR_NAME	"S5K4ECGX N\n"
#define FRONT_SENSOR_NAME	"SR030PC50M N\n"

#elif defined(CONFIG_MACH_GRANDNEOVE3G)

#define REAR_SENSOR_NAME	"S5K4EC N\n"
#define FRONT_SENSOR_NAME	"SR200PC20M N\n"

#elif defined(CONFIG_MACH_GOYAVE3G_SWA)
#define REAR_SENSOR_NAME	"S5K4ECGX N\n"

#elif defined(CONFIG_MACH_GOYAVE3G) || defined(CONFIG_MACH_GOYAVEWIFI) || defined(CONFIG_MACH_GOYAVE3G_SEA) || defined(CONFIG_MACH_GOYAVEWIFI_SEA_XTC)

#define REAR_SENSOR_NAME	"SR200PC20 N\n"

#if defined(CONFIG_MACH_GOYAVE3G_SEA) || defined(CONFIG_MACH_GOYAVEWIFI_SEA_XTC)
#define FRONT_SENSOR_NAME	"SR200PC20M N\n"
#endif

#elif defined(CONFIG_MACH_J13G) 

#define REAR_SENSOR_NAME	"SR541 N\n"
#define FRONT_SENSOR_NAME	"SR200PC20M N\n"

#elif defined(CONFIG_MACH_GTEL3G) || defined(CONFIG_MACH_GTELWIFI)

#define REAR_SENSOR_NAME	"S5K4ECGX N\n"
#define FRONT_SENSOR_NAME	"SR200PC20M N\n"

#else

#define REAR_SENSOR_NAME	"S5K4ECGX N\n"
#define FRONT_SENSOR_NAME	"SR200PC20M N\n"

#endif

#define SENSOR_TYPE "SOC\n"

#if defined (CONFIG_MACH_CORE3_W) || defined(CONFIG_MACH_J13G) || defined (CONFIG_MACH_GRANDNEOVE3G)\
    || defined (CONFIG_MACH_VIVALTO5MVE3G) || defined(CONFIG_MACH_VIVALTO3MVE3G_LTN)	\
    || defined(CONFIG_MACH_VIVALTO3MVEML3G) || defined(CONFIG_MACH_YOUNG33G)

LOCAL int _Sensor_K_SetTorch(uint32_t flash_mode)
{
	printk("_Sensor_K_SetTorch : mode %d, flash_torch_status = %d\n", flash_mode, flash_torch_status);
	switch (flash_mode)
	{
		case 1: /* For torch */
			flash_torch_status=1;
			sm5701_led_ready(MOVIE_MODE);
			sm5701_set_fleden(SM5701_FLEDEN_ON_MOVIE);
			break;

		case 0:
			flash_torch_status=0;
			sm5701_set_fleden(SM5701_FLEDEN_DISABLED);
			sm5701_led_ready(LED_DISABLE);
			break;

		default:
			printk("_Sensor_K_SetTorch : Un-know mode : flash_mode : 0x%x\n", flash_mode);
			break;
	}
	printk("_Sensor_K_SetTorch : Flash_mode : 0x%x\n", flash_mode);

	return 0;
}
#endif

static ssize_t Rear_Cam_Sensor_ID(struct device *dev, struct device_attribute *attr, char *buf)
{
	SENSOR_PRINT("Rear_Cam_Sensor_ID\n");
	return sprintf(buf, REAR_SENSOR_NAME);
}

static ssize_t Rear_Cam_FW_Store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	 SENSOR_PRINT("Rear_Cam_FW_Store value\n");
	 return 0;
}

static ssize_t Rear_Cam_Type_Store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	 SENSOR_PRINT("Rear_Cam_Type_Store value \n");
	 return 0;
}

static ssize_t Cam_Sensor_TYPE(struct device *dev, struct device_attribute *attr, char *buf)
{
	SENSOR_PRINT("Cam_Sensor_type\n");
	return sprintf(buf, SENSOR_TYPE);
}


#if defined (CONFIG_MACH_CORE3_W) || defined(CONFIG_MACH_J13G) || defined (CONFIG_MACH_GRANDNEOVE3G)\
    || defined (CONFIG_MACH_VIVALTO5MVE3G) || defined(CONFIG_MACH_VIVALTO3MVE3G_LTN)	\
    || defined(CONFIG_MACH_VIVALTO3MVEML3G) || defined(CONFIG_MACH_YOUNG33G)

static ssize_t Rear_Cam_store_flash(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	sscanf(buf, "%d", &value);
	printk("Rear_Cam_store_flash value = %d\n", value);
	_Sensor_K_SetTorch(value);
	return size;
}

static ssize_t Rear_Cam_show_flash(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	SENSOR_PRINT("Rear_Cam_show_flash value\n");
	return sprintf(buf, "%d", flash_torch_status);
}
#endif

#if defined(FRONT_SENSOR_NAME)
static ssize_t Front_Cam_Sensor_ID(struct device *dev, struct device_attribute *attr, char *buf)
{
	SENSOR_PRINT("Front_Cam_Sensor_ID\n");
	return sprintf(buf, FRONT_SENSOR_NAME);
}

static ssize_t Front_Cam_FW_Store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	 SENSOR_PRINT("Front_Cam_FW_Store value\n");
	 return 0;
}

static ssize_t Front_Cam_Type_Store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	 SENSOR_PRINT("Front_Cam_Type_Store value \n");
	 return 0;
}
#endif

#if defined(CONFIG_MACH_CORE3_W) || defined(CONFIG_MACH_GRANDNEOVE3G) || defined(CONFIG_MACH_VIVALTO5MVE3G)	\
	|| defined(CONFIG_MACH_J13G) || defined(CONFIG_MACH_GOYAVE3G_SWA)	\
	|| defined(CONFIG_MACH_GTEL3G) || defined(CONFIG_MACH_GTELWIFI)	\
	|| defined(CONFIG_MACH_YOUNG33G)
static ssize_t S5K4ECGX_camera_vendorid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	count = sprintf(buf, "0x%04X", VENDOR_ID);
	printk("%s : vendor ID is 0x%04X\n", __func__, VENDOR_ID);
	return count;
}

static ssize_t S5K4ECGX_camera_vendorid_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int tmp = 0;
	sscanf(buf, "%x", &tmp);
	VENDOR_ID = tmp;
	printk("%s : vendor ID is 0x%04X\n", __func__, VENDOR_ID);
	return size;
}

static ssize_t S5K4ECGX_camera_antibanding_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	count = sprintf(buf, "%d", camera_antibanding_val);
	printk("%s : antibanding is %d\n", __func__, camera_antibanding_val);
	return count;
}

static ssize_t S5K4ECGX_camera_antibanding_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int tmp = 0;
	sscanf(buf, "%d", &tmp);
	camera_antibanding_val = tmp;
	printk("%s : antibanding is %d\n", __func__, camera_antibanding_val);
	return size;
}

#elif defined(CONFIG_USE_CSC_FEATURE_FOR_ANTIBANDING)

static ssize_t Sensor_camera_antibanding_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	count = sprintf(buf, "%d", camera_antibanding_val);
	SENSOR_PRINT("%s : antibanding is %d\n", __func__, camera_antibanding_val);
	return count;
}

static ssize_t Sensor_camera_antibanding_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int tmp = 0;
	sscanf(buf, "%d", &tmp);
	camera_antibanding_val = tmp;
	SENSOR_PRINT("%s : antibanding is %d\n", __func__, camera_antibanding_val);
	return size;
}
#endif


static DEVICE_ATTR(rear_camfw, S_IRUGO | S_IXOTH, Rear_Cam_Sensor_ID, NULL); // Read(User, Group, Other), Execute(Other)
static DEVICE_ATTR(rear_type, S_IRUGO | S_IXOTH, Cam_Sensor_TYPE, NULL); // Read(User, Group, Other), Execute(Other)

#if defined (CONFIG_MACH_CORE3_W) || defined(CONFIG_MACH_J13G) || defined (CONFIG_MACH_GRANDNEOVE3G)\
    || defined (CONFIG_MACH_VIVALTO5MVE3G) || defined(CONFIG_MACH_VIVALTO3MVE3G_LTN)	\
    || defined(CONFIG_MACH_VIVALTO3MVEML3G) || defined(CONFIG_MACH_YOUNG33G)
static DEVICE_ATTR(rear_flash, S_IWUSR | S_IWGRP | S_IXOTH, Rear_Cam_show_flash, Rear_Cam_store_flash); // Write(User, Group), Execute(Other)
#endif


#if defined(FRONT_SENSOR_NAME)
static DEVICE_ATTR(front_camfw, S_IRUGO | S_IXOTH, Front_Cam_Sensor_ID, NULL); // Read(User, Group, Other), Execute(Other)
#endif

static DEVICE_ATTR(front_type, S_IRUGO | S_IXOTH, Cam_Sensor_TYPE, NULL); // Read(User, Group, Other), Execute(Other)

#if  defined(CONFIG_MACH_CORE3_W)|| defined(CONFIG_MACH_GRANDNEOVE3G) || defined(CONFIG_MACH_VIVALTO5MVE3G)	\
	|| defined(CONFIG_MACH_J13G) || defined(CONFIG_MACH_GOYAVE3G_SWA) || defined(CONFIG_MACH_YOUNG33G)	\
	|| defined(CONFIG_MACH_GTEL3G) || defined(CONFIG_MACH_GTELWIFI)

static struct device_attribute S5K4ECGX_camera_vendorid_attr = {
	.attr = {
		.name = "rear_vendorid",
		.mode = (S_IRUSR|S_IRGRP | S_IWUSR|S_IWGRP)},
	.show = S5K4ECGX_camera_vendorid_show,
	.store = S5K4ECGX_camera_vendorid_store
};

static struct device_attribute S5K4ECGX_camera_antibanding_attr = {
	.attr = {
		.name = "Cam_antibanding",
		.mode = (S_IRUSR|S_IRGRP | S_IWUSR|S_IWGRP)},
	.show = S5K4ECGX_camera_antibanding_show,
	.store = S5K4ECGX_camera_antibanding_store
};

#elif defined(CONFIG_USE_CSC_FEATURE_FOR_ANTIBANDING)

static struct device_attribute Sensor_camera_antibanding_attr = {
	.attr = {
		.name = "Cam_antibanding",
		.mode = (S_IRUSR|S_IRGRP | S_IWUSR|S_IWGRP)
	},
	.show = Sensor_camera_antibanding_show,
	.store = Sensor_camera_antibanding_store
};

#endif


int __init sensor_k_init(void)
{
	printk(KERN_INFO "sensor_k_init : E\n");

	int i = 0;
	int err = 0;
	int numberOfPin = sizeof(camera_stby_rst_pinmap)/sizeof(camera_pinmap_t);

	s_p_sensor_mod = (struct sensor_module *)vzalloc(sizeof(struct sensor_module));
	SENSOR_CHECK_ZERO(s_p_sensor_mod);

	s_p_sensor_mod->sensor_id = SENSOR_ID_MAX;
	mutex_init(&s_p_sensor_mod->sensor_lock);

	if (platform_driver_register(&sensor_dev_driver) != 0)
	{
		printk("sensor_k_init : platform device register Failed\n");
		vfree(s_p_sensor_mod);
		s_p_sensor_mod=NULL;
		return SENSOR_K_FAIL;
	}

	flash_torch_status = 0;

	struct device *dev_t_rear;
	struct device *dev_t_front;

	camera_class = class_create(THIS_MODULE, "camera");
	if (IS_ERR(camera_class))
	{
		SENSOR_PRINT("Failed to create camera_class!\n");
		platform_driver_unregister(&sensor_dev_driver);
		return PTR_ERR( camera_class );
	}
	
	dev_t_rear = device_create(camera_class, NULL, 0, "%s", "rear");
	if (IS_ERR(dev_t_rear)) 
	{
		platform_driver_unregister(&sensor_dev_driver);
		class_destroy(camera_class);
		SENSOR_PRINT("Failed to create camera_dev!\n");
		return PTR_ERR( dev_t_rear );
	}

	err = device_create_file(dev_t_rear, &dev_attr_rear_camfw);
	if(err)
	{
		SENSOR_PRINT("Failed to create device file(%s)!\n", dev_attr_rear_camfw.attr.name);
		goto err_make_rear_camfw_file;
	}

	err = device_create_file(dev_t_rear, &dev_attr_rear_type);
	if(err)
	{
		SENSOR_PRINT("Failed to create device file(%s)!\n", dev_attr_rear_type.attr.name);
		goto err_make_rear_type_file;
	}

#if defined (CONFIG_MACH_CORE3_W) || defined(CONFIG_MACH_J13G) || defined (CONFIG_MACH_GRANDNEOVE3G)\
    || defined (CONFIG_MACH_VIVALTO5MVE3G) || defined(CONFIG_MACH_VIVALTO3MVE3G_LTN)	\
    || defined(CONFIG_MACH_VIVALTO3MVEML3G) || defined(CONFIG_MACH_YOUNG33G)

	err = device_create_file(dev_t_rear, &dev_attr_rear_flash);
	if(err)
	{
		SENSOR_PRINT("Failed to create device file(%s)!\n", dev_attr_rear_flash.attr.name);
		goto err_make_rear_flash_file;
	}
#endif

#if defined(FRONT_SENSOR_NAME)
	dev_t_front = device_create(camera_class, NULL, 0, "%s", "front");
	if (IS_ERR(dev_t_front))
	{
		SENSOR_PRINT("Failed to create camera_dev front!\n");
		err = PTR_ERR( dev_t_front );
		goto err_make_front_device;
	}

	err = device_create_file(dev_t_front, &dev_attr_front_camfw);
	if(err)
	{
		SENSOR_PRINT("Failed to create device file(%s)!\n", dev_attr_front_camfw.attr.name);
		goto err_make_front_camfw_file;
	}
	
	err = device_create_file(dev_t_front, &dev_attr_front_type);
	if(err)
	{
		SENSOR_PRINT("Failed to create device file(%s)!\n", dev_attr_front_type.attr.name);
		goto err_make_front_type_file;
	}
#endif

#if  defined(CONFIG_MACH_CORE3_W)|| defined(CONFIG_MACH_GRANDNEOVE3G) || defined(CONFIG_MACH_VIVALTO5MVE3G)	\
	|| defined(CONFIG_MACH_J13G) || defined(CONFIG_MACH_GOYAVE3G_SWA) || defined(CONFIG_MACH_YOUNG33G)	\
	|| defined(CONFIG_MACH_GTEL3G) || defined(CONFIG_MACH_GTELWIFI)
	err = device_create_file(dev_t_rear, &S5K4ECGX_camera_vendorid_attr);
	if(err)
	{
		SENSOR_PRINT("Failed to create device file(%s)!\n", S5K4ECGX_camera_vendorid_attr.attr.name);
		goto err_make_camera_vendorid;
	}

	err = device_create_file(dev_t_rear, &S5K4ECGX_camera_antibanding_attr);
	if(err)
	{
		SENSOR_PRINT("Failed to create device file(%s)!\n", S5K4ECGX_camera_antibanding_attr.attr.name);
		goto err_make_camera_antibanding;
	}
#elif defined(CONFIG_USE_CSC_FEATURE_FOR_ANTIBANDING)
	err = device_create_file(dev_t_rear, &Sensor_camera_antibanding_attr);
	if(err)
	{
		SENSOR_PRINT("Failed to create device file(%s)!\n", Sensor_camera_antibanding_attr.attr.name);
		goto err_make_camera_antibanding;
	}
#endif

	return 0;


#if defined(FRONT_SENSOR_NAME)
	err_make_front_type_file:
		device_remove_file(dev_t_front, &dev_attr_front_type);
	err_make_front_camfw_file:
		device_remove_file(dev_t_front, &dev_attr_front_camfw);
	err_make_front_device:
		device_destroy(camera_class, dev_t_front);
#endif		

#if defined (CONFIG_MACH_CORE3_W) || defined(CONFIG_MACH_J13G) || defined (CONFIG_MACH_GRANDNEOVE3G)\
    || defined (CONFIG_MACH_VIVALTO5MVE3G) || defined(CONFIG_MACH_VIVALTO3MVE3G_LTN)	\
    || defined(CONFIG_MACH_VIVALTO3MVEML3G) || defined(CONFIG_MACH_YOUNG33G)
	err_make_rear_flash_file:
		device_remove_file(dev_t_rear, &dev_attr_rear_flash);
#endif

#if  defined(CONFIG_MACH_CORE3_W)|| defined(CONFIG_MACH_GRANDNEOVE3G) || defined(CONFIG_MACH_VIVALTO5MVE3G)	\
	|| defined(CONFIG_MACH_J13G) || defined(CONFIG_MACH_GOYAVE3G_SWA) ||	defined(CONFIG_MACH_YOUNG33G)\
	|| defined(CONFIG_MACH_GTEL3G) || defined(CONFIG_MACH_GTELWIFI)
	err_make_camera_vendorid:
		device_remove_file(dev_t_rear, &S5K4ECGX_camera_vendorid_attr);
	err_make_camera_antibanding:
		device_remove_file(dev_t_rear, &S5K4ECGX_camera_antibanding_attr);
#elif defined(CONFIG_USE_CSC_FEATURE_FOR_ANTIBANDING)
	err_make_camera_antibanding:
		device_remove_file(dev_t_rear, &Sensor_camera_antibanding_attr);
#endif


	err_make_rear_type_file:
		device_remove_file(dev_t_rear, &dev_attr_rear_type);
	err_make_rear_camfw_file:
		vfree(s_p_sensor_mod);
		s_p_sensor_mod=NULL;
		device_destroy(camera_class,dev_t_rear);
		class_destroy(camera_class);
		platform_driver_unregister(&sensor_dev_driver);

	return err;
=======
int __init sensor_k_init(void)
{
	printk(KERN_INFO "sensor_k_init called !\n");

	if (platform_driver_register(&sensor_dev_driver) != 0) {
		printk("platform device register Failed \n");
		return SENSOR_K_FAIL;
	}
	return 0;
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
}

void sensor_k_exit(void)
{
<<<<<<< HEAD
	printk(KERN_INFO "sensor_k_exit\n");
	platform_driver_unregister(&sensor_dev_driver);

	if (SENSOR_ADDR_INVALID(s_p_sensor_mod))
	{
		printk("sensor_k_exit : Invalid addr, 0x%x", (uint32_t)s_p_sensor_mod);
	}
	else
	{
		vfree(s_p_sensor_mod);
		s_p_sensor_mod = NULL;
	}
=======
	printk(KERN_INFO "sensor_k_exit called !\n");
	platform_driver_unregister(&sensor_dev_driver);
>>>>>>> ef4e2d99b0e... drivers: Merge upstream camera source for scx30g
}

module_init(sensor_k_init);
module_exit(sensor_k_exit);

MODULE_DESCRIPTION("Sensor Driver");
MODULE_LICENSE("GPL");

#if defined(CONFIG_MACH_VIVALTO3MVE3G_LTN) || defined(CONFIG_MACH_VIVALTO3MVEML3G)
#define CONFIG_MACH_VIVALTO5MVE3G y
#endif
