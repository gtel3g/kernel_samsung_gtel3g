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
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/spinlock_types.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <video/isp_drv_kernel.h>
#include <mach/sci.h>
#include <linux/clk.h>
#include <asm/cacheflush.h>
#include <mach/hardware.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/vmalloc.h>

#if IS_ENABLED(VERSION3T) || IS_ENABLED(VERSION3L)
#include "Shark_reg_isp.h"
#elif IS_ENABLED(VERSION2)
#include "Tiger_reg_isp.h"
#else
#error "Unknown architecture specification"
#endif

#include "parse_hwinfo.h"


#define DEBUG_ISP_DRV
#ifdef DEBUG_ISP_DRV
#define ISP_PRINT   printk
#else
#define ISP_PRINT(...)
#endif

#define ISP_QUEUE_LENGTH 16
#define SA_SHIRQ	IRQF_SHARED

#define ISP_MINOR		MISC_DYNAMIC_MINOR/*isp minor number*/
#define init_MUTEX(sem)		sema_init(sem, 1)
#define init_MUTEX_LOCKED(sem)	sema_init(sem, 0)
#define IO_PTR          volatile void __iomem *
#define ISP_READL(a)	__raw_readl((IO_PTR)a)
#define ISP_WRITEL(a,v)	__raw_writel(v,(IO_PTR)a)
#define ISP_OWR(a,v)	__raw_writel((__raw_readl((IO_PTR)a) | v), (IO_PTR)a)
#define ISP_AWR(a,v)	__raw_writel((__raw_readl((IO_PTR)a) & v), (IO_PTR)a)
#define ISP_NAWR(a,v)	__raw_writel((__raw_readl((IO_PTR)a) & ~v), (IO_PTR)a)
#define ISP_REG_RD(a)	ISP_READL(((IO_PTR)a))
#define DEBUG_STR                         "Error L %d, %s \n"
#define DEBUG_ARGS                        __LINE__,__FUNCTION__
#define ISP_LOWEST_ADDR                0x800
#define ISP_ADDR_INVALID(addr)         ((uint32_t)(addr) < ISP_LOWEST_ADDR)
#define ISP_CHECK_ZERO(a)                                      \
	do {                                                       \
		if (ISP_ADDR_INVALID(a)) {                       \
			printk("isp_k, zero pointer \n");           \
			printk(DEBUG_STR, DEBUG_ARGS);               \
			return -EFAULT;                              \
		}                                                   \
	} while(0)

#define ISP_CHECK_ZERO_VOID(a)                                 \
	do {                                                       \
		if (ISP_ADDR_INVALID(a)) {                       \
			printk("isp_k, zero pointer \n");           \
			printk(DEBUG_STR, DEBUG_ARGS);               \
			return -EFAULT;                                      \
		}                                                   \
	} while(0)

#define ISP_DCAM_IRQ_MASK	0x03
#define ISP_DCAM_IRQ_NUM	0x02
#if IS_ENABLED(VERSION2)
#define ISP_BUF_MAX_SIZE ISP_TMP_BUF_SIZE_MAX_V0000
#define ISP_IRQ_NUM ISP_IRQ_NUM_V0000
#define ISP_IRQ_HW_MASK ISP_IRQ_HW_MASK_V0000
#elif IS_ENABLED(VERSION3L) || IS_ENABLED(VERSION3T)
#define ISP_BUF_MAX_SIZE ISP_TMP_BUF_SIZE_MAX_V0001
#define ISP_IRQ_NUM ISP_IRQ_NUM_V0001
#define ISP_IRQ_HW_MASK ISP_IRQ_HW_MASK_V0001
#endif
#define ISP_TIME_OUT_MAX (500)

struct isp_node {
	uint32_t	isp_irq_val;
	uint32_t	dcam_irq_val;
};

struct isp_queue {
	struct isp_node 	node[ISP_QUEUE_LENGTH];
	struct isp_node 	*write;
	struct isp_node 	*read;
};

struct isp_k_file
{
	uint32_t reg_base_addr;/*the pointer of isp register context*/
	uint32_t size;/* struct size*/
	uint32_t buf_addr;
	uint32_t buf_len;
	struct isp_queue queue;
	struct semaphore sem_isr;/*for interrupts*/
	struct semaphore sem_isp;/*for the isp device, protect the isp hardware; protect  only  one caller use the oi*/ 
				/*controll/read/write functions*/
	struct clk* s_isp_clk_mm_i;
	struct clk* s_isp_clk;
	atomic_t s_isp_users;
	unsigned long s_isp_alloc_addr;
	unsigned long s_isp_alloc_order;
	unsigned long s_isp_alloc_len;
	struct mutex s_isp_lock;
	spinlock_t isp_spin_lock;
};

struct isp_buf_phy {
	unsigned long buf_phy_ptr;
	unsigned long buf_vir_ptr;
	unsigned long buf_len;
};

uint32_t s_dcam_int_eb = 0x00;

static int32_t _isp_set_clk(void *handle, enum isp_clk_sel clk_sel);
static int32_t _isp_module_eb(void *handle);
static int32_t _isp_module_dis(void *handle);
static int  _isp_registerirq(void *handle);
static void _isp_unregisterirq(void *handle);
static irqreturn_t _isp_irq_root(int irq, void *handle);
void _dcam_isp_root(void);
static int _isp_queue_init(struct isp_queue *queue);
static int _isp_queue_write(struct isp_queue *queue, struct isp_node *node);
static int _isp_queue_read(struct isp_queue *queue, struct isp_node *node);
static inline void _isp_regread(char *dst,  char *src, size_t n);
static inline void  _isp_regwrite(char *dst,  char *src, size_t n);
static void _read_reg(struct isp_reg_bits *reg_bits_ptr, uint32_t counts);
static void _write_reg(struct isp_reg_bits *reg_bits_ptr, uint32_t counts);

/**file operation functions declare**/
static int32_t _isp_kernel_open(struct inode *node, struct file *filp);
static int32_t _isp_kernel_release(struct inode *node, struct file *filp);
static long _isp_kernel_ioctl( struct file *fl, unsigned int cmd, unsigned long param);
/**driver'  functions declare**/
static int32_t _isp_probe(struct platform_device *pdev);
static int32_t _isp_remove(struct platform_device *dev);
/**module'  functions declare**/
static int32_t __init isp_kernel_init(void);
static void isp_kernel_exit(void);

static struct file_operations isp_fops = {
	.owner	= THIS_MODULE,
	.open	= _isp_kernel_open,
	.unlocked_ioctl	= _isp_kernel_ioctl,
	.release	= _isp_kernel_release,
};

static struct miscdevice isp_dev = {
	.minor	= ISP_MINOR,
	.name	= "sprd_isp",
	.fops	= &isp_fops,
};

static const struct of_device_id of_match_table_isp[] = {
	{ .compatible = "sprd,sprd_isp", },
	{ },
};

static struct platform_driver isp_driver = {
	.probe	= _isp_probe,
	.remove	= _isp_remove,
	.driver	= {
		.owner = THIS_MODULE,
		.name = "sprd_isp",
		.of_match_table = of_match_ptr(of_match_table_isp),
	},
};

static int32_t _isp_module_eb(void *handle)
{
	int32_t ret = 0;
	struct isp_k_file *fd = (struct isp_k_file*)handle;
	if (NULL == handle) {
		ISP_PRINT("_isp_module_eb handle null\n");
		return -ENOENT;
	}
	if (0x01 == atomic_inc_return(&fd->s_isp_users)) {

		ret = clk_mm_i_eb(isp_dev.this_device->of_node,1);
#if IS_ENABLED(VERSION3L) || IS_ENABLED(VERSION3T)
		ret = _isp_set_clk(fd, ISP_CLK_312M);
#else
		ret = _isp_set_clk(fd, ISP_CLK_256M);
#endif
		if (unlikely(0 != ret)) {
			ISP_PRINT("isp_k: set clock error\n");
			ret = -EIO;
		}
	}

	ISP_PRINT("_isp_module_eb: end\n");

	return ret;
}

static int32_t _isp_module_dis(void *handle)
{
	int32_t	ret = 0;
	struct isp_k_file *fd = (struct isp_k_file*)handle;
	if (NULL == handle) {
		ISP_PRINT("_isp_module_eb handle null\n");
		return -ENOENT;
	}
	if (0x00 == atomic_dec_return(&fd->s_isp_users)) {

		ret = _isp_set_clk(fd, ISP_CLK_NONE);
		if (unlikely(0 != ret)) {
			ISP_PRINT("isp_k: close clock error\n");
			ret = -EFAULT;
			return ret;
		}
		ret =  clk_mm_i_eb(isp_dev.this_device->of_node,0);
	}
	return ret;
}

static int32_t _isp_module_rst(void *handle)
{
	int32_t ret = 0;
	uint32_t reg_value=0x00;
	int32_t time_out_cnt = 0;
	struct isp_k_file *fd = (struct isp_k_file*)handle;
	if (NULL == handle) {
		ISP_PRINT("_isp_module_eb handle null\n");
		return -ENOENT;
	}
	if (0x00 != atomic_read(&fd->s_isp_users)) {

#if IS_ENABLED(VERSION3L) || IS_ENABLED(VERSION3T)
		ISP_OWR(ISP_AXI_MASTER_STOP, BIT_0);
#endif
		reg_value=ISP_READL(ISP_AXI_MASTER);
		while((0x00==(reg_value&0x08)) && (time_out_cnt < ISP_TIME_OUT_MAX))
		{
			time_out_cnt++;
			udelay(50);
			reg_value=ISP_READL(ISP_AXI_MASTER);
		}
		if (time_out_cnt >= ISP_TIME_OUT_MAX) {
			ret = -1;
			ISP_PRINT("_isp_module_rst: time out\n");
		}

		ISP_WRITEL(ISP_INT_CLEAR, ISP_IRQ_HW_MASK);

#if IS_ENABLED(VERSION3L) || IS_ENABLED(VERSION3T)
		sci_glb_set(ISP_MODULE_RESET, ISP_RST_LOG_BIT);
		sci_glb_set(ISP_MODULE_RESET, ISP_RST_CFG_BIT);
		sci_glb_set(ISP_MODULE_RESET, ISP_RST_LOG_BIT);
		sci_glb_set(ISP_MODULE_RESET, ISP_RST_CFG_BIT);
		sci_glb_set(ISP_MODULE_RESET, ISP_RST_LOG_BIT);
		sci_glb_set(ISP_MODULE_RESET, ISP_RST_CFG_BIT);

		sci_glb_clr(ISP_MODULE_RESET, ISP_RST_CFG_BIT);
		sci_glb_clr(ISP_MODULE_RESET, ISP_RST_LOG_BIT);
#elif IS_ENABLED(VERSION2)
		sci_glb_set(ISP_MODULE_RESET, ISP_RST_BIT);
		sci_glb_set(ISP_MODULE_RESET, ISP_RST_BIT);
		sci_glb_set(ISP_MODULE_RESET, ISP_RST_BIT);

		sci_glb_clr(ISP_MODULE_RESET, ISP_RST_BIT);
#else
#error "Unknown architecture specification"
#endif

	}

	ISP_PRINT("_isp_module_rst: exit\n");

	return ret;
}

static int32_t _isp_lnc_param_load(void *handle, struct isp_reg_bits *reg_bits_ptr, uint32_t counts)
{
	int32_t ret = 0;
	int32_t time_out_cnt = 0;
	uint32_t reg_value=0x00;
	struct isp_k_file *fd = (struct isp_k_file*)handle;
	if (NULL == handle) {
		ISP_PRINT("_isp_module_eb handle null\n");
		return -ENOENT;
	}
	if((0x00 != fd->s_isp_alloc_addr)
		&&(0x00 != fd->s_isp_alloc_len)) {

		void *ptr = (void*)fd->s_isp_alloc_addr;
		uint32_t len = fd->s_isp_alloc_len;
		dmac_flush_range(ptr, ptr + len);
		outer_flush_range(__pa(ptr), __pa(ptr) + len);

		reg_bits_ptr->reg_value = (uint32_t)__pa(reg_bits_ptr->reg_value);
#if defined(CONFIG_MACH_CORE3)
		{
			unsigned long flags;

			local_irq_save(flags);

			reg_value=ISP_READL(ISP_LNC_STATUS);
			while((0x00==(reg_value&ISP_LNC_STATUS_OK)) && (time_out_cnt < (ISP_TIME_OUT_MAX*1000))) {
				udelay(1);
				reg_value=ISP_READL(ISP_LNC_STATUS);
				time_out_cnt++;
			}
			if (time_out_cnt >= (ISP_TIME_OUT_MAX*1000)) {
				ret = -1;
				ISP_PRINT("isp_k: isp lnc status time out\n");
			}

			_write_reg(reg_bits_ptr, counts);
			local_irq_restore(flags);
		}
#else
		_write_reg(reg_bits_ptr, counts);
#endif
		reg_value=ISP_READL(ISP_INT_RAW);

		while ((0x00 == (reg_value&ISP_INT_LENS_LOAD)) && (time_out_cnt < ISP_TIME_OUT_MAX)) {
			udelay(1);
			reg_value=ISP_READL(ISP_INT_RAW);
			time_out_cnt++;
		}
		if (time_out_cnt >= ISP_TIME_OUT_MAX) {
			ret = -1;
			ISP_PRINT("isp_k: isp load lnc param time out\n");
		}
		ISP_OWR(ISP_INT_CLEAR, ISP_INT_LENS_LOAD);
	}else {
		ISP_PRINT("isp_k: isp load lnc param error\n");
	}

	return ret;
}

static int32_t _isp_lnc_param_set(void *handle, uint32_t* addr, uint32_t len)
{
	int32_t ret = 0;
	struct isp_k_file *fd = (struct isp_k_file*)handle;
	if (NULL == handle) {
		ISP_PRINT("_isp_module_eb handle null\n");
		return -ENOENT;
	}
	if((0x00 != fd->s_isp_alloc_addr)
		&&(0x00!=addr)) {
		memcpy((void*)fd->s_isp_alloc_addr, (void*)addr, len);
	}

	return ret;
}

static int32_t _isp_update(void *handle)
{
	int32_t ret = 0;
	struct isp_k_file *fd = (struct isp_k_file*)handle;
	if (NULL == handle) {
		ISP_PRINT("_isp_module_eb handle null\n");
		return -ENOENT;
	}
	if((0x00 != fd->s_isp_alloc_addr)
		&&(0x00 != fd->s_isp_alloc_order)) {
		fd->s_isp_alloc_order = 0x00;
		fd->s_isp_alloc_addr = 0x00;
		fd->s_isp_alloc_len=0x00;
	}

	return ret;
}

static int32_t _isp_set_clk(void *handle, enum isp_clk_sel clk_sel)
{
	struct clk              *clk_parent;
	char                    *parent = "clk_256m";
	int32_t       rtn = 0;
	struct isp_k_file *fd = (struct isp_k_file*)handle;
	ISP_CHECK_ZERO(fd);

#if IS_ENABLED(VERSION3L) || IS_ENABLED(VERSION3T)
	switch (clk_sel) {
	case ISP_CLK_312M:
		parent = "clk_312m";
		break;
	case ISP_CLK_256M:
		parent = "clk_256m";
		break;
	case ISP_CLK_128M:
		parent = "clk_128m";
		break;
	case ISP_CLK_48M:
		parent = "clk_48m";
		break;
	case ISP_CLK_76M8:
		parent = "clk_76p8m";
		break;

	case ISP_CLK_NONE:
		ISP_PRINT("isp_k: ISP close CLK %d \n", (int)clk_get_rate(fd->s_isp_clk));
		if (fd->s_isp_clk) {
			clk_disable(fd->s_isp_clk);
			clk_put(fd->s_isp_clk);
			fd->s_isp_clk = NULL;
		}
		return 0;
	default:
		parent = "clk_128m";
		break;
	}

	if (NULL == fd->s_isp_clk) {
		fd->s_isp_clk = parse_clk(isp_dev.this_device->of_node, "clk_isp");
		if (IS_ERR(fd->s_isp_clk)) {
			ISP_PRINT("isp_k: parse_clk fail, %d \n", (int)fd->s_isp_clk);
			return -1;
		} else {
			ISP_PRINT("isp_k: get clk_parent ok \n");
		}
	} else {
		clk_disable(fd->s_isp_clk);
	}

	clk_parent = clk_get(NULL, parent);
	if (IS_ERR(clk_parent)) {
		ISP_PRINT("isp_k: dcam_set_clk fail, %d \n", (int)clk_parent);
		return -1;
	} else {
		ISP_PRINT("isp_k: get clk_parent ok \n");
	}

	rtn = clk_set_parent(fd->s_isp_clk, clk_parent);
	if(rtn){
		ISP_PRINT("isp_k: clk_set_parent fail, %d \n", rtn);
	}

	rtn = clk_enable(fd->s_isp_clk);
	if (rtn) {
		ISP_PRINT("isp_k: enable isp clk error.\n");
		return -1;
	}
#endif

	return rtn;
}

static int _isp_queue_init(struct isp_queue *queue)
{
	if (NULL == queue)
		return -EINVAL;

	memset(queue, 0x00, sizeof(*queue));
	queue->write = &queue->node[0];
	queue->read  = &queue->node[0];

	return 0;
}

static int _isp_queue_write(struct isp_queue *queue, struct isp_node *node)
{
	struct isp_node 	*ori_node;

	if (NULL == queue || NULL == node)
	return -EINVAL;

	ori_node = queue->write;

	//ISP_PRINT("_isp_queue_write called!\n");
	*queue->write++ = *node;
	if (queue->write > &queue->node[ISP_QUEUE_LENGTH-1]) {
		queue->write = &queue->node[0];
	}

	if (queue->write == queue->read) {
	queue->write = ori_node;
	}

	//ISP_PRINT("_isp_queue_write finished!\n");
	return 0;
}

static int _isp_queue_read(struct isp_queue *queue, struct isp_node *node)
{
	int ret = 0;

	if (NULL == queue || NULL == node)
		return -EINVAL;
	//ISP_PRINT("_isp_queue_read called!\n");
	if (queue->read != queue->write) {
		*node = *queue->read++;
		if (queue->read > &queue->node[ISP_QUEUE_LENGTH-1]) {
			queue->read = &queue->node[0];
		}
	}
	//ISP_PRINT("_isp_queue_read finished!\n");
	return ret;
}

static inline void _isp_regread(char *dst,  char *src, size_t n)
{
	char tmp  = 0;
	uint32_t tmp2 = 0;
	char *char_src = 0,*d = 0;
	uint32_t *d2 = (uint32_t*) dst;
	uint32_t *int_src =  (uint32_t*) src;
	uint32_t counts = (n>>2)<<2;
	uint32_t res_counts = n -counts;
	counts = counts>>2;

	while (counts--) {
		tmp2 = ISP_READL(int_src);
		*d2++ = tmp2;
		int_src++;
	}

	if(res_counts) {
		d = (char*) d2;
		char_src = (char*) int_src;
		while(res_counts--) {
			tmp = __raw_readb(char_src);
			*d = tmp;
			char_src++;
		}
	}
}

static inline void _isp_regwrite(char *dst,  char *src, size_t n)
{
	uint32_t tmp2 = 0;
	char *char_src = 0, *d = 0;
	uint32_t *int_src = 0, *d2 = 0;
	uint32_t counts = 0, res_counts = 0;

	int_src = (uint32_t*) src;
	d2 = (uint32_t*) dst;
	counts = (n>>2)<<2;
	res_counts = n - counts;
	counts = counts>>2;

	while (counts--) {
		tmp2 = *int_src++;
		ISP_WRITEL(d2, tmp2);
		d2++;
	}

	if(res_counts) {
		d = (char*) d2;
		char_src = (char*) int_src;

		while(res_counts--) {
			tmp2 = *char_src++;
			 __raw_writeb(tmp2, d);
			d++;
		}
	}

}
static int32_t _isp_get_ctlr(void *param)
{
	struct isp_k_file *dev_ptr = (struct isp_k_file *) param;

	down(&dev_ptr->sem_isp);

	return 0;
}
static int32_t _isp_put_ctlr(void *param)
{
	struct isp_k_file *dev_ptr = (struct isp_k_file *) param;

	up(&dev_ptr->sem_isp);

	return 0;
}

static int _isp_en_irq(unsigned long int_num)
{
	uint32_t ret = 0;

	ISP_WRITEL(ISP_INT_CLEAR, ISP_IRQ_HW_MASK);
	ISP_WRITEL(ISP_INT_EN, int_num);

	return ret;
}

static int _isp_registerirq(void *handle)
{
	uint32_t ret = 0;
	struct isp_k_file *fd = (struct isp_k_file*)handle;
	if (NULL == handle) {
		ISP_PRINT("_isp_module_eb handle null\n");
		return -ENOENT;
	}

	ret = request_irq(ISP_IRQ, _isp_irq_root, IRQF_SHARED, "ISP", fd);

	return ret;
}
static void _isp_unregisterirq(void *handle)
{
	free_irq (ISP_IRQ, handle);
}

static int _isp_cfg_dcam_int(uint32_t param)
{
	uint32_t ret = 0;

	s_dcam_int_eb = param;

	return ret;
}

static void _read_reg(struct isp_reg_bits *reg_bits_ptr, uint32_t counts)
{
	uint32_t i = 0;
	uint32_t reg_val = 0, reg_addr = 0;

	for (i = 0; i<counts; ++i) {
		//reg_addr = reg_bits_ptr[i].reg_addr;
		reg_addr = ISP_BASE_ADDR + reg_bits_ptr[i].reg_addr;
		reg_val = ISP_READL(reg_addr);
		reg_bits_ptr[i].reg_value = reg_val;
	}

}

static void _write_reg(struct isp_reg_bits *reg_bits_ptr, uint32_t counts)
{
	uint32_t i = 0;
	uint32_t reg_val = 0, reg_addr = 0;

	for (i = 0; i<counts; ++i) {
		reg_addr = reg_bits_ptr[i].reg_addr+ISP_BASE_ADDR;
		reg_val = reg_bits_ptr[i].reg_value;
		ISP_WRITEL(reg_addr, reg_val);
	}
}

/**********************************************************
*open the device
*
***********************************************************/
static int32_t _isp_kernel_open (struct inode *node, struct file *pf)
{
	int32_t ret = 0;
	uint32_t addr = 0;
	struct isp_k_file *fd = NULL;
	struct miscdevice *md = pf->private_data;
	struct isp_buf_phy *isp_buf = NULL;
	ISP_PRINT ("isp_k: open start \n");

	fd = (struct isp_k_file*)vzalloc(sizeof(struct isp_k_file));
	ISP_CHECK_ZERO(fd);
	atomic_set(&fd->s_isp_users, 0);
	mutex_init(&fd->s_isp_lock);
	init_MUTEX(&fd->sem_isp);
	init_MUTEX_LOCKED(&fd->sem_isr); /*for interrupt */
	spin_lock_init(&fd->isp_spin_lock);
	isp_buf = (struct isp_buf_phy*)md->this_device->platform_data;
	if (!isp_buf) {
		ret = -EFAULT;
		printk("isp_kernel_open fail isp_buf NULL \n");
		goto exit;
	}

	fd->s_isp_alloc_addr = isp_buf->buf_vir_ptr;
	fd->s_isp_alloc_len = isp_buf->buf_len;
	fd->s_isp_alloc_order = get_order(isp_buf->buf_len);
	fd->s_isp_clk = NULL;
	fd->s_isp_clk_mm_i = NULL;
	fd->buf_addr = 0;
	fd->buf_len = 0;

	fd->buf_addr = (uint32_t)vzalloc(ISP_BUF_MAX_SIZE);
	if (0 == fd->buf_addr) {
		ret = -1;
		ISP_PRINT ("isp_kernel_init 1: alloc  error \n");
		goto ISP_K_OPEN_ERROR_EXIT1;
	}
	fd->buf_len = ISP_BUF_MAX_SIZE;

	ret =  _isp_get_ctlr(fd);
	if (unlikely(ret)) {
		ISP_PRINT ("isp_k: get control error \n");
		ret = -EFAULT;
		return ret;
	}

	fd->reg_base_addr = (uint32_t)ISP_BASE_ADDR;
	fd->size = ISP_REG_MAX_SIZE;

	ret = _isp_module_eb(fd);
	if (unlikely(0 != ret)) {
		ISP_PRINT("isp_k: enable isp module error\n");
		ret = -EIO;
		goto ISP_K_OPEN_ERROR_EXIT2;
	}

	ret = _isp_module_rst(fd);
	if (unlikely(0 != ret)) {
		ISP_PRINT("isp_k: reset isp module error \n");
		ret = -EIO;
		goto ISP_K_OPEN_ERROR_EXIT2;
	}

	ret = _isp_registerirq(fd);
	ISP_PRINT ("isp_k: base addr = 0x%x, size = %d \n", fd->reg_base_addr,
			fd->size);

	ret = _isp_queue_init(&(fd->queue));
	pf->private_data = fd;
	ISP_PRINT ("isp_k: open end \n");

	return ret;

ISP_K_OPEN_ERROR_EXIT2:
	 _isp_put_ctlr(fd);
ISP_K_OPEN_ERROR_EXIT1:
	_isp_update(fd);
	if (fd->buf_addr) {
		vfree((void *)fd->buf_addr);
		fd->buf_addr = 0;
		fd->buf_len = 0;
	}

exit:
	if (fd) {
		vfree(fd);
		fd = 0;
	}

	return ret;
}

static irqreturn_t _isp_irq_root(int irq, void *handle)
{
	int32_t	    ret = 0;
	uint32_t	status = 0;
	uint32_t	irq_line = 0;
	uint32_t	irq_status = 0;
	unsigned long flag = 0;
	int32_t     i = 0;
	struct isp_node    node = { 0 };
	struct isp_k_file *fd = (struct isp_k_file*)handle;
	if (NULL == handle) {
		ISP_PRINT("_isp_module_eb handle null\n");
		return -ENOENT;
	}

	status = ISP_REG_RD(ISP_INT_STATUS);
	irq_line = status&ISP_IRQ_HW_MASK;
	//ISP_PRINT("ISP_RAW:isp_k: isp irq: 0x%x\n", irq_line);
	if ( 0 == irq_line ) {
		return IRQ_NONE;
	}

	spin_lock_irqsave(&fd->isp_spin_lock,flag);
	for (i = ISP_IRQ_NUM- 1; i >= 0; i--) {
		if (irq_line & (1 << (uint32_t)i)) {
		irq_status |= 1 << (uint32_t)i;
		}
		irq_line &= ~(uint32_t)(1 << (uint32_t)i); //clear the interrupt flag
		if(!irq_line) //no interrupt source left
		break;
	}

	node.isp_irq_val = irq_status;
	ISP_WRITEL(ISP_INT_CLEAR, irq_status);
	ret = _isp_queue_write((struct isp_queue *)&fd->queue, (struct isp_node*)&node);
	spin_unlock_irqrestore(&fd->isp_spin_lock, flag);
	up(&fd->sem_isr);

	return IRQ_HANDLED;
}
/*
//static irqreturn_t _dcam_irq_root(int irq, void *dev_id)
void _dcam_isp_root(void)
{
	int32_t	ret = 0;
	unsigned long flag = 0;
	struct isp_node node = { 0 };

	//ISP_PRINT ("ISP_RAW: isp_k: _dcam_isp_root %d \n", s_dcam_int_eb);

	if(0x00 !=s_dcam_int_eb)
	{
		spin_lock_irqsave(&isp_spin_lock,flag);
		#if defined(CONFIG_MACH_CORE3)
		node.dcam_irq_val = ISP_INT_FETCH_EOF;
		#else
		node.dcam_irq_val = ISP_INT_FETCH_SOF;
		#endif

		//ISP_PRINT("isp_k: dcam sof irq :0x%x\n", node.dcam_irq_val);
		ret = _isp_queue_write((struct isp_queue *)&g_isp_dev_ptr->queue, (struct isp_node*)&node);
		spin_unlock_irqrestore(&isp_spin_lock, flag);
		up(&g_isp_dev_ptr->sem_isr);
	}

	//return IRQ_HANDLED;
}
*/
/**********************************************************
*release the device
*
***********************************************************/
static int32_t _isp_kernel_release (struct inode *node, struct file *pf)
{
	int ret = 0;
	struct isp_k_file *fd = NULL;

	ISP_PRINT ("isp_k: release start \n");
	fd = (struct isp_k_file *)pf->private_data;
	ISP_CHECK_ZERO(fd);
	mutex_lock(&fd->s_isp_lock);

	_isp_unregisterirq(fd);
	ret = _isp_module_dis(fd);

	mutex_unlock(&fd->s_isp_lock);

	ret = _isp_put_ctlr(fd);
	if (unlikely (ret) ) {
		ISP_PRINT ("isp_k: release control error \n");
		return -EFAULT;
	}

	mutex_destroy(&fd->s_isp_lock);
	_isp_update(fd);
	ISP_CHECK_ZERO_VOID(fd);
	if (fd->buf_addr) {
		vfree((void *)fd->buf_addr);
		fd->buf_addr = 0;
		fd->buf_len = 0;
	}
	vfree(fd);
	fd = NULL;
	ISP_PRINT ("isp_k: release end \n");
	return ret;
}
/**********************************************************
*read info from  file
*size_t size:
*loff_t p:
***********************************************************/
#if 0
static int32_t _isp_kernel_proc_read (char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int	 len = 0;
	uint32_t	 reg_buf_len = 200;
	uint32_t	 print_len = 0, print_cnt = 0;
	uint32_t	*reg_ptr = 0;

	ISP_PRINT ("isp_k: _isp_kernel_proc_read 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x \n", (uint32_t)page, (uint32_t)start, (uint32_t)off, (uint32_t)count, (uint32_t)eof, (uint32_t)data);

	(void)start; (void)off; (void)count; (void)eof;

	if(0x00==g_isp_device.reg_base_addr)
	{
		return 0x00;
	}

	reg_ptr = (uint32_t*)g_isp_device.reg_base_addr;
	len += sprintf(page + len, "Context for ISP device \n");
	len += sprintf(page + len, "********************************************* \n");
	while (print_len < reg_buf_len) {
	len += sprintf(page + len, "offset 0x%x : 0x%x, 0x%x, 0x%x, 0x%x \n",
		print_len,
		reg_ptr[print_cnt],
		reg_ptr[print_cnt+1],
		reg_ptr[print_cnt+2],
		reg_ptr[print_cnt+3]);
	print_cnt += 4;
	print_len += 16;
	}
	len += sprintf(page + len, "********************************************* \n");
	len += sprintf(page + len, "The end of ISP device \n");
	return len;
}
#endif
/**********************************************************
*the io controller of isp
*unsigned int cmd:
*unsigned long param:
***********************************************************/
static long _isp_kernel_ioctl(struct file *fl, unsigned int cmd, unsigned long param)
{
	long ret = 0;
	uint32_t isp_irq, dcam_irq;
	struct isp_irq_param irq_param = { 0 };
	struct isp_node isp_node = { 0 };
	struct isp_reg_param reg_param = { 0 };
	struct isp_reg_bits *reg_bits_ptr = 0;
	struct isp_k_file *fd = NULL;
	//ISP_PRINT("isp_k:_ioctl called, cmd: %x\n", cmd);

	if (!fl) {
		return -EINVAL;
	}
	fd = fl->private_data;
	ISP_CHECK_ZERO(fd);

	if(ISP_IO_IRQ==cmd)
	{
		ret = down_interruptible(&fd->sem_isr);
		if (ret) {
			ISP_PRINT("isp_k: ioctl irq: down failed ret = %ld",ret);
			memset(&irq_param, 0, sizeof(irq_param));
			irq_param.ret_val = ret;
			ret = copy_to_user ((void*) param, (void*)&irq_param, sizeof(irq_param));
			if ( 0 != ret) {
				ISP_PRINT("isp_k: ioctl irq: copy_to_user failed ret = %ld", ret);
			}
			ret = -ERESTARTSYS;
			goto ISP_IOCTL_EXIT;
		}
		ret=_isp_queue_read(&fd->queue, &isp_node);
		if ( 0 != ret) {

			ISP_PRINT("isp_k: ioctl irq: _isp_queue_read error, ret = 0x%x", (uint32_t)ret);
			ret = -EFAULT;
			goto ISP_IOCTL_EXIT;
		}

		irq_param.dcam_irq_val = isp_node.dcam_irq_val;
		irq_param.isp_irq_val = isp_node.isp_irq_val;
		isp_irq = isp_node.isp_irq_val;
		dcam_irq = isp_node.dcam_irq_val;
		irq_param.irq_val = dcam_irq|isp_irq;
		ret = copy_to_user ((void*) param, (void*)&irq_param, sizeof(struct isp_irq_param));
		if ( 0 != ret) {

			ISP_PRINT("isp_k: ioctl irq: copy_to_user error, ret = 0x%x", (uint32_t)ret);
			ret = -EFAULT;
		}

	} else {

		mutex_lock(&fd->s_isp_lock);

		switch (cmd)
		{
			case ISP_IO_READ: {
			uint32_t buf_size = 0;
			//ISP_PRINT("  isp_k:_ioctl read called \n");
			ret = copy_from_user((void*)&reg_param, (void*)param, sizeof(struct isp_reg_param));
			if ( 0 != ret) {

				ISP_PRINT("isp_k: ioctl read: copy_to_user error, ret = 0x%x", (uint32_t)ret);
				ret = -EFAULT;
				goto IO_READ_EXIT;
			}
			buf_size = reg_param.counts*sizeof(struct isp_reg_bits);
			if (buf_size > fd->buf_len) {
				ret  =-EFAULT;
				goto IO_READ_EXIT;
			}
			reg_bits_ptr = (struct isp_reg_bits*) fd->buf_addr;
			ret = copy_from_user((void*)reg_bits_ptr, (void*)reg_param.reg_param, buf_size);
			if ( 0 != ret) {

				ISP_PRINT("isp_k: ioctl read: copy_to_user error, ret = 0x%x", (uint32_t)ret);
				ret  =-EFAULT;
				goto IO_READ_EXIT;
			}
			_read_reg(reg_bits_ptr, reg_param.counts);

			ret = copy_to_user((void*)reg_param.reg_param, (void*)reg_bits_ptr, buf_size);
			if ( 0 != ret) {

				ISP_PRINT("isp_k: ioctl read: copy_to_user error, ret = 0x%x", (uint32_t)ret);
				ret = -EFAULT;
				goto IO_READ_EXIT;
			}
			IO_READ_EXIT:
			if(reg_bits_ptr) {
				memset((void *)fd->buf_addr, 0x00, buf_size);
				reg_bits_ptr = NULL;
			}
			}
			break;

			case ISP_IO_WRITE: {
			uint32_t buf_size = 0;
			//ISP_PRINT(" isp_k:_ioctl write called \n");
			ret = copy_from_user((void*)&reg_param, (void*)param, sizeof(struct isp_reg_param));
			if ( 0 != ret) {

				ISP_PRINT("isp_k: ioctl write: copy_to_user error, ret = 0x%x", (uint32_t)ret);
				ret = -EFAULT;
				goto IO_WRITE_EXIT;
			}
			buf_size = reg_param.counts*sizeof(struct isp_reg_bits);
			if(buf_size > fd->buf_len)
			{
				ret = -EFAULT;
				goto IO_WRITE_EXIT;
			}
			reg_bits_ptr = (struct isp_reg_bits*) fd->buf_addr;

			ret = copy_from_user((void*)reg_bits_ptr, (void*)reg_param.reg_param, buf_size);
			if ( 0 != ret) {

				ISP_PRINT("isp_k: ioctl write: copy_to_user error, ret = 0x%x", (uint32_t)ret);
				ret = -EFAULT;
				goto IO_WRITE_EXIT;
			}
			_write_reg(reg_bits_ptr, reg_param.counts);

			IO_WRITE_EXIT:
			if(reg_bits_ptr) {
				memset((void *)fd->buf_addr, 0x00, buf_size);
				reg_bits_ptr = NULL;
			}

			}
			break;

			case ISP_IO_RST: {
			ISP_PRINT(" isp_k:ioctl restet start \n");
			ret = _isp_module_rst(fd);
			if (ret) {

				ISP_PRINT("isp_k: ioctl restet error!\n");
				ret = -EFAULT;
			}
			}
			break;

			case ISP_IO_SETCLK: {
			ISP_PRINT(" isp_k:ioctl set clock start \n");
			}
			break;

			case ISP_IO_STOP: {
			unsigned long flag = 0;
			struct isp_node node = { 0 };
			ret = _isp_en_irq(0);//dis-enable the interrupt
			ISP_PRINT("isp_k: ioctl  stop start !\n");
			spin_lock_irqsave(&fd->isp_spin_lock,flag);
			node.dcam_irq_val = ISP_INT_STOP;
			ret = _isp_queue_write((struct isp_queue *)&fd->queue, (struct isp_node*)&node);
			spin_unlock_irqrestore(&fd->isp_spin_lock, flag);
			up(&fd->sem_isr);
			}
			break;

			case ISP_IO_INT: {
				unsigned long int_num;
				ret = copy_from_user((void*)&int_num, (void*)param, 0x04);
				if (ret) {
					ISP_PRINT ("isp_k:io int copy param error, ret = %d \n", (uint32_t)ret);
					ret = -EFAULT;
					goto ISP_IOCTL_LOCKED_CMD_EXIT;
				}
				ret = _isp_en_irq(int_num);
				//ret = _isp_registerirq();
				if (unlikely(ret)) {
					ISP_PRINT ("isp_k:enable  interrupt error \n");
					ret = -EFAULT;
				}
			}
			break;

			case ISP_IO_DCAM_INT: {
				unsigned long int_param;
				ret = copy_from_user((void*)&int_param, (void*)param, 0x04);
				if (ret) {
					ISP_PRINT ("isp_k:dcam int copy params error, ret = %d \n", (uint32_t)ret);
					ret = -EFAULT;
					goto ISP_IOCTL_LOCKED_CMD_EXIT;
				}
				ret = _isp_cfg_dcam_int(int_param);
				if (unlikely(ret)) {
					ISP_PRINT ("isp_k:cfg dcam interrupt error \n");
					ret = -EFAULT;
				}
			}
			break;

			case ISP_IO_LNC_PARAM: {
				uint32_t buf_size = 0;
				uint32_t* addr = 0;
				ret = copy_from_user((void*)&reg_param, (void*)param, sizeof(struct isp_reg_param));
				if ( 0 != ret) {

					ISP_PRINT("isp_k: ioctl lnc param: copy_to_user error, ret = 0x%x", (uint32_t)ret);
					ret = -EFAULT;
					goto IO_LNC_PARAM_EXIT;
				}
				buf_size = reg_param.counts;
				if(buf_size > fd->buf_len){

					ret = -EFAULT;
					goto IO_LNC_PARAM_EXIT;
				}
				addr = (uint32_t*) fd->buf_addr;
				ret = copy_from_user((void*)addr, (void*)reg_param.reg_param, buf_size);
				if ( 0 != ret) {

					ISP_PRINT("isp_k: ioctl lnc param: copy_to_user error, ret = 0x%x", (uint32_t)ret);
					ret = -EFAULT;
					goto IO_LNC_PARAM_EXIT;
				}
				ret = _isp_lnc_param_set(fd, addr, buf_size);
				if ( 0 != ret) {
					ISP_PRINT("isp_k: ioctl lnc param error, ret = 0x%x", (uint32_t)ret);
					ret = -EFAULT;
					goto IO_LNC_PARAM_EXIT;
				}

				IO_LNC_PARAM_EXIT:
				if(addr) {
					memset((void *)fd->buf_addr, 0x00, buf_size);
					addr = NULL;
				}
			}
			break;

			case ISP_IO_LNC: {
				uint32_t buf_size = 0;
				ret = copy_from_user((void*)&reg_param, (void*)param, sizeof(struct isp_reg_param));
				if ( 0 != ret) {
					ISP_PRINT("isp_k: ioctl lnc: copy_to_user error, ret = 0x%x", (uint32_t)ret);
					ret = -EFAULT;
					goto IO_LNC_EXIT;
				}
				buf_size = reg_param.counts*sizeof(struct isp_reg_bits);
				if(buf_size > fd->buf_len)
				{
					ret = -EFAULT;
					ISP_PRINT("isp_k: isp_io_lnc: buf len failed\n");
					goto IO_LNC_EXIT;
				}
				reg_bits_ptr = (struct isp_reg_bits*) fd->buf_addr;
				ret = copy_from_user((void*)reg_bits_ptr, (void*)reg_param.reg_param, buf_size);
				if ( 0 != ret) {
					ISP_PRINT("isp_k: ioctl lnc: copy_to_user error, ret = 0x%x", (uint32_t)ret);
					ret = -EFAULT;
					goto IO_LNC_EXIT;
				}

				ret = _isp_lnc_param_load(fd,reg_bits_ptr, reg_param.counts);
				if (unlikely(ret)) {
					ISP_PRINT ("isp_k:load lnc error \n");
					ret = -EFAULT;
				}
				IO_LNC_EXIT:
				if(reg_bits_ptr) {
					memset((void *)fd->buf_addr, 0x00, buf_size);
					reg_bits_ptr = NULL;
				}
			}
			break;

			case ISP_IO_ALLOC: {

				ret = copy_from_user((void*)&reg_param, (void*)param, sizeof(struct isp_reg_param));
				if ( 0 != ret) {
					ISP_PRINT("isp_k: ioctl write: copy_to_user error, ret = 0x%x", (uint32_t)ret);
					ret = -EFAULT;
					break;
				}
				reg_param.reg_param = fd->s_isp_alloc_addr;
				reg_param.counts = fd->s_isp_alloc_len;
				ret = copy_to_user((void*)param, (void*)&reg_param, sizeof(struct isp_reg_param));
			}
			break;

			default:
			mutex_unlock(&fd->s_isp_lock);
			ISP_PRINT("isp_k:_ioctl cmd is unsupported, cmd = %x\n", (int32_t)cmd);
			return -EFAULT;
		}

		ISP_IOCTL_LOCKED_CMD_EXIT:
		mutex_unlock(&fd->s_isp_lock);
	}

	//ISP_PRINT("isp_k:_ioctl finished\n");
	ISP_IOCTL_EXIT:
	return ret;
}

static int _isp_probe(struct platform_device *pdev)
{
	int ret = 0;
	uint32_t order = 0;
	struct isp_buf_phy *isp_buf = NULL;

	ISP_PRINT ("isp_k:probe start\n");

	isp_buf = devm_kzalloc(&pdev->dev, sizeof(struct isp_buf_phy), GFP_KERNEL);
	if (!isp_buf) {
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, isp_buf);
	ret = misc_register(&isp_dev);
	if (ret) {
		ISP_PRINT ( "isp_k:probe cannot register miscdev on minor=%d (%d)\n",(int32_t)ISP_MINOR, (int32_t)ret);
		goto probe_out;
	}

	isp_buf->buf_len = ISP_BUF_MAX_SIZE;
	order = get_order(isp_buf->buf_len);
	isp_buf->buf_vir_ptr = __get_free_pages(GFP_KERNEL | __GFP_COMP, order);
	if (NULL == (void*)isp_buf->buf_vir_ptr ) {
		ISP_PRINT("ISP_RAW:_isp_alloc order:0x%x, len:0x%x error\n", order, isp_buf->buf_len);
		ret = 1;
		goto probe_out;
	}
	isp_buf->buf_phy_ptr = virt_to_phys((volatile void *)isp_buf->buf_vir_ptr);
	dmac_flush_range(isp_buf->buf_vir_ptr, isp_buf->buf_vir_ptr + isp_buf->buf_len);
	outer_flush_range(__pa(isp_buf->buf_phy_ptr), __pa(isp_buf->buf_phy_ptr) + isp_buf->buf_len);

	isp_dev.this_device->of_node = pdev->dev.of_node;
	isp_dev.this_device->platform_data = (void *)isp_buf;

	ISP_PRINT (" isp_k:probe end\n");
	goto exit;
probe_out:
	devm_kfree(&pdev->dev, isp_buf);
	platform_set_drvdata(pdev, NULL);
exit:
	return ret;
}

static int _isp_remove(struct platform_device * dev)
{
	struct isp_buf_phy *isp_buf;
	uint32_t order = 0;

	ISP_PRINT ("isp_k: remove start \n");
	isp_buf = platform_get_drvdata(dev);
	if (!isp_buf)
		goto remove_exit;

	misc_deregister(&isp_dev);
/*
	if (isp_proc_file) {
		remove_proc_entry("driver/sprd_isp", NULL);
	}
*/
	order = get_order(isp_buf->buf_len);
	free_pages(isp_buf->buf_vir_ptr, order);
	devm_kfree(&dev->dev, isp_buf);
	platform_set_drvdata(dev, NULL);

	ISP_PRINT ("isp_k: remove end !\n");

remove_exit:
	return 0;
}

static int32_t __init isp_kernel_init(void)
{
	int32_t ret = 0;

	ISP_PRINT ("isp_k: init start \n");
	if (platform_driver_register(&isp_driver) != 0) {
		ISP_PRINT ("isp_kernel_init: platform device register error \n");
		return -1;
	}

	ISP_PRINT ("isp_k: init end\n");

	return ret;
}

static void isp_kernel_exit(void)
{
	ISP_PRINT ("isp_k: exit start \n");

	platform_driver_unregister(&isp_driver);

	ISP_PRINT ("isp_k: exit end \n");
}

module_init(isp_kernel_init);
module_exit(isp_kernel_exit);

MODULE_DESCRIPTION("Isp Driver");
MODULE_LICENSE("GPL");
