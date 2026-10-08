// SPDX-License-Identifier: GPL-2.0-only
/*
 * H3C HM2004-DU Micro-OLT FPGA Management Driver (FMCS)
 *
 * Core driver lifecycle, IOCTL interface, FPGA bitstream loading, and IRQ.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/mod_devicetable.h>
#include "fmcs.h"

static struct fmcs_priv *g_fmcs_priv;

static int fmcs_dev_open(struct inode *inode, struct file *file)
{
	file->private_data = g_fmcs_priv;
	return 0;
}

static int fmcs_dev_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long fmcs_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct fmcs_priv *priv = file->private_data;
	int ret = 0;

	if (_IOC_TYPE(cmd) != FMCS_IOC_MAGIC)
		return -ENOTTY;

	if (!priv)
		return -ENODEV;

	switch (cmd) {
	case FMCS_IOC_WRITE_FPGA_REG: {
		struct fmcs_reg_op op;
		if (copy_from_user(&op, (void __user *)arg, sizeof(op))) {
			ret = -EFAULT;
			break;
		}
		ret = fmcs_spi_write_fpga_reg(priv, op.addr, op.val);
		break;
	}
	case FMCS_IOC_READ_FPGA_REG: {
		struct fmcs_reg_op op;
		if (copy_from_user(&op, (void __user *)arg, sizeof(op))) {
			ret = -EFAULT;
			break;
		}
		ret = fmcs_spi_read_fpga_reg(priv, op.addr, &op.val);
		if (!ret && copy_to_user((void __user *)arg, &op, sizeof(op)))
			ret = -EFAULT;
		break;
	}
	case FMCS_IOC_WRITE_BOSA_REG: {
		struct fmcs_bosa_op op;
		if (copy_from_user(&op, (void __user *)arg, sizeof(op))) {
			ret = -EFAULT;
			break;
		}
		ret = fmcs_spi_write_bosa_reg(priv, op.addr, op.val);
		break;
	}
	case FMCS_IOC_READ_BOSA_REG: {
		struct fmcs_bosa_op op;
		if (copy_from_user(&op, (void __user *)arg, sizeof(op))) {
			ret = -EFAULT;
			break;
		}
		ret = fmcs_spi_read_bosa_reg(priv, op.addr, &op.val);
		if (!ret && copy_to_user((void __user *)arg, &op, sizeof(op)))
			ret = -EFAULT;
		break;
	}
	case FMCS_IOC_DEV_ATTR_MODIFY: {
		/* Used for setting debug flags and multicast GEM ports */
		break;
	}
	case FMCS_IOC_RECV_PLOAM: {
		struct fmcs_ploam_msg msg;
		ret = wait_event_interruptible_timeout(priv->wq, priv->event_pending, msecs_to_jiffies(1000));
		if (ret > 0 && priv->event_pending) {
			size_t len = 0;
			ret = fmcs_spi_recv_ploam(priv, msg.data, &len);
			msg.len = len;
			priv->event_pending = false;
			if (!ret && copy_to_user((void __user *)arg, &msg, sizeof(msg)))
				ret = -EFAULT;
		} else if (ret == 0) {
			ret = -ETIMEDOUT;
		}
		break;
	}
	case FMCS_IOC_SEND_PACKET: {
		struct fmcs_ploam_msg msg;
		if (copy_from_user(&msg, (void __user *)arg, sizeof(msg))) {
			ret = -EFAULT;
			break;
		}
		ret = fmcs_spi_send_ploam(priv, msg.data, msg.len);
		break;
	}
	case FMCS_IOC_SET_CARRIER: {
		struct fmcs_carrier_req req;
		if (copy_from_user(&req, (void __user *)arg, sizeof(req))) {
			ret = -EFAULT;
			break;
		}
		if (req.port < 1 || req.port > FMCS_MAX_FTTR_PORTS) {
			ret = -EINVAL;
			break;
		}
		if (!priv->fttr_devs[req.port - 1]) {
			ret = -ENODEV;
			break;
		}
		if (req.carrier)
			netif_carrier_on(priv->fttr_devs[req.port - 1]);
		else
			netif_carrier_off(priv->fttr_devs[req.port - 1]);
		ret = 0;
		break;
	}
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct file_operations fmcs_fops = {
	.owner          = THIS_MODULE,
	.open           = fmcs_dev_open,
	.release        = fmcs_dev_release,
	.unlocked_ioctl = fmcs_dev_ioctl,
};

static irqreturn_t fmcs_irq_handler(int irq, void *dev_id)
{
	struct fmcs_priv *priv = dev_id;
	u32 status_b0 = 0;
	u8 ploam_rx[16] = { 0 };
	size_t ploam_len = 0;

	mutex_lock(&priv->lock);
	/* Read FPGA event register 0x000000B0 */
	fmcs_spi_read_fpga_reg(priv, 0x000000B0, &status_b0);

	/* Bit 0 of 0xB0 indicates an incoming PLOAM frame from an ONU */
	if (status_b0 & 0x1) {
		if (fmcs_spi_recv_ploam(priv, ploam_rx, &ploam_len) == 0 && ploam_len > 0) {
			memcpy(priv->last_ploam.data, ploam_rx, min_t(size_t, ploam_len, 16));
			priv->last_ploam.len = ploam_len;
			priv->event_pending = true;
			fmcs_rx_ploam_frame(priv, ploam_rx, ploam_len);
		}
	}
	mutex_unlock(&priv->lock);

	priv->event_pending = true;
	wake_up_interruptible(&priv->wq);

	return IRQ_HANDLED;
}

static int fmcs_load_fpga_bitstream(struct fmcs_priv *priv, const char *fw_name)
{
	const struct firmware *fw;
	size_t i;
	int bit;
	int ret;

	if (!priv->gpiod_clk || !priv->gpiod_data) {
		dev_warn(priv->dev, "FPGA programming GPIOs not configured\n");
		return -ENODEV;
	}

	ret = request_firmware(&fw, fw_name, priv->dev);
	if (ret) {
		dev_warn(priv->dev, "FPGA bitstream %s not found (%d)\n", fw_name, ret);
		return ret;
	}

	dev_info(priv->dev, "Programming FPGA bitstream %s (%zu bytes)...\n", fw_name, fw->size);

	gpiod_direction_output(priv->gpiod_clk, 0);
	gpiod_direction_output(priv->gpiod_data, 0);
	udelay(100);

	for (i = 0; i < fw->size; i++) {
		u8 b = fw->data[i];
		for (bit = 7; bit >= 0; bit--) {
			gpiod_set_value(priv->gpiod_data, (b >> bit) & 1);
			gpiod_set_value(priv->gpiod_clk, 1);
			ndelay(50);
			gpiod_set_value(priv->gpiod_clk, 0);
			ndelay(50);
		}
		if ((i & 0x7FFF) == 0)
			cond_resched();
	}

	dev_info(priv->dev, "FPGA bitstream programmed successfully!\n");
	release_firmware(fw);
	return 0;
}

static ssize_t reload_fpga_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct fmcs_priv *priv = dev_get_drvdata(dev);

	if (priv)
		fmcs_load_fpga_bitstream(priv, "FTTR_TOP.sbit");

	return count;
}
static DEVICE_ATTR_WO(reload_fpga);

static int fmcs_match_spi_child(struct device *dev, const void *data)
{
	return 1;
}

static struct spi_controller *fmcs_get_spi_controller(struct device_node *np)
{
	struct platform_device *pdev;
	struct device *child;

	pdev = of_find_device_by_node(np);
	if (!pdev)
		return NULL;

	child = device_find_child(&pdev->dev, NULL, fmcs_match_spi_child);
	platform_device_put(pdev);
	if (!child)
		return NULL;

	return container_of(child, struct spi_controller, dev);
}

static int fmcs_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fmcs_priv *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	mutex_init(&priv->lock);
	init_waitqueue_head(&priv->wq);

	/* Map AN7581 SNFI/SPI and NFI MMIO Registers */
	priv->spi_base = ioremap(AN7581_SPI_BASE_PHYS, AN7581_SPI_SIZE);
	if (!priv->spi_base) {
		dev_err(dev, "Failed to ioremap SPI controller at 0x%08x\n", AN7581_SPI_BASE_PHYS);
		return -ENOMEM;
	}

	priv->nfi_base = ioremap(AN7581_NFI_BASE_PHYS, AN7581_NFI_SIZE);
	if (!priv->nfi_base) {
		dev_err(dev, "Failed to ioremap NFI controller at 0x%08x\n", AN7581_NFI_BASE_PHYS);
		iounmap(priv->spi_base);
		return -ENOMEM;
	}

	dev_info(dev, "Mapped AN7581 MMIO: SPI=%px, NFI=%px\n", priv->spi_base, priv->nfi_base);

	/* Enable SPI CS1 pad/pinmux on AN7581 (Discovered from stock vmlinux probe at 0x576ae8) */
	{
		void __iomem *pad_base = ioremap(0x1FA20218, 4);
		if (pad_base) {
			u32 val = readl(pad_base);
			dev_info(dev, "AN7581 SPI CS1 pad reg (0x1FA20218): before=0x%08x\n", val);
			writel(val | 1, pad_base);
			dev_info(dev, "AN7581 SPI CS1 pad reg (0x1FA20218): after=0x%08x\n", readl(pad_base));
			iounmap(pad_base);
		}
	}

	/* Bind to SPI Controller for hardware bus arbitration locking */
	{
		struct device_node *spi_node = of_parse_phandle(dev->of_node, "spi-controller", 0);
		if (!spi_node)
			spi_node = of_find_compatible_node(NULL, NULL, "airoha,en7581-snand");
		if (spi_node) {
			priv->spi_ctrl = fmcs_get_spi_controller(spi_node);
			of_node_put(spi_node);
			if (priv->spi_ctrl)
				dev_info(dev, "Bound to SPI controller %s for bus arbitration locking\n",
					 dev_name(&priv->spi_ctrl->dev));
			else
				dev_warn(dev, "SPI controller found in DT but child device not found\n");
		}
	}

	/* Parse GPIO and Interrupts from Device Tree */
	priv->gpiod_int = devm_gpiod_get_optional(dev, "fpga-spi-int", GPIOD_IN);
	priv->gpiod_clk = devm_gpiod_get_optional(dev, "fpga-clk", GPIOD_OUT_LOW);
	priv->gpiod_data = devm_gpiod_get_optional(dev, "fpga-data", GPIOD_OUT_LOW);

	if (priv->gpiod_int)
		priv->irq = gpiod_to_irq(priv->gpiod_int);
	else
		priv->irq = platform_get_irq_optional(pdev, 0);

	if (priv->irq > 0) {
		ret = devm_request_threaded_irq(dev, priv->irq, NULL,
						fmcs_irq_handler,
						IRQF_TRIGGER_HIGH | IRQF_ONESHOT | IRQF_SHARED,
						"fmcs-irq", priv);
		if (ret)
			dev_warn(dev, "Failed to request IRQ %d: %d\n", priv->irq, ret);
		else
			dev_info(dev, "Registered FMCS IRQ %d\n", priv->irq);
	}

	/* Register Character Device /dev/fmcs_mci */
	ret = alloc_chrdev_region(&priv->devno, 0, 1, FMCS_DEV_NAME);
	if (ret)
		goto err_maps;

	cdev_init(&priv->cdev, &fmcs_fops);
	priv->cdev.owner = THIS_MODULE;
	ret = cdev_add(&priv->cdev, priv->devno, 1);
	if (ret)
		goto err_cdev;

	priv->class = class_create(FMCS_DEV_NAME);
	if (IS_ERR(priv->class)) {
		ret = PTR_ERR(priv->class);
		goto err_class;
	}

	device_create(priv->class, NULL, priv->devno, NULL, FMCS_DEV_NAME);

	/* Initialize molt_omci and molt_ploam */
	ret = fmcs_netdevs_init(priv);
	if (ret)
		dev_warn(dev, "Failed to register virtual netdevs: %d\n", ret);

	/* Automatically load FPGA bitstream if available */
	fmcs_load_fpga_bitstream(priv, "FTTR_TOP.sbit");
	ret = device_create_file(dev, &dev_attr_reload_fpga);
	if (ret)
		dev_warn(dev, "Failed to create reload_fpga sysfs attribute: %d\n", ret);

	g_fmcs_priv = priv;
	platform_set_drvdata(pdev, priv);

	dev_info(dev, "H3C Micro-OLT FPGA Management Driver (FMCS) initialized successfully.\n");
	return 0;

err_class:
	cdev_del(&priv->cdev);
err_cdev:
	unregister_chrdev_region(priv->devno, 1);
err_maps:
	iounmap(priv->nfi_base);
	iounmap(priv->spi_base);
	return ret;
}

static void fmcs_remove(struct platform_device *pdev)
{
	struct fmcs_priv *priv = platform_get_drvdata(pdev);

	if (!priv)
		return;

	device_remove_file(priv->dev, &dev_attr_reload_fpga);
	fmcs_netdevs_exit(priv);
	device_destroy(priv->class, priv->devno);
	class_destroy(priv->class);
	cdev_del(&priv->cdev);
	unregister_chrdev_region(priv->devno, 1);

	if (priv->spi_base)
		iounmap(priv->spi_base);
	if (priv->nfi_base)
		iounmap(priv->nfi_base);
	if (priv->spi_ctrl)
		spi_controller_put(priv->spi_ctrl);

	g_fmcs_priv = NULL;
}

static const struct of_device_id fmcs_of_match[] = {
	{ .compatible = "h3c,fmcs" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fmcs_of_match);

static struct platform_driver fmcs_driver = {
	.probe  = fmcs_probe,
	.remove = fmcs_remove,
	.driver = {
		.name           = FMCS_DRV_NAME,
		.of_match_table = fmcs_of_match,
	},
};

module_platform_driver(fmcs_driver);

MODULE_AUTHOR("LazuliKao");
MODULE_DESCRIPTION("H3C HM2004-DU Micro-OLT FPGA Management Driver");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("FTTR_TOP.sbit");
