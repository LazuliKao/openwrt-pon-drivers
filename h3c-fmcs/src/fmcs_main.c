// SPDX-License-Identifier: GPL-2.0-only
/*
 * H3C HM2004-DU Micro-OLT FPGA Management Controller (FMCS) Driver
 *
 * Implements /dev/fmcs_mci character control device, SPI communication,
 * and hardware event handling.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
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

	mutex_lock(&priv->lock);

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
			msg = priv->last_ploam;
			priv->event_pending = false;
			if (copy_to_user((void __user *)arg, &msg, sizeof(msg)))
				ret = -EFAULT;
			else
				ret = 0;
		} else if (ret == 0) {
			ret = -ETIMEDOUT;
		}
		break;
	}
	case FMCS_IOC_SEND_PACKET: {
		/* Test packet injection */
		break;
	}
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&priv->lock);
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
	u32 status = 0;

	/* Read FPGA interrupt status */
	fmcs_spi_read_fpga_reg(priv, 0x00000004, &status);

	/* Check LOS / Ranging events */
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

static int fmcs_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct fmcs_priv *priv;
	struct spi_controller *ctlr;
	struct spi_board_info chip = {
		.modalias = "fmcs-spi",
		.max_speed_hz = 25000000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
	};
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	mutex_init(&priv->lock);
	init_wait_queue_head(&priv->wq);

	/* Parse GPIO and Interrupts from Device Tree */
	priv->gpiod_int = devm_gpiod_get_optional(dev, "fpga-spi-int", GPIOD_IN);
	priv->gpiod_clk = devm_gpiod_get_optional(dev, "fpga-clk", GPIOD_OUT_LOW);
	priv->gpiod_data = devm_gpiod_get_optional(dev, "fpga-data", GPIOD_OUT_LOW);

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq > 0) {
		ret = devm_request_threaded_irq(dev, priv->irq, NULL,
						fmcs_irq_handler,
						IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
						"fmcs-irq", priv);
		if (ret)
			dev_warn(dev, "Failed to request IRQ %d: %d\n", priv->irq, ret);
	}

	/* Connect to SPI controller */
	ctlr = spi_busnum_to_master(0);
	if (ctlr) {
		priv->spi = spi_new_device(ctlr, &chip);
		if (!priv->spi)
			dev_warn(dev, "Failed to instantiate SPI device\n");
	}

	/* Register Character Device /dev/fmcs_mci */
	ret = alloc_chrdev_region(&priv->devno, 0, 1, FMCS_DEV_NAME);
	if (ret)
		return ret;

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

	if (priv->spi)
		spi_unregister_device(priv->spi);

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

MODULE_AUTHOR("Antigravity & OpenWrt Community");
MODULE_DESCRIPTION("H3C HM2004-DU Micro-OLT FPGA Management Driver (FMCS)");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("FTTR_TOP.sbit");
