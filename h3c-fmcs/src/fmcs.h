/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * H3C HM2004-DU Micro-OLT FPGA Management Control System (FMCS)
 *
 * Header file defining IOCTL interface, SPI wire protocol, and driver state.
 */

#ifndef _H3C_FMCS_H_
#define _H3C_FMCS_H_

#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/spi/spi.h>
#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/wait.h>

#define FMCS_DRV_NAME "h3c-fmcs"
#define FMCS_DEV_NAME "fmcs_mci"

/* 
 * IOCTL Interface (Magic: 0xA5)
 * Discovered via reverse engineering of FmcsMciIoctl.
 */
#define FMCS_IOC_MAGIC 0xA5

struct fmcs_reg_op {
	__u32 addr;
	__u32 val;
};

struct fmcs_bosa_op {
	__u32 addr;
	__u32 val;
};

struct fmcs_attr_op {
	__u32 cmd;
	__u32 param;
};

struct fmcs_ploam_msg {
	__u8  data[16];
	__u32 len;
};

#define FMCS_IOC_WRITE_FPGA_REG  _IOW(FMCS_IOC_MAGIC, 0, struct fmcs_reg_op)
#define FMCS_IOC_READ_FPGA_REG   _IOWR(FMCS_IOC_MAGIC, 1, struct fmcs_reg_op)
#define FMCS_IOC_WRITE_BOSA_REG  _IOW(FMCS_IOC_MAGIC, 2, struct fmcs_bosa_op)
#define FMCS_IOC_READ_BOSA_REG   _IOWR(FMCS_IOC_MAGIC, 3, struct fmcs_bosa_op)
#define FMCS_IOC_DEV_ATTR_MODIFY _IOW(FMCS_IOC_MAGIC, 4, struct fmcs_attr_op)
#define FMCS_IOC_RECV_PLOAM      _IOR(FMCS_IOC_MAGIC, 5, struct fmcs_ploam_msg)
#define FMCS_IOC_SEND_PACKET     _IOW(FMCS_IOC_MAGIC, 6, __u8)

/*
 * SPI Wire Protocol Commands
 */
#include <linux/gpio/consumer.h>

#define FMCS_SPI_CMD_WRITE_REG   0xA8
#define FMCS_SPI_CMD_READ_REQ    0xA0
#define FMCS_SPI_CMD_READ_RESP   0x50

/* Driver private structure */
struct fmcs_priv {
	struct device *dev;
	struct spi_device *spi;
	int irq;
	struct gpio_desc *gpiod_int;
	struct gpio_desc *gpiod_clk;
	struct gpio_desc *gpiod_data;

	/* Char device */
	dev_t devno;
	struct cdev cdev;
	struct class *class;

	/* Virtual Net Devices */
	struct net_device *omci_dev;
	struct net_device *ploam_dev;

	/* Synchronization */
	struct mutex lock;
	wait_queue_head_t wq;
	bool event_pending;
	struct fmcs_ploam_msg last_ploam;
};

/* Function Prototypes */
int fmcs_spi_write_fpga_reg(struct fmcs_priv *priv, u32 addr, u32 val);
int fmcs_spi_read_fpga_reg(struct fmcs_priv *priv, u32 addr, u32 *val);
int fmcs_spi_write_bosa_reg(struct fmcs_priv *priv, u32 addr, u32 val);
int fmcs_spi_read_bosa_reg(struct fmcs_priv *priv, u32 addr, u32 *val);

int fmcs_netdevs_init(struct fmcs_priv *priv);
void fmcs_netdevs_exit(struct fmcs_priv *priv);
void fmcs_rx_omci_frame(struct fmcs_priv *priv, const u8 *data, size_t len);
void fmcs_rx_ploam_frame(struct fmcs_priv *priv, const u8 *data, size_t len);

#endif /* _H3C_FMCS_H_ */
