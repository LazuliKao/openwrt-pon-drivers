// SPDX-License-Identifier: GPL-2.0-only
/*
 * H3C HM2004-DU Micro-OLT FPGA SPI Transport Implementation
 * Direct MMIO control for Airoha AN7581 SNFI/SPI controller (CS1)
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#include <linux/io.h>
#include "fmcs.h"

static int an7581_spi_op_write(struct fmcs_priv *priv, u32 cmd, u32 len)
{
	int timeout = 10000;
	u32 val = ((cmd & 0x1f) << 9) | (len & 0x1ff);

	writel(val, priv->spi_base + REG_SPI_OPFIFO_WDATA);

	while (readl(priv->spi_base + REG_SPI_OPFIFO_FULL) && --timeout)
		cpu_relax();
	if (!timeout)
		return -ETIMEDOUT;

	writel(1, priv->spi_base + REG_SPI_OPFIFO_WR);

	timeout = 10000;
	while (!readl(priv->spi_base + REG_SPI_OPFIFO_EMPTY) && --timeout)
		cpu_relax();
	if (!timeout)
		return -ETIMEDOUT;

	return 0;
}

static int an7581_spi_write_bytes(struct fmcs_priv *priv, const u8 *buf, u32 len)
{
	u32 rem = len;
	const u8 *ptr = buf;

	while (rem > 0) {
		u32 chunk = min_t(u32, rem, 511);
		int ret = an7581_spi_op_write(priv, SPI_OP_TX_DFIFO, chunk);
		if (ret)
			return ret;

		for (u32 i = 0; i < chunk; i++) {
			int timeout = 10000;
			while (readl(priv->spi_base + REG_SPI_DFIFO_W_FULL) && --timeout)
				cpu_relax();
			if (!timeout)
				return -ETIMEDOUT;
			writel(*ptr++, priv->spi_base + REG_SPI_DFIFO_WDATA);
		}
		rem -= chunk;
	}
	return 0;
}

static int an7581_spi_read_bytes(struct fmcs_priv *priv, u8 *buf, u32 len)
{
	u32 rem = len;
	u8 *ptr = buf;

	while (rem > 0) {
		u32 chunk = min_t(u32, rem, 511);
		int ret = an7581_spi_op_write(priv, SPI_OP_RX_DFIFO, chunk);
		if (ret)
			return ret;

		for (u32 i = 0; i < chunk; i++) {
			int timeout = 10000;
			while (readl(priv->spi_base + REG_SPI_DFIFO_R_EMPTY) && --timeout)
				cpu_relax();
			if (!timeout)
				return -ETIMEDOUT;
			*ptr++ = (u8)readl(priv->spi_base + REG_SPI_DFIFO_RDATA);
			writel(1, priv->spi_base + REG_SPI_DFIFO_RD);
		}
		rem -= chunk;
	}
	return 0;
}

static int fmcs_raw_spi_write(struct fmcs_priv *priv, const u8 *buf, u32 len)
{
	int timeout = 10000;
	int ret;

	if (!priv->spi_base || !priv->nfi_base)
		return -ENODEV;

	/* Enable Manual Mode */
	writel(9, priv->spi_base + REG_SPI_MANUAL_OP_CTRL);
	writel(1, priv->spi_base + REG_SPI_CTRL_MANUAL_EN);

	/* Switch NFI to CS1 (FPGA) */
	writel(1, priv->nfi_base + REG_NFI_CS_SEL);

	/* Assert CS (Active Low) */
	ret = an7581_spi_op_write(priv, SPI_OP_ASSERT_CS, 1);
	if (!ret)
		ret = an7581_spi_write_bytes(priv, buf, len);

	/* Deassert CS */
	an7581_spi_op_write(priv, SPI_OP_DEASSERT_CS, 1);

	/* Restore NFI to CS0 (NAND) */
	writel(0, priv->nfi_base + REG_NFI_CS_SEL);

	/* Restore Auto Mode */
	writel(0, priv->spi_base + REG_SPI_MANUAL_OP_CTRL);
	writel(0, priv->spi_base + REG_SPI_CTRL_MANUAL_EN);

	return ret;
}

static int fmcs_raw_spi_read(struct fmcs_priv *priv, const u8 *send_buf, u32 send_len,
			     u8 *recv_buf, u32 recv_len)
{
	int timeout = 10000;
	int ret;

	if (!priv->spi_base || !priv->nfi_base)
		return -ENODEV;

	/* Enable Manual Mode */
	writel(9, priv->spi_base + REG_SPI_MANUAL_OP_CTRL);
	writel(1, priv->spi_base + REG_SPI_CTRL_MANUAL_EN);

	/* Switch NFI to CS1 (FPGA) */
	writel(1, priv->nfi_base + REG_NFI_CS_SEL);

	/* Assert CS (Active Low) */
	ret = an7581_spi_op_write(priv, SPI_OP_ASSERT_CS, 1);
	if (!ret && send_buf && send_len > 0)
		ret = an7581_spi_write_bytes(priv, send_buf, send_len);

	if (!ret && recv_buf && recv_len > 0)
		ret = an7581_spi_read_bytes(priv, recv_buf, recv_len);

	/* Deassert CS */
	an7581_spi_op_write(priv, SPI_OP_DEASSERT_CS, 1);

	/* Restore NFI to CS0 (NAND) */
	writel(0, priv->nfi_base + REG_NFI_CS_SEL);

	/* Restore Auto Mode */
	writel(0, priv->spi_base + REG_SPI_MANUAL_OP_CTRL);
	writel(0, priv->spi_base + REG_SPI_CTRL_MANUAL_EN);

	return ret;
}

int fmcs_spi_write_fpga_reg(struct fmcs_priv *priv, u32 addr, u32 val)
{
	u8 tx_buf[9];

	tx_buf[0] = FMCS_SPI_CMD_WRITE_REG;
	*(__be32 *)&tx_buf[1] = cpu_to_be32(addr);
	*(__be32 *)&tx_buf[5] = cpu_to_be32(val);

	return fmcs_raw_spi_write(priv, tx_buf, sizeof(tx_buf));
}

int fmcs_spi_read_fpga_reg(struct fmcs_priv *priv, u32 addr, u32 *val)
{
	u8 tx_req[5];
	u8 rx_cmd = FMCS_SPI_CMD_READ_RESP;
	u8 rx_resp[4] = { 0 };
	int ret;

	if (!val)
		return -EINVAL;

	/* Phase 1: Send Read Request (0xA0 + addr) */
	tx_req[0] = FMCS_SPI_CMD_READ_REQ;
	*(__be32 *)&tx_req[1] = cpu_to_be32(addr);

	ret = fmcs_raw_spi_write(priv, tx_req, sizeof(tx_req));
	if (ret)
		return ret;

	/* FPGA internal processing delay */
	udelay(30);

	/* Phase 2: Read 4 bytes of register data with command 0x50 */
	ret = fmcs_raw_spi_read(priv, &rx_cmd, 1, rx_resp, sizeof(rx_resp));
	if (ret)
		return ret;

	*val = be32_to_cpup((__be32 *)rx_resp);
	return 0;
}

int fmcs_spi_write_bosa_reg(struct fmcs_priv *priv, u32 addr, u32 val)
{
	if (addr < 0x10000)
		addr = 0x06006300 | addr;
	return fmcs_spi_write_fpga_reg(priv, addr, val);
}

int fmcs_spi_read_bosa_reg(struct fmcs_priv *priv, u32 addr, u32 *val)
{
	if (addr < 0x10000)
		addr = 0x06006300 | addr;
	return fmcs_spi_read_fpga_reg(priv, addr, val);
}

int fmcs_spi_send_ploam(struct fmcs_priv *priv, const u8 *data, size_t len)
{
	u8 buf[18] = { 0 };

	if (len > 16)
		len = 16;
	buf[0] = 0xAA;
	memcpy(&buf[1], data, len);
	return fmcs_raw_spi_write(priv, buf, len + 1);
}

int fmcs_spi_recv_ploam(struct fmcs_priv *priv, u8 *data, size_t *len)
{
	u8 req[5] = { 0xA2, 0x10, 0x00, 0x00, 0x00 };
	u8 cmd = 0x52;
	u8 rx[13] = { 0 };
	int ret;

	ret = fmcs_raw_spi_write(priv, req, sizeof(req));
	if (ret)
		return ret;

	udelay(30);

	ret = fmcs_raw_spi_read(priv, &cmd, 1, rx, sizeof(rx));
	if (ret)
		return ret;

	memcpy(data, rx, sizeof(rx));
	if (len)
		*len = sizeof(rx);
	return 0;
}
