// SPDX-License-Identifier: GPL-2.0-only
/*
 * H3C HM2004-DU Micro-OLT FPGA SPI Transport Implementation
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/byteorder/generic.h>
#include "fmcs.h"

int fmcs_spi_write_fpga_reg(struct fmcs_priv *priv, u32 addr, u32 val)
{
	u8 tx_buf[9];
	struct spi_transfer t = {
		.tx_buf = tx_buf,
		.len = sizeof(tx_buf),
	};
	struct spi_message m;

	if (!priv->spi)
		return -ENODEV;

	/*
	 * Protocol:
	 * Byte 0: 0xA8 (Write Reg)
	 * Bytes 1-4: Address (Big-Endian)
	 * Bytes 5-8: Value (Big-Endian)
	 */
	tx_buf[0] = FMCS_SPI_CMD_WRITE_REG;
	*(__be32 *)&tx_buf[1] = cpu_to_be32(addr);
	*(__be32 *)&tx_buf[5] = cpu_to_be32(val);

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return spi_sync(priv->spi, &m);
}

int fmcs_spi_read_fpga_reg(struct fmcs_priv *priv, u32 addr, u32 *val)
{
	u8 tx_req[5];
	u8 tx_resp[5] = { FMCS_SPI_CMD_READ_RESP, 0, 0, 0, 0 };
	u8 rx_resp[5] = { 0 };
	struct spi_transfer t_req = {
		.tx_buf = tx_req,
		.len = sizeof(tx_req),
	};
	struct spi_transfer t_resp = {
		.tx_buf = tx_resp,
		.rx_buf = rx_resp,
		.len = sizeof(tx_resp),
	};
	struct spi_message m;
	int ret;

	if (!priv->spi || !val)
		return -ENODEV;

	/* Phase 1: Send Read Request (0xA0 + addr) */
	tx_req[0] = FMCS_SPI_CMD_READ_REQ;
	*(__be32 *)&tx_req[1] = cpu_to_be32(addr);

	spi_message_init(&m);
	spi_message_add_tail(&t_req, &m);
	ret = spi_sync(priv->spi, &m);
	if (ret)
		return ret;

	/* Phase 2: Read 4 bytes of data with opcode 0x50 */
	spi_message_init(&m);
	spi_message_add_tail(&t_resp, &m);
	ret = spi_sync(priv->spi, &m);
	if (ret)
		return ret;

	/* Extract big-endian 32-bit register value */
	*val = be32_to_cpup((__be32 *)&rx_resp[1]);
	return 0;
}

int fmcs_spi_write_bosa_reg(struct fmcs_priv *priv, u32 addr, u32 val)
{
	/* BOSA UX3326 controller is mapped at FPGA sub-address 0x06006300 */
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

