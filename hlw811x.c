/*
 * SPDX-FileCopyrightText: 2024 Kyunghwan Kwon <k@libmcu.org>
 *
 * SPDX-License-Identifier: MIT
 */

#include "hlw811x.h"
#include "hlw811x_regs.h"
#include "hlw811x_overrides.h"

#if !defined(HLW811X_MCLK)
#define HLW811X_MCLK		(3579545UL) /* Hz (= 3.579545MHz) */
#endif

#if !defined(HLW811X_DEBUG)
#define HLW811X_DEBUG(...)
#endif
#if !defined(HLW811X_INFO)
#define HLW811X_INFO(...)
#endif
#if !defined(HLW811X_ERROR)
#define HLW811X_ERROR(...)
#endif

enum {
	CMD_ENABLE_WRITE	= 0xE5u,
	CMD_DISABLE_WRITE	= 0xDCu,
	CMD_SET_CHANNEL_A	= 0x5Au,
	CMD_SET_CHANNEL_B	= 0xA5u,
	CMD_RESET_CHIP		= 0x96u,
};

static int32_t convert_24bit_to_int32(const int32_t value)
{
	int32_t result = value;

	if (result & 0x00800000) {
		return (int32_t)(result | (int32_t)0xFF000000);
	}

	return (int32_t)result;
}

static hlw811x_error_t write_reg(hlw811x_reg_addr_t addr,
		const uint8_t *data, size_t datalen)
{
	uint8_t txbuf[3] = { addr | 0x80u, 0, 0 };

	if (datalen > 2 || (data == NULL && datalen != 0)) {
		HLW811X_ERROR("Invalid parameter");
		return HLW811X_INVALID_PARAM;
	}

	for (size_t i = 0; i < datalen; i++) {
		txbuf[i + 1] = data[i];
	}

	if (hlw811x_ll_write(txbuf, datalen+1) != 0) {
		HLW811X_ERROR("hlw811x_ll_write() failed");
		return HLW811X_IO_ERROR;
	}

	return HLW811X_ERROR_NONE;
}

static hlw811x_error_t read_reg(hlw811x_reg_addr_t addr,
		uint8_t *buf, size_t bytes_to_read)
{
	hlw811x_error_t err = write_reg(addr, 0, 0);

	if (err != HLW811X_ERROR_NONE) {
		return err;
	}

	int rc = hlw811x_ll_read(buf, bytes_to_read);

	if (rc < 0) {
		HLW811X_ERROR("hlw811x_ll_read() failed");
		return HLW811X_IO_ERROR;
	} else if (rc != (int)bytes_to_read) {
		HLW811X_ERROR("hlw811x_ll_read() returned %d", rc);
		return HLW811X_INCORRECT_RESPONSE;
	}

	return HLW811X_ERROR_NONE;
}

static hlw811x_error_t reset_chip(void)
{
	uint8_t cmd = CMD_RESET_CHIP;
	return write_reg(HLW811X_REG_COMMAND, &cmd, 1);
}

static hlw811x_error_t enable_write(void)
{
	uint8_t cmd = CMD_ENABLE_WRITE;
	return write_reg(HLW811X_REG_COMMAND, &cmd, 1);
}

static hlw811x_error_t disable_write(void)
{
	uint8_t cmd = CMD_DISABLE_WRITE;
	return write_reg(HLW811X_REG_COMMAND, &cmd, 1);
}

static hlw811x_error_t select_channel(hlw811x_channel_t channel)
{
	uint8_t cmd = (channel == HLW811X_CHANNEL_A)?
		CMD_SET_CHANNEL_A : CMD_SET_CHANNEL_B;
	return write_reg(HLW811X_REG_COMMAND, &cmd, 1);
}

hlw811x_error_t hlw811x_init(void)
{
	return reset_chip();
}
