/*
 * SPDX-FileCopyrightText: 2024 Kyunghwan Kwon <k@libmcu.org>
 *
 * SPDX-License-Identifier: MIT
 */

#include "hlw811x.h"
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

typedef hlw811x_error_t (*encoder_t)(uint8_t *buf, size_t bufsize,
		const uint8_t *data, size_t datalen, size_t *encoded_len);
typedef hlw811x_error_t (*decoder_t)(uint8_t *buf, size_t bufsize,
		const uint8_t *tx, size_t tx_len,
		const uint8_t *rx, size_t rx_len, size_t *decoded_len);

static hlw811x_interface_t iface;

static int32_t convert_24bit_to_int32(const int32_t value)
{
	int32_t result = value;

	if (result & 0x00800000) {
		return (int32_t)(result | (int32_t)0xFF000000);
	}

	return (int32_t)result;
}

static hlw811x_error_t encode_uart(uint8_t *buf, size_t bufsize,
		const uint8_t *data, size_t datalen, size_t *encoded_len)
{
	if (bufsize < datalen + 2) {
		HLW811X_ERROR("Buffer size is too small");
		return HLW811X_BUFFER_TOO_SMALL;
	}

	buf[0] = 0xA5;
	uint8_t chksum = buf[0];

	for (size_t i = 0; i < datalen; i++) {
		buf[i + 1] = data[i];
		chksum += data[i];
	}

	buf[datalen + 1] = ~chksum;
	*encoded_len = datalen + 2;

	return HLW811X_ERROR_NONE;
}

static hlw811x_error_t decode_uart(uint8_t *buf, size_t bufsize,
		const uint8_t *tx, size_t tx_len,
		const uint8_t *rx, size_t rx_len, size_t *decoded_len)
{
	if (tx_len < 1) {
		HLW811X_ERROR("Invalid tx_len");
		return HLW811X_INVALID_PARAM;
	}

	if (rx_len < 3 || bufsize < rx_len - 1) {
		HLW811X_ERROR("Invalid rx_len");
		return HLW811X_INVALID_PARAM;
	}

	uint8_t chksum = ~tx[tx_len - 1];

	for (size_t i = 0; i < rx_len - 1; i++) {
		buf[i] = rx[i];
		chksum += rx[i];
	}

	if ((uint8_t)~chksum != rx[rx_len - 1]) {
		HLW811X_ERROR("chksum mismatch %x : %x", ~chksum, rx[rx_len-1]);
		return HLW811X_CHECKSUM_MISMATCH;
	}

	*decoded_len = rx_len - 1;

	return HLW811X_ERROR_NONE;
}

static hlw811x_error_t encode(uint8_t *buf, size_t bufsize,
		const uint8_t *data, size_t datalen, size_t *len)
{
	encoder_t encoder = encode_uart;

	if (iface == HLW811X_UART) {
	} else {
		HLW811X_ERROR("Not implemented");
		return HLW811X_NOT_IMPLEMENTED;
	}

	return (*encoder)(buf, bufsize, data, datalen, len);

	return HLW811X_ERROR_NONE;
}

static hlw811x_error_t decode(uint8_t *buf, size_t bufsize,
		const uint8_t *tx, size_t tx_len,
		const uint8_t *rx, size_t rx_len, size_t *len)
{
	decoder_t decoder = decode_uart;

	if (iface == HLW811X_UART) {
	} else {
		HLW811X_ERROR("Not implemented");
		return HLW811X_NOT_IMPLEMENTED;
	}

	return (*decoder)(buf, bufsize, tx, tx_len, rx, rx_len, len);
}

static hlw811x_error_t encode_frame(hlw811x_reg_addr_t addr,
		uint8_t *txbuf, size_t txbuf_len,
		const uint8_t *data, size_t datalen,
		size_t *frame_len)
{
	uint8_t payload[datalen + 1];

	if (datalen > 2 || (data == NULL && datalen != 0)) {
		HLW811X_ERROR("Invalid parameter");
		return HLW811X_INVALID_PARAM;
	}

	payload[0] = (uint8_t)addr;
	for (size_t i = 0; i < datalen; i++) {
		payload[i + 1] = data[i];
	}

	return encode(txbuf, txbuf_len, payload, datalen+1, frame_len);
}

static hlw811x_error_t decode_frame(uint8_t *buf, size_t bufsize,
		const uint8_t *tx, size_t tx_len,
		const uint8_t *rx, size_t rx_len)
{
	size_t len;
	hlw811x_error_t err;

	if ((err = decode(buf, bufsize, tx, tx_len, rx, rx_len, &len))
			!= HLW811X_ERROR_NONE) {
		return err;
	} else if (len != bufsize) {
		HLW811X_ERROR("decoder returned %d", len);
		return HLW811X_INCORRECT_RESPONSE;
	}

	return HLW811X_ERROR_NONE;
}

static hlw811x_error_t send_frame(const uint8_t *data, size_t datalen)
{
	int err;

	if ((err = hlw811x_ll_write(data, datalen)) < 0) {
		HLW811X_ERROR("hlw811x_ll_write() failed: %x", err);
		return HLW811X_IO_ERROR;
	}

	if ((size_t)err != datalen) {
		HLW811X_ERROR("tx len mismatch: %d != %d", err, datalen);
		return HLW811X_IO_MISSING_BYTES;
	}

	return HLW811X_ERROR_NONE;
}

static hlw811x_error_t write_reg(hlw811x_reg_addr_t addr,
		const uint8_t *data, size_t datalen)
{
	uint8_t frame[datalen + 1/*addr*/ + 2/*header+chksum*/];
	size_t frame_len;
	hlw811x_error_t err;

	if ((err = encode_frame(addr | 0x80u, frame, sizeof(frame),
				data, datalen, &frame_len))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	return send_frame(frame, frame_len);
}

static hlw811x_error_t read_reg(hlw811x_reg_addr_t addr,
		uint8_t *buf, size_t bytes_to_read)
{
	hlw811x_error_t err;
	int bytes_received;
	uint8_t rx[bytes_to_read + 1];
	uint8_t tx[3];
	size_t encoded_len;
	size_t tx_len;

	if ((err = encode_frame(addr, tx, sizeof(tx), 0, 0, &encoded_len))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	tx_len = encoded_len;
	if (iface == HLW811X_UART) {
		tx_len -= 1; /* do not send chksum */
	}

	if ((err = send_frame(tx, tx_len)) != HLW811X_ERROR_NONE) {
		return err;
	}

	if ((bytes_received = hlw811x_ll_read(rx, sizeof(rx))) < 0) {
		HLW811X_ERROR("hlw811x_ll_read() failed");
		return HLW811X_IO_ERROR;
	}

	return decode_frame(buf, bytes_to_read, tx, encoded_len,
			rx, (size_t)bytes_received);
}

static hlw811x_error_t reset_chip(void)
{
	const uint8_t cmd = CMD_RESET_CHIP;
	return write_reg(HLW811X_REG_COMMAND, &cmd, 1);
}

static hlw811x_error_t enable_write(void)
{
	const uint8_t cmd = CMD_ENABLE_WRITE;
	return write_reg(HLW811X_REG_COMMAND, &cmd, 1);
}

static hlw811x_error_t disable_write(void)
{
	const uint8_t cmd = CMD_DISABLE_WRITE;
	return write_reg(HLW811X_REG_COMMAND, &cmd, 1);
}

static hlw811x_error_t select_channel(hlw811x_channel_t channel)
{
	const uint8_t cmd = (channel == HLW811X_CHANNEL_A)?
		CMD_SET_CHANNEL_A : CMD_SET_CHANNEL_B;
	return write_reg(HLW811X_REG_COMMAND, &cmd, 1);
}

hlw811x_error_t hlw811x_write_reg(hlw811x_reg_addr_t addr,
		const uint8_t *data, size_t datalen)
{
	return write_reg(addr, data, datalen);
}

hlw811x_error_t hlw811x_read_reg(hlw811x_reg_addr_t addr,
		uint8_t *buf, size_t bufsize)
{
	return read_reg(addr, buf, bufsize);
}

hlw811x_error_t hlw811x_select_channel(hlw811x_channel_t channel)
{
	return select_channel(channel);
}

hlw811x_error_t hlw811x_reset(void)
{
	return reset_chip();
}

hlw811x_error_t hlw811x_init(hlw811x_interface_t interface)
{
	iface = interface;
	return HLW811X_ERROR_NONE;
}
