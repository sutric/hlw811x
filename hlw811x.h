/*
 * SPDX-FileCopyrightText: 2024 Kyunghwan Kwon <k@libmcu.org>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef HLW811X_H
#define HLW811X_H

#if defined(__cplusplus)
extern "C" {
#endif

typedef enum {
	HLW811X_ERROR_NONE,
	HLW811X_INVALID_PARAM,
	HLW811X_IO_ERROR,
	HLW811X_INCORRECT_RESPONSE,
} hlw811x_error_t;

typedef enum {
	HLW811X_CHANNEL_A,
	HLW811X_CHANNEL_B,
} hlw811x_channel_t;

/**
 * @brief Initializes the HLW811x chip.
 *
 * This function is used to initialize the HLW811x chip. It does this by
 * calling the `reset_chip` function, which presumably resets the chip to
 * a known state.
 *
 * @note At least 60ms delay is required after initialization before calling
 * any other functions because the chip needs time to stabilize such as crystal
 * oscillator start-up time.
 *
 * @return hlw811x_error_t
 */
hlw811x_error_t hlw811x_init(void);

#if defined(__cplusplus)
}
#endif

#endif /* HLW811X_H */
