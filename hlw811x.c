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

hlw811x_error_t hlw811x_init(void)
{
	return HLW811X_ERROR_NONE;
}
