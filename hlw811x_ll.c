/**
 * @file hlw811x_ll.c
 * @brief Implementation of Low Level functions in hlw811x_overrides.h.
 */

/*----------------------------------------------------*/
/*--------------------- Inlcudes ---------------------*/

#include "stdint.h"
#include "stddef.h"
#include "gpio.h"
#include "spi.h"
/*----------------------------------------------------*/
/*----------------- Macro definitions ----------------*/

#define HAL_SPI_DELAY 10000 // ms since

/*----------------------------------------------------*/
/*-------------------- Data types --------------------*/

/*----------------------------------------------------*/
/*-------------------- Variables ---------------------*/

/*----------------------------------------------------*/
/*-------------- Functions Declarations --------------*/

/*----------------------------------------------------*/
/*-------------- Functions Definitions --------------*/

/**
 * @brief Writes data to the HLW811X device at a low level.
 *
 * This function sends the specified data to the HLW811X device.
 *
 * @param[in] data Pointer to the data to be written.
 * @param[in] datalen Length of the data to be written.
 * @param[in] ctx Pointer to the context or additional parameters required for
 *            the write operation.
 *
 * @return int Returns 0 on success, or a negative error code on failure.
 */
int hlw811x_ll_write(const uint8_t *data, size_t datalen, void *ctx)
{
    HAL_StatusTypeDef result;
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET); // CS low
    result = HAL_SPI_Transmit(&hspi1, data, datalen, HAL_SPI_DELAY);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET); // CS high
    if (result == HAL_OK)
    {
        return datalen;
    }
    return -1;
}

/**
 * @brief Reads data from the HLW811X device at a low level.
 *
 * This function reads data from the HLW811X device into the specified buffer.
 *
 * @param[out] buf Pointer to the buffer where the read data will be stored.
 * @param[in] bufsize Size of the buffer.
 * @param[in] ctx Pointer to the context or additional parameters required for
 *            the read operation.
 *
 * @return int Returns the number of bytes read on success, or a negative error
 *             code on failure.
 */
int hlw811x_ll_read(uint8_t *buf, size_t bufsize, void *ctx)
{
    HAL_StatusTypeDef result;
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET); // CS low
    result = HAL_SPI_Receive(&hspi1, buf, bufsize, HAL_SPI_DELAY);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET); // CS high
    if (result == HAL_OK)
    {
        return bufsize;
    }
    return -1;
}
