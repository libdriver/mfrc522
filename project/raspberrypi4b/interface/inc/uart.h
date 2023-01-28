/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      uart.h
 * @brief     uart header file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2022-11-11
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/11/11  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#ifndef UART_H
#define UART_H

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup uart uart function
 * @brief    uart function modules
 * @{
 */

/**
 * @brief      uart init
 * @param[in]  *name points to a device name buffer
 * @param[out] *fd points to a uart handler buffer
 * @param[in]  baud_rate is the baud rate
 * @param[in]  data_bits is the data bits
 * @param[in]  parity is the data parity
 * @param[in]  stop_bits is the stop bits
 * @return     status code
 *             - 0 success
 *             - 1 uart init failed
 * @note       none
 */
uint8_t uart_init(char *name, int *fd, uint32_t baud_rate, uint8_t data_bits, char parity, uint8_t stop_bits);

/**
 * @brief     uart deinit
 * @param[in] fd is the uart handle
 * @return    status code
 *            - 0 success
 *            - 1 deinit failed
 * @note      none
 */
uint8_t uart_deinit(int fd);

/**
 * @brief     uart write data
 * @param[in] fd is the uart handle
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t uart_write(int fd, uint8_t *buf, uint32_t len);

/**
 * @brief          uart read data
 * @param[in]      fd is the uart handle
 * @param[out]     *buf points to a data buffer
 * @param[in, out] *len points to a length of the data buffer
 * @return         status code
 *                 - 0 success
 *                 - 1 read failed
 * @note           none
 */
uint8_t uart_read(int fd, uint8_t *buf, uint32_t *len);

/**
 * @brief     uart flush
 * @param[in] fd is the uart handle
 * @return    status code
 *            - 0 success
 *            - 1 flush failed
 * @note      none
 */
uint8_t uart_flush(int fd);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif 
