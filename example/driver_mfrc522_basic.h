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
 * @file      driver_mfrc522_basic.h
 * @brief     driver mfrc522 basic header file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2022-05-31
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/05/31  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#ifndef DRIVER_MFRC522_BASIC_H
#define DRIVER_MFRC522_BASIC_H

#include "driver_mfrc522_interface.h"

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @defgroup mfrc522_example_driver mfrc522 example driver function
 * @brief    mfrc522 example driver modules
 * @ingroup  mfrc522_driver
 * @{
 */

/**
 * @brief mfrc522 basic example default definition
 */
#define MFRC522_BASIC_DEFAULT_INTERRUPT1_PIN_INVERT        MFRC522_BOOL_TRUE                                         /**< enable */
#define MFRC522_BASIC_DEFAULT_INTERRUPT_PIN_TYPE           MFRC522_INTERRUPT_PIN_TYPE_STANDARD_CMOS                  /**< standard cmos */
#define MFRC522_BASIC_DEFAULT_FORCE_IIC_HIGH_SPEED         MFRC522_BOOL_FALSE                                        /**< disable */
#define MFRC522_BASIC_DEFAULT_CLEAR_TEMPERATURE_ERROR      MFRC522_BOOL_FALSE                                        /**< disable */
#define MFRC522_BASIC_DEFAULT_WATER_LEVEL                  8                                                         /**< water level */
#define MFRC522_BASIC_DEFAULT_RX_ALIGN                     MFRC522_RX_ALIGN_0                                        /**< align 0 */
#define MFRC522_BASIC_DEFAULT_TX_CRC_GENERATION            MFRC522_BOOL_FALSE                                        /**< disable */
#define MFRC522_BASIC_DEFAULT_TX_SPEED                     MFRC522_SPEED_106_KBD                                     /**< 106 kBd */
#define MFRC522_BASIC_DEFAULT_MODULATION_INVERT            MFRC522_BOOL_FALSE                                        /**< disable */
#define MFRC522_BASIC_DEFAULT_RX_CRC_GENERATION            MFRC522_BOOL_FALSE                                        /**< disable */
#define MFRC522_BASIC_DEFAULT_RX_SPEED                     MFRC522_SPEED_106_KBD                                     /**< 106 kBd */
#define MFRC522_BASIC_DEFAULT_RX_NO_ERROR                  MFRC522_BOOL_TRUE                                         /**< enable */
#define MFRC522_BASIC_DEFAULT_RX_MULTIPLE                  MFRC522_BOOL_FALSE                                        /**< disable */
#define MFRC522_BASIC_DEFAULT_TX_INPUT                     MFRC522_TX_INPUT_INTERNAL_ENCODER                         /**< internal encoder */
#define MFRC522_BASIC_DEFAULT_MFOUT_INPUT                  MFRC522_MFOUT_INPUT_3_STATE                               /**< 3 state */
#define MFRC522_BASIC_DEFAULT_MINI_LEVEL                   0x8                                                       /**< mini level 8 */
#define MFRC522_BASIC_DEFAULT_COLLISION_LEVEL              0x4                                                       /**< collision level 4 */
#define MFRC522_BASIC_DEFAULT_CHANNEL_RECEPTION            MFRC522_CHANNEL_RECEPTION_STRONGER_FREEZE_SELECTED        /**< stronger channel and 
                                                                                                                          freezes the selected 
                                                                                                                          channel during communication*/
#define MFRC522_BASIC_DEFAULT_FIX_IQ                       MFRC522_BOOL_FALSE                                        /**< disable */
#define MFRC522_BASIC_DEFAULT_TIMER_PRESCAL_EVEN           MFRC522_BOOL_FALSE                                        /**< disable */
#define MFRC522_BASIC_DEFAULT_TIMER_CONSTANT_RECEPTION     0x3                                                       /**< constant reception 0x3 */
#define MFRC522_BASIC_DEFAULT_TIMER_CONSTANT_SYNC          0x1                                                       /**< constant sync 0x01 */
#define MFRC522_BASIC_DEFAULT_TX_WAIT                      0x2                                                       /**< tx wait 0x2 */
#define MFRC522_BASIC_DEFAULT_PARITY_DISABLE               MFRC522_BOOL_FALSE                                        /**< disable */
#define MFRC522_BASIC_DEFAULT_SERIAL_SPEED_T0              0x07                                                      /**< 9600 */
#define MFRC522_BASIC_DEFAULT_SERIAL_SPEED_T1              0x0B                                                      /**< 9600 */
#define MFRC522_BASIC_DEFAULT_MODULATION_WIDTH             0x26                                                      /**< modulation width 0x26 */
#define MFRC522_BASIC_DEFAULT_CWGSN                        0x8                                                       /**< cwgsn 0x8 */
#define MFRC522_BASIC_DEFAULT_MODGSN                       0x8                                                       /**< modgsn 0x8 */
#define MFRC522_BASIC_DEFAULT_CWGSP                        0x20                                                      /**< cwgsp 0x20 */
#define MFRC522_BASIC_DEFAULT_MODGSP                       0x20                                                      /**< modgsp 0x20 */
#define MFRC522_BASIC_DEFAULT_INTERRUPT1_TX                MFRC522_BOOL_FALSE                                        /**< disable */
#define MFRC522_BASIC_DEFAULT_INTERRUPT1_RX                MFRC522_BOOL_TRUE                                         /**< enable */
#define MFRC522_BASIC_DEFAULT_INTERRUPT1_IDLE              MFRC522_BOOL_TRUE                                         /**< enable */
#define MFRC522_BASIC_DEFAULT_INTERRUPT1_HI_ALERT          MFRC522_BOOL_FALSE                                        /**< disable */
#define MFRC522_BASIC_DEFAULT_INTERRUPT1_LO_ALERT          MFRC522_BOOL_FALSE                                        /**< disable */
#define MFRC522_BASIC_DEFAULT_INTERRUPT1_ERR               MFRC522_BOOL_TRUE                                         /**< enable */
#define MFRC522_BASIC_DEFAULT_INTERRUPT1_TIMER             MFRC522_BOOL_TRUE                                         /**< enable */
#define MFRC522_BASIC_DEFAULT_INTERRUPT2_MFIN_ACT          MFRC522_BOOL_FALSE                                        /**< disable */
#define MFRC522_BASIC_DEFAULT_INTERRUPT2_CRC               MFRC522_BOOL_TRUE                                         /**< enable */
#define MFRC522_BASIC_DEFAULT_CRC_MSB_FIRST                MFRC522_BOOL_FALSE                                        /**< disable */
#define MFRC522_BASIC_DEFAULT_TX_WAIT_RF                   MFRC522_BOOL_TRUE                                         /**< enable */
#define MFRC522_BASIC_DEFAULT_MFIN_POLARITY                MFRC522_MFIN_POLARITY_HIGH                                /**< mfin polarity high */
#define MFRC522_BASIC_DEFAULT_CRC_PRESET                   MFRC522_CRC_PRESET_6363                                   /**< 0x6363 */
#define MFRC522_BASIC_DEFAULT_FORCE_100_ASK                MFRC522_BOOL_TRUE                                         /**< enable */
#define MFRC522_BASIC_DEFAULT_CONTACTLESS_UART_INPUT       MFRC522_CONTACTLESS_UART_INTERNAL_ANALOG_MODULE           /**< analog module */
#define MFRC522_BASIC_DEFAULT_RX_WAIT                      0x6                                                       /**< rx wait 0x6 */
#define MFRC522_BASIC_DEFAULT_RX_GAIN                      MFRC522_RX_GAIN_48_DB                                     /**< rx gain 48 db */
#define MFRC522_BASIC_DEFAULT_TIMER_AUTO                   MFRC522_BOOL_TRUE                                         /**< enable */
#define MFRC522_BASIC_DEFAULT_TIMER_GATED_MODE             MFRC522_TIMER_GATED_MODE_NONE                             /**< gated mode none */
#define MFRC522_BASIC_DEFAULT_TIMER_AUTO_RESTART           MFRC522_BOOL_FALSE                                        /**< disable */
#define MFRC522_BASIC_DEFAULT_TIMER_PRESCALER              0xD3E                                                     /**< 0xD3E */
#define MFRC522_BASIC_DEFAULT_TIMER_RELOAD                 0x001E                                                    /**< 0x001E */

/**
 * @brief  interrupt irq
 * @return status code
 *         - 0 success
 *         - 1 run failed
 * @note   none
 */
uint8_t mfrc522_interrupt_irq_handler(void);

/**
 * @brief     basic example init
 * @param[in] interface bus interface
 * @param[in] addr iic device address
 * @param[in] *callback pointer to a callback function
 * @return    status code
 *            - 0 success
 *            - 1 init failed
 * @note      none
 */
uint8_t mfrc522_basic_init(mfrc522_interface_t interface, uint8_t addr, void (*callback)(uint16_t type));

/**
 * @brief  basic example deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t mfrc522_basic_deinit(void);

/**
 * @brief         basic example transceiver
 * @param[in]     *in_buf pointer to a input buffer
 * @param[in]     in_len input length
 * @param[out]    *out_buf pointer to a output buffer
 * @param[in,out] *out_len pointer to a output length buffer
 * @return        status code
 *                - 0 success
 *                - 1 transceiver failed
 * @note          none
 */
uint8_t mfrc522_basic_transceiver(uint8_t *in_buf, uint8_t in_len, uint8_t *out_buf, uint8_t *out_len);

/**
 * @brief      basic example calculate the crc
 * @param[in]  *buf pointer to a buffer
 * @param[in]  len buffer length
 * @param[out] *crc pointer to a crc buffer
 * @return     status code
 *             - 0 success
 *             - 1 calculate crc failed
 * @note       none
 */
uint8_t mfrc522_basic_calculate_crc(uint8_t *buf, uint8_t len, uint16_t *crc);

/**
 * @brief      basic example generate the random
 * @param[out] *buf pointer to a random buffer
 * @return     status code
 *             - 0 success
 *             - 1 generate random failed
 * @note       none
 */
uint8_t mfrc522_basic_generate_random(uint8_t buf[25]);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
