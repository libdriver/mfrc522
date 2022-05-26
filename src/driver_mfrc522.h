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
 * @file      driver_mfrc522.h
 * @brief     driver mfrc522 header file
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

#ifndef DRIVER_MFRC522_H
#define DRIVER_MFRC522_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @defgroup mfrc522_driver mfrc522 driver function
 * @brief    mfrc522 driver modules
 * @{
 */

/**
 * @addtogroup mfrc522_basic_driver
 * @{
 */

/**
 * @brief mfrc522 interface enumeration definition
 */
typedef enum
{
    MFRC522_INTERFACE_IIC  = 0x00,       /**< iic interface */
    MFRC522_INTERFACE_SPI  = 0x01,       /**< spi interface */
    MFRC522_INTERFACE_UART = 0x02,       /**< uart interface */
} mfrc522_interface_t;

/**
 * @brief mfrc522 bool enumeration definition
 */
typedef enum
{
    MFRC522_BOOL_FALSE = 0x00,        /**< disable function */
    MFRC522_BOOL_TRUE  = 0x01,        /**< enable function */
} mfrc522_bool_t;

/**
 * @brief mfrc522 command enumeration definition
 */
typedef enum
{
    MFRC522_COMMAND_IDLE       = 0x00,        /**< no action, cancels current command execution */
    MFRC522_COMMAND_MEM        = 0x01,        /**< stores 25 bytes into the internal buffer */
    MFRC522_COMMAND_RANDOM_ID  = 0x02,        /**< generates a 10-byte random id number */
    MFRC522_COMMAND_CALC_CRC   = 0x03,        /**< activates the crc coprocessor or performs a self test */
    MFRC522_COMMAND_TRANSMIT   = 0x04,        /**< transmits data from the fifo buffer */
    MFRC522_COMMAND_NO_CHANGE  = 0x07,        /**< no command change */
    MFRC522_COMMAND_RECEIVE    = 0x08,        /**< activates the receiver circuits */
    MFRC522_COMMAND_TRANSCEIVE = 0x0C,        /**< transmits data from fifo buffer to antenna and automatically
                                                   activates the receiver after transmission */
    MFRC522_COMMAND_MF_AUTHENT = 0x0E,        /**< performs the MIFARE standard authentication as a reader */
    MFRC522_COMMAND_SOFT_RESET = 0x0F,        /**< resets the chip */
} mfrc522_command_t;

/**
 * @brief mfrc522 interrupt1 enumeration definition
 */
typedef enum
{
    MFRC522_INTERRUPT1_TX       = 6,        /**< allows the transmitter interrupt request to be propagated to pin irq */
    MFRC522_INTERRUPT1_RX       = 5,        /**< allows the receiver interrupt request to be propagated to pin irq */
    MFRC522_INTERRUPT1_IDLE     = 4,        /**< allows the idle interrupt request to be propagated to pin irq */
    MFRC522_INTERRUPT1_HI_ALERT = 3,        /**< allows the high alert interrupt request to be propagated to pin irq */
    MFRC522_INTERRUPT1_LO_ALERT = 2,        /**< allows the low alert interrupt request to be propagated to pin irq */
    MFRC522_INTERRUPT1_ERR      = 1,        /**< allows the error interrupt request to be propagated to pin irq */
    MFRC522_INTERRUPT1_TIMER    = 0,        /**< allows the timer interrupt request to be propagated to pin irq */
} mfrc522_interrupt1_t;

/**
 * @brief mfrc522 interrupt2 enumeration definition
 */
typedef enum
{
    MFRC522_INTERRUPT2_MFIN_ACT = 4,        /**< allows the MFIN active interrupt request to be propagated to pin irq */
    MFRC522_INTERRUPT2_CRC      = 2,        /**< allows the CRC interrupt request to be propagated to pin irq */
} mfrc522_interrupt2_t;

/**
 * @brief mfrc522 interrupt1 enumeration definition
 */
typedef enum
{
    MFRC522_INTERRUPT_MFIN_ACT = MFRC522_INTERRUPT2_MFIN_ACT + 8,        /**< allows the MFIN active interrupt request to be propagated to pin irq */
    MFRC522_INTERRUPT_CRC      = MFRC522_INTERRUPT2_CRC + 8,             /**< allows the CRC interrupt request to be propagated to pin irq */
    MFRC522_INTERRUPT_TX       = MFRC522_INTERRUPT1_TX,                  /**< allows the transmitter interrupt request to be propagated to pin irq */
    MFRC522_INTERRUPT_RX       = MFRC522_INTERRUPT1_RX,                  /**< allows the receiver interrupt request to be propagated to pin irq */
    MFRC522_INTERRUPT_IDLE     = MFRC522_INTERRUPT1_IDLE,                /**< allows the idle interrupt request to be propagated to pin irq */
    MFRC522_INTERRUPT_HI_ALERT = MFRC522_INTERRUPT1_HI_ALERT,            /**< allows the high alert interrupt request to be propagated to pin irq */
    MFRC522_INTERRUPT_LO_ALERT = MFRC522_INTERRUPT1_LO_ALERT,            /**< allows the low alert interrupt request to be propagated to pin irq */
    MFRC522_INTERRUPT_ERR      = MFRC522_INTERRUPT1_ERR,                 /**< allows the error interrupt request to be propagated to pin irq */
    MFRC522_INTERRUPT_TIMER    = MFRC522_INTERRUPT1_TIMER,               /**< allows the timer interrupt request to be propagated to pin irq */
} mfrc522_interrupt_t;

/**
 * @brief mfrc522 interrupt pin type enumeration definition
 */
typedef enum
{
    MFRC522_INTERRUPT_PIN_TYPE_STANDARD_CMOS = 1,        /**< standard cmos output pin */
    MFRC522_INTERRUPT_PIN_TYPE_OPEN_DRAIN    = 0,        /**< open drain output pin */
} mfrc522_interrupt_pin_type_t;

/**
 * @brief mfrc522 interrupt mark enumeration definition
 */
typedef enum
{
    MFRC522_INTERRUPT_MARK_SET     = 1,        /**< marked 1 */
    MFRC522_INTERRUPT_MARK_CLEARED = 0,        /**< marked 0 */
} mfrc522_interrupt_mark_t;

/**
 * @brief mfrc522 error enumeration definition
 */
typedef enum
{
    MFRC522_ERROR_WR               = (1 << 7),        /**< write error */
    MFRC522_ERROR_TEMP             = (1 << 6),        /**< overheating */
    MFRC522_ERROR_BUFFER_OVER_FLOW = (1 << 4),        /**< buffer is full */
    MFRC522_ERROR_COLL             = (1 << 3),        /**< a bit collision is detected */
    MFRC522_ERROR_CRC              = (1 << 2),        /**< crc calculation fails */
    MFRC522_ERROR_PARITY           = (1 << 1),        /**< parity check failed */
    MFRC522_ERROR_PROTOCOL         = (1 << 0),        /**< sof is incorrect */
} mfrc522_error_t;

/**
 * @brief mfrc522 status1 enumeration definition
 */
typedef enum
{
    MFRC522_STATUS1_CRC_OK     = (1 << 6),        /**< crc is ok */
    MFRC522_STATUS1_CRC_READY  = (1 << 5),        /**< the crc calculation has finished */
    MFRC522_STATUS1_IRQ        = (1 << 4),        /**< irq */
    MFRC522_STATUS1_TRUNNING   = (1 << 3),        /**< timer unit is running */
    MFRC522_STATUS1_HI_ALERT   = (1 << 1),        /**< hi alert = (64 – fifo length) <= water level */
    MFRC522_STATUS1_LO_ALERT   = (1 << 0),        /**< lo alert = fifo length <= water level */
} mfrc522_status1_t;

/**
 * @brief mfrc522 status2 enumeration definition
 */
typedef enum
{
    MFRC522_STATUS2_MFCRYPTO1_ON = (1 << 3),    /**< indicates that the mifare crypto1 unit is switched on */
} mfrc522_status2_t;

/**
 * @brief mfrc522 modem state enumeration definition
 */
typedef enum
{
    MFRC522_MODEM_STATE_IDLE             = 0x00,    /**< idle */
    MFRC522_MODEM_STATE_WAIT_BIT_FRAMING = 0x01,    /**< wait for the start send bit */
    MFRC522_MODEM_STATE_TX_WAIT          = 0x02,    /**< wait until rf field is present */
    MFRC522_MODEM_STATE_TRANSMITTING     = 0x03,    /**< transmitting */
    MFRC522_MODEM_STATE_RX_WAIT          = 0x04,    /**< wait until rf field is present */
    MFRC522_MODEM_STATE_WAIT_DATA        = 0x05,    /**< wait for data */
    MFRC522_MODEM_STATE_RECEIVING        = 0x06,    /**< receiving */
} mfrc522_modem_state_t;

/**
 * @brief mfrc522 rx align enumeration definition
 */
typedef enum
{
    MFRC522_RX_ALIGN_0 = 0,    /**< lsb of the received bit is stored at bit position 0, the second
                                    received bit is stored at bit position 1 */
    MFRC522_RX_ALIGN_1 = 1,    /**< lsb of the received bit is stored at bit position 1, the second
                                    received bit is stored at bit position 2 */
    MFRC522_RX_ALIGN_7 = 7,    /**< lsb of the received bit is stored at bit position 7, the second
                                    received bit is stored in the next byte that follows at bit
                                    position 0 */
} mfrc522_rx_align_t;

/**
 * @brief mfrc522 mfin polarity enumeration definition
 */
typedef enum
{
    MFRC522_MFIN_POLARITY_LOW  = 0,        /**< polarity of pin mfin is active low */
    MFRC522_MFIN_POLARITY_HIGH = 1,        /**< polarity of pin mfin is active high */
} mfrc522_mfin_polarity_t;

/**
 * @brief mfrc522 crc preset enumeration definition
 */
typedef enum
{
    MFRC522_CRC_PRESET_0000 = 0,        /**< 0000h */
    MFRC522_CRC_PRESET_6363 = 1,        /**< 6363h */
    MFRC522_CRC_PRESET_A671 = 2,        /**< A671h */
    MFRC522_CRC_PRESET_FFFF = 3,        /**< FFFFh */
} mfrc522_crc_preset_t;

/**
 * @brief mfrc522 speed enumeration definition
 */
typedef enum
{
    MFRC522_SPEED_106_KBD = 0,        /**< 106 kBd */
    MFRC522_SPEED_212_KBD = 1,        /**< 212 kBd */
    MFRC522_SPEED_424_KBD = 2,        /**< 424 kBd */
    MFRC522_SPEED_848_KBD = 3,        /**< 848 kBd */
} mfrc522_speed_t;

/**
 * @brief mfrc522 antenna driver enumeration definition
 */
typedef enum
{
    MFRC522_ANTENNA_DRIVER_INV_TX2_RF_ON  = 7,        /**< output signal on pin tx2 inverted when driver tx2 is enabled */
    MFRC522_ANTENNA_DRIVER_INV_TX1_RF_ON  = 6,        /**< output signal on pin tx1 inverted when driver tx1 is enabled */
    MFRC522_ANTENNA_DRIVER_INV_TX2_RF_OFF = 5,        /**< output signal on pin tx2 inverted when driver tx2 is disabled */
    MFRC522_ANTENNA_DRIVER_INV_TX1_RF_OFF = 4,        /**< output signal on pin tx1 inverted when driver tx1 is disabled */
    MFRC522_ANTENNA_DRIVER_TX2_CW         = 3,        /**< output signal on pin tx2 continuously delivers the unmodulated 13.56 MHz energy carrier
                                                           tx2cw bit is enabled to modulate the 13.56 MHz energy carrier */
    MFRC522_ANTENNA_DRIVER_TX2_RF         = 1,        /**< output signal on pin tx2 delivers the 13.56 MHz energy carrier modulated by 
                                                           the transmission data */
    MFRC522_ANTENNA_DRIVER_TX1_RF         = 0,        /**< output signal on pin tx1 delivers the 13.56 MHz energy carrier modulated by
                                                           the transmission data */
} mfrc522_antenna_driver_t;

/**
 * @brief mfrc522 tx input enumeration definition
 */
typedef enum
{
    MFRC522_TX_INPUT_3_STATE          = 0,        /**< 3 state, in soft power-down the drivers are only in 3-state
                                                       mode if the DriverSel[1:0] value is set to 3-state mode */
    MFRC522_TX_INPUT_INTERNAL_ENCODER = 1,        /**< modulation signal (envelope) from the internal encoder, miller
                                                       pulse encoded */
    MFRC522_TX_INPUT_MFIN_PIN         = 2,        /**< modulation signal (envelope) from pin mfin */
    MFRC522_TX_INPUT_CONTROL          = 3,        /**< HIGH, the HIGH level depends on the setting of bits
                                                       InvTx1RFOn/InvTx1RFOff and InvTx2RFOn/InvTx2RFOff */
} mfrc522_tx_input_t;

/**
 * @brief mfrc522 mfout input enumeration definition
 */
typedef enum
{
    MFRC522_MFOUT_INPUT_3_STATE          = 0,        /**< 3 state */
    MFRC522_MFOUT_INPUT_LOW              = 1,        /**< low */
    MFRC522_MFOUT_INPUT_HIGH             = 2,        /**< high */
    MFRC522_MFOUT_INPUT_TEST             = 3,        /**< test bus signal as defined by the testsel1reg register’s tstbusbitsel[2:0] value */
    MFRC522_MFOUT_INPUT_INTERNAL_ENCODER = 4,        /**< modulation signal (envelope) from the internal encoder, miller pulse encoded */
    MFRC522_MFOUT_INPUT_TRANSMITTED      = 5,        /**< serial data stream to be transmitted, data stream before miller encoder */
    MFRC522_MFOUT_INPUT_RECEIVED         = 7,        /**< serial data stream received, data stream after manchester decoder */
} mfrc522_mfout_input_t;

/**
 * @brief mfrc522 contactless uart input enumeration definition
 */
typedef enum
{
    MFRC522_CONTACTLESS_UART_INPUT_CONSTANT_LOW     = 0,        /**< constant low */
    MFRC522_CONTACTLESS_UART_MFIN_PIN               = 1,        /**< manchester with subcarrier from pin mfin */
    MFRC522_CONTACTLESS_UART_INTERNAL_ANALOG_MODULE = 2,        /**< modulated signal from the internal analog module */
    MFRC522_CONTACTLESS_UART_NRZ                    = 3,        /**< NRZ coding without subcarrier from pin MFIN which is only valid
                                                                     for transfer speeds above 106 kBd */
} mfrc522_contactless_uart_input_t;

/**
 * @brief mfrc522 channel reception enumeration definition
 */
typedef enum
{
    MFRC522_CHANNEL_RECEPTION_STRONGER = 0,                        /**< selects the stronger channel */
    MFRC522_CHANNEL_RECEPTION_STRONGER_FREEZE_SELECTED = 1,        /**< selects the stronger channel and 
                                                                        freezes the selected channel during communication */
} mfrc522_channel_reception_t;

/**
 * @brief mfrc522 rx gain enumeration definition
 */
typedef enum
{
    MFRC522_RX_GAIN_18_DB = 2,        /**< 18 dB */
    MFRC522_RX_GAIN_23_DB = 3,        /**< 23 dB */
    MFRC522_RX_GAIN_33_DB = 4,        /**< 33 dB */
    MFRC522_RX_GAIN_38_DB = 5,        /**< 38 dB */
    MFRC522_RX_GAIN_43_DB = 6,        /**< 43 dB */
    MFRC522_RX_GAIN_48_DB = 7,        /**< 48 dB */
} mfrc522_rx_gain_t;

/**
 * @brief mfrc522 timer gated mode enumeration definition
 */
typedef enum
{
    MFRC522_TIMER_GATED_MODE_NONE = 0,        /**< non gated mode */
    MFRC522_TIMER_GATED_MODE_MFIN = 1,        /**< gated by pin MFIN */
    MFRC522_TIMER_GATED_MODE_AUX1 = 2,        /**< gated by pin AUX1 */
} mfrc522_timer_gated_mode_t;

/**
 * @brief mfrc522 test analog control enumeration definition
 */
typedef enum
{
    MFRC522_TEST_ANALOG_CONTROL_3_STATE                   = 0x0,        /**< 3 state */
    MFRC522_TEST_ANALOG_CONTROL_OUTPUT                    = 0x1,        /**< output of test dac1, output of test dac2 */
    MFRC522_TEST_ANALOG_CONTROL_TEST_SIGNAL_CORR1         = 0x2,        /**< test signal corr1 */
    MFRC522_TEST_ANALOG_CONTROL_DAC_TEST_SIGNAL_MIN_LEVEL = 0x4,        /**< dac test signal min level */
    MFRC522_TEST_ANALOG_CONTROL_DAC_TEST_SIGNAL_ADC_I     = 0x5,        /**< dac test signal adc i */
    MFRC522_TEST_ANALOG_CONTROL_DAC_TEST_SIGNAL_ADC_Q     = 0x6,        /**< dac test signal adc q */
    MFRC522_TEST_ANALOG_CONTROL_SIGNAL_FOR_PRODUCTION     = 0x8,        /**< test signal for production */
    MFRC522_TEST_ANALOG_CONTROL_HIGH                      = 0xA,        /**< high */
    MFRC522_TEST_ANALOG_CONTROL_LOW                       = 0xB,        /**< low */
    MFRC522_TEST_ANALOG_CONTROL_TX_ACTIVE                 = 0xC,        /**< tx active */
    MFRC522_TEST_ANALOG_CONTROL_RX_ACTIVE                 = 0xD,        /**< rx active */
    MFRC522_TEST_ANALOG_CONTROL_SUBCARRIER_DETECTED       = 0xE,        /**< subcarrier detected */
    MFRC522_TEST_ANALOG_CONTROL_DEFINED_BIT               = 0xF,        /**< test bus bit as defined by the testsel1reg register’s
                                                                             tstbusbitsel[2:0] bits */
} mfrc522_test_analog_control_t;

/**
 * @brief mfrc522 handle structure definition
 */
typedef struct mfrc522_handle_s
{
    uint8_t (*reset_gpio_init)(void);                                                   /**< point to a reset_gpio_init function address */
    uint8_t (*reset_gpio_deinit)(void);                                                 /**< point to a reset_gpio_deinit function address */
    uint8_t (*reset_gpio_write)(uint8_t data);                                          /**< point to a reset_gpio_write function address */
    uint8_t (*iic_init)(void);                                                          /**< point to a iic_init function address */
    uint8_t (*iic_deinit)(void);                                                        /**< point to a iic_deinit function address */
    uint8_t (*iic_write)(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);        /**< point to a iic_write function address */
    uint8_t (*iic_read)(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);         /**< point to a iic_read function address */
    uint8_t (*uart_init)(void);                                                         /**< point to a uart_init function address */
    uint8_t (*uart_deinit)(void);                                                       /**< point to a uart_deinit function address */
    uint16_t (*uart_read)(uint8_t *buf, uint16_t len);                                  /**< point to a uart_read function address */
    uint8_t (*uart_flush)(void);                                                        /**< point to a uart_flush function address */
    uint8_t (*uart_write)(uint8_t *buf, uint16_t len);                                  /**< point to a uart_write function address */
    uint8_t (*spi_init)(void);                                                          /**< point to a spi_init function address */
    uint8_t (*spi_deinit)(void);                                                        /**< point to a spi_deinit function address */
    uint8_t (*spi_read)(uint8_t reg, uint8_t *buf, uint16_t len);                       /**< point to a spi_read function address */
    uint8_t (*spi_write)(uint8_t reg, uint8_t *buf, uint16_t len);                      /**< point to a spi_write function address */
    void (*receive_callback)(uint16_t type);                                            /**< point to a receive_callback function address */
    void (*delay_ms)(uint32_t ms);                                                      /**< point to a delay_ms function address */
    void (*debug_print)(const char *const fmt, ...);                                    /**< point to a debug_print function address */
    uint8_t inited;                                                                     /**< inited flag */
    uint8_t iic_addr;                                                                   /**< iic address */
    uint8_t iic_spi_uart;                                                               /**< iic spi uart */
    uint16_t irq_flag;                                                                  /**< set the irq flag */
} mfrc522_handle_t;

/**
 * @brief mfrc522 information structure definition
 */
typedef struct mfrc522_info_s
{
    char chip_name[32];                /**< chip name */
    char manufacturer_name[32];        /**< manufacturer name */
    char interface[32];                /**< chip interface name */
    float supply_voltage_min_v;        /**< chip min supply voltage */
    float supply_voltage_max_v;        /**< chip max supply voltage */
    float max_current_ma;              /**< chip max current */
    float temperature_min;             /**< chip min operating temperature */
    float temperature_max;             /**< chip max operating temperature */
    uint32_t driver_version;           /**< driver version */
} mfrc522_info_t;

/**
 * @}
 */

/**
 * @defgroup mfrc522_link_driver mfrc522 link driver function
 * @brief    mfrc522 link driver modules
 * @ingroup  mfrc522_driver
 * @{
 */

/**
 * @brief     initialize mfrc522_handle_t structure
 * @param[in] HANDLE points to a mfrc522 handle structure
 * @param[in] STRUCTURE is mfrc522_handle_t
 * @note      none
 */
#define DRIVER_MFRC522_LINK_INIT(HANDLE, STRUCTURE)         memset(HANDLE, 0, sizeof(STRUCTURE))

/**
 * @brief     link reset_gpio_init function
 * @param[in] HANDLE points to a mfrc522 handle structure
 * @param[in] FUC points to a reset_gpio_init function address
 * @note      none
 */
#define DRIVER_MFRC522_LINK_RESET_GPIO_INIT(HANDLE, FUC)    (HANDLE)->reset_gpio_init = FUC

/**
 * @brief     link reset_gpio_deinit function
 * @param[in] HANDLE points to a mfrc522 handle structure
 * @param[in] FUC points to a reset_gpio_deinit function address
 * @note      none
 */
#define DRIVER_MFRC522_LINK_RESET_GPIO_DEINIT(HANDLE, FUC)  (HANDLE)->reset_gpio_deinit = FUC

/**
 * @brief     link reset_gpio_write function
 * @param[in] HANDLE points to a mfrc522 handle structure
 * @param[in] FUC points to a reset_gpio_write function address
 * @note      none
 */
#define DRIVER_MFRC522_LINK_RESET_GPIO_WRITE(HANDLE, FUC)   (HANDLE)->reset_gpio_write = FUC

/**
 * @brief     link iic_init function
 * @param[in] HANDLE points to a mfrc522 handle structure
 * @param[in] FUC points to a iic_init function address
 * @note      none
 */
#define DRIVER_MFRC522_LINK_IIC_INIT(HANDLE, FUC)           (HANDLE)->iic_init = FUC

/**
 * @brief     link iic_deinit function
 * @param[in] HANDLE points to a mfrc522 handle structure
 * @param[in] FUC points to a iic_deinit function address
 * @note      none
 */
#define DRIVER_MFRC522_LINK_IIC_DEINIT(HANDLE, FUC)         (HANDLE)->iic_deinit = FUC

/**
 * @brief     link iic_write function
 * @param[in] HANDLE points to a mfrc522 handle structure
 * @param[in] FUC points to a iic_write function address
 * @note      none
 */
#define DRIVER_MFRC522_LINK_IIC_WRITE(HANDLE, FUC)          (HANDLE)->iic_write = FUC

/**
 * @brief     link iic_read function
 * @param[in] HANDLE points to a mfrc522 handle structure
 * @param[in] FUC points to a iic_read function address
 * @note      none
 */
#define DRIVER_MFRC522_LINK_IIC_READ(HANDLE, FUC)           (HANDLE)->iic_read = FUC

/**
 * @brief     link uart_init function
 * @param[in] HANDLE points to a mfrc522 handle structure
 * @param[in] FUC points to a uart_init function address
 * @note      none
 */
#define DRIVER_MFRC522_LINK_UART_INIT(HANDLE, FUC)          (HANDLE)->uart_init = FUC

/**
 * @brief     link uart_deinit function
 * @param[in] HANDLE points to a mfrc522 handle structure
 * @param[in] FUC points to a uart_deinit function address
 * @note      none
 */
#define DRIVER_MFRC522_LINK_UART_DEINIT(HANDLE, FUC)        (HANDLE)->uart_deinit = FUC

/**
 * @brief     link uart_read function
 * @param[in] HANDLE points to a mfrc522 handle structure
 * @param[in] FUC points to a uart_read function address
 * @note      none
 */
#define DRIVER_MFRC522_LINK_UART_READ(HANDLE, FUC)          (HANDLE)->uart_read = FUC

/**
 * @brief     link uart_write function
 * @param[in] HANDLE points to a mfrc522 handle structure
 * @param[in] FUC points to a uart_write function address
 * @note      none
 */
#define DRIVER_MFRC522_LINK_UART_WRITE(HANDLE, FUC)         (HANDLE)->uart_write = FUC

/**
 * @brief     link uart_flush function
 * @param[in] HANDLE points to a mfrc522 handle structure
 * @param[in] FUC points to a uart_flush function address
 * @note      none
 */
#define DRIVER_MFRC522_LINK_UART_FLUSH(HANDLE, FUC)         (HANDLE)->uart_flush = FUC

/**
 * @brief     link spi_init function
 * @param[in] HANDLE points to a mfrc522 handle structure
 * @param[in] FUC points to a spi_init function address
 * @note      none
 */
#define DRIVER_MFRC522_LINK_SPI_INIT(HANDLE, FUC)           (HANDLE)->spi_init = FUC

/**
 * @brief     link spi_deinit function
 * @param[in] HANDLE points to a mfrc522 handle structure
 * @param[in] FUC points to a spi_deinit function address
 * @note      none
 */
#define DRIVER_MFRC522_LINK_SPI_DEINIT(HANDLE, FUC)         (HANDLE)->spi_deinit = FUC

/**
 * @brief     link spi_read function
 * @param[in] HANDLE points to a mfrc522 handle structure
 * @param[in] FUC points to a spi_read function address
 * @note      none
 */
#define DRIVER_MFRC522_LINK_SPI_READ(HANDLE, FUC)           (HANDLE)->spi_read = FUC

/**
 * @brief     link spi_write function
 * @param[in] HANDLE points to a mfrc522 handle structure
 * @param[in] FUC points to a spi_write function address
 * @note      none
 */
#define DRIVER_MFRC522_LINK_SPI_WRITE(HANDLE, FUC)          (HANDLE)->spi_write = FUC

/**
 * @brief     link delay_ms function
 * @param[in] HANDLE points to a mfrc522 handle structure
 * @param[in] FUC points to a delay_ms function address
 * @note      none
 */
#define DRIVER_MFRC522_LINK_DELAY_MS(HANDLE, FUC)           (HANDLE)->delay_ms = FUC

/**
 * @brief     link debug_print function
 * @param[in] HANDLE points to a mfrc522 handle structure
 * @param[in] FUC points to a debug_print function address
 * @note      none
 */
#define DRIVER_MFRC522_LINK_DEBUG_PRINT(HANDLE, FUC)        (HANDLE)->debug_print = FUC

/**
 * @brief     link receive_callback function
 * @param[in] HANDLE points to a mfrc522 handle structure
 * @param[in] FUC points to a receive_callback function address
 * @note      none
 */
#define DRIVER_MFRC522_LINK_RECEIVE_CALLBACK(HANDLE, FUC)   (HANDLE)->receive_callback = FUC

/**
 * @}
 */

/**
 * @defgroup mfrc522_basic_driver mfrc522 basic driver function
 * @brief    mfrc522 basic driver modules
 * @ingroup  mfrc522_driver
 * @{
 */

/**
 * @brief      get chip information
 * @param[out] *info points to a mfrc522 info structure
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 * @note       none
 */
uint8_t mfrc522_info(mfrc522_info_t *info);

/**
 * @brief     set the chip interface
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] interface is the chip interface
 * @return    status code
 *            - 0 success
 *            - 2 handle is NULL
 * @note      none
 */
uint8_t mfrc522_set_interface(mfrc522_handle_t *handle, mfrc522_interface_t interface);

/**
 * @brief      get the chip interface
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *interface points to a chip interface buffer
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 * @note       none
 */
uint8_t mfrc522_get_interface(mfrc522_handle_t *handle, mfrc522_interface_t *interface);

/**
 * @brief     set the iic address pin
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] addr_pin is the address pin
 * @return    status code
 *            - 0 success
 *            - 2 handle is NULL
 * @note      none
 */
uint8_t mfrc522_set_addr_pin(mfrc522_handle_t *handle, uint8_t addr_pin);

/**
 * @brief      get the iic address pin
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *addr_pin points to a address pin buffer
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 * @note       none
 */
uint8_t mfrc522_get_addr_pin(mfrc522_handle_t *handle, uint8_t *addr_pin);

/**
 * @brief     irq handler
 * @param[in] *handle points to a mfrc522 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 run failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_irq_handler(mfrc522_handle_t *handle);

/**
 * @brief     initialize the chip
 * @param[in] *handle points to a mfrc522 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 iic, spi or uart initialization failed
 *            - 2 handle is NULL
 *            - 3 linked functions is NULL
 *            - 4 interface is invalid
 *            - 5 get id failed
 *            - 6 check id failed
 * @note      none
 */
uint8_t mfrc522_init(mfrc522_handle_t *handle);

/**
 * @brief     close the chip
 * @param[in] *handle points to a mfrc522 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 iic, spi or uart deinit failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 interface is invalid
 *            - 5 power down failed
 * @note      none
 */
uint8_t mfrc522_deinit(mfrc522_handle_t *handle);

/**
 * @brief         mfrc522 transceiver
 * @param[in]     *handle points to a mfrc522 handle structure
 * @param[in]     command is the set command
 * @param[in]     *in_buf points to a input buffer
 * @param[in]     in_len is the input length
 * @param[out]    *out_buf points to a output buffer
 * @param[in,out] *out_len points to a output length buffer
 * @param[out]    *err points to an error buffer
 * @param[in]     ms is the timeout in ms
 * @return        status code
 *                - 0 success
 *                - 1 transmit failed
 *                - 2 handle is NULL
 *                - 3 linked functions is NULL
 *                - 4 buffer is NULL
 *                - 5 in_len is over 64
 *                - 6 read timeout
 *                - 7 timer timeout
 *                - 8 find error
 * @note          none
 */
uint8_t mfrc522_transceiver(mfrc522_handle_t *handle,
                            mfrc522_command_t command,
                            uint8_t *in_buf, uint8_t in_len,
                            uint8_t *out_buf, uint8_t *out_len,
                            uint8_t *err, uint32_t ms);

/**
 * @brief     enable or disable the analog part of the receiver
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set receiver analog failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_receiver_analog(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the analog part of the receiver status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get receiver analog failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_receiver_analog(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief     enable or disable power down
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set power down failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_power_down(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get power down status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get power down failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_power_down(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief     set the command
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] command is the set command
 * @return    status code
 *            - 0 success
 *            - 1 set command failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_command(mfrc522_handle_t *handle, mfrc522_command_t command);

/**
 * @brief      get the command
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *command points to a set command buffer
 * @return     status code
 *             - 0 success
 *             - 1 get command failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_command(mfrc522_handle_t *handle, mfrc522_command_t *command);

/**
 * @brief     enable or disable the interrupt1
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] type is the interrupt1 type
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set interrupt1 failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_interrupt1(mfrc522_handle_t *handle, mfrc522_interrupt1_t type, mfrc522_bool_t enable);

/**
 * @brief      get the interrupt1 status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[in]  type is the interrupt1 type
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get interrupt1 failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_interrupt1(mfrc522_handle_t *handle, mfrc522_interrupt1_t type, mfrc522_bool_t *enable);

/**
 * @brief     enable or disable interrupt1 pin invert
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set interrupt1 pin invert failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_interrupt1_pin_invert(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the interrupt1 pin invert status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get interrupt1 pin invert failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_interrupt1_pin_invert(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief     set the interrupt1 mark
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] mark is the interrupt1 mark type
 * @return    status code
 *            - 0 success
 *            - 1 set interrupt1 mark failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_interrupt1_mark(mfrc522_handle_t *handle, mfrc522_interrupt_mark_t mark);

/**
 * @brief     enable or disable the interrupt2
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] type is the interrupt2 type
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set interrupt2 failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_interrupt2(mfrc522_handle_t *handle, mfrc522_interrupt2_t type, mfrc522_bool_t enable);

/**
 * @brief      get the interrupt2 status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[in]  type is the interrupt2 type
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get interrupt2 failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_interrupt2(mfrc522_handle_t *handle, mfrc522_interrupt2_t type, mfrc522_bool_t *enable);

/**
 * @brief     set the interrupt pin type
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] type is the interrupt pin type
 * @return    status code
 *            - 0 success
 *            - 1 set interrupt pin type failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_interrupt_pin_type(mfrc522_handle_t *handle, mfrc522_interrupt_pin_type_t type);

/**
 * @brief      get the interrupt pin type
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *type points to an interrupt pin type buffer
 * @return     status code
 *             - 0 success
 *             - 1 get interrupt pin type failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_interrupt_pin_type(mfrc522_handle_t *handle, mfrc522_interrupt_pin_type_t *type);

/**
 * @brief     set the interrupt2 mark
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] mark is the interrupt2 mark type
 * @return    status code
 *            - 0 success
 *            - 1 set interrupt2 mark failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_interrupt2_mark(mfrc522_handle_t *handle, mfrc522_interrupt_mark_t mark);

/**
 * @brief      get the interrupt1 status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *status points to a status buffer
 * @return     status code
 *             - 0 success
 *             - 1 get interrupt1 status failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_interrupt1_status(mfrc522_handle_t *handle, uint8_t *status);

/**
 * @brief      get the interrupt2 status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *status points to a status buffer
 * @return     status code
 *             - 0 success
 *             - 1 get interrupt2 status failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_interrupt2_status(mfrc522_handle_t *handle, uint8_t *status);

/**
 * @brief      get the error
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *err points to an error buffer
 * @return     status code
 *             - 0 success
 *             - 1 get error failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_error(mfrc522_handle_t *handle, uint8_t *err);

/**
 * @brief      get the status1
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *status points to a status buffer
 * @return     status code
 *             - 0 success
 *             - 1 get status1 failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_status1(mfrc522_handle_t *handle, uint8_t *status);

/**
 * @brief      get the status2
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *status points to a status buffer
 * @return     status code
 *             - 0 success
 *             - 1 get status2 failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_status2(mfrc522_handle_t *handle, uint8_t *status);

/**
 * @brief      get the modem state
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *state points to a modem state buffer
 * @return     status code
 *             - 0 success
 *             - 1 get modem state failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_modem_state(mfrc522_handle_t *handle, mfrc522_modem_state_t *state);

/**
 * @brief     enable or disable mifare crypto1 on
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set mifare crypto1 on failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_mifare_crypto1_on(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief     enable or disable force iic high speed
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set force iic high speed failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_force_iic_high_speed(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the iic high speed status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get force iic high speed failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_force_iic_high_speed(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief     enable or disable clear temperature error
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set clear temperature error failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_clear_temperature_error(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the clear temperature error status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get clear temperature error failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_clear_temperature_error(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief     set the fifo data
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] *data points to a data buffer
 * @param[in] len is the data length
 * @return    status code
 *            - 0 success
 *            - 1 set fifo data failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 len is over 64
 * @note      len <= 64
 */
uint8_t mfrc522_set_fifo_data(mfrc522_handle_t *handle, uint8_t *data, uint8_t len);

/**
 * @brief      get the fifo data
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *data points to a data buffer
 * @param[in]  len is the data length
 * @return     status code
 *             - 0 success
 *             - 1 get fifo data failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 *             - 4 len is over 64
 * @note       len <= 64
 */
uint8_t mfrc522_get_fifo_data(mfrc522_handle_t *handle, uint8_t *data, uint8_t len);

/**
 * @brief      get the fifo level
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *level points to a level buffer
 * @return     status code
 *             - 0 success
 *             - 1 get fifo level failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_fifo_level(mfrc522_handle_t *handle, uint8_t *level);

/**
 * @brief      flush the fifo
 * @param[in]  *handle points to a mfrc522 handle structure
 * @return     status code
 *             - 0 success
 *             - 1 flush fifo failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_flush_fifo(mfrc522_handle_t *handle);

/**
 * @brief     set the water level
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] level is the water level
 * @return    status code
 *            - 0 success
 *            - 1 set water level failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 level is over 0x3F
 * @note      level <= 0x3F
 */
uint8_t mfrc522_set_water_level(mfrc522_handle_t *handle, uint8_t level);

/**
 * @brief      get the water level
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *level points to a level buffer
 * @return     status code
 *             - 0 success
 *             - 1 get water level failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_water_level(mfrc522_handle_t *handle, uint8_t *level);

/**
 * @brief     stop the timer
 * @param[in] *handle points to a mfrc522 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 stop timer failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_stop_timer(mfrc522_handle_t *handle);

/**
 * @brief     start the timer
 * @param[in] *handle points to a mfrc522 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 start timer failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_start_timer(mfrc522_handle_t *handle);

/**
 * @brief      get the rx last bits
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *bits points to a bits buffer
 * @return     status code
 *             - 0 success
 *             - 1 get rx last bits failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_rx_last_bits(mfrc522_handle_t *handle, uint8_t *bits);

/**
 * @brief     start the transmission of data
 * @param[in] *handle points to a mfrc522 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 start send failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_start_send(mfrc522_handle_t *handle);

/**
 * @brief     stop the transmission of data
 * @param[in] *handle points to a mfrc522 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 stop send failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_stop_send(mfrc522_handle_t *handle);

/**
 * @brief     set the tx last bits
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] bits is the set bits
 * @return    status code
 *            - 0 success
 *            - 1 set tx last bits failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 bits is over 7
 * @note      none
 */
uint8_t mfrc522_set_tx_last_bits(mfrc522_handle_t *handle, uint8_t bits);

/**
 * @brief      get the tx last bits
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *bits points to a bits buffer
 * @return     status code
 *             - 0 success
 *             - 1 get tx last bits failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_tx_last_bits(mfrc522_handle_t *handle, uint8_t *bits);

/**
 * @brief     set the rx align
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] align is the rx align
 * @return    status code
 *            - 0 success
 *            - 1 set rx align failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_rx_align(mfrc522_handle_t *handle, mfrc522_rx_align_t align);

/**
 * @brief      get the rx align
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *align points to a rx align buffer
 * @return     status code
 *             - 0 success
 *             - 1 get rx align failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_rx_align(mfrc522_handle_t *handle, mfrc522_rx_align_t *align);

/**
 * @brief     enable or disable value clear after coll
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set value clear after coll failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_value_clear_after_coll(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the value clear after coll status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get value clear after coll failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_value_clear_after_coll(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief      get the collision position not valid bit status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get collision position not valid failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_collision_position_not_valid(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief      get the collision position
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *pos points to a position buffer
 * @return     status code
 *             - 0 success
 *             - 1 get collision position failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_collision_position(mfrc522_handle_t *handle, uint8_t *pos);

/**
 * @brief     enable or disable the crc msb first
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set crc msb first failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_crc_msb_first(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the crc msb first
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get crc msb first failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_crc_msb_first(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief     enable or disable the rf tx wait
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set tx wait rf failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_tx_wait_rf(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the rf tx wait status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get tx wait rf failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_tx_wait_rf(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief     set the mfin polarity
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] polarity is the mfin polarity
 * @return    status code
 *            - 0 success
 *            - 1 set mfin polarity failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_mfin_polarity(mfrc522_handle_t *handle, mfrc522_mfin_polarity_t polarity);

/**
 * @brief      get the mfin polarity
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *polarity points to a mfin polarity buffer
 * @return     status code
 *             - 0 success
 *             - 1 get mfin polarity failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_mfin_polarity(mfrc522_handle_t *handle, mfrc522_mfin_polarity_t *polarity);

/**
 * @brief     set the crc preset
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] preset is the crc preset
 * @return    status code
 *            - 0 success
 *            - 1 set crc preset failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_crc_preset(mfrc522_handle_t *handle, mfrc522_crc_preset_t preset);

/**
 * @brief      get the crc preset
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *preset points to a crc preset buffer
 * @return     status code
 *             - 0 success
 *             - 1 get crc preset failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_crc_preset(mfrc522_handle_t *handle, mfrc522_crc_preset_t *preset);

/**
 * @brief     enable or disable tx crc generation
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set tx crc generation failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_tx_crc_generation(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the tx crc generation status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get tx crc generation failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_tx_crc_generation(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief     set the tx speed
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] speed is the tx speed
 * @return    status code
 *            - 0 success
 *            - 1 set tx speed failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_tx_speed(mfrc522_handle_t *handle, mfrc522_speed_t speed);

/**
 * @brief      get the tx speed
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *speed points to a tx speed buffer
 * @return     status code
 *             - 0 success
 *             - 1 get tx speed failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_tx_speed(mfrc522_handle_t *handle, mfrc522_speed_t *speed);

/**
 * @brief     enable or disable the modulation invert
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set modulation invert failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_modulation_invert(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the modulation invert status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get modulation invert failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_modulation_invert(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief     enable or disable the rx crc generation
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set rx crc generation failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_rx_crc_generation(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the rx crc generation status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get rx crc generation failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_rx_crc_generation(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief     set the rx speed
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] speed is the rx speed
 * @return    status code
 *            - 0 success
 *            - 1 set rx speed failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_rx_speed(mfrc522_handle_t *handle, mfrc522_speed_t speed);

/**
 * @brief      get the rx speed
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *speed points to a rx speed buffer
 * @return     status code
 *             - 0 success
 *             - 1 get rx speed failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_rx_speed(mfrc522_handle_t *handle, mfrc522_speed_t *speed);

/**
 * @brief     enable or disable rx no error
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set rx no error failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_rx_no_error(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the rx no error status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get rx no error failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_rx_no_error(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief     enable or disable rx multiple
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set rx multiple failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_rx_multiple(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the rx multiple status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get rx multiple failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_rx_multiple(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief     enable or disable the antenna driver
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] driver is the antenna driver type
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set antenna driver failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_antenna_driver(mfrc522_handle_t *handle, mfrc522_antenna_driver_t driver, mfrc522_bool_t enable);

/**
 * @brief      get the antenna driver status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[in]  driver is the antenna driver type
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get antenna driver failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_antenna_driver(mfrc522_handle_t *handle, mfrc522_antenna_driver_t driver, mfrc522_bool_t *enable);

/**
 * @brief     enable or disable force 100 ask
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set force 100 ask failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_force_100_ask(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the force 100 ask status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get force 100 ask failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_force_100_ask(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief     set the tx input
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] input is the tx input
 * @return    status code
 *            - 0 success
 *            - 1 set tx input failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_tx_input(mfrc522_handle_t *handle, mfrc522_tx_input_t input);

/**
 * @brief      get the tx input
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *input points to a tx input buffer
 * @return     status code
 *             - 0 success
 *             - 1 get tx input failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_tx_input(mfrc522_handle_t *handle, mfrc522_tx_input_t *input);

/**
 * @brief     set the mfout input
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] input is the mfout input
 * @return    status code
 *            - 0 success
 *            - 1 set mfout input failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_mfout_input(mfrc522_handle_t *handle, mfrc522_mfout_input_t input);

/**
 * @brief      get the mfout input
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *input points to a mfout input buffer
 * @return     status code
 *             - 0 success
 *             - 1 get mfout input failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_mfout_input(mfrc522_handle_t *handle, mfrc522_mfout_input_t *input);

/**
 * @brief     set the contactless uart input
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] input is the contactless uart input
 * @return    status code
 *            - 0 success
 *            - 1 set contactless uart input failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_contactless_uart_input(mfrc522_handle_t *handle, mfrc522_contactless_uart_input_t input);

/**
 * @brief      get the contactless uart input
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *input points to a contactless uart input buffer
 * @return     status code
 *             - 0 success
 *             - 1 get contactless uart input failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_contactless_uart_input(mfrc522_handle_t *handle, mfrc522_contactless_uart_input_t *input);

/**
 * @brief     set the rx wait
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] t is the rx wait
 * @return    status code
 *            - 0 success
 *            - 1 set rx wait failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 t is over 0x3F
 * @note      none
 */
uint8_t mfrc522_set_rx_wait(mfrc522_handle_t *handle, uint8_t t);

/**
 * @brief      get the rx wait
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *t points to a rx wait buffer
 * @return     status code
 *             - 0 success
 *             - 1 get rx wait failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_rx_wait(mfrc522_handle_t *handle, uint8_t *t);

/**
 * @brief     set the min level
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] level is the min level
 * @return    status code
 *            - 0 success
 *            - 1 set min level failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 level is over 0xF
 * @note      none
 */
uint8_t mfrc522_set_min_level(mfrc522_handle_t *handle, uint8_t level);

/**
 * @brief      get the min level
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *level points to a min level buffer
 * @return     status code
 *             - 0 success
 *             - 1 get min level failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_min_level(mfrc522_handle_t *handle, uint8_t *level);

/**
 * @brief     set the collision level
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] level is the collision level
 * @return    status code
 *            - 0 success
 *            - 1 set collision level failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 level is over 7
 * @note      none
 */
uint8_t mfrc522_set_collision_level(mfrc522_handle_t *handle, uint8_t level);

/**
 * @brief      get the collision level
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *level points to a collision level buffer
 * @return     status code
 *             - 0 success
 *             - 1 get collision level failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_collision_level(mfrc522_handle_t *handle, uint8_t *level);

/**
 * @brief     set the channel reception
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] reception is the channel reception
 * @return    status code
 *            - 0 success
 *            - 1 set channel reception failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_channel_reception(mfrc522_handle_t *handle, mfrc522_channel_reception_t reception);

/**
 * @brief      get the channel reception
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *reception points to a channel reception buffer
 * @return     status code
 *             - 0 success
 *             - 1 get channel reception failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_channel_reception(mfrc522_handle_t *handle, mfrc522_channel_reception_t *reception);

/**
 * @brief     enable or disable fix iq
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set fix iq failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_fix_iq(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the fix iq status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get fix iq failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_fix_iq(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief     enable or disable timer prescal even
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set timer prescal even failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_timer_prescal_even(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the timer prescal even status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get timer prescal even failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_timer_prescal_even(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief     set the timer constant reception
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] t is the reception
 * @return    status code
 *            - 0 success
 *            - 1 set timer constant reception failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 t is over 3
 * @note      none
 */
uint8_t mfrc522_set_timer_constant_reception(mfrc522_handle_t *handle, uint8_t t);

/**
 * @brief      get the timer constant reception
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *t points to a reception buffer
 * @return     status code
 *             - 0 success
 *             - 1 get timer constant reception failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_timer_constant_reception(mfrc522_handle_t *handle, uint8_t *t);

/**
 * @brief     set the timer constant sync
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] t is the sync
 * @return    status code
 *            - 0 success
 *            - 1 set timer constant sync failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 t is over 3
 * @note      none
 */
uint8_t mfrc522_set_timer_constant_sync(mfrc522_handle_t *handle, uint8_t t);

/**
 * @brief      get the timer constant sync
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *t points to a sync buffer
 * @return     status code
 *             - 0 success
 *             - 1 get timer constant sync failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_timer_constant_sync(mfrc522_handle_t *handle, uint8_t *t);

/**
 * @brief     set the tx wait
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] t is the wait
 * @return    status code
 *            - 0 success
 *            - 1 set tx wait failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 t is over 3
 * @note      none
 */
uint8_t mfrc522_set_tx_wait(mfrc522_handle_t *handle, uint8_t t);

/**
 * @brief      get the tx wait
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *t points to a wait buffer
 * @return     status code
 *             - 0 success
 *             - 1 get tx wait failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_tx_wait(mfrc522_handle_t *handle, uint8_t *t);

/**
 * @brief     enable or disable parity disable
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set parity disable failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_parity_disable(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the parity disable status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get parity disable failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_parity_disable(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief     set the serial speed
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] t0 is the speed parm0
 * @param[in] t1 is the speed parm1
 * @return    status code
 *            - 0 success
 *            - 1 set serial speed failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 t0 is over 0x7
 *            - 5 t1 is over 0x1F
 * @note       speed      t0        t1
 *            7200       0x07      0x14
 *            9600       0x07      0x0B
 *            14400      0x06      0x1A
 *            19200      0x06      0x0B
 *            38400      0x05      0x0B
 *            57600      0x04      0x1A
 *            115200     0x03      0x1A
 *            128000     0x03      0x14
 *            230400     0x02      0x1A
 *            460800     0x01      0x1A
 *            921600     0x00      0x1C
 *            1228800    0x00      0x15
 */
uint8_t mfrc522_set_serial_speed(mfrc522_handle_t *handle, uint8_t t0, uint8_t t1);

/**
 * @brief      get the serial speed
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *t0 points to a speed parm0 buffer
 * @param[out] *t1 points to a speed parm1 buffer
 * @return     status code
 *             - 0 success
 *             - 1 get serial speed failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_serial_speed(mfrc522_handle_t *handle, uint8_t *t0, uint8_t *t1);

/**
 * @brief      get the crc
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *crc points to a crc buffer
 * @return     status code
 *             - 0 success
 *             - 1 get crc failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_crc(mfrc522_handle_t *handle, uint16_t *crc);

/**
 * @brief     set the modulation width
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] width is the modulation width
 * @return    status code
 *            - 0 success
 *            - 1 set modulation width failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_modulation_width(mfrc522_handle_t *handle, uint8_t width);

/**
 * @brief      get the modulation width
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *width points to a modulation width buffer
 * @return     status code
 *             - 0 success
 *             - 1 get modulation width failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_modulation_width(mfrc522_handle_t *handle, uint8_t *width);

/**
 * @brief     set the rx gain
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] gain is the rx gain
 * @return    status code
 *            - 0 success
 *            - 1 set rx gain failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_rx_gain(mfrc522_handle_t *handle, mfrc522_rx_gain_t gain);

/**
 * @brief      get the rx gain
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *gain points to a rx gain buffer
 * @return     status code
 *             - 0 success
 *             - 1 get rx gain failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_rx_gain(mfrc522_handle_t *handle, mfrc522_rx_gain_t *gain);

/**
 * @brief     set the cwgsn
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] n is the param
 * @return    status code
 *            - 0 success
 *            - 1 set cwgsn failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 n is over 0xF
 * @note      none
 */
uint8_t mfrc522_set_cwgsn(mfrc522_handle_t *handle, uint8_t n);

/**
 * @brief      get the cwgsn
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *n points to a param buffer
 * @return     status code
 *             - 0 success
 *             - 1 get cwgsn failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_cwgsn(mfrc522_handle_t *handle, uint8_t *n);

/**
 * @brief     set the modgsn
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] n is the param
 * @return    status code
 *            - 0 success
 *            - 1 set modgsn failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 n is over 0xF
 * @note      none
 */
uint8_t mfrc522_set_modgsn(mfrc522_handle_t *handle, uint8_t n);

/**
 * @brief      get the modgsn
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *n points to a param buffer
 * @return     status code
 *             - 0 success
 *             - 1 get modgsn failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_modgsn(mfrc522_handle_t *handle, uint8_t *n);

/**
 * @brief     set the cwgsp
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] n is the param
 * @return    status code
 *            - 0 success
 *            - 1 set cwgsp failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 n is over 0x3F
 * @note      none
 */
uint8_t mfrc522_set_cwgsp(mfrc522_handle_t *handle, uint8_t n);

/**
 * @brief      get the cwgsp
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *n ponts to a param buffer
 * @return     status code
 *             - 0 success
 *             - 1 get cwgsp failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_get_cwgsp(mfrc522_handle_t *handle, uint8_t *n);

/**
 * @brief     set the modgsp
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] n is the param
 * @return    status code
 *            - 0 success
 *            - 1 set modgsp failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 n is over 0x3F
 * @note      none
 */
uint8_t mfrc522_set_modgsp(mfrc522_handle_t *handle, uint8_t n);

/**
 * @brief      get the modgsp
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *n points to a param buffer
 * @return     status code
 *             - 0 success
 *             - 1 get modgsp failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_modgsp(mfrc522_handle_t *handle, uint8_t *n);

/**
 * @brief     enable or disable timer auto
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set timer auto failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_timer_auto(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the timer auto status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get timer auto failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_timer_auto(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief     set the timer gated mode
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] mode is the timer gated mode
 * @return    status code
 *            - 0 success
 *            - 1 set timer gated mode failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_timer_gated_mode(mfrc522_handle_t *handle, mfrc522_timer_gated_mode_t mode);

/**
 * @brief      get the timer gated mode
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *mode points to a timer gated mode buffer
 * @return     status code
 *             - 0 success
 *             - 1 get timer gated mode failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_timer_gated_mode(mfrc522_handle_t *handle, mfrc522_timer_gated_mode_t *mode);

/**
 * @brief     enable or disable timer auto restart
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set timer auto restart failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_timer_auto_restart(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the timer auto restart status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get timer auto restart failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_timer_auto_restart(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief     set the timer prescaler
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] t is the prescaler
 * @return    status code
 *            - 0 success
 *            - 1 set timer prescaler failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 t is over 0xFFF
 * @note      none
 */
uint8_t mfrc522_set_timer_prescaler(mfrc522_handle_t *handle, uint16_t t);

/**
 * @brief      get the timer prescaler
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *t points to a prescaler buffer
 * @return     status code
 *             - 0 success
 *             - 1 get timer prescaler failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_timer_prescaler(mfrc522_handle_t *handle, uint16_t *t);

/**
 * @brief     set the timer reload
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] reload is the reload
 * @return    status code
 *            - 0 success
 *            - 1 set timer reload failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_timer_reload(mfrc522_handle_t *handle, uint16_t reload);

/**
 * @brief      get the timer reload
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *reload points to a reload buffer
 * @return     status code
 *             - 0 success
 *             - 1 get timer reload failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_timer_reload(mfrc522_handle_t *handle, uint16_t *reload);

/**
 * @brief      get the timer counter
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *cnt points to a counter buffer
 * @return     status code
 *             - 0 success
 *             - 1 get timer counter failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_timer_counter(mfrc522_handle_t *handle, uint16_t *cnt);

/**
 * @}
 */

/**
 * @defgroup mfrc522_chip_test_driver mfrc522 chip test driver function
 * @brief    mfrc522 chip test driver modules
 * @ingroup  mfrc522_driver
 * @{
 */

/**
 * @brief     set the test bus signal 1
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] s is the set signal
 * @return    status code
 *            - 0 success
 *            - 1 set test bus signal 1 failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 s is over 7
 * @note      none
 */
uint8_t mfrc522_set_test_bus_signal_1(mfrc522_handle_t *handle, uint8_t s);

/**
 * @brief      get the test bus signal 1
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *s points to a set signal buffer
 * @return     status code
 *             - 0 success
 *             - 1 get test bus signal 1 failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_test_bus_signal_1(mfrc522_handle_t *handle, uint8_t *s);

/**
 * @brief     set the test bus signal 2
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] s is the set signal
 * @return    status code
 *            - 0 success
 *            - 1 set test bus signal 2 failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 s is over 0x1F
 * @note      none
 */
uint8_t mfrc522_set_test_bus_signal_2(mfrc522_handle_t *handle, uint8_t s);

/**
 * @brief      get the test bus signal 2
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *s points to a set signal buffer
 * @return     status code
 *             - 0 success
 *             - 1 get test bus signal 2 failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_test_bus_signal_2(mfrc522_handle_t *handle, uint8_t *s);

/**
 * @brief     enable or disable test bus flip
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set test bus flip failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_test_bus_flip(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the test bus flip status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get test bus flip failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_test_bus_flip(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief     enable or disable test prbs9
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set test prbs9 failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_test_prbs9(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the test prbs9 status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get test prbs9 failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_test_prbs9(mfrc522_handle_t *handle, mfrc522_bool_t *enable);
 
/**
 * @brief     enable or disable test prbs15
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set test prbs15 failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_test_prbs15(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the test prbs15 status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get test prbs15 failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_test_prbs15(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief     enable or disable test rs232 line
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set test rs232 line failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_test_rs232_line(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the test rs232 line status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get test rs232 line failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_test_rs232_line(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief     set the test pin enable
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] pin is the set pin map
 * @return    status code
 *            - 0 success
 *            - 1 set test pin enable failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - pin is over 0x3F
 * @note      none
 */
uint8_t mfrc522_set_test_pin_enable(mfrc522_handle_t *handle, uint8_t pin);

/**
 * @brief      get the test pin enable
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *pin points to a pin map buffer
 * @return     status code
 *             - 0 success
 *             - 1 get test pin enable failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_get_test_pin_enable(mfrc522_handle_t *handle, uint8_t *pin);

/**
 * @brief     enable or disable test port io
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set test port io failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_test_port_io(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the test port io status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get test port io failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_test_port_io(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief     set the test pin value
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] value is the set value
 * @return    status code
 *            - 0 success
 *            - 1 set test pin enable failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 value is over 0x3F
 * @note      none
 */
uint8_t mfrc522_set_test_pin_value(mfrc522_handle_t *handle, uint8_t value);

/**
 * @brief      get the test pin value
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *value points to a set value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get test pin enable failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_test_pin_value(mfrc522_handle_t *handle, uint8_t *value);

/**
 * @brief      get the test bus
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *bus points to a bus buffer
 * @return     status code
 *             - 0 success
 *             - 1 get test bus failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_test_bus(mfrc522_handle_t *handle, uint8_t *bus);

/**
 * @brief     enable or disable test amp rcv
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] enable is a bool value
 * @return    status code
 *            - 0 success
 *            - 1 set test amp rcv failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_test_amp_rcv(mfrc522_handle_t *handle, mfrc522_bool_t enable);

/**
 * @brief      get the test amp rcv status
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *enable points to a bool value buffer
 * @return     status code
 *             - 0 success
 *             - 1 get test amp rcv failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_test_amp_rcv(mfrc522_handle_t *handle, mfrc522_bool_t *enable);

/**
 * @brief     set the self test
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] test is the self test param
 * @return    status code
 *            - 0 success
 *            - 1 set self test failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 test is over 0xF
 * @note      none
 */
uint8_t mfrc522_set_self_test(mfrc522_handle_t *handle, uint8_t test);

/**
 * @brief      get the self test
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *test points to a self test buffer
 * @return     status code
 *             - 0 success
 *             - 1 get self test failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_self_test(mfrc522_handle_t *handle, uint8_t *test);

/**
 * @brief      get the version
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *id points to an id buffer
 * @param[out] *version points to a version buffer
 * @return     status code
 *             - 0 success
 *             - 1 get version failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_version(mfrc522_handle_t *handle, uint8_t *id, uint8_t *version);

/**
 * @brief     set the test analog control aux 1
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] control is the aux control
 * @return    status code
 *            - 0 success
 *            - 1 set test analog control aux 1
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_test_analog_control_aux_1(mfrc522_handle_t *handle, mfrc522_test_analog_control_t control);

/**
 * @brief      get the test analog control aux 1
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *control points to a aux control buffer
 * @return     status code
 *             - 0 success
 *             - 1 get test analog control aux 1
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_test_analog_control_aux_1(mfrc522_handle_t *handle, mfrc522_test_analog_control_t *control);

/**
 * @brief     set the test analog control aux 2
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] control is the aux control
 * @return    status code
 *            - 0 success
 *            - 1 set test analog control aux 2
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_test_analog_control_aux_2(mfrc522_handle_t *handle, mfrc522_test_analog_control_t control);

/**
 * @brief      get the test analog control aux 2
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *control points to a aux control buffer
 * @return     status code
 *             - 0 success
 *             - 1 get test analog control aux 2
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_test_analog_control_aux_2(mfrc522_handle_t *handle, mfrc522_test_analog_control_t *control);

/**
 * @brief     set the test dac 1
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] dac is the set dac
 * @return    status code
 *            - 0 success
 *            - 1 set test dac 1
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 dac is over 0x3F
 * @note      none
 */
uint8_t mfrc522_set_test_dac_1(mfrc522_handle_t *handle, uint8_t dac);

/**
 * @brief      get the test dac 1
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *dac points to a set dac buffer
 * @return     status code
 *             - 0 success
 *             - 1 get test dac 1
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_test_dac_1(mfrc522_handle_t *handle, uint8_t *dac);

/**
 * @brief     set the test dac 2
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] dac is the set dac
 * @return    status code
 *            - 0 success
 *            - 1 set test dac 2
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 dac is over 0x3F
 * @note      none
 */
uint8_t mfrc522_set_test_dac_2(mfrc522_handle_t *handle, uint8_t dac);

/**
 * @brief      get the test dac 2
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *dac points to a set dac buffer
 * @return     status code
 *             - 0 success
 *             - 1 get test dac 2
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_test_dac_2(mfrc522_handle_t *handle, uint8_t *dac);

/**
 * @brief      get the test adc
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *adc_i points to an adc i buffer
 * @param[out] *adc_q points to an adc q buffer
 * @return     status code
 *             - 0 success
 *             - 1 get test adc
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_test_adc(mfrc522_handle_t *handle, uint8_t *adc_i, uint8_t *adc_q);

/**
 * @}
 */

/**
 * @defgroup mfrc522_extern_driver mfrc522 extern driver function
 * @brief    mfrc522 extern driver modules
 * @ingroup  mfrc522_driver
 * @{
 */

/**
 * @brief     set the chip register
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] reg is the register address
 * @param[in] *buf points to a data buffer
 * @param[in] len is the data buffer length
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mfrc522_set_reg(mfrc522_handle_t *handle, uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief      get the chip register
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[in]  reg is the register address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the data buffer length
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mfrc522_get_reg(mfrc522_handle_t *handle, uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
