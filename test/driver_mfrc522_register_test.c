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
 * @file      driver_mfrc522_register_test.c
 * @brief     driver mfrc522 register test source file
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

#include "driver_mfrc522_register_test.h"
#include <stdlib.h>

static mfrc522_handle_t gs_handle;        /**< mfrc522 handle */

/**
 * @brief     register test
 * @param[in] interface bus interface
 * @param[in] addr iic device address
 * @return    status code
 *            - 0 success
 *            - 1 test failed
 * @note      none
 */
uint8_t mfrc522_register_test(mfrc522_interface_t interface, uint8_t addr)
{
    uint8_t res;
    uint8_t i;
    uint8_t reg;
    uint8_t level, level_check;
    uint8_t addr_pin, addr_pin_check;
    uint8_t t0, t0_check;
    uint8_t t1, t1_check;
    uint8_t id, version;
    uint8_t adc_i;
    uint8_t adc_q;
    uint8_t buf[64];
    uint8_t buf_check[64];
    uint16_t crc;
    uint16_t t, t_check;
    mfrc522_interface_t interface_check;
    mfrc522_bool_t enable;
    mfrc522_info_t info;
    mfrc522_interrupt_pin_type_t pin_type;
    mfrc522_modem_state_t modem_state;
    mfrc522_rx_align_t align;
    mfrc522_mfin_polarity_t polarity;
    mfrc522_crc_preset_t preset;
    mfrc522_speed_t speed;
    mfrc522_tx_input_t tx_input;
    mfrc522_mfout_input_t mfout_input;
    mfrc522_contactless_uart_input_t uart_input;
    mfrc522_channel_reception_t reception;
    mfrc522_rx_gain_t gain;
    mfrc522_timer_gated_mode_t gated_mode;
    mfrc522_test_analog_control_t control;
    
    /* link interface function */
    DRIVER_MFRC522_LINK_INIT(&gs_handle, mfrc522_handle_t);
    DRIVER_MFRC522_LINK_RESET_GPIO_INIT(&gs_handle, mfrc522_interface_reset_gpio_init);
    DRIVER_MFRC522_LINK_RESET_GPIO_DEINIT(&gs_handle, mfrc522_interface_reset_gpio_deinit);
    DRIVER_MFRC522_LINK_RESET_GPIO_WRITE(&gs_handle, mfrc522_interface_reset_gpio_write);
    DRIVER_MFRC522_LINK_IIC_INIT(&gs_handle, mfrc522_interface_iic_init);
    DRIVER_MFRC522_LINK_IIC_DEINIT(&gs_handle, mfrc522_interface_iic_deinit);
    DRIVER_MFRC522_LINK_IIC_WRITE(&gs_handle, mfrc522_interface_iic_write);
    DRIVER_MFRC522_LINK_IIC_READ(&gs_handle, mfrc522_interface_iic_read);
    DRIVER_MFRC522_LINK_UART_INIT(&gs_handle, mfrc522_interface_uart_init);
    DRIVER_MFRC522_LINK_UART_DEINIT(&gs_handle, mfrc522_interface_uart_deinit);
    DRIVER_MFRC522_LINK_UART_READ(&gs_handle, mfrc522_interface_uart_read);
    DRIVER_MFRC522_LINK_UART_WRITE(&gs_handle, mfrc522_interface_uart_write);
    DRIVER_MFRC522_LINK_UART_FLUSH(&gs_handle, mfrc522_interface_uart_flush);
    DRIVER_MFRC522_LINK_SPI_INIT(&gs_handle, mfrc522_interface_spi_init);
    DRIVER_MFRC522_LINK_SPI_DEINIT(&gs_handle, mfrc522_interface_spi_deinit);
    DRIVER_MFRC522_LINK_SPI_READ(&gs_handle, mfrc522_interface_spi_read);
    DRIVER_MFRC522_LINK_SPI_WRITE(&gs_handle, mfrc522_interface_spi_write);
    DRIVER_MFRC522_LINK_DELAY_MS(&gs_handle, mfrc522_interface_delay_ms);
    DRIVER_MFRC522_LINK_DEBUG_PRINT(&gs_handle, mfrc522_interface_debug_print);
    DRIVER_MFRC522_LINK_RECEIVE_CALLBACK(&gs_handle, mfrc522_interface_receive_callback);
    
    /* get information */
    res = mfrc522_info(&info);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get info failed.\n");
        
        return 1;
    }
    else
    {
        /* print chip info */
        mfrc522_interface_debug_print("mfrc522: chip is %s.\n", info.chip_name);
        mfrc522_interface_debug_print("mfrc522: manufacturer is %s.\n", info.manufacturer_name);
        mfrc522_interface_debug_print("mfrc522: interface is %s.\n", info.interface);
        mfrc522_interface_debug_print("mfrc522: driver version is %d.%d.\n", info.driver_version / 1000, (info.driver_version % 1000) / 100);
        mfrc522_interface_debug_print("mfrc522: min supply voltage is %0.1fV.\n", info.supply_voltage_min_v);
        mfrc522_interface_debug_print("mfrc522: max supply voltage is %0.1fV.\n", info.supply_voltage_max_v);
        mfrc522_interface_debug_print("mfrc522: max current is %0.2fmA.\n", info.max_current_ma);
        mfrc522_interface_debug_print("mfrc522: max temperature is %0.1fC.\n", info.temperature_max);
        mfrc522_interface_debug_print("mfrc522: min temperature is %0.1fC.\n", info.temperature_min);
    }
    
    /* start register test */
    mfrc522_interface_debug_print("mfrc522: start register test.\n");
    
    /* mfrc522_set_addr_pin/mfrc522_get_addr_pin test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_addr_pin/mfrc522_get_addr_pin test.\n");
    
    addr_pin = rand() % 256;
    res = mfrc522_set_addr_pin(&gs_handle, addr_pin);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set addr pin failed.\n");
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set addr 0x%02X.\n", addr_pin);
    res = mfrc522_get_addr_pin(&gs_handle, &addr_pin_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get addr pin failed.\n");
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check addr pin %s.\n", addr_pin_check == addr_pin ? "ok" : "error");
    
    /* mfrc522_set_interface/mfrc522_get_interface test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_interface/mfrc522_get_interface test.\n");
    
    /* iic interface */
    res = mfrc522_set_interface(&gs_handle, MFRC522_INTERFACE_IIC);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interface failed.\n");
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set interface iic.\n");
    res = mfrc522_get_interface(&gs_handle, &interface_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interface failed.\n");
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check interface %s.\n", interface_check == MFRC522_INTERFACE_IIC ? "ok" : "error");
    
    /* spi interface */
    res = mfrc522_set_interface(&gs_handle, MFRC522_INTERFACE_SPI);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interface failed.\n");
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set interface spi.\n");
    res = mfrc522_get_interface(&gs_handle, &interface_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interface failed.\n");
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check interface %s.\n", interface_check == MFRC522_INTERFACE_SPI ? "ok" : "error");
    
    /* uart interface */
    res = mfrc522_set_interface(&gs_handle, MFRC522_INTERFACE_UART);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interface failed.\n");
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set interface uart.\n");
    res = mfrc522_get_interface(&gs_handle, &interface_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interface failed.\n");
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check interface %s.\n", interface_check == MFRC522_INTERFACE_UART ? "ok" : "error");

    /* set the interface */
    res = mfrc522_set_interface(&gs_handle, interface);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interface failed.\n");
        
        return 1;
    }
    
    /* set the addr pin */
    res = mfrc522_set_addr_pin(&gs_handle, addr);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set addr pin failed.\n");
        
        return 1;
    }
    
    /* init failed */
    res = mfrc522_init(&gs_handle);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: init failed.\n");
        
        return 1;
    }
    
    /* mfrc522_set_receiver_analog/mfrc522_get_receiver_analog test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_receiver_analog/mfrc522_get_receiver_analog test.\n");
    
    /* enable */
    res = mfrc522_set_receiver_analog(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set receiver analog failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set receiver analog enable.\n");
    res = mfrc522_get_receiver_analog(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get receiver analog failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check receiver analog %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_receiver_analog(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set receiver analog failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set receiver analog disable.\n");
    res = mfrc522_get_receiver_analog(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get receiver analog failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check receiver analog %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_power_down/mfrc522_get_power_down test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_power_down/mfrc522_get_power_down test.\n");
    
    /* if not uart interface */
    if (interface != MFRC522_INTERFACE_UART)
    {
        /* enable */
        res = mfrc522_set_power_down(&gs_handle, MFRC522_BOOL_TRUE);
        if (res != 0)
        {
            mfrc522_interface_debug_print("mfrc522: set power down failed.\n");
            (void)mfrc522_deinit(&gs_handle);
            
            return 1;
        }
        mfrc522_interface_debug_print("mfrc522: set power down enable.\n");
        res = mfrc522_get_power_down(&gs_handle, &enable);
        if (res != 0)
        {
            mfrc522_interface_debug_print("mfrc522: get power down failed.\n");
            (void)mfrc522_deinit(&gs_handle);
            
            return 1;
        }
        mfrc522_interface_debug_print("mfrc522: check power down %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    }
    
    /* disable */
    res = mfrc522_set_power_down(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set power down failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set power down disable.\n");
    res = mfrc522_get_power_down(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get power down failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check power down %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");

    /* mfrc522_set_interrupt1/mfrc522_get_interrupt1 test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_interrupt1/mfrc522_get_interrupt1 test.\n");
    
    /* enable tx interrupt1 */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_TX, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set tx interrupt1 enable.\n");
    res = mfrc522_get_interrupt1(&gs_handle, MFRC522_INTERRUPT1_TX, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check interrupt1 %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable tx interrupt1 */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_TX, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set tx interrupt1 disable.\n");
    res = mfrc522_get_interrupt1(&gs_handle, MFRC522_INTERRUPT1_TX, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check interrupt1 %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* enable rx interrupt1 */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_RX, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set rx interrupt1 enable.\n");
    res = mfrc522_get_interrupt1(&gs_handle, MFRC522_INTERRUPT1_RX, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check interrupt1 %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable rx interrupt1 */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_RX, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set rx interrupt1 disable.\n");
    res = mfrc522_get_interrupt1(&gs_handle, MFRC522_INTERRUPT1_RX, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check interrupt1 %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* enable idle interrupt1 */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_IDLE, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set idle interrupt1 enable.\n");
    res = mfrc522_get_interrupt1(&gs_handle, MFRC522_INTERRUPT1_IDLE, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check interrupt1 %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable idle interrupt1 */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_IDLE, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set idle interrupt1 disable.\n");
    res = mfrc522_get_interrupt1(&gs_handle, MFRC522_INTERRUPT1_IDLE, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check interrupt1 %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* enable hi alert interrupt1 */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_HI_ALERT, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set hi alert interrupt1 enable.\n");
    res = mfrc522_get_interrupt1(&gs_handle, MFRC522_INTERRUPT1_HI_ALERT, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check interrupt1 %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable hi alert interrupt1 */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_HI_ALERT, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set hi alert interrupt1 disable.\n");
    res = mfrc522_get_interrupt1(&gs_handle, MFRC522_INTERRUPT1_HI_ALERT, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check interrupt1 %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* enable lo alert interrupt1 */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_LO_ALERT, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set lo alert interrupt1 enable.\n");
    res = mfrc522_get_interrupt1(&gs_handle, MFRC522_INTERRUPT1_LO_ALERT, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check interrupt1 %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable lo alert interrupt1 */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_LO_ALERT, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set lo alert interrupt1 disable.\n");
    res = mfrc522_get_interrupt1(&gs_handle, MFRC522_INTERRUPT1_LO_ALERT, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check interrupt1 %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* enable err interrupt1 */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_ERR, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set err interrupt1 enable.\n");
    res = mfrc522_get_interrupt1(&gs_handle, MFRC522_INTERRUPT1_ERR, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check interrupt1 %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable err interrupt1 */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_ERR, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set err interrupt1 disable.\n");
    res = mfrc522_get_interrupt1(&gs_handle, MFRC522_INTERRUPT1_ERR, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check interrupt1 %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* enable timer interrupt1 */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_TIMER, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set timer interrupt1 enable.\n");
    res = mfrc522_get_interrupt1(&gs_handle, MFRC522_INTERRUPT1_TIMER, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check interrupt1 %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable err interrupt1 */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_TIMER, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set timer interrupt1 disable.\n");
    res = mfrc522_get_interrupt1(&gs_handle, MFRC522_INTERRUPT1_TIMER, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check interrupt1 %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_interrupt1_pin_invert/mfrc522_get_interrupt1_pin_invert test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_interrupt1_pin_invert/mfrc522_get_interrupt1_pin_invert test.\n");
    
    /* enable */
    res = mfrc522_set_interrupt1_pin_invert(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 pin invert failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set interrupt1 pin invert enable.\n");
    res = mfrc522_get_interrupt1_pin_invert(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt1 pin invert failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check invert %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_interrupt1_pin_invert(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 pin invert failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set interrupt1 pin invert disable.\n");
    res = mfrc522_get_interrupt1_pin_invert(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt1 pin invert failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check invert %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_interrupt1_trigger test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_interrupt1_trigger test.\n");
    
    /* set */
    res = mfrc522_set_interrupt1_mark(&gs_handle, MFRC522_INTERRUPT_MARK_SET);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 mark failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set interrupt1 trigger set.\n");
    mfrc522_interface_debug_print("mfrc522: check interrupt1 trigger %s.\n", res == 0 ? "ok" : "error");
    
    /* cleared */
    res = mfrc522_set_interrupt1_mark(&gs_handle, MFRC522_INTERRUPT_MARK_CLEARED);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 mark failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set interrupt1 trigger cleared.\n");
    mfrc522_interface_debug_print("mfrc522: check interrupt1 trigger %s.\n", res == 0 ? "ok" : "error");
    
    /* mfrc522_set_interrupt2/mfrc522_get_interrupt2 test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_interrupt2/mfrc522_get_interrupt2 test.\n");
    
    /* mafin act enable */
    res = mfrc522_set_interrupt2(&gs_handle, MFRC522_INTERRUPT2_MFIN_ACT, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set mafin act interrupt2 enable.\n");
    res = mfrc522_get_interrupt2(&gs_handle, MFRC522_INTERRUPT2_MFIN_ACT, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check interrupt2 %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* mafin act disable */
    res = mfrc522_set_interrupt2(&gs_handle, MFRC522_INTERRUPT2_MFIN_ACT, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set mafin act interrupt2 disable.\n");
    res = mfrc522_get_interrupt2(&gs_handle, MFRC522_INTERRUPT2_MFIN_ACT, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check interrupt2 %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* crc enable */
    res = mfrc522_set_interrupt2(&gs_handle, MFRC522_INTERRUPT2_CRC, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set crc interrupt2 enable.\n");
    res = mfrc522_get_interrupt2(&gs_handle, MFRC522_INTERRUPT2_CRC, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check interrupt2 %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* crc disable */
    res = mfrc522_set_interrupt2(&gs_handle, MFRC522_INTERRUPT2_CRC, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set crc interrupt2 disable.\n");
    res = mfrc522_get_interrupt2(&gs_handle, MFRC522_INTERRUPT2_CRC, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check interrupt2 %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_interrupt_pin_type/mfrc522_get_interrupt_pin_type test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_interrupt_pin_type/mfrc522_get_interrupt_pin_type test.\n");
    
    /* set standard cmos */
    res = mfrc522_set_interrupt_pin_type(&gs_handle, MFRC522_INTERRUPT_PIN_TYPE_STANDARD_CMOS);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt pin type failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set interrupt pin type standard cmos.\n");
    res = mfrc522_get_interrupt_pin_type(&gs_handle, &pin_type);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt pin type failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check interrupt pin type %s.\n", pin_type == MFRC522_INTERRUPT_PIN_TYPE_STANDARD_CMOS ? "ok" : "error");
    
    /* set open drain */
    res = mfrc522_set_interrupt_pin_type(&gs_handle, MFRC522_INTERRUPT_PIN_TYPE_OPEN_DRAIN);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt pin type failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set interrupt pin type open drain.\n");
    res = mfrc522_get_interrupt_pin_type(&gs_handle, &pin_type);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt pin type failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check interrupt pin type %s.\n", pin_type == MFRC522_INTERRUPT_PIN_TYPE_OPEN_DRAIN ? "ok" : "error");
    
    /* mfrc522_set_interrupt1_mark test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_interrupt1_mark test.\n");
    
    /* set */
    res = mfrc522_set_interrupt1_mark(&gs_handle, MFRC522_INTERRUPT_MARK_SET);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 mark failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set interrupt1 mark set.\n");
    mfrc522_interface_debug_print("mfrc522: check interrupt1 mark %s.\n", res == 0 ? "ok" : "error");
    
    /* cleared */
    res = mfrc522_set_interrupt1_mark(&gs_handle, MFRC522_INTERRUPT_MARK_CLEARED);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 mark failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set interrupt1 mark cleared.\n");
    mfrc522_interface_debug_print("mfrc522: check interrupt1 mark %s.\n", res == 0 ? "ok" : "error");
    
    /* mfrc522_set_interrupt2_mark test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_interrupt2_mark test.\n");
    
    /* set */
    res = mfrc522_set_interrupt2_mark(&gs_handle, MFRC522_INTERRUPT_MARK_SET);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt2 mark failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set interrupt2 mark set.\n");
    mfrc522_interface_debug_print("mfrc522: check interrupt2 mark %s.\n", res == 0 ? "ok" : "error");
    
    /* cleared */
    res = mfrc522_set_interrupt2_mark(&gs_handle, MFRC522_INTERRUPT_MARK_CLEARED);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt2 mark failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set interrupt2 mark cleared.\n");
    mfrc522_interface_debug_print("mfrc522: check interrupt2 mark %s.\n", res == 0 ? "ok" : "error");
    
    /* mfrc522_get_interrupt1_status */
    mfrc522_interface_debug_print("mfrc522: mfrc522_get_interrupt1_status test.\n");
    
    /* get the interrupt1 status */
    res = mfrc522_get_interrupt1_status(&gs_handle, &reg);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt1 status failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: interrupt1 status is 0x%02X.\n", reg);
    
    /* mfrc522_get_interrupt2_status */
    mfrc522_interface_debug_print("mfrc522: mfrc522_get_interrupt2_status test.\n");
    
    /* get the interrupt2 status */
    res = mfrc522_get_interrupt2_status(&gs_handle, &reg);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get interrupt2 status failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: interrupt2 status is 0x%02X.\n", reg);
    
    /* mfrc522_get_error */
    mfrc522_interface_debug_print("mfrc522: mfrc522_get_error test.\n");
    
    /* get the error */
    res = mfrc522_get_error(&gs_handle, &reg);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get error failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: error is 0x%02X.\n", reg);
    
    /* mfrc522_get_status1 */
    mfrc522_interface_debug_print("mfrc522: mfrc522_get_status1 test.\n");
    
    /* get the status1 */
    res = mfrc522_get_status1(&gs_handle, &reg);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get status1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: status1 is 0x%02X.\n", reg);
    
    /* mfrc522_get_status2 */
    mfrc522_interface_debug_print("mfrc522: mfrc522_get_status2 test.\n");
    
    /* get the status2 */
    res = mfrc522_get_status2(&gs_handle, &reg);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get status2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: status2 is 0x%02X.\n", reg);
    
    /* mfrc522_get_modem_state */
    mfrc522_interface_debug_print("mfrc522: mfrc522_get_modem_state test.\n");
    
    /* get modem state */
    res = mfrc522_get_modem_state(&gs_handle, &modem_state);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get modem state failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: modem state is 0x%02X.\n", modem_state);
    
    /* mfrc522_set_mifare_crypto1_on/mfrc522_get_mifare_crypto1_on test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_mifare_crypto1_on/mfrc522_get_mifare_crypto1_on test.\n");
    
    /* enable */
    res = mfrc522_set_mifare_crypto1_on(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set mifare crypto1 on failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set mifare crypto1 on enable.\n");
    mfrc522_interface_debug_print("mfrc522: check mifare crypto1 on %s.\n", res == 0 ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_mifare_crypto1_on(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set mifare crypto1 on failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set mifare crypto1 on disable.\n");
    mfrc522_interface_debug_print("mfrc522: check mifare crypto1 on %s.\n", res == 0 ? "ok" : "error");
    
    /* mfrc522_set_force_iic_high_speed/mfrc522_get_force_iic_high_speed test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_force_iic_high_speed/mfrc522_get_force_iic_high_speed test.\n");
    
    /* enable */
    res = mfrc522_set_force_iic_high_speed(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set force iic high speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set force iic high speed enable.\n");
    res = mfrc522_get_force_iic_high_speed(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get force iic high speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check force iic high speed %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_force_iic_high_speed(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set force iic high speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set force iic high speed disable.\n");
    res = mfrc522_get_force_iic_high_speed(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get force iic high speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check force iic high speed %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_clear_temperature_error/mfrc522_get_clear_temperature_error test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_clear_temperature_error/mfrc522_get_clear_temperature_error test.\n");
    
    /* enable */
    res = mfrc522_set_clear_temperature_error(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set clear temperature error failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set clear temperature error enable.\n");
    res = mfrc522_get_clear_temperature_error(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get clear temperature error failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check clear temperature error enable %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_clear_temperature_error(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set clear temperature error failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set clear temperature error disable.\n");
    res = mfrc522_get_clear_temperature_error(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get clear temperature error failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check clear temperature error enable %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_get_fifo_level test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_get_fifo_level test.\n");
    
    /* get fifo level */
    res = mfrc522_get_fifo_level(&gs_handle, &level);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get fifo level failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: fifo level is 0x%02X.\n", level);
    
    /* mfrc522_flush_fifo test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_flush_fifo test.\n");
     
    /* flush fifo */
    res = mfrc522_flush_fifo(&gs_handle);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: flush fifo failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check flush fifo %s.\n", res == 0 ? "ok" : "error");
    
    /* mfrc522_set_water_level/mfrc522_get_water_level test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_water_level/mfrc522_get_water_level test.\n");
    
    level = rand() % 0x40;
    res = mfrc522_set_water_level(&gs_handle, level);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set water level failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set water level 0x%02X.\n", level);
    res = mfrc522_get_water_level(&gs_handle, &level_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get water level failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check water level %s.\n", level == level_check ? "ok" : "error");
    
    /* mfrc522_start_timer test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_start_timer test.\n");
    
    /* start the timer */
    res = mfrc522_start_timer(&gs_handle);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: start timer failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check start timer %s.\n", res == 0 ? "ok" : "error");
    
    /* mfrc522_stop_timer test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_stop_timer test.\n");
    
    /* stop the timer */
    res = mfrc522_start_timer(&gs_handle);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: stop timer failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check stop timer %s.\n", res == 0 ? "ok" : "error");
    
    /* mfrc522_get_rx_last_bits test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_get_rx_last_bits test.\n");
    
    /* get rx last bits */
    res = mfrc522_get_rx_last_bits(&gs_handle, &reg);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get rx last bits failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: rx last bits is 0x%02X.\n", reg);
    
    /* mfrc522_start_send test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_start_send test.\n");
    
    /* start send */
    res = mfrc522_start_send(&gs_handle);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: start send failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check start send %s.\n", res == 0 ? "ok" : "error");
    
    /* mfrc522_stop_send test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_stop_send test.\n");
    
    /* stop send */
    res = mfrc522_stop_send(&gs_handle);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: stop send failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check stop send %s.\n", res == 0 ? "ok" : "error");
    
    /* mfrc522_set_tx_last_bits/mfrc522_get_tx_last_bits test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_tx_last_bits/mfrc522_get_tx_last_bits test.\n");
    
    level = rand() % 8;
    res = mfrc522_set_tx_last_bits(&gs_handle, level);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx last bits failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set tx last bits 0x%02X.\n", level);
    res = mfrc522_get_tx_last_bits(&gs_handle, &level_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get tx last bits failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check tx last bits %s.\n", level == level_check ? "ok" : "error");
    
    /* mfrc522_set_rx_align/mfrc522_get_rx_align test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_rx_align/mfrc522_get_rx_align test.\n");
    
    /* set rx align 0 */
    res = mfrc522_set_rx_align(&gs_handle, MFRC522_RX_ALIGN_0);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx align failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set rx align 0.\n");
    res = mfrc522_get_rx_align(&gs_handle, &align);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get rx align failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check rx align %s.\n", align == MFRC522_RX_ALIGN_0 ? "ok" : "error");
    
    /* set rx align 1 */
    res = mfrc522_set_rx_align(&gs_handle, MFRC522_RX_ALIGN_1);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx align failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set rx align 1.\n");
    res = mfrc522_get_rx_align(&gs_handle, &align);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get rx align failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check rx align %s.\n", align == MFRC522_RX_ALIGN_1 ? "ok" : "error");
    
    /* set rx align 7 */
    res = mfrc522_set_rx_align(&gs_handle, MFRC522_RX_ALIGN_7);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx align failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set rx align 7.\n");
    res = mfrc522_get_rx_align(&gs_handle, &align);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get rx align failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check rx align %s.\n", align == MFRC522_RX_ALIGN_7 ? "ok" : "error");
    
    /* mfrc522_set_value_clear_after_coll/mfrc522_get_value_clear_after_coll test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_value_clear_after_coll/mfrc522_get_value_clear_after_coll test.\n");
    
    /* enable */
    res = mfrc522_set_value_clear_after_coll(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set value clear after coll failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set value clear after coll enable.\n");
    res = mfrc522_get_value_clear_after_coll(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get value clear after coll failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check value clear after coll %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_value_clear_after_coll(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set value clear after coll failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set value clear after coll disable.\n");
    res = mfrc522_get_value_clear_after_coll(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get value clear after coll failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check value clear after coll %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_get_collision_position_not_valid test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_get_collision_position_not_valid test.\n");
    
    /* get the bool */
    res = mfrc522_get_collision_position_not_valid(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get collision position not valid failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: collision position not valid is %s.\n", enable == MFRC522_BOOL_TRUE ? "true" : "false");
    
    /* mfrc522_get_collision_position_not_valid test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_get_collision_position_not_valid test.\n");
    
    /* get collision position */
    res = mfrc522_get_collision_position(&gs_handle, &reg);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get collision position failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: collision position is 0x%02X.\n", reg);
    
    /* mfrc522_set_crc_msb_first/mfrc522_get_crc_msb_first test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_crc_msb_first/mfrc522_get_crc_msb_first test.\n");
    
    /* enable */
    res = mfrc522_set_crc_msb_first(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set crc msb first failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set crc msb first enable.\n");
    res = mfrc522_get_crc_msb_first(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get crc msb first failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check get crc msb first %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_crc_msb_first(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set crc msb first failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set crc msb first disable.\n");
    res = mfrc522_get_crc_msb_first(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get crc msb first failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check get crc msb first %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_tx_wait_rf/mfrc522_get_tx_wait_rf test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_tx_wait_rf/mfrc522_get_tx_wait_rf test.\n");
    
    /* enable */
    res = mfrc522_set_tx_wait_rf(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx wait rf failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set tx wait rf enable .\n");
    res = mfrc522_get_tx_wait_rf(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get tx wait rf failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check tx wait rf %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_tx_wait_rf(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx wait rf failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set tx wait rf disable .\n");
    res = mfrc522_get_tx_wait_rf(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get tx wait rf failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check tx wait rf %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_mfin_polarity/mfrc522_get_mfin_polarity test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_mfin_polarity/mfrc522_get_mfin_polarity test.\n");
    
    /* set low */
    res = mfrc522_set_mfin_polarity(&gs_handle, MFRC522_MFIN_POLARITY_LOW);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set mfin polarity failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set mfin polarity low.\n");
    res = mfrc522_get_mfin_polarity(&gs_handle, &polarity);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get mfin polarity failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check mfin polarity %s.\n", polarity == MFRC522_MFIN_POLARITY_LOW ? "ok" : "error");
    
    /* set high */
    res = mfrc522_set_mfin_polarity(&gs_handle, MFRC522_MFIN_POLARITY_HIGH);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set mfin polarity failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set mfin polarity high.\n");
    res = mfrc522_get_mfin_polarity(&gs_handle, &polarity);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get mfin polarity failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check mfin polarity %s.\n", polarity == MFRC522_MFIN_POLARITY_HIGH ? "ok" : "error");
    
    /* mfrc522_set_crc_preset/mfrc522_get_crc_preset test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_crc_preset/mfrc522_get_crc_preset test.\n");
    
    /* 0000 */
    res = mfrc522_set_crc_preset(&gs_handle, MFRC522_CRC_PRESET_0000);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set crc preset failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set crc preset 0000.\n");
    res = mfrc522_get_crc_preset(&gs_handle, &preset);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get crc preset failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check crc preset %s.\n", preset == MFRC522_CRC_PRESET_0000 ? "ok" : "error");
    
    /* 6363 */
    res = mfrc522_set_crc_preset(&gs_handle, MFRC522_CRC_PRESET_6363);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set crc preset failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set crc preset 6363.\n");
    res = mfrc522_get_crc_preset(&gs_handle, &preset);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get crc preset failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check crc preset %s.\n", preset == MFRC522_CRC_PRESET_6363 ? "ok" : "error");
    
    /* A671 */
    res = mfrc522_set_crc_preset(&gs_handle, MFRC522_CRC_PRESET_A671);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set crc preset failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set crc preset A671.\n");
    res = mfrc522_get_crc_preset(&gs_handle, &preset);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get crc preset failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check crc preset %s.\n", preset == MFRC522_CRC_PRESET_A671 ? "ok" : "error");
    
    /* FFFF */
    res = mfrc522_set_crc_preset(&gs_handle, MFRC522_CRC_PRESET_FFFF);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set crc preset failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set crc preset FFFF.\n");
    res = mfrc522_get_crc_preset(&gs_handle, &preset);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get crc preset failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check crc preset %s.\n", preset == MFRC522_CRC_PRESET_FFFF ? "ok" : "error");
    
    /* mfrc522_set_tx_crc_generation/mfrc522_get_tx_crc_generation test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_tx_crc_generation/mfrc522_get_tx_crc_generation test.\n");
    
    /* enable */
    res = mfrc522_set_tx_crc_generation(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx crc generation failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set tx crc generation enable.\n");
    res = mfrc522_get_tx_crc_generation(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get tx crc generation failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check tx crc generation %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_tx_crc_generation(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx crc generation failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set tx crc generation disable.\n");
    res = mfrc522_get_tx_crc_generation(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get tx crc generation failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check tx crc generation %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_tx_speed/mfrc522_get_tx_speed test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_tx_speed/mfrc522_get_tx_speed test.\n");
    
    /* 106 kBd*/
    res = mfrc522_set_tx_speed(&gs_handle, MFRC522_SPEED_106_KBD);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set tx speed 106 kBd.\n");
    res = mfrc522_get_tx_speed(&gs_handle, &speed);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get tx speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check tx speed %s.\n", speed == MFRC522_SPEED_106_KBD ? "ok" : "error");
    
    /* 212 kBd*/
    res = mfrc522_set_tx_speed(&gs_handle, MFRC522_SPEED_212_KBD);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set tx speed 212 kBd.\n");
    res = mfrc522_get_tx_speed(&gs_handle, &speed);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get tx speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check tx speed %s.\n", speed == MFRC522_SPEED_212_KBD ? "ok" : "error");
    
    /* 424 kBd*/
    res = mfrc522_set_tx_speed(&gs_handle, MFRC522_SPEED_424_KBD);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set tx speed 424 kBd.\n");
    res = mfrc522_get_tx_speed(&gs_handle, &speed);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get tx speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check tx speed %s.\n", speed == MFRC522_SPEED_424_KBD ? "ok" : "error");
    
    /* 848 kBd*/
    res = mfrc522_set_tx_speed(&gs_handle, MFRC522_SPEED_848_KBD);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set tx speed 848 kBd.\n");
    res = mfrc522_get_tx_speed(&gs_handle, &speed);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get tx speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check tx speed %s.\n", speed == MFRC522_SPEED_848_KBD ? "ok" : "error");
    
    /* mfrc522_set_modulation_invert/mfrc522_get_modulation_invert test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_modulation_invert/mfrc522_get_modulation_invert test.\n");
    
    /* enable */
    res = mfrc522_set_modulation_invert(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set modulation invert failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set modulation invert enable.\n");
    res = mfrc522_get_modulation_invert(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get modulation invert failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check modulation invert %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_modulation_invert(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set modulation invert failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set modulation invert disable.\n");
    res = mfrc522_get_modulation_invert(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get modulation invert failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check modulation invert %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_rx_crc_generation/mfrc522_get_rx_crc_generation test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_rx_crc_generation/mfrc522_get_rx_crc_generation test.\n");
    
    /* enable */
    res = mfrc522_set_rx_crc_generation(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx crc generation failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set rx crc generation enable.\n");
    res = mfrc522_get_rx_crc_generation(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get rx crc generation failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check rx crc generation %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_rx_crc_generation(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx crc generation failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set rx crc generation disable.\n");
    res = mfrc522_get_rx_crc_generation(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get rx crc generation failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check rx crc generation %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_rx_speed/mfrc522_get_rx_speed test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_rx_speed/mfrc522_get_rx_speed test.\n");
    
    /* 106 kBd */
    res = mfrc522_set_rx_speed(&gs_handle, MFRC522_SPEED_106_KBD);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set rx speed 106 kBd.\n");
    res = mfrc522_get_rx_speed(&gs_handle, &speed);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get rx speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check rx speed %s.\n", speed == MFRC522_SPEED_106_KBD ? "ok" : "error");
    
    /* 212 kBd */
    res = mfrc522_set_rx_speed(&gs_handle, MFRC522_SPEED_212_KBD);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set rx speed 212 kBd.\n");
    res = mfrc522_get_rx_speed(&gs_handle, &speed);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get rx speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check rx speed %s.\n", speed == MFRC522_SPEED_212_KBD ? "ok" : "error");
    
    /* 424 kBd */
    res = mfrc522_set_rx_speed(&gs_handle, MFRC522_SPEED_424_KBD);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set rx speed 424 kBd.\n");
    res = mfrc522_get_rx_speed(&gs_handle, &speed);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get rx speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check rx speed %s.\n", speed == MFRC522_SPEED_424_KBD ? "ok" : "error");
    
    /* 848 kBd */
    res = mfrc522_set_rx_speed(&gs_handle, MFRC522_SPEED_848_KBD);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set rx speed 848 kBd.\n");
    res = mfrc522_get_rx_speed(&gs_handle, &speed);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get rx speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check rx speed %s.\n", speed == MFRC522_SPEED_848_KBD ? "ok" : "error");
    
    /* mfrc522_set_rx_no_error/mfrc522_get_rx_no_error test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_rx_no_error/mfrc522_get_rx_no_error test.\n");
    
    /* enable */
    res = mfrc522_set_rx_no_error(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx no error failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set rx no error enable.\n");
    res = mfrc522_get_rx_no_error(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get rx no error failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check rx no error %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_rx_no_error(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx no error failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set rx no error disable.\n");
    res = mfrc522_get_rx_no_error(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get rx no error failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check rx no error %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_rx_multiple/mfrc522_get_rx_multiple test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_rx_multiple/mfrc522_get_rx_multiple test.\n");
    
    /* enable */
    res = mfrc522_set_rx_multiple(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx multiple failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set rx multiple enable.\n");
    res = mfrc522_get_rx_multiple(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get rx multiple failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check rx multiple %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_rx_multiple(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx multiple failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set rx multiple disable.\n");
    res = mfrc522_get_rx_multiple(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get rx multiple failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check rx multiple %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_antenna_driver/mfrc522_get_antenna_driver test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_antenna_driver/mfrc522_get_antenna_driver test.\n");
    
    /* enable */
    res = mfrc522_set_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_INV_TX2_RF_ON, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set antenna driver inv tx2 rf on enable.\n");
    res = mfrc522_get_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_INV_TX2_RF_ON, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check antenna driver %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_INV_TX2_RF_ON, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set antenna driver inv tx2 rf on disable.\n");
    res = mfrc522_get_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_INV_TX2_RF_ON, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check antenna driver %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* enable */
    res = mfrc522_set_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_INV_TX1_RF_ON, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set antenna driver inv tx1 rf on enable.\n");
    res = mfrc522_get_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_INV_TX1_RF_ON, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check antenna driver %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_INV_TX1_RF_ON, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set antenna driver inv tx1 rf on disable.\n");
    res = mfrc522_get_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_INV_TX1_RF_ON, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check antenna driver %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* enable */
    res = mfrc522_set_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_INV_TX2_RF_OFF, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set antenna driver inv tx2 rf off enable.\n");
    res = mfrc522_get_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_INV_TX2_RF_OFF, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check antenna driver %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_INV_TX2_RF_OFF, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set antenna driver inv tx2 rf off disable.\n");
    res = mfrc522_get_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_INV_TX2_RF_OFF, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check antenna driver %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* enable */
    res = mfrc522_set_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_INV_TX1_RF_OFF, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set antenna driver inv tx1 rf off enable.\n");
    res = mfrc522_get_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_INV_TX1_RF_OFF, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check antenna driver %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_INV_TX1_RF_OFF, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set antenna driver inv tx1 rf off disable.\n");
    res = mfrc522_get_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_INV_TX1_RF_OFF, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check antenna driver %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* enable */
    res = mfrc522_set_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_TX2_CW, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set antenna driver tx2 cw enable.\n");
    res = mfrc522_get_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_TX2_CW, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check antenna driver %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_TX2_CW, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set antenna driver tx2 cw disable.\n");
    res = mfrc522_get_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_TX2_CW, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check antenna driver %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* enable */
    res = mfrc522_set_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_TX2_RF, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set antenna driver tx2 rf enable.\n");
    res = mfrc522_get_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_TX2_RF, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check antenna driver %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_TX2_RF, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set antenna driver tx2 rf disable.\n");
    res = mfrc522_get_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_TX2_RF, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check antenna driver %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* enable */
    res = mfrc522_set_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_TX1_RF, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set antenna driver tx1 rf enable.\n");
    res = mfrc522_get_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_TX1_RF, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check antenna driver %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_TX1_RF, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set antenna driver tx1 rf disable.\n");
    res = mfrc522_get_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_TX1_RF, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check antenna driver %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_force_100_ask/mfrc522_get_force_100_ask test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_force_100_ask/mfrc522_get_force_100_ask test.\n");
    
    /* enable */
    res = mfrc522_set_force_100_ask(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set force 100 ask failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set force 100 ask enable.\n");
    res = mfrc522_get_force_100_ask(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get force 100 ask failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check force 100 ask %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_force_100_ask(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set force 100 ask failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set force 100 ask disable.\n");
    res = mfrc522_get_force_100_ask(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get force 100 ask failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check force 100 ask %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_tx_input/mfrc522_get_tx_input test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_tx_input/mfrc522_get_tx_input test.\n");
    
    /* 3 state */
    res = mfrc522_set_tx_input(&gs_handle, MFRC522_TX_INPUT_3_STATE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set tx input 3 state.\n");
    res = mfrc522_get_tx_input(&gs_handle, &tx_input);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get tx input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check tx input %s.\n", tx_input == MFRC522_TX_INPUT_3_STATE ? "ok" : "error");
    
    /* internal encoder */
    res = mfrc522_set_tx_input(&gs_handle, MFRC522_TX_INPUT_INTERNAL_ENCODER);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set tx input internal encoder.\n");
    res = mfrc522_get_tx_input(&gs_handle, &tx_input);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get tx input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check tx input %s.\n", tx_input == MFRC522_TX_INPUT_INTERNAL_ENCODER ? "ok" : "error");
    
    /* mfin pin */
    res = mfrc522_set_tx_input(&gs_handle, MFRC522_TX_INPUT_MFIN_PIN);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set tx input mfin pin.\n");
    res = mfrc522_get_tx_input(&gs_handle, &tx_input);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get tx input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check tx input %s.\n", tx_input == MFRC522_TX_INPUT_MFIN_PIN ? "ok" : "error");
    
    /* control */
    res = mfrc522_set_tx_input(&gs_handle, MFRC522_TX_INPUT_CONTROL);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set tx input control.\n");
    res = mfrc522_get_tx_input(&gs_handle, &tx_input);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get tx input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check tx input %s.\n", tx_input == MFRC522_TX_INPUT_CONTROL ? "ok" : "error");
    
    /* mfrc522_set_mfout_input/mfrc522_get_mfout_input test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_mfout_input/mfrc522_get_mfout_input test.\n");
    
    /* 3 state */
    res = mfrc522_set_mfout_input(&gs_handle, MFRC522_MFOUT_INPUT_3_STATE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set mfout input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set mfout input 3 state.\n");
    res = mfrc522_get_mfout_input(&gs_handle, &mfout_input);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get mfout input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check mfout input %s.\n", mfout_input == MFRC522_MFOUT_INPUT_3_STATE ? "ok" : "error");
    
    /* low */
    res = mfrc522_set_mfout_input(&gs_handle, MFRC522_MFOUT_INPUT_LOW);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set mfout input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set mfout input low.\n");
    res = mfrc522_get_mfout_input(&gs_handle, &mfout_input);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get mfout input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check mfout input %s.\n", mfout_input == MFRC522_MFOUT_INPUT_LOW ? "ok" : "error");
    
    /* high */
    res = mfrc522_set_mfout_input(&gs_handle, MFRC522_MFOUT_INPUT_HIGH);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set mfout input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set mfout input high.\n");
    res = mfrc522_get_mfout_input(&gs_handle, &mfout_input);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get mfout input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check mfout input %s.\n", mfout_input == MFRC522_MFOUT_INPUT_HIGH ? "ok" : "error");
    
    /* test */
    res = mfrc522_set_mfout_input(&gs_handle, MFRC522_MFOUT_INPUT_TEST);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set mfout input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set mfout input test.\n");
    res = mfrc522_get_mfout_input(&gs_handle, &mfout_input);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get mfout input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check mfout input %s.\n", mfout_input == MFRC522_MFOUT_INPUT_TEST ? "ok" : "error");
    
    /* internal encoder */
    res = mfrc522_set_mfout_input(&gs_handle, MFRC522_MFOUT_INPUT_INTERNAL_ENCODER);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set mfout input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set mfout input internal encoder.\n");
    res = mfrc522_get_mfout_input(&gs_handle, &mfout_input);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get mfout input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check mfout input %s.\n", mfout_input == MFRC522_MFOUT_INPUT_INTERNAL_ENCODER ? "ok" : "error");
    
    /* transmitted */
    res = mfrc522_set_mfout_input(&gs_handle, MFRC522_MFOUT_INPUT_TRANSMITTED);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set mfout input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set mfout input transmitted.\n");
    res = mfrc522_get_mfout_input(&gs_handle, &mfout_input);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get mfout input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check mfout input %s.\n", mfout_input == MFRC522_MFOUT_INPUT_TRANSMITTED ? "ok" : "error");
    
    /* received */
    res = mfrc522_set_mfout_input(&gs_handle, MFRC522_MFOUT_INPUT_RECEIVED);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set mfout input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set mfout input received.\n");
    res = mfrc522_get_mfout_input(&gs_handle, &mfout_input);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get mfout input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check mfout input %s.\n", mfout_input == MFRC522_MFOUT_INPUT_RECEIVED ? "ok" : "error");
    
    /* mfrc522_set_contactless_uart_input/mfrc522_get_contactless_uart_input test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_contactless_uart_input/mfrc522_get_contactless_uart_input test.\n");
    
    /* set constant low */
    res = mfrc522_set_contactless_uart_input(&gs_handle, MFRC522_CONTACTLESS_UART_INPUT_CONSTANT_LOW);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set contactless uart input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set contactless uart input constant low.\n");
    res = mfrc522_get_contactless_uart_input(&gs_handle, &uart_input);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get contactless uart input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check contactless uart input %s.\n", uart_input == MFRC522_CONTACTLESS_UART_INPUT_CONSTANT_LOW ? "ok" : "error");
    
    /* set mfin pin */
    res = mfrc522_set_contactless_uart_input(&gs_handle, MFRC522_CONTACTLESS_UART_MFIN_PIN);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set contactless uart input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set contactless uart input mfin pin.\n");
    res = mfrc522_get_contactless_uart_input(&gs_handle, &uart_input);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get contactless uart input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check contactless uart input %s.\n", uart_input == MFRC522_CONTACTLESS_UART_MFIN_PIN ? "ok" : "error");
    
    /* set internal analog module */
    res = mfrc522_set_contactless_uart_input(&gs_handle, MFRC522_CONTACTLESS_UART_INTERNAL_ANALOG_MODULE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set contactless uart input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set contactless uart input internal analog module.\n");
    res = mfrc522_get_contactless_uart_input(&gs_handle, &uart_input);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get contactless uart input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check contactless uart input %s.\n", uart_input == MFRC522_CONTACTLESS_UART_INTERNAL_ANALOG_MODULE ? "ok" : "error");
    
    /* set nrz */
    res = mfrc522_set_contactless_uart_input(&gs_handle, MFRC522_CONTACTLESS_UART_NRZ);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set contactless uart input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set contactless uart input nrz.\n");
    res = mfrc522_get_contactless_uart_input(&gs_handle, &uart_input);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get contactless uart input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check contactless uart input %s.\n", uart_input == MFRC522_CONTACTLESS_UART_NRZ ? "ok" : "error");
    
    /* mfrc522_set_rx_wait/mfrc522_get_rx_wait test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_rx_wait/mfrc522_get_rx_wait test.\n");
    
    level = rand() % 0x40;
    res = mfrc522_set_rx_wait(&gs_handle, level);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx wait failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set rx wait 0x%02X.\n", level);
    res = mfrc522_get_rx_wait(&gs_handle, &level_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get rx wait failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check rx wait %s.\n", level_check == level ? "ok" : "error");
    
    /* mfrc522_set_min_level/mfrc522_get_min_level test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_min_level/mfrc522_get_min_level test.\n");
    
    level = rand() % 0xF;
    res = mfrc522_set_min_level(&gs_handle, level);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set min level failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set min level 0x%02X.\n", level);
    res = mfrc522_get_min_level(&gs_handle, &level_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get min level failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check min level %s.\n", level_check == level ? "ok" : "error");
    
    /* mfrc522_set_collision_level/mfrc522_get_collision_level test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_collision_level/mfrc522_get_collision_level test.\n");
    
    level = rand() % 8;
    res = mfrc522_set_collision_level(&gs_handle, level);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set collision level failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set collision level 0x%02X.\n", level);
    res = mfrc522_get_collision_level(&gs_handle, &level_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get collision level failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check collision level %s.\n", level_check == level ? "ok" : "error");
    
    /* mfrc522_set_channel_reception/mfrc522_get_channel_reception test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_channel_reception/mfrc522_get_channel_reception test.\n");
    
    /* stronger channel */
    res = mfrc522_set_channel_reception(&gs_handle, MFRC522_CHANNEL_RECEPTION_STRONGER);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set channel reception failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set channel reception stronger channel.\n");
    res = mfrc522_get_channel_reception(&gs_handle, &reception);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get channel reception failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check channel reception %s.\n", reception == MFRC522_CHANNEL_RECEPTION_STRONGER ? "ok" : "error");
    
    /* stronger channel and freezes the selected channel */
    res = mfrc522_set_channel_reception(&gs_handle, MFRC522_CHANNEL_RECEPTION_STRONGER_FREEZE_SELECTED);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set channel reception failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set channel reception stronger channel and freezes the selected channel.\n");
    res = mfrc522_get_channel_reception(&gs_handle, &reception);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get channel reception failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check channel reception %s.\n", reception == MFRC522_CHANNEL_RECEPTION_STRONGER_FREEZE_SELECTED ? "ok" : "error");
    
    /* mfrc522_set_fix_iq/mfrc522_get_fix_iq test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_fix_iq/mfrc522_get_fix_iq test.\n");
    
    /* enable */
    res = mfrc522_set_fix_iq(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set fix iq failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set fix iq enable.\n");
    res = mfrc522_get_fix_iq(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get fix iq failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check fix iq %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_fix_iq(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set fix iq failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set fix iq disable.\n");
    res = mfrc522_get_fix_iq(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get fix iq failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check fix iq %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_timer_prescal_even/mfrc522_get_timer_prescal_even test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_timer_prescal_even/mfrc522_get_timer_prescal_even test.\n");
    
    /* enable */
    res = mfrc522_set_timer_prescal_even(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer prescal even failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set timer prescal even enable.\n");
    res = mfrc522_get_timer_prescal_even(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get timer prescal even failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check timer prescal even %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_timer_prescal_even(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer prescal even failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set timer prescal even disable.\n");
    res = mfrc522_get_timer_prescal_even(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get timer prescal even failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check timer prescal even %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_timer_constant_reception/mfrc522_get_timer_constant_reception test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_timer_constant_reception/mfrc522_get_timer_constant_reception test.\n");
    
    level = rand() % 3;
    res = mfrc522_set_timer_constant_reception(&gs_handle, level);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer constant reception failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set timer constant reception 0x%02X.\n", level);
    res = mfrc522_get_timer_constant_reception(&gs_handle, &level_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get timer constant reception failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check timer constant reception %s.\n", level == level_check ? "ok" : "error");
    
    /* mfrc522_set_timer_constant_sync/mfrc522_get_timer_constant_sync test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_timer_constant_sync/mfrc522_get_timer_constant_sync test.\n");
    
    level = rand() % 3;
    res = mfrc522_set_timer_constant_sync(&gs_handle, level);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer constant sync failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set timer constant sync 0x%02X.\n", level);
    res = mfrc522_get_timer_constant_sync(&gs_handle, &level_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get timer constant sync failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check timer constant sync %s.\n", level == level_check ? "ok" : "error");
    
    /* mfrc522_set_tx_wait/mfrc522_get_tx_wait test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_tx_wait/mfrc522_get_tx_wait test.\n");
    
    level = rand() % 3;
    res = mfrc522_set_tx_wait(&gs_handle, level);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx wait failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set tx wait 0x%02X.\n", level);
    res = mfrc522_get_tx_wait(&gs_handle, &level_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get tx wait failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check tx wait %s.\n", level == level_check ? "ok" : "error");
    
    /* mfrc522_set_parity_disable/mfrc522_get_parity_disable test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_parity_disable/mfrc522_get_parity_disable test.\n");
    
    /* enable */
    res = mfrc522_set_parity_disable(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set parity disable failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set parity disable enable.\n");
    res = mfrc522_get_parity_disable(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get parity disable failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check parity disable %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_parity_disable(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set parity disable failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set parity disable disable.\n");
    res = mfrc522_get_parity_disable(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get parity disable failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check parity disable %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    if (interface != MFRC522_INTERFACE_UART)
    {
        /* mfrc522_set_serial_speed/mfrc522_get_serial_speed test */
        mfrc522_interface_debug_print("mfrc522: mfrc522_set_serial_speed/mfrc522_get_serial_speed test.\n");
        
        t0 = rand() % 0x8;
        t1 = rand() % 0x20;
        res = mfrc522_set_serial_speed(&gs_handle, t0, t1);
        if (res != 0)
        {
            mfrc522_interface_debug_print("mfrc522: set serial speed failed.\n");
            (void)mfrc522_deinit(&gs_handle);
            
            return 1;
        }
        mfrc522_interface_debug_print("mfrc522: set serial speed t0 0x%02X.\n", t0);
        mfrc522_interface_debug_print("mfrc522: set serial speed t1 0x%02X.\n", t1);
        res = mfrc522_get_serial_speed(&gs_handle, &t0_check, &t1_check);
        if (res != 0)
        {
            mfrc522_interface_debug_print("mfrc522: get serial speed failed.\n");
            (void)mfrc522_deinit(&gs_handle);
            
            return 1;
        }
        mfrc522_interface_debug_print("mfrc522: check serial speed t0 %s.\n", t0 == t0_check ? "ok" : "error");
        mfrc522_interface_debug_print("mfrc522: check serial speed t1 %s.\n", t1 == t1_check ? "ok" : "error");
    }
    
    /* mfrc522_get_crc test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_get_crc test.\n");
    
    /* get the crc */
    res = mfrc522_get_crc(&gs_handle, &crc);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get crc failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: crc is 0x%04X.\n", crc);
    
    /* mfrc522_set_modulation_width/mfrc522_get_modulation_width test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_modulation_width/mfrc522_get_modulation_width test.\n");
    
    level = rand() % 256;
    res = mfrc522_set_modulation_width(&gs_handle, level);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set modulation width failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set modulation width 0x%02X.\n", level);
    res = mfrc522_get_modulation_width(&gs_handle, &level_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get modulation width failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check modulation width %s.\n", level == level_check ? "ok" : "error");
    
    /* mfrc522_set_rx_gain/mfrc522_get_rx_gain test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_rx_gain/mfrc522_get_rx_gain test.\n");
    
    /* set the 18 db */
    res = mfrc522_set_rx_gain(&gs_handle, MFRC522_RX_GAIN_18_DB);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx gain failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set rx gain 18 db.\n");
    res = mfrc522_get_rx_gain(&gs_handle, &gain);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get rx gain failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check rx gain %s.\n", gain == MFRC522_RX_GAIN_18_DB ? "ok" : "error");
    
    /* set the 23 db */
    res = mfrc522_set_rx_gain(&gs_handle, MFRC522_RX_GAIN_23_DB);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx gain failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set rx gain 23 db.\n");
    res = mfrc522_get_rx_gain(&gs_handle, &gain);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get rx gain failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check rx gain %s.\n", gain == MFRC522_RX_GAIN_23_DB ? "ok" : "error");
    
    /* set the 33 db */
    res = mfrc522_set_rx_gain(&gs_handle, MFRC522_RX_GAIN_33_DB);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx gain failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set rx gain 33 db.\n");
    res = mfrc522_get_rx_gain(&gs_handle, &gain);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get rx gain failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check rx gain %s.\n", gain == MFRC522_RX_GAIN_33_DB ? "ok" : "error");
    
    /* set the 38 db */
    res = mfrc522_set_rx_gain(&gs_handle, MFRC522_RX_GAIN_38_DB);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx gain failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set rx gain 38 db.\n");
    res = mfrc522_get_rx_gain(&gs_handle, &gain);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get rx gain failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check rx gain %s.\n", gain == MFRC522_RX_GAIN_38_DB ? "ok" : "error");
    
    /* set the 43 db */
    res = mfrc522_set_rx_gain(&gs_handle, MFRC522_RX_GAIN_43_DB);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx gain failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set rx gain 43 db.\n");
    res = mfrc522_get_rx_gain(&gs_handle, &gain);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get rx gain failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check rx gain %s.\n", gain == MFRC522_RX_GAIN_43_DB ? "ok" : "error");
    
    /* set the 48 db */
    res = mfrc522_set_rx_gain(&gs_handle, MFRC522_RX_GAIN_48_DB);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx gain failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set rx gain 48 db.\n");
    res = mfrc522_get_rx_gain(&gs_handle, &gain);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get rx gain failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check rx gain %s.\n", gain == MFRC522_RX_GAIN_48_DB ? "ok" : "error");
    
    /* mfrc522_set_cwgsn/mfrc522_get_cwgsn test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_cwgsn/mfrc522_get_cwgsn test.\n");
    
    level = rand() % 0xF;
    res = mfrc522_set_cwgsn(&gs_handle, level);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set cwgsn failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set cwgsn 0x%02X.\n", level);
    res = mfrc522_get_cwgsn(&gs_handle, &level_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get cwgsn failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check cwgsn %s.\n", level == level_check ? "ok" : "error");
    
    /* mfrc522_set_modgsn/mfrc522_get_modgsn test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_modgsn/mfrc522_get_modgsn test.\n");
    
    level = rand() % 0xF;
    res = mfrc522_set_modgsn(&gs_handle, level);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set modgsn failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set modgsn 0x%02X.\n", level);
    res = mfrc522_get_modgsn(&gs_handle, &level_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get modgsn failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check modgsn %s.\n", level == level_check ? "ok" : "error");
    
    /* mfrc522_set_cwgsp/mfrc522_get_cwgsp test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_cwgsp/mfrc522_get_cwgsp test.\n");
    
    level = rand() % 0x40;
    res = mfrc522_set_cwgsp(&gs_handle, level);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set cwgsp failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set cwgsp 0x%02X.\n", level);
    res = mfrc522_get_cwgsp(&gs_handle, &level_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get cwgsp failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check cwgsp %s.\n", level == level_check ? "ok" : "error");
    
    /* mfrc522_set_modgsp/mfrc522_get_modgsp test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_modgsp/mfrc522_get_modgsp test.\n");
    
    level = rand() % 0x40;
    res = mfrc522_set_modgsp(&gs_handle, level);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set modgsp failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set modgsp 0x%02X.\n", level);
    res = mfrc522_get_modgsp(&gs_handle, &level_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get modgsp failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check modgsp %s.\n", level == level_check ? "ok" : "error");
    
    /* mfrc522_set_timer_auto/mfrc522_get_timer_auto test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_timer_auto/mfrc522_get_timer_auto test.\n");
    
    /* enable */
    res = mfrc522_set_timer_auto(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer auto failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set timer auto enable.\n");
    res = mfrc522_get_timer_auto(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get timer auto failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check timer auto %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_timer_auto(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer auto failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set timer auto disable.\n");
    res = mfrc522_get_timer_auto(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get timer auto failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check timer auto %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_timer_gated_mode/mfrc522_get_timer_gated_mode test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_timer_gated_mode/mfrc522_get_timer_gated_mode test.\n");
    
    /* none */
    res = mfrc522_set_timer_gated_mode(&gs_handle, MFRC522_TIMER_GATED_MODE_NONE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer gated mode failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set timer gated mode none.\n");
    res = mfrc522_get_timer_gated_mode(&gs_handle, &gated_mode);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get timer gated mode failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check timer gated mode %s.\n", gated_mode == MFRC522_TIMER_GATED_MODE_NONE ? "ok" : "error");
    
    /* mfin */
    res = mfrc522_set_timer_gated_mode(&gs_handle, MFRC522_TIMER_GATED_MODE_MFIN);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer gated mode failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set timer gated mode mfin.\n");
    res = mfrc522_get_timer_gated_mode(&gs_handle, &gated_mode);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get timer gated mode failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check timer gated mode %s.\n", gated_mode == MFRC522_TIMER_GATED_MODE_MFIN ? "ok" : "error");
    
    /* aux1 */
    res = mfrc522_set_timer_gated_mode(&gs_handle, MFRC522_TIMER_GATED_MODE_AUX1);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer gated mode failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set timer gated mode aux1.\n");
    res = mfrc522_get_timer_gated_mode(&gs_handle, &gated_mode);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get timer gated mode failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check timer gated mode %s.\n", gated_mode == MFRC522_TIMER_GATED_MODE_AUX1 ? "ok" : "error");
    
    /* mfrc522_set_timer_auto_restart/mfrc522_get_timer_auto_restart test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_timer_auto_restart/mfrc522_get_timer_auto_restart test.\n");
    
    /* enable */
    res = mfrc522_set_timer_auto_restart(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer_auto restart failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set timer auto restart enable.\n");
    res = mfrc522_get_timer_auto_restart(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get timer_auto restart failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check timer auto restart %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_timer_auto_restart(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer_auto restart failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set timer auto restart disable.\n");
    res = mfrc522_get_timer_auto_restart(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get timer_auto restart failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check timer auto restart %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_timer_prescaler/mfrc522_get_timer_prescaler test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_timer_prescaler/mfrc522_get_timer_prescaler test.\n");
    
    t = rand() % 0xFFF;
    res = mfrc522_set_timer_prescaler(&gs_handle, t);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer prescaler failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set timer prescaler 0x%04X.\n", t);
    res = mfrc522_get_timer_prescaler(&gs_handle, &t_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get timer prescaler failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check timer prescaler %s.\n", t_check == t ? "ok" : "error");
    
    /* mfrc522_set_timer_reload/mfrc522_get_timer_reload test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_timer_reload/mfrc522_get_timer_reload test.\n");
    
    t = rand() % 0xFFFFU;
    res = mfrc522_set_timer_reload(&gs_handle, t);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer reload failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set timer reload 0x%04X.\n", t);
    res = mfrc522_get_timer_reload(&gs_handle, &t_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get timer reload failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check timer reload %s.\n", t_check == t ? "ok" : "error");
    
    /* mfrc522_get_timer_counter test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_get_timer_counter test.\n");
    
    res = mfrc522_get_timer_counter(&gs_handle, &t_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get timer reload failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check timer reload 0x%04X.\n", t_check);
    
    /* mfrc522_set_fifo_data/mfrc522_get_fifo_data test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_fifo_data/mfrc522_get_fifo_data test.\n");
    
    for (i = 0; i < 64; i++)
    {
        buf[i] = rand() % 256;
    }
    res = mfrc522_set_fifo_data(&gs_handle, buf, 64);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set fifo data failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    res = mfrc522_get_fifo_data(&gs_handle, buf_check, 64);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set fifo data failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check fifo data %s.\n", memcmp(buf, buf_check, 64) == 0 ? "ok" : "error");
    
    /* mfrc522_set_test_bus_signal_1/mfrc522_get_test_bus_signal_1 test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_test_bus_signal_1/mfrc522_get_test_bus_signal_1 test.\n");
    
    level = rand() % 8;
    res = mfrc522_set_test_bus_signal_1(&gs_handle, level);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test bus signal 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test bus signal 1 0x%02X.\n", level);
    res = mfrc522_get_test_bus_signal_1(&gs_handle, &level_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test bus signal 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test bus signal 1 %s.\n", level == level_check ? "ok" : "error");
    
    /* mfrc522_set_test_bus_signal_2/mfrc522_get_test_bus_signal_2 test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_test_bus_signal_2/mfrc522_get_test_bus_signal_2 test.\n");
    
    level = rand() % 0x20;
    res = mfrc522_set_test_bus_signal_2(&gs_handle, level);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test bus signal 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test bus signal 2 0x%02X.\n", level);
    res = mfrc522_get_test_bus_signal_2(&gs_handle, &level_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test bus signal 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test bus signal 2 %s.\n", level == level_check ? "ok" : "error");
    
    /* mfrc522_set_test_bus_flip/mfrc522_get_test_bus_flip test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_test_bus_flip/mfrc522_get_test_bus_flip test.\n");
    
    /* enable */
    res = mfrc522_set_test_bus_flip(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test bus flip failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test bus flip enable.\n");
    res = mfrc522_get_test_bus_flip(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test bus flip failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test bus flip %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_test_bus_flip(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test bus flip failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test bus flip disable.\n");
    res = mfrc522_get_test_bus_flip(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test bus flip failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test bus flip %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_test_prbs9/mfrc522_get_test_prbs9 test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_test_prbs9/mfrc522_get_test_prbs9 test.\n");
    
    /* enable */
    res = mfrc522_set_test_prbs9(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test prbs9 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test prbs9 enable.\n");
    res = mfrc522_get_test_prbs9(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test prbs9 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test prbs9 %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_test_prbs9(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test prbs9 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test prbs9 disable.\n");
    res = mfrc522_get_test_prbs9(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test prbs9 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test prbs9 %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_test_prbs15/mfrc522_get_test_prbs15 test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_test_prbs15/mfrc522_get_test_prbs15 test.\n");
    
    /* enable */
    res = mfrc522_set_test_prbs15(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test prbs15 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test prbs15 enable.\n");
    res = mfrc522_get_test_prbs15(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test prbs15 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test prbs15 %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_test_prbs15(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test prbs15 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test prbs15 disable.\n");
    res = mfrc522_get_test_prbs15(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test prbs15 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test prbs15 %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_test_rs232_line/mfrc522_get_test_rs232_line test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_test_rs232_line/mfrc522_get_test_rs232_line test.\n");
    
    /* enable */
    res = mfrc522_set_test_rs232_line(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test rs232 line failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test rs232 line enable.\n");
    res = mfrc522_get_test_rs232_line(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test rs232 line failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test rs232 line %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_test_rs232_line(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test rs232 line failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test rs232 line disable.\n");
    res = mfrc522_get_test_rs232_line(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test rs232 line failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test rs232 line %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_test_pin_enable/mfrc522_get_test_pin_enable test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_test_pin_enable/mfrc522_get_test_pin_enable test.\n");
    
    level = rand() % 0x40;
    res = mfrc522_set_test_pin_enable(&gs_handle, level);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test pin enable failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test pin enable 0x%02X.\n", level);
    res = mfrc522_get_test_pin_enable(&gs_handle, &level_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test pin enable failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test pin enable %s.\n", level == level_check ? "ok" : "error");
    
    /* mfrc522_set_test_port_io/mfrc522_get_test_port_io test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_test_port_io/mfrc522_get_test_port_io test.\n");
    
    /* enable */
    res = mfrc522_set_test_port_io(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test port io failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test port io enable.\n");
    res = mfrc522_get_test_port_io(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test port io failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test port io %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_test_port_io(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test port io failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test port io disable.\n");
    res = mfrc522_get_test_port_io(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test port io failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test port io %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_test_pin_value/mfrc522_get_test_pin_value test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_test_pin_value/mfrc522_get_test_pin_value test.\n");
    
    level = rand() % 0x40;
    res = mfrc522_set_test_pin_value(&gs_handle, level);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test pin value failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test pin value 0x%02X.\n", level);
    res = mfrc522_get_test_pin_value(&gs_handle, &level_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test pin value failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test pin value %s.\n", level == level_check ? "ok" : "error");
    
    /* mfrc522_get_test_bus test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_get_test_bus test.\n");
    
    res = mfrc522_get_test_bus(&gs_handle, &level);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test bus failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: get test bus 0x%02X.\n", level);
    
    /* mfrc522_set_test_amp_rcv/mfrc522_get_test_amp_rcv test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_test_amp_rcv/mfrc522_get_test_amp_rcv test.\n");
    
    /* enable */
    res = mfrc522_set_test_amp_rcv(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test amp rcv failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test amp rcv enable.\n");
    res = mfrc522_get_test_amp_rcv(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test amp rcv failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test amp rcv %s.\n", enable == MFRC522_BOOL_TRUE ? "ok" : "error");
    
    /* disable */
    res = mfrc522_set_test_amp_rcv(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test amp rcv failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test amp rcv disable.\n");
    res = mfrc522_get_test_amp_rcv(&gs_handle, &enable);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test amp rcv failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test amp rcv %s.\n", enable == MFRC522_BOOL_FALSE ? "ok" : "error");
    
    /* mfrc522_set_self_test/mfrc522_get_self_test test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_self_test/mfrc522_get_self_test test.\n");
    
    level = 0x00;
    res = mfrc522_set_self_test(&gs_handle, level);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set self test failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set self test 0x%02X.\n", level);
    res = mfrc522_get_self_test(&gs_handle, &level_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get self test failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check self test %s.\n", level == level_check ? "ok" : "error");
    
    /* mfrc522_get_version test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_get_version test.\n");
    
    res = mfrc522_get_version(&gs_handle, &id, &version);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get version failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: id is 0x%02X, version is 0x%02X.\n", id, version);
    
    /* mfrc522_set_test_analog_control_aux_1/mfrc522_get_test_analog_control_aux_1 test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_test_analog_control_aux_1/mfrc522_get_test_analog_control_aux_1 test.\n");
    
    /* 3 state */
    res = mfrc522_set_test_analog_control_aux_1(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_3_STATE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 3 state.\n");
    res = mfrc522_get_test_analog_control_aux_1(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 1 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_3_STATE ? "ok" : "error");
    
    /* output */
    res = mfrc522_set_test_analog_control_aux_1(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_OUTPUT);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 output.\n");
    res = mfrc522_get_test_analog_control_aux_1(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 1 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_OUTPUT ? "ok" : "error");
    
    /* test signal corr1 */
    res = mfrc522_set_test_analog_control_aux_1(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_TEST_SIGNAL_CORR1);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 test signal corr1.\n");
    res = mfrc522_get_test_analog_control_aux_1(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 1 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_TEST_SIGNAL_CORR1 ? "ok" : "error");
    
    /* dac test signal min level */
    res = mfrc522_set_test_analog_control_aux_1(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_DAC_TEST_SIGNAL_MIN_LEVEL);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 dac test signal min level.\n");
    res = mfrc522_get_test_analog_control_aux_1(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 1 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_DAC_TEST_SIGNAL_MIN_LEVEL ? "ok" : "error");
    
    /* dac test signal adc i */
    res = mfrc522_set_test_analog_control_aux_1(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_DAC_TEST_SIGNAL_ADC_I);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 dac test signal adc i.\n");
    res = mfrc522_get_test_analog_control_aux_1(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 1 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_DAC_TEST_SIGNAL_ADC_I ? "ok" : "error");
    
    /* dac test signal adc q */
    res = mfrc522_set_test_analog_control_aux_1(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_DAC_TEST_SIGNAL_ADC_Q);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 dac test signal adc q.\n");
    res = mfrc522_get_test_analog_control_aux_1(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 1 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_DAC_TEST_SIGNAL_ADC_Q ? "ok" : "error");
    
    /* test signal for production */
    res = mfrc522_set_test_analog_control_aux_1(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_SIGNAL_FOR_PRODUCTION);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 test signal for production.\n");
    res = mfrc522_get_test_analog_control_aux_1(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 1 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_SIGNAL_FOR_PRODUCTION ? "ok" : "error");
    
    /* high */
    res = mfrc522_set_test_analog_control_aux_1(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_HIGH);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 high.\n");
    res = mfrc522_get_test_analog_control_aux_1(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 1 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_HIGH ? "ok" : "error");
    
    /* low */
    res = mfrc522_set_test_analog_control_aux_1(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_LOW);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 low.\n");
    res = mfrc522_get_test_analog_control_aux_1(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 1 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_LOW ? "ok" : "error");
    
    /* tx active */
    res = mfrc522_set_test_analog_control_aux_1(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_TX_ACTIVE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 tx active.\n");
    res = mfrc522_get_test_analog_control_aux_1(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 1 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_TX_ACTIVE ? "ok" : "error");
    
    /* rx active */
    res = mfrc522_set_test_analog_control_aux_1(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_RX_ACTIVE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 rx active.\n");
    res = mfrc522_get_test_analog_control_aux_1(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 1 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_RX_ACTIVE ? "ok" : "error");
    
    /* subcarrier detected */
    res = mfrc522_set_test_analog_control_aux_1(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_SUBCARRIER_DETECTED);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 subcarrier detected.\n");
    res = mfrc522_get_test_analog_control_aux_1(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 1 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_SUBCARRIER_DETECTED ? "ok" : "error");
    
    /* defined bit */
    res = mfrc522_set_test_analog_control_aux_1(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_DEFINED_BIT);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 1 defined bit.\n");
    res = mfrc522_get_test_analog_control_aux_1(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 1 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_DEFINED_BIT ? "ok" : "error");
    
    /* mfrc522_set_test_analog_control_aux_2/mfrc522_get_test_analog_control_aux_2 test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_test_analog_control_aux_2/mfrc522_get_test_analog_control_aux_2 test.\n");
    
    /* 3 state */
    res = mfrc522_set_test_analog_control_aux_2(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_3_STATE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 3 state.\n");
    res = mfrc522_get_test_analog_control_aux_2(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 2 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_3_STATE ? "ok" : "error");
    
    /* output */
    res = mfrc522_set_test_analog_control_aux_2(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_OUTPUT);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 output.\n");
    res = mfrc522_get_test_analog_control_aux_2(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 2 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_OUTPUT ? "ok" : "error");
    
    /* test signal corr1 */
    res = mfrc522_set_test_analog_control_aux_2(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_TEST_SIGNAL_CORR1);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 test signal corr1.\n");
    res = mfrc522_get_test_analog_control_aux_2(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 2 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_TEST_SIGNAL_CORR1 ? "ok" : "error");
    
    /* dac test signal min level */
    res = mfrc522_set_test_analog_control_aux_2(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_DAC_TEST_SIGNAL_MIN_LEVEL);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 dac test signal min level.\n");
    res = mfrc522_get_test_analog_control_aux_2(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 2 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_DAC_TEST_SIGNAL_MIN_LEVEL ? "ok" : "error");
    
    /* dac test signal adc i */
    res = mfrc522_set_test_analog_control_aux_2(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_DAC_TEST_SIGNAL_ADC_I);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 dac test signal adc i.\n");
    res = mfrc522_get_test_analog_control_aux_2(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 2 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_DAC_TEST_SIGNAL_ADC_I ? "ok" : "error");
    
    /* dac test signal adc q */
    res = mfrc522_set_test_analog_control_aux_2(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_DAC_TEST_SIGNAL_ADC_Q);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 dac test signal adc q.\n");
    res = mfrc522_get_test_analog_control_aux_2(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 2 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_DAC_TEST_SIGNAL_ADC_Q ? "ok" : "error");
    
    /* test signal for production */
    res = mfrc522_set_test_analog_control_aux_2(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_SIGNAL_FOR_PRODUCTION);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 test signal for production.\n");
    res = mfrc522_get_test_analog_control_aux_2(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 2 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_SIGNAL_FOR_PRODUCTION ? "ok" : "error");
    
    /* high */
    res = mfrc522_set_test_analog_control_aux_2(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_HIGH);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 high.\n");
    res = mfrc522_get_test_analog_control_aux_2(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 2 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_HIGH ? "ok" : "error");
    
    /* low */
    res = mfrc522_set_test_analog_control_aux_2(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_LOW);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 low.\n");
    res = mfrc522_get_test_analog_control_aux_2(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 2 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_LOW ? "ok" : "error");
    
    /* tx active */
    res = mfrc522_set_test_analog_control_aux_2(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_TX_ACTIVE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 tx active.\n");
    res = mfrc522_get_test_analog_control_aux_2(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 2 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_TX_ACTIVE ? "ok" : "error");
    
    /* rx active */
    res = mfrc522_set_test_analog_control_aux_2(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_RX_ACTIVE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 rx active.\n");
    res = mfrc522_get_test_analog_control_aux_2(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 2 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_RX_ACTIVE ? "ok" : "error");
    
    /* subcarrier detected */
    res = mfrc522_set_test_analog_control_aux_2(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_SUBCARRIER_DETECTED);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 subcarrier detected.\n");
    res = mfrc522_get_test_analog_control_aux_2(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 2 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_SUBCARRIER_DETECTED ? "ok" : "error");
    
    /* defined bit */
    res = mfrc522_set_test_analog_control_aux_2(&gs_handle, MFRC522_TEST_ANALOG_CONTROL_DEFINED_BIT);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: set test analog control aux 2 defined bit.\n");
    res = mfrc522_get_test_analog_control_aux_2(&gs_handle, &control);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test analog control aux 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: check test analog control aux 2 %s.\n", control == MFRC522_TEST_ANALOG_CONTROL_DEFINED_BIT ? "ok" : "error");
    
    /* mfrc522_set_test_dac_1/mfrc522_get_test_dac_1 test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_test_dac_1/mfrc522_get_test_dac_1 test.\n");
    
    level = rand() % 0x40;
    res = mfrc522_set_test_dac_1(&gs_handle, level);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test dac 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: set test dac 1 0x%02X.\n", level);
    res = mfrc522_get_test_dac_1(&gs_handle, &level_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test dac 1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: check test dac 1 %s.\n", level == level_check ? "ok" : "error");
    
    /* mfrc522_set_test_dac_2/mfrc522_get_test_dac_2 test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_set_test_dac_2/mfrc522_get_test_dac_2 test.\n");
    
    level = rand() % 0x40;
    res = mfrc522_set_test_dac_2(&gs_handle, level);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set test dac 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: set test dac 2 0x%02X.\n", level);
    res = mfrc522_get_test_dac_2(&gs_handle, &level_check);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test dac 2 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: check test dac 2 %s.\n", level == level_check ? "ok" : "error");
    
    /* mfrc522_get_test_adc test */
    mfrc522_interface_debug_print("mfrc522: mfrc522_get_test_adc test.\n");
    
    res = mfrc522_get_test_adc(&gs_handle, &adc_i, &adc_q);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get test adc failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 2;
    }
    mfrc522_interface_debug_print("mfrc522: test adc i is 0x%02X adc q is 0x%02X.\n", adc_i, adc_q);
    
    /* finish register */
    mfrc522_interface_debug_print("mfrc522: finish register test.\n");
    (void)mfrc522_deinit(&gs_handle);
    
    return 0;
}
