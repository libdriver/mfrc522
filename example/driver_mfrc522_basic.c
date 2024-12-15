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
 * @file      driver_mfrc522_basic.c
 * @brief     driver mfrc522 basic source file
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

#include "driver_mfrc522_basic.h"

static mfrc522_handle_t gs_handle;        /**< mfrc522 handle */

/**
 * @brief  interrupt irq
 * @return status code
 *         - 0 success
 *         - 1 run failed
 * @note   none
 */
uint8_t mfrc522_interrupt_irq_handler(void)
{
    if (mfrc522_irq_handler(&gs_handle) != 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

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
uint8_t mfrc522_basic_init(mfrc522_interface_t interface, uint8_t addr, void (*callback)(uint16_t type))
{
    uint8_t res;
    
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
    DRIVER_MFRC522_LINK_RECEIVE_CALLBACK(&gs_handle, callback);
    
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
    
    /* enable set_receiver analog */
    res = mfrc522_set_receiver_analog(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set receiver analog failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default interrupt1 pin invert */
    res = mfrc522_set_interrupt1_pin_invert(&gs_handle, MFRC522_BASIC_DEFAULT_INTERRUPT1_PIN_INVERT);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default interrupt pin type */
    res = mfrc522_set_interrupt_pin_type(&gs_handle, MFRC522_BASIC_DEFAULT_INTERRUPT_PIN_TYPE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default force iic high speed */
    res = mfrc522_set_force_iic_high_speed(&gs_handle, MFRC522_BASIC_DEFAULT_FORCE_IIC_HIGH_SPEED);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default clear temperature error */
    res = mfrc522_set_clear_temperature_error(&gs_handle, MFRC522_BASIC_DEFAULT_CLEAR_TEMPERATURE_ERROR);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set clear temperature error failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default water level */
    res = mfrc522_set_water_level(&gs_handle, MFRC522_BASIC_DEFAULT_WATER_LEVEL);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set water level failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* stop timer */
    res = mfrc522_stop_timer(&gs_handle);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: stop timer failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default rx align */
    res = mfrc522_set_rx_align(&gs_handle, MFRC522_BASIC_DEFAULT_RX_ALIGN);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx align failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* disable value clear after coll */
    res = mfrc522_set_value_clear_after_coll(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set value clear after coll failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default tx crc generation */
    res = mfrc522_set_tx_crc_generation(&gs_handle, MFRC522_BASIC_DEFAULT_TX_CRC_GENERATION);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx crc generation failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default tx speed */
    res = mfrc522_set_tx_speed(&gs_handle, MFRC522_BASIC_DEFAULT_TX_SPEED);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default modulation invert */
    res = mfrc522_set_modulation_invert(&gs_handle, MFRC522_BASIC_DEFAULT_MODULATION_INVERT);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set modulation invert failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default rx crc generation */
    res = mfrc522_set_rx_crc_generation(&gs_handle, MFRC522_BASIC_DEFAULT_RX_CRC_GENERATION);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx crc generation failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default rx speed */
    res = mfrc522_set_rx_speed(&gs_handle, MFRC522_BASIC_DEFAULT_RX_SPEED);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default rx no error */
    res = mfrc522_set_rx_no_error(&gs_handle, MFRC522_BASIC_DEFAULT_RX_NO_ERROR);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx no error failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* stop send */
    res = mfrc522_stop_send(&gs_handle);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: stop send failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default rx multiple */
    res = mfrc522_set_rx_multiple(&gs_handle, MFRC522_BASIC_DEFAULT_RX_MULTIPLE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx multiple failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default tx input */
    res = mfrc522_set_tx_input(&gs_handle, MFRC522_BASIC_DEFAULT_TX_INPUT);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default mfout input */
    res = mfrc522_set_mfout_input(&gs_handle, MFRC522_BASIC_DEFAULT_MFOUT_INPUT);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set mfout input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set  the default min level */
    res = mfrc522_set_min_level(&gs_handle, MFRC522_BASIC_DEFAULT_MINI_LEVEL);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set min level failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default collision level */
    res = mfrc522_set_collision_level(&gs_handle, MFRC522_BASIC_DEFAULT_COLLISION_LEVEL);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set collision level failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default channel reception */
    res = mfrc522_set_channel_reception(&gs_handle, MFRC522_BASIC_DEFAULT_CHANNEL_RECEPTION);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set channel reception failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default fix iq */
    res = mfrc522_set_fix_iq(&gs_handle, MFRC522_BASIC_DEFAULT_FIX_IQ);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set fix iq failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default timer prescal even */
    res = mfrc522_set_timer_prescal_even(&gs_handle, MFRC522_BASIC_DEFAULT_TIMER_PRESCAL_EVEN);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer prescal even failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default timer constant reception */
    res = mfrc522_set_timer_constant_reception(&gs_handle, MFRC522_BASIC_DEFAULT_TIMER_CONSTANT_RECEPTION);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer constant reception failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default timer constant sync */
    res = mfrc522_set_timer_constant_sync(&gs_handle, MFRC522_BASIC_DEFAULT_TIMER_CONSTANT_SYNC);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer constant sync failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default tx wait */
    res = mfrc522_set_tx_wait(&gs_handle, MFRC522_BASIC_DEFAULT_TX_WAIT);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx wait failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default parity disable */
    res = mfrc522_set_parity_disable(&gs_handle, MFRC522_BASIC_DEFAULT_PARITY_DISABLE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set parity disable failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default serial speed */
    res = mfrc522_set_serial_speed(&gs_handle, MFRC522_BASIC_DEFAULT_SERIAL_SPEED_T0, MFRC522_BASIC_DEFAULT_SERIAL_SPEED_T1);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set serial speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default modulation width */
    res = mfrc522_set_modulation_width(&gs_handle, MFRC522_BASIC_DEFAULT_MODULATION_WIDTH);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set modulation width failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default cwgsn */
    res = mfrc522_set_cwgsn(&gs_handle, MFRC522_BASIC_DEFAULT_CWGSN);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set cwgsn failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default modgsn */
    res = mfrc522_set_modgsn(&gs_handle, MFRC522_BASIC_DEFAULT_MODGSN);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set modgsn failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default cwgsp */
    res = mfrc522_set_cwgsp(&gs_handle, MFRC522_BASIC_DEFAULT_CWGSP);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set cwgsp failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default modgsp */
    res = mfrc522_set_modgsp(&gs_handle, MFRC522_BASIC_DEFAULT_MODGSP);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set modgsp failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default interrupt1 tx */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_TX, MFRC522_BASIC_DEFAULT_INTERRUPT1_TX);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default interrupt1 rx */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_RX, MFRC522_BASIC_DEFAULT_INTERRUPT1_RX);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default interrupt1 idle */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_IDLE, MFRC522_BASIC_DEFAULT_INTERRUPT1_IDLE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default interrupt1 hi alert */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_HI_ALERT, MFRC522_BASIC_DEFAULT_INTERRUPT1_HI_ALERT);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default interrupt1 lo alert */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_LO_ALERT, MFRC522_BASIC_DEFAULT_INTERRUPT1_LO_ALERT);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default interrupt1 err */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_ERR, MFRC522_BASIC_DEFAULT_INTERRUPT1_ERR);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default interrupt1 timer */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_TIMER, MFRC522_BASIC_DEFAULT_INTERRUPT1_TIMER);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default interrupt2 mfin act */
    res = mfrc522_set_interrupt2(&gs_handle, MFRC522_INTERRUPT2_MFIN_ACT, MFRC522_BASIC_DEFAULT_INTERRUPT2_MFIN_ACT);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default interrupt2 crc */
    res = mfrc522_set_interrupt2(&gs_handle, MFRC522_INTERRUPT2_CRC, MFRC522_BASIC_DEFAULT_INTERRUPT2_CRC);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default crc msb first */
    res = mfrc522_set_crc_msb_first(&gs_handle, MFRC522_BASIC_DEFAULT_CRC_MSB_FIRST);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set crc msb first failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default tx wait rf */
    res = mfrc522_set_tx_wait_rf(&gs_handle, MFRC522_BASIC_DEFAULT_TX_WAIT_RF);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx wait rf failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default mfin polarity */
    res = mfrc522_set_mfin_polarity(&gs_handle, MFRC522_BASIC_DEFAULT_MFIN_POLARITY);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx wait rf failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default crc preset */
    res = mfrc522_set_crc_preset(&gs_handle, MFRC522_BASIC_DEFAULT_CRC_PRESET);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set crc preset failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default force 100 ask */
    res = mfrc522_set_force_100_ask(&gs_handle, MFRC522_BASIC_DEFAULT_FORCE_100_ASK);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set force 100 ask failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default contactless uart input */
    res = mfrc522_set_contactless_uart_input(&gs_handle, MFRC522_BASIC_DEFAULT_CONTACTLESS_UART_INPUT);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set contactless uart input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default rx wait */
    res = mfrc522_set_rx_wait(&gs_handle, MFRC522_BASIC_DEFAULT_RX_WAIT);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx wait failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default rx gain */
    res = mfrc522_set_rx_gain(&gs_handle, MFRC522_BASIC_DEFAULT_RX_GAIN);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx gain failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default timer auto */
    res = mfrc522_set_timer_auto(&gs_handle, MFRC522_BASIC_DEFAULT_TIMER_AUTO);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set auto failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default timer gated mode */
    res = mfrc522_set_timer_gated_mode(&gs_handle, MFRC522_BASIC_DEFAULT_TIMER_GATED_MODE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer gated mode failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default timer auto restart */
    res = mfrc522_set_timer_auto_restart(&gs_handle, MFRC522_BASIC_DEFAULT_TIMER_AUTO_RESTART);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer auto restart failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default timer prescaler */
    res = mfrc522_set_timer_prescaler(&gs_handle, MFRC522_BASIC_DEFAULT_TIMER_PRESCALER);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer prescaler failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the default timer reload */
    res = mfrc522_set_timer_reload(&gs_handle, MFRC522_BASIC_DEFAULT_TIMER_RELOAD);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer reload failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* antenna on */
    res = mfrc522_set_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_TX1_RF, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* antenna on */
    res = mfrc522_set_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_TX2_RF, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set antenna driver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* disable cypto1 on */
    res = mfrc522_set_mifare_crypto1_on(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set mifare crypto1 on failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    return 0;
}

/**
 * @brief  basic example deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t mfrc522_basic_deinit(void)
{
    uint8_t res;
    
    /* antenna off */
    res = mfrc522_set_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_TX1_RF, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        return 1;
    }
    
    /* antenna off */
    res = mfrc522_set_antenna_driver(&gs_handle, MFRC522_ANTENNA_DRIVER_TX2_RF, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        return 1;
    }
    
    /* deinit */
    res = mfrc522_deinit(&gs_handle);
    if (res != 0)
    {
        return 1;
    }
    
    return 0;
}

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
uint8_t mfrc522_basic_transceiver(uint8_t *in_buf, uint8_t in_len, uint8_t *out_buf, uint8_t *out_len)
{
    uint8_t res;
    uint8_t err;
    
    /* request */
    if ((in_len == 1) && (in_buf[0] == 0x26))
    {
        /* disable cypto1 on */
        res = mfrc522_set_mifare_crypto1_on(&gs_handle, MFRC522_BOOL_FALSE);
        if (res != 0)
        {
            return 1;
        }
        
        /* set tx last bits 7 */
        res = mfrc522_set_tx_last_bits(&gs_handle, 7);
        if (res != 0)
        {
            return 1;
        }
        
        /* transceiver */
        res = mfrc522_transceiver(&gs_handle, MFRC522_COMMAND_TRANSCEIVE, in_buf, in_len, out_buf, out_len, &err, 1000);
        if (res != 0)
        {
            return 1;
        }
        
        return 0;
    }
    /* wake up */
    else if ((in_len == 1) && (in_buf[0] == 0x52))
    {
        /* disable cypto1 on */
        res = mfrc522_set_mifare_crypto1_on(&gs_handle, MFRC522_BOOL_FALSE);
        if (res != 0)
        {
            return 1;
        }
        
        /* set tx last bits 7 */
        res = mfrc522_set_tx_last_bits(&gs_handle, 7);
        if (res != 0)
        {
            return 1;
        }
        
        /* transceiver */
        res = mfrc522_transceiver(&gs_handle, MFRC522_COMMAND_TRANSCEIVE, in_buf, in_len, out_buf, out_len, &err, 1000);
        if (res != 0)
        {
            return 1;
        }
        
        return 0;
    }
    /* anti collision cl1 */
    else if ((in_len == 2) && (in_buf[0] == 0x93) && (in_buf[1] == 0x20))
    {
        /* disable cypto1 on */
        res = mfrc522_set_mifare_crypto1_on(&gs_handle, MFRC522_BOOL_FALSE);
        if (res != 0)
        {
            return 1;
        }
        
        /* set tx last bits 0 */
        res = mfrc522_set_tx_last_bits(&gs_handle, 0);
        if (res != 0)
        {
            return 1;
        }
        
        /* enable value clear after coll */
        res = mfrc522_set_value_clear_after_coll(&gs_handle, MFRC522_BOOL_TRUE);
        if (res != 0)
        {
            return 1;
        }
        
        /* transceiver */
        res = mfrc522_transceiver(&gs_handle, MFRC522_COMMAND_TRANSCEIVE, in_buf, in_len, out_buf, out_len, &err, 1000);
        if (res != 0)
        {
            return 1;
        }
        
        /* disable value clear after coll */
        res = mfrc522_set_value_clear_after_coll(&gs_handle, MFRC522_BOOL_FALSE);
        if (res != 0)
        {
            return 1;
        }
        
        return 0;
    }
    /* anti collision cl2 */
    else if ((in_len == 2) && (in_buf[0] == 0x95) && (in_buf[1] == 0x20))
    {
        /* disable cypto1 on */
        res = mfrc522_set_mifare_crypto1_on(&gs_handle, MFRC522_BOOL_FALSE);
        if (res != 0)
        {
            return 1;
        }
        
        /* set tx last bits 0 */
        res = mfrc522_set_tx_last_bits(&gs_handle, 0);
        if (res != 0)
        {
            return 1;
        }
        
        /* enable value clear after coll */
        res = mfrc522_set_value_clear_after_coll(&gs_handle, MFRC522_BOOL_TRUE);
        if (res != 0)
        {
            return 1;
        }
        
        /* transceiver */
        res = mfrc522_transceiver(&gs_handle, MFRC522_COMMAND_TRANSCEIVE, in_buf, in_len, out_buf, out_len, &err, 1000);
        if (res != 0)
        {
            return 1;
        }
        
        /* disable value clear after coll */
        res = mfrc522_set_value_clear_after_coll(&gs_handle, MFRC522_BOOL_FALSE);
        if (res != 0)
        {
            return 1;
        }
        
        return 0;
    }
    /* select cl1 */
    else if ((in_len == 9) && (in_buf[0] == 0x93) && (in_buf[1] == 0x70))
    {
        /* disable cypto1 on */
        res = mfrc522_set_mifare_crypto1_on(&gs_handle, MFRC522_BOOL_FALSE);
        if (res != 0)
        {
            return 1;
        }
        
        /* transceiver */
        res = mfrc522_transceiver(&gs_handle, MFRC522_COMMAND_TRANSCEIVE, in_buf, in_len, out_buf, out_len, &err, 1000);
        if (res != 0)
        {
            return 1;
        }
        
        return 0;
    }
    /* select cl2 */
    else if ((in_len == 9) && (in_buf[0] == 0x95) && (in_buf[1] == 0x70))
    {
        /* disable cypto1 on */
        res = mfrc522_set_mifare_crypto1_on(&gs_handle, MFRC522_BOOL_FALSE);
        if (res != 0)
        {
            return 1;
        }
        
        /* transceiver */
        res = mfrc522_transceiver(&gs_handle, MFRC522_COMMAND_TRANSCEIVE, in_buf, in_len, out_buf, out_len, &err, 1000);
        if (res != 0)
        {
            return 1;
        }
        
        return 0;
    }
    /* authentication key a or key b */
    else if ((in_len == 12) && ((in_buf[0] == 0x60) || (in_buf[0] == 0x61)))
    {
        uint8_t status;
        
        /* transceiver */
        res = mfrc522_transceiver(&gs_handle, MFRC522_COMMAND_MF_AUTHENT, in_buf, in_len, out_buf, out_len, &err, 1000);
        if (res != 0)
        {
            return 1;
        }
        
        /* get the mfcrypto1 on bit */
        res = mfrc522_get_status2(&gs_handle, &status);
        if (res != 0)
        {
            return 1;
        }
        
        /* check the result */
        if ((status & 0x08) == 0)
        {
            return 1;
        }
        
        return 0;
    }
    /* the others */
    else
    {
        /* transceiver */
        res = mfrc522_transceiver(&gs_handle, MFRC522_COMMAND_TRANSCEIVE, in_buf, in_len, out_buf, out_len, &err, 1000);
        if (res != 0)
        {
            return 1;
        }
        
        return 0;
    }
}

/**
 * @brief      basic example generate the random
 * @param[out] *buf pointer to a random buffer
 * @return     status code
 *             - 0 success
 *             - 1 generate random failed
 * @note       none
 */
uint8_t mfrc522_basic_generate_random(uint8_t buf[25])
{
    uint8_t res;
    uint8_t in_buf;
    uint8_t out_buf;
    uint8_t out_len;
    uint8_t err;
    
    /* generate the random */
    out_len = 0;
    res = mfrc522_transceiver(&gs_handle, MFRC522_COMMAND_RANDOM_ID, &in_buf, 0, &out_buf, &out_len, &err, 1000);
    if (res != 0)
    {
        return 1;
    }
    
    /* copy from mem to fifo buffer */
    out_len = 25;
    res = mfrc522_transceiver(&gs_handle, MFRC522_COMMAND_MEM, &in_buf, 0, buf, &out_len, &err, 1000);
    if (res != 0)
    {
        return 1;
    }
    
    return 0;
}

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
uint8_t mfrc522_basic_calculate_crc(uint8_t *buf, uint8_t len, uint16_t *crc)
{
    uint8_t res;
    uint8_t tmp;
    uint8_t tmp_len;
    uint8_t err;
    
    /* calculate the crc */
    tmp_len = 1;
    res = mfrc522_transceiver(&gs_handle, MFRC522_COMMAND_CALC_CRC, buf, len, &tmp, &tmp_len, &err, 1000);
    if (res != 0)
    {
        return 1;
    }
    
    /* get the crc */
    res = mfrc522_get_crc(&gs_handle, crc);
    if (res != 0)
    {
        return 1;
    }
    
    return 0;
}
