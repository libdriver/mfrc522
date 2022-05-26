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
 * @file      driver_mfrc522_mifare_test.c
 * @brief     driver mfrc522 mifare test source file
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

#include "driver_mfrc522_mifare_test.h"
#include <stdlib.h>

static mfrc522_handle_t gs_handle;        /**< mfrc522 handle */

/**
 * @brief  mifare test irq
 * @return status code
 *         - 0 success
 *         - 1 run failed
 * @note   none
 */
uint8_t mfrc522_mifare_test_irq_handler(void)
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
 * @brief     mifare test
 * @param[in] interface is the bus interface
 * @param[in] addr is the iic device address
 * @return    status code
 *            - 0 success
 *            - 1 test failed
 * @note      none
 */
uint8_t mfrc522_mifare_test(mfrc522_interface_t interface, uint8_t addr)
{
    uint8_t res;
    uint8_t i;
    uint8_t err;
    uint8_t out_len;
    uint8_t number[4];
    uint8_t in_buf[16];
    uint8_t out_buf[64];
    uint16_t crc16;
    uint16_t type;
    mfrc522_info_t info;
    
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
    
    /* start mifare test */
    mfrc522_interface_debug_print("mfrc522: start mifare test.\n");
    
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
    
    /* set the invert */
    res = mfrc522_set_interrupt1_pin_invert(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* standard cmos */
    res = mfrc522_set_interrupt_pin_type(&gs_handle, MFRC522_INTERRUPT_PIN_TYPE_STANDARD_CMOS);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* disable iic high speed */
    res = mfrc522_set_force_iic_high_speed(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* disable clear temperature error */
    res = mfrc522_set_clear_temperature_error(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set clear temperature error failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set water level 8 */
    res = mfrc522_set_water_level(&gs_handle, 8);
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
    
    /* set rx align 0 */
    res = mfrc522_set_rx_align(&gs_handle, MFRC522_RX_ALIGN_0);
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
    
    /* disable tx crc generation */
    res = mfrc522_set_tx_crc_generation(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx crc generation failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* tx 106 kBd */
    res = mfrc522_set_tx_speed(&gs_handle, MFRC522_SPEED_106_KBD);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* disable modulation invert */
    res = mfrc522_set_modulation_invert(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set modulation invert failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* disable rx crc generation */
    res = mfrc522_set_rx_crc_generation(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx crc generation failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* rx 106 kBd */
    res = mfrc522_set_rx_speed(&gs_handle, MFRC522_SPEED_106_KBD);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* enable rx no error */
    res = mfrc522_set_rx_no_error(&gs_handle, MFRC522_BOOL_TRUE);
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
    
    /* disable rx multiple */
    res = mfrc522_set_rx_multiple(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx multiple failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* tx input internal encoder */
    res = mfrc522_set_tx_input(&gs_handle, MFRC522_TX_INPUT_INTERNAL_ENCODER);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* 3 state */
    res = mfrc522_set_mfout_input(&gs_handle, MFRC522_MFOUT_INPUT_3_STATE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set mfout input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set min level 8 */
    res = mfrc522_set_min_level(&gs_handle, 0x8);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set min level failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set collision level 4 */
    res = mfrc522_set_collision_level(&gs_handle, 0x4);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set collision level failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the param */
    res = mfrc522_set_channel_reception(&gs_handle, MFRC522_CHANNEL_RECEPTION_STRONGER_FREEZE_SELECTED);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set channel reception failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* disable fix iq */
    res = mfrc522_set_fix_iq(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set fix iq failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* disable timer prescal even */
    res = mfrc522_set_timer_prescal_even(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer prescal even failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set timer constant reception 3 */
    res = mfrc522_set_timer_constant_reception(&gs_handle, 0x3);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer constant reception failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set timer constant sync 1 */
    res = mfrc522_set_timer_constant_sync(&gs_handle, 0x1);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer constant sync failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set tx wait 2 */
    res = mfrc522_set_tx_wait(&gs_handle, 0x2);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx wait failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* disable */
    res = mfrc522_set_parity_disable(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set parity disable failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* 9600 */
    res = mfrc522_set_serial_speed(&gs_handle, 0x07, 0x0B);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set serial speed failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set modulation width 0x26 */
    res = mfrc522_set_modulation_width(&gs_handle, 0x26);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set modulation width failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set cwgsn 8 */
    res = mfrc522_set_cwgsn(&gs_handle, 0x8);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set cwgsn failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set modgsn 8 */
    res = mfrc522_set_modgsn(&gs_handle, 0x8);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set modgsn failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set cwgsp 0x20 */
    res = mfrc522_set_cwgsp(&gs_handle, 0x20);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set cwgsp failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set modgsp 0x20 */
    res = mfrc522_set_modgsp(&gs_handle, 0x20);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set modgsp failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* tx disable */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_TX, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* rx enable */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_RX, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* idle enable */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_IDLE, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* hi alert disable */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_HI_ALERT, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* lo alert disable */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_LO_ALERT, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* err enable */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_ERR, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* timer enable */
    res = mfrc522_set_interrupt1(&gs_handle, MFRC522_INTERRUPT1_TIMER, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* mfin act disable */
    res = mfrc522_set_interrupt2(&gs_handle, MFRC522_INTERRUPT2_MFIN_ACT, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* crc enable */
    res = mfrc522_set_interrupt2(&gs_handle, MFRC522_INTERRUPT2_CRC, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set interrupt1 failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* disable crc msb first */
    res = mfrc522_set_crc_msb_first(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set crc msb first failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* enable tx wait rf */
    res = mfrc522_set_tx_wait_rf(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx wait rf failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set mfin polarity high */
    res = mfrc522_set_mfin_polarity(&gs_handle, MFRC522_MFIN_POLARITY_HIGH);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx wait rf failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* preset 0x6363 */
    res = mfrc522_set_crc_preset(&gs_handle, MFRC522_CRC_PRESET_6363);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set crc preset failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* enable */
    res = mfrc522_set_force_100_ask(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set force 100 ask failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set analog module */
    res = mfrc522_set_contactless_uart_input(&gs_handle, MFRC522_CONTACTLESS_UART_INTERNAL_ANALOG_MODULE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set contactless uart input failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set rx wait */
    res = mfrc522_set_rx_wait(&gs_handle, 0x6);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx wait failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set 48 db */
    res = mfrc522_set_rx_gain(&gs_handle, MFRC522_RX_GAIN_48_DB);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set rx gain failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* enable timer auto */
    res = mfrc522_set_timer_auto(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set auto failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set gated mode none */
    res = mfrc522_set_timer_gated_mode(&gs_handle, MFRC522_TIMER_GATED_MODE_NONE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer gated mode failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* disable auto restart */
    res = mfrc522_set_timer_auto_restart(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer auto restart failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the timer prescaler */
    res = mfrc522_set_timer_prescaler(&gs_handle, 0xD3E);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set timer prescaler failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set the reload */
    res = mfrc522_set_timer_reload(&gs_handle, 0x001E);
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
    
    /* mifare random test */
    mfrc522_interface_debug_print("mfrc522: mifare random test.\n");
    
    /* generate the random */
    out_len = 0;
    res = mfrc522_transceiver(&gs_handle, MFRC522_COMMAND_RANDOM_ID, in_buf, 0, out_buf, &out_len, &err, 1000);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: transceiver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* copy from mem to fifo to buffer */
    out_len = 64;
    res = mfrc522_transceiver(&gs_handle, MFRC522_COMMAND_MEM, in_buf, 0, out_buf, &out_len, &err, 1000);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: transceiver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    for (i = 0; i < out_len; i++)
    {
        mfrc522_interface_debug_print("0x%02X ", out_buf[i]);
    }
    mfrc522_interface_debug_print("\n");
    
    /* mifare crc test */
    mfrc522_interface_debug_print("mfrc522: mifare crc test.\n");
    out_len = 0;
    in_buf[0] = 'm';
    in_buf[1] = 'f';
    in_buf[2] = 'r';
    in_buf[3] = 'c';
    in_buf[4] = '5';
    in_buf[5] = '2';
    in_buf[6] = '2';
    res = mfrc522_transceiver(&gs_handle, MFRC522_COMMAND_CALC_CRC, in_buf, 7, out_buf, &out_len, &err, 1000);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: transceiver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    res = mfrc522_get_crc(&gs_handle, &crc16);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: get crc failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
     mfrc522_interface_debug_print("mfrc522: mfrc522 crc is 0x%04X and checked %s.\n", crc16, 
                                    crc16 == 0x0564 ? "ok" : "error");
    
    /* mifare find card test */
    mfrc522_interface_debug_print("mfrc522: mifare find card test.\n");
    
    /* disable cypto1 on */
    res = mfrc522_set_mifare_crypto1_on(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set mifare crypto1 on failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set tx last bits 7 */
    res = mfrc522_set_tx_last_bits(&gs_handle, 7);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx last bits failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* mifare find card */
    out_len = 64;
    in_buf[0] = 0x26;
    res = mfrc522_transceiver(&gs_handle, MFRC522_COMMAND_TRANSCEIVE, in_buf, 1, out_buf, &out_len, &err, 1000);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: transceiver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    type = (uint16_t)out_buf[0] << 8 | out_buf[1];
    if (type == 0x4400)
    {
        mfrc522_interface_debug_print("mfrc522: find mifare ultralight.\n");
    }
    else if (type == 0x0400)
    {
        mfrc522_interface_debug_print("mfrc522: find mifare S50.\n");
    }
    else if (type == 0x0200)
    {
        mfrc522_interface_debug_print("mfrc522: find mifare S70.\n");
    }
    else if (type == 0x0800)
    {
        mfrc522_interface_debug_print("mfrc522: find mifare pro(x).\n");
    }
    else if (type == 0x4403)
    {
        mfrc522_interface_debug_print("mfrc522: find mifare desfire.\n");
    }
    else
    {
        mfrc522_interface_debug_print("mfrc522: unknown type.\n");
    }
    
    /* mifare anticoll test */
    mfrc522_interface_debug_print("mfrc522: mifare anticoll test.\n");
    
    /* disable cypto1 on */
    res = mfrc522_set_mifare_crypto1_on(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set mifare crypto1 on failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set tx last bits 0 */
    res = mfrc522_set_tx_last_bits(&gs_handle, 0);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set tx last bits failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* enable value clear after coll */
    res = mfrc522_set_value_clear_after_coll(&gs_handle, MFRC522_BOOL_TRUE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set value clear after coll failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* mifare anticoll */
    out_len = 64;
    in_buf[0] = 0x93;
    in_buf[1] = 0x20;
    res = mfrc522_transceiver(&gs_handle, MFRC522_COMMAND_TRANSCEIVE, in_buf, 2, out_buf, &out_len, &err, 1000);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: transceiver failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    number[0] = out_buf[0];
    number[1] = out_buf[1];
    number[2] = out_buf[2];
    number[3] = out_buf[3];
    err = 0;
    err ^= out_buf[0];
    err ^= out_buf[1];
    err ^= out_buf[2];
    err ^= out_buf[3];
    if (err == out_buf[4])
    {
        mfrc522_interface_debug_print("mfrc522: id is 0x%02X 0x%02X 0x%02X 0x%02X.\n",
                                      number[0], number[1], number[2], number[3]);
    }
    else
    {
        mfrc522_interface_debug_print("mfrc522: id check failed.\n");
    }
    
    /* disable value clear after coll */
    res = mfrc522_set_value_clear_after_coll(&gs_handle, MFRC522_BOOL_FALSE);
    if (res != 0)
    {
        mfrc522_interface_debug_print("mfrc522: set value clear after coll failed.\n");
        (void)mfrc522_deinit(&gs_handle);
        
        return 1;
    }
    
    /* finish mifare test */
    mfrc522_interface_debug_print("mfrc522: finish mifare test.\n");
    (void)mfrc522_deinit(&gs_handle);
    
    return 0;
}
