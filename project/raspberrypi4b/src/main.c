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
 * @file      main.c
 * @brief     main source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2021-07-25
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2021/07/25  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_mfrc522_register_test.h"
#include "driver_mfrc522_mifare_test.h"
#include "driver_mfrc522_basic.h"
#include "gpio.h"
#include <stdlib.h>

uint8_t g_flag;                            /**< interrupt flag */
uint8_t (*g_gpio_irq)(void) = NULL;        /**< gpio irq function address */

/**
 * @brief     interface receive callback
 * @param[in] type is the irq type
 * @note      none
 */
static void a_callback(uint16_t type)
{
    switch (type)
    {
        case MFRC522_INTERRUPT_MFIN_ACT :
        {
            mfrc522_interface_debug_print("mfrc522: irq mfin act.\n");
            
            break;
        }
        case MFRC522_INTERRUPT_CRC :
        {
            break;
        }
        case MFRC522_INTERRUPT_TX :
        {
            break;
        }
        case MFRC522_INTERRUPT_RX :
        {
            break;
        }
        case MFRC522_INTERRUPT_IDLE :
        {
            break;
        }
        case MFRC522_INTERRUPT_HI_ALERT :
        {
            break;
        }
        case MFRC522_INTERRUPT_LO_ALERT :
        {
            break;
        }
        case MFRC522_INTERRUPT_ERR :
        {
            mfrc522_interface_debug_print("mfrc522: irq err.\n");
            
            break;
        }
        case MFRC522_INTERRUPT_TIMER :
        {
            mfrc522_interface_debug_print("mfrc522: irq timer.\n");
            
            break;
        }
        default :
        {
            mfrc522_interface_debug_print("mfrc522: irq unknown code.\n");
            
            break;
        }
    }
}

/**
 * @brief     mfrc522 full function
 * @param[in] argc is arg numbers
 * @param[in] **argv is the arg address
 * @return    status code
 *             - 0 success
 *             - 1 run failed
 *             - 5 param is invalid
 * @note      none
 */
uint8_t mfrc522(uint8_t argc, char **argv)
{
    if (argc == 1)
    {
        goto help;
    }
    else if (argc == 2)
    {
        if (strcmp("-i", argv[1]) == 0)
        {
            mfrc522_info_t info;
            
            /* print mfrc522 info */
            mfrc522_info(&info);
            mfrc522_interface_debug_print("mfrc522: chip is %s.\n", info.chip_name);
            mfrc522_interface_debug_print("mfrc522: manufacturer is %s.\n", info.manufacturer_name);
            mfrc522_interface_debug_print("mfrc522: interface is %s.\n", info.interface);
            mfrc522_interface_debug_print("mfrc522: driver version is %d.%d.\n", info.driver_version / 1000, (info.driver_version % 1000) / 100);
            mfrc522_interface_debug_print("mfrc522: min supply voltage is %0.1fV.\n", info.supply_voltage_min_v);
            mfrc522_interface_debug_print("mfrc522: max supply voltage is %0.1fV.\n", info.supply_voltage_max_v);
            mfrc522_interface_debug_print("mfrc522: max current is %0.2fmA.\n", info.max_current_ma);
            mfrc522_interface_debug_print("mfrc522: max temperature is %0.1fC.\n", info.temperature_max);
            mfrc522_interface_debug_print("mfrc522: min temperature is %0.1fC.\n", info.temperature_min);
            
            return 0;
        }
        else if (strcmp("-p", argv[1]) == 0)
        {
            /* print pin connection */
            mfrc522_interface_debug_print("mfrc522: SPI interface SCK connected to GPIO11(BCM).\n");
            mfrc522_interface_debug_print("mfrc522: SPI interface MISO connected to GPIO9(BCM).\n");
            mfrc522_interface_debug_print("mfrc522: SPI interface MOSI connected to GPIO10(BCM).\n");
            mfrc522_interface_debug_print("mfrc522: SPI interface CS connected to GPIO8(BCM).\n");
            mfrc522_interface_debug_print("mfrc522: IIC interface SCL connected to GPIO3(BCM).\n");
            mfrc522_interface_debug_print("mfrc522: IIC interface SDA connected to GPIO2(BCM).\n");
            mfrc522_interface_debug_print("mfrc522: UART interface TX connected to GPIO14(BCM).\n");
            mfrc522_interface_debug_print("mfrc522: UART interface RX connected to GPIO15(BCM).\n");
            mfrc522_interface_debug_print("mfrc522: INT connected to GPIO17(BCM).\n");
            mfrc522_interface_debug_print("mfrc522: RESET connected to GPIO26(BCM).\n");
            
            return 0;
        }
        else if (strcmp("-h", argv[1]) == 0)
        {
            /* show mfrc522 help */
            help:
            
            mfrc522_interface_debug_print("mfrc522 -i\n\tshow mfrc522 chip and driver information.\n");
            mfrc522_interface_debug_print("mfrc522 -h\n\tshow mfrc522 help.\n");
            mfrc522_interface_debug_print("mfrc522 -p\n\tshow mfrc522 pin connections of the current board.\n");
            mfrc522_interface_debug_print("mfrc522 -t reg (-spi | -uart | -iic <addr>)\n\trun mfrc522 register test.addr is the iic address.\n");
            mfrc522_interface_debug_print("mfrc522 -t mifare (-spi | -uart | -iic <addr>)\n\trun mfrc522 mifare test.addr is the iic address.\n");
            mfrc522_interface_debug_print("mfrc522 -c mifare (-spi | -uart | -iic <addr>) <data>\n\trun mfrc522 mifare function."
                                          "addr is the iic address, data is the send data and data is hexadecimal.\n");
            mfrc522_interface_debug_print("mfrc522 -c crc (-spi | -uart | -iic <addr>) <data>\n\trun mfrc522 crc function."
                                          "addr is the iic address, data is the send data and data is hexadecimal.\n");
            mfrc522_interface_debug_print("mfrc522 -c random (-spi | -uart | -iic <addr>)\n\trun mfrc522 random function.addr is the iic address.\n");
            
            return 0;
        }
        else
        {
            return 5;
        }
    }
    else if (argc == 4)
    {
        if (strcmp("-t", argv[1]) == 0)
        {
            if (strcmp("reg", argv[2]) == 0)
            {
                if (strcmp("-spi", argv[3]) == 0)
                {
                    /* run register test */
                    if (mfrc522_register_test(MFRC522_INTERFACE_SPI, 0x00) != 0)
                    {
                        return 1;
                    }
                    else
                    {
                        return 0;
                    }
                }
                else if (strcmp("-uart", argv[3]) == 0)
                {
                    /* run register test */
                    if (mfrc522_register_test(MFRC522_INTERFACE_UART, 0x00) != 0)
                    {
                        return 1;
                    }
                    else
                    {
                        return 0;
                    }
                }
                else
                {
                    return 5;
                }
            }
            else if (strcmp("mifare", argv[2]) == 0)
            {
                uint8_t res;
                
                if (strcmp("-spi", argv[3]) == 0)
                {
                    /* run mifare test */
                    g_gpio_irq = mfrc522_mifare_test_irq_handler;
                    res = gpio_interrupt_init();
                    if (res != 0)
                    {
                        return 1;
                    }
                    if (mfrc522_mifare_test(MFRC522_INTERFACE_SPI, 0x00) != 0)
                    {
                        (void)gpio_interrupt_deinit();
                         g_gpio_irq = NULL;
                        
                        return 1;
                    }
                    
                    (void)gpio_interrupt_deinit();
                     g_gpio_irq = NULL;
                    
                    return 0;
                }
                else if (strcmp("-uart", argv[3]) == 0)
                {
                    /* run mifare test */
                    g_gpio_irq = mfrc522_mifare_test_irq_handler;
                    res = gpio_interrupt_init();
                    if (res != 0)
                    {
                        return 1;
                    }
                    if (mfrc522_mifare_test(MFRC522_INTERFACE_UART, 0x00) != 0)
                    {
                        (void)gpio_interrupt_deinit();
                         g_gpio_irq = NULL;
                        
                        return 1;
                    }
                    
                    (void)gpio_interrupt_deinit();
                     g_gpio_irq = NULL;
                    
                    return 0;
                }
                else
                {
                    return 5;
                }
            }
            else
            {
                return 5;
            }
        }
        else if (strcmp("-c", argv[1]) == 0)
        {
            uint8_t res;
            
            if (strcmp("random", argv[2]) == 0)
            {
                if (strcmp("-spi", argv[3]) == 0)
                {
                    uint8_t i;
                    uint8_t buf[25];
                    
                    /* get the random */
                    g_gpio_irq = mfrc522_interrupt_irq_handler;
                    res = gpio_interrupt_init();
                    if (res != 0)
                    {
                        return 1;
                    }
                    res = mfrc522_basic_init(MFRC522_INTERFACE_SPI, 0x00, a_callback);
                    if (res != 0)
                    {
                        (void)gpio_interrupt_deinit();
                         g_gpio_irq = NULL;
                        
                        return 1;
                    }
                    res = mfrc522_basic_generate_random(buf);
                    if (res != 0)
                    {
                        (void)gpio_interrupt_deinit();
                         g_gpio_irq = NULL;
                        
                        return 1;
                    }
                    for (i = 0; i < 25; i++)
                    {
                        mfrc522_interface_debug_print("0x%02X ", buf[i]);
                    }
                    mfrc522_interface_debug_print("\n");
                    
                    (void)mfrc522_basic_deinit();
                    (void)gpio_interrupt_deinit();
                    g_gpio_irq = NULL;
                     
                    return 0;
                }
                else if (strcmp("-uart", argv[3]) == 0)
                {
                    uint8_t i;
                    uint8_t buf[25];
                    
                    /* get the random */
                    g_gpio_irq = mfrc522_interrupt_irq_handler;
                    res = gpio_interrupt_init();
                    if (res != 0)
                    {
                        return 1;
                    }
                    res = mfrc522_basic_init(MFRC522_INTERFACE_UART, 0x00, a_callback);
                    if (res != 0)
                    {
                        (void)gpio_interrupt_deinit();
                         g_gpio_irq = NULL;
                        
                        return 1;
                    }
                    res = mfrc522_basic_generate_random(buf);
                    if (res != 0)
                    {
                        (void)gpio_interrupt_deinit();
                         g_gpio_irq = NULL;
                        
                        return 1;
                    }
                    for (i = 0; i < 25; i++)
                    {
                        mfrc522_interface_debug_print("0x%02X ", buf[i]);
                    }
                    mfrc522_interface_debug_print("\n");
                    
                    (void)mfrc522_basic_deinit();
                    (void)gpio_interrupt_deinit();
                    g_gpio_irq = NULL;
                     
                    return 0;
                }
                else
                {
                    return 5;
                }
            }
            else
            {
                return 5;
            }
        }
        else
        {
            return 5;
        }
    }
    else if (argc == 5)
    {
        if (strcmp("-t", argv[1]) == 0)
        {
            if (strcmp("reg", argv[2]) == 0)
            {
                if (strcmp("-iic", argv[3]) == 0)
                {
                    /* run register test */
                    if (mfrc522_register_test(MFRC522_INTERFACE_IIC, (uint8_t)atoi(argv[4])) != 0)
                    {
                        return 1;
                    }
                    else
                    {
                        return 0;
                    }
                }
                else
                {
                    return 5;
                }
            }
            else if (strcmp("mifare", argv[2]) == 0)
            {
                uint8_t res;
                
                if (strcmp("-iic", argv[3]) == 0)
                {
                    /* run mifare test */
                    g_gpio_irq = mfrc522_mifare_test_irq_handler;
                    res = gpio_interrupt_init();
                    if (res != 0)
                    {
                        return 1;
                    }
                    if (mfrc522_mifare_test(MFRC522_INTERFACE_IIC, (uint8_t)atoi(argv[4])) != 0)
                    {
                        (void)gpio_interrupt_deinit();
                         g_gpio_irq = NULL;
                        
                        return 1;
                    }
                    
                    (void)gpio_interrupt_deinit();
                     g_gpio_irq = NULL;
                    
                    return 0;
                }
                else
                {
                    return 5;
                }
            }
            else
            {
                return 5;
            }
        }
        else if (strcmp("-c", argv[1]) == 0)
        {
            uint8_t res;
            
            if (strcmp("random", argv[2]) == 0)
            {
                if (strcmp("-iic", argv[3]) == 0)
                {
                    uint8_t i;
                    uint8_t buf[25];
                    
                    /* get the random */
                    g_gpio_irq = mfrc522_interrupt_irq_handler;
                    res = gpio_interrupt_init();
                    if (res != 0)
                    {
                        return 1;
                    }
                    res = mfrc522_basic_init(MFRC522_INTERFACE_IIC, (uint8_t)atoi(argv[4]), a_callback);
                    if (res != 0)
                    {
                        (void)gpio_interrupt_deinit();
                         g_gpio_irq = NULL;
                        
                        return 1;
                    }
                    res = mfrc522_basic_generate_random(buf);
                    if (res != 0)
                    {
                        (void)gpio_interrupt_deinit();
                         g_gpio_irq = NULL;
                        
                        return 1;
                    }
                    for (i = 0; i < 25; i++)
                    {
                        mfrc522_interface_debug_print("0x%02X ", buf[i]);
                    }
                    mfrc522_interface_debug_print("\n");
                    
                    (void)mfrc522_basic_deinit();
                    (void)gpio_interrupt_deinit();
                    g_gpio_irq = NULL;
                     
                    return 0;
                }
                else
                {
                    return 5;
                }
            }
            else if (strcmp("crc", argv[2]) == 0)
            {
                if (strcmp("-spi", argv[3]) == 0)
                {
                    uint16_t crc;
                    
                    /* run the crc */
                    g_gpio_irq = mfrc522_interrupt_irq_handler;
                    res = gpio_interrupt_init();
                    if (res != 0)
                    {
                        return 1;
                    }
                    res = mfrc522_basic_init(MFRC522_INTERFACE_SPI, 0x00, a_callback);
                    if (res != 0)
                    {
                        (void)gpio_interrupt_deinit();
                         g_gpio_irq = NULL;
                        
                        return 1;
                    }
                    res = mfrc522_basic_calculate_crc((uint8_t *)argv[4], (uint8_t)strlen(argv[4]), &crc);
                    if (res != 0)
                    {
                        (void)gpio_interrupt_deinit();
                         g_gpio_irq = NULL;
                        
                        return 1;
                    }
                    mfrc522_interface_debug_print("%s crc is 0x%04X.\n", argv[4], crc);
                    
                    (void)mfrc522_basic_deinit();
                    (void)gpio_interrupt_deinit();
                    g_gpio_irq = NULL;
                     
                    return 0;
                }
                else if (strcmp("-uart", argv[3]) == 0)
                {
                    uint16_t crc;
                    
                    /* run the crc */
                    g_gpio_irq = mfrc522_interrupt_irq_handler;
                    res = gpio_interrupt_init();
                    if (res != 0)
                    {
                        return 1;
                    }
                    res = mfrc522_basic_init(MFRC522_INTERFACE_UART, 0x00, a_callback);
                    if (res != 0)
                    {
                        (void)gpio_interrupt_deinit();
                         g_gpio_irq = NULL;
                        
                        return 1;
                    }
                    res = mfrc522_basic_calculate_crc((uint8_t *)argv[4], (uint8_t)strlen(argv[4]), &crc);
                    if (res != 0)
                    {
                        (void)gpio_interrupt_deinit();
                         g_gpio_irq = NULL;
                        
                        return 1;
                    }
                    mfrc522_interface_debug_print("%s crc is 0x%04X.\n", argv[4], crc);
                    
                    (void)mfrc522_basic_deinit();
                    (void)gpio_interrupt_deinit();
                    g_gpio_irq = NULL;
                     
                    return 0;
                }
                else
                {
                    return 5;
                }
            }
            else if (strcmp("mifare", argv[2]) == 0)
            {
                uint8_t i;
                uint8_t in_buf[64];
                uint8_t in_len;
                uint8_t out_len;
                uint8_t out_buf[64];
                uint16_t l;
                
                if (strcmp("-spi", argv[3]) == 0)
                {
                    /* sent the mifare command */
                    g_gpio_irq = mfrc522_interrupt_irq_handler;
                    res = gpio_interrupt_init();
                    if (res != 0)
                    {
                        return 1;
                    }
                    res = mfrc522_basic_init(MFRC522_INTERFACE_SPI, 0x00, a_callback);
                    if (res != 0)
                    {
                        (void)gpio_interrupt_deinit();
                         g_gpio_irq = NULL;
                        
                        return 1;
                    }
                    l = (uint16_t)strlen(argv[4]);
                    for (i = 0; (i < 64) & (l != 0); i++)
                    {
                        if ((int16_t)(l - 2) < 0)
                        {
                            break;
                        }
                        in_buf[i] = (argv[4][i * 2 + 0] - '0') * 16 + (argv[4][i * 2 + 1] - '0'); 
                        l -= 2;
                    }
                    in_len = i;
                    out_len = 64;
                    res = mfrc522_basic_transceiver(in_buf, in_len, out_buf, &out_len);
                    if (res != 0)
                    {
                        (void)gpio_interrupt_deinit();
                         g_gpio_irq = NULL;
                        
                        return 1;
                    }
                    for (i = 0; i < out_len; i++)
                    {
                        mfrc522_interface_debug_print("0x%02X ", out_buf[i]);
                    }
                    mfrc522_interface_debug_print("\n");
                    
                    (void)mfrc522_basic_deinit();
                    (void)gpio_interrupt_deinit();
                    g_gpio_irq = NULL;
                     
                    return 0;
                }
                if (strcmp("-uart", argv[3]) == 0)
                {
                    /* sent the mifare command */
                    g_gpio_irq = mfrc522_interrupt_irq_handler;
                    res = gpio_interrupt_init();
                    if (res != 0)
                    {
                        return 1;
                    }
                    res = mfrc522_basic_init(MFRC522_INTERFACE_UART, 0x00, a_callback);
                    if (res != 0)
                    {
                        (void)gpio_interrupt_deinit();
                         g_gpio_irq = NULL;
                        
                        return 1;
                    }
                    l = (uint16_t)strlen(argv[4]);
                    for (i = 0; (i < 64) & (l != 0); i++)
                    {
                        if ((int16_t)(l - 2) < 0)
                        {
                            break;
                        }
                        in_buf[i] = (argv[4][i * 2 + 0] - '0') * 16 + (argv[4][i * 2 + 1] - '0'); 
                        l -= 2;
                    }
                    in_len = i;
                    out_len = 64;
                    res = mfrc522_basic_transceiver(in_buf, in_len, out_buf, &out_len);
                    if (res != 0)
                    {
                        (void)gpio_interrupt_deinit();
                         g_gpio_irq = NULL;
                        
                        return 1;
                    }
                    for (i = 0; i < out_len; i++)
                    {
                        mfrc522_interface_debug_print("0x%02X ", out_buf[i]);
                    }
                    mfrc522_interface_debug_print("\n");
                    
                    (void)mfrc522_basic_deinit();
                    (void)gpio_interrupt_deinit();
                    g_gpio_irq = NULL;
                     
                    return 0;
                }
                else
                {
                    return 5;
                }
            }
            else
            {
                return 5;
            }
        }
        else
        {
            return 5;
        }
    }
    else if (argc == 6)
    {
        if (strcmp("-c", argv[1]) == 0)
        {
            uint8_t res;
            
            /* sent the mifare command */
            if (strcmp("mifare", argv[2]) == 0)
            {
                uint8_t i;
                uint8_t in_buf[64];
                uint8_t in_len;
                uint8_t out_len;
                uint8_t out_buf[64];
                uint16_t l;
                
                if (strcmp("-iic", argv[3]) == 0)
                {
                    g_gpio_irq = mfrc522_interrupt_irq_handler;
                    res = gpio_interrupt_init();
                    if (res != 0)
                    {
                        return 1;
                    }
                    res = mfrc522_basic_init(MFRC522_INTERFACE_IIC, (uint8_t)atoi(argv[4]), a_callback);
                    if (res != 0)
                    {
                        (void)gpio_interrupt_deinit();
                         g_gpio_irq = NULL;
                        
                        return 1;
                    }
                    l = (uint16_t)strlen(argv[5]);
                    for (i = 0; (i < 64) & (l != 0); i++)
                    {
                        if ((int16_t)(l - 2) < 0)
                        {
                            break;
                        }
                        in_buf[i] = (argv[5][i * 2 + 0] - '0') * 16 + (argv[5][i * 2 + 1] - '0'); 
                        l -= 2;
                    }
                    in_len = i;
                    out_len = 64;
                    res = mfrc522_basic_transceiver(in_buf, in_len, out_buf, &out_len);
                    if (res != 0)
                    {
                        (void)gpio_interrupt_deinit();
                         g_gpio_irq = NULL;
                        
                        return 1;
                    }
                    for (i = 0; i < out_len; i++)
                    {
                        mfrc522_interface_debug_print("0x%02X ", out_buf[i]);
                    }
                    mfrc522_interface_debug_print("\n");
                    
                    (void)mfrc522_basic_deinit();
                    (void)gpio_interrupt_deinit();
                    g_gpio_irq = NULL;
                     
                    return 0;
                }
                else
                {
                    return 5;
                }
            }
            else if (strcmp("crc", argv[2]) == 0)
            {
                if (strcmp("-iic", argv[3]) == 0)
                {
                    uint16_t crc;
                    
                    /* run the crc */
                    g_gpio_irq = mfrc522_interrupt_irq_handler;
                    res = gpio_interrupt_init();
                    if (res != 0)
                    {
                        return 1;
                    }
                    res = mfrc522_basic_init(MFRC522_INTERFACE_IIC, (uint8_t)atoi(argv[4]), a_callback);
                    if (res != 0)
                    {
                        (void)gpio_interrupt_deinit();
                         g_gpio_irq = NULL;
                        
                        return 1;
                    }
                    res = mfrc522_basic_calculate_crc((uint8_t *)argv[5], (uint8_t)strlen(argv[5]), &crc);
                    if (res != 0)
                    {
                        (void)gpio_interrupt_deinit();
                         g_gpio_irq = NULL;
                        
                        return 1;
                    }
                    mfrc522_interface_debug_print("%s crc is 0x%04X.\n", argv[4], crc);
                    
                    (void)mfrc522_basic_deinit();
                    (void)gpio_interrupt_deinit();
                    g_gpio_irq = NULL;
                     
                    return 0;
                }
                else
                {
                    return 5;
                }
            }
            else
            {
                return 5;
            }
        }
        else
        {
            return 5;
        }
    }
    /* param is invalid */
    else
    {
        return 5;
    }
}

/**
 * @brief     main function
 * @param[in] argc is arg numbers
 * @param[in] **argv is the arg address
 * @return    status code
 *             - 0 success
 * @note      none
 */
int main(uint8_t argc, char **argv)
{
    uint8_t res;

    res = mfrc522(argc, argv);
    if (res == 0)
    {
        /* run success */
    }
    else if (res == 1)
    {
        mfrc522_interface_debug_print("mfrc522: run failed.\n");
    }
    else if (res == 5)
    {
        mfrc522_interface_debug_print("mfrc522: param is invalid.\n");
    }
    else
    {
        mfrc522_interface_debug_print("mfrc522: unknow status code.\n");
    }

    return 0;
}
