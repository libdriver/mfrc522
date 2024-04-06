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
#include <getopt.h>
#include <math.h>
#include <stdlib.h>

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
    int c;
    int longindex = 0;
    const char short_options[] = "hipe:t:";
    const struct option long_options[] =
    {
        {"help", no_argument, NULL, 'h'},
        {"information", no_argument, NULL, 'i'},
        {"port", no_argument, NULL, 'p'},
        {"example", required_argument, NULL, 'e'},
        {"test", required_argument, NULL, 't'},
        {"addr", required_argument, NULL, 1},
        {"data", required_argument, NULL, 2},
        {"interface", required_argument, NULL, 3},
        {"str", required_argument, NULL, 4},
        {NULL, 0, NULL, 0},
    };
    char type[33] = "unknown";
    uint8_t addr = 0x00;
    uint64_t data = 0x00;
    uint8_t data_flag = 0;
    uint8_t str_flag = 0;
    mfrc522_interface_t interface = MFRC522_INTERFACE_SPI;
    char str[49] = {0};
    
    /* if no params */
    if (argc == 1)
    {
        /* goto the help */
        goto help;
    }
    
    /* init 0 */
    optind = 0;
    
    /* parse */
    do
    {
        /* parse the args */
        c = getopt_long(argc, argv, short_options, long_options, &longindex);
        
        /* judge the result */
        switch (c)
        {
            /* help */
            case 'h' :
            {
                /* set the type */
                memset(type, 0, sizeof(char) * 33);
                snprintf(type, 32, "h");
                
                break;
            }
            
            /* information */
            case 'i' :
            {
                /* set the type */
                memset(type, 0, sizeof(char) * 33);
                snprintf(type, 32, "i");
                
                break;
            }
            
            /* port */
            case 'p' :
            {
                /* set the type */
                memset(type, 0, sizeof(char) * 33);
                snprintf(type, 32, "p");
                
                break;
            }
            
            /* example */
            case 'e' :
            {
                /* set the type */
                memset(type, 0, sizeof(char) * 33);
                snprintf(type, 32, "e_%s", optarg);
                
                break;
            }
            
            /* test */
            case 't' :
            {
                /* set the type */
                memset(type, 0, sizeof(char) * 33);
                snprintf(type, 32, "t_%s", optarg);
                
                break;
            }
            
            /* addr */
            case 1 :
            {
                /* set the addr pin */
                addr = atol(optarg);
                
                break;
            }
            
            /* data */
            case 2 :
            {
                char *p;
                uint16_t l;
                uint16_t i;
                uint64_t hex_data;

                /* set the data */
                l = strlen(optarg);

                /* check the header */
                if (l >= 2)
                {
                    if (strncmp(optarg, "0x", 2) == 0)
                    {
                        p = optarg + 2;
                        l -= 2;
                    }
                    else if (strncmp(optarg, "0X", 2) == 0)
                    {
                        p = optarg + 2;
                        l -= 2;
                    }
                    else
                    {
                        p = optarg;
                    }
                }
                else
                {
                    p = optarg;
                }
                
                /* init 0 */
                hex_data = 0;

                /* loop */
                for (i = 0; i < l; i++)
                {
                    if ((p[i] <= '9') && (p[i] >= '0'))
                    {
                        hex_data += (p[i] - '0') * (uint32_t)pow(16, l - i - 1);
                    }
                    else if ((p[i] <= 'F') && (p[i] >= 'A'))
                    {
                        hex_data += ((p[i] - 'A') + 10) * (uint32_t)pow(16, l - i - 1);
                    }
                    else if ((p[i] <= 'f') && (p[i] >= 'a'))
                    {
                        hex_data += ((p[i] - 'a') + 10) * (uint32_t)pow(16, l - i - 1);
                    }
                    else
                    {
                        return 5;
                    }
                }
                
                /* copy the data */
                data = hex_data;
                data_flag = 1;

                break;
            }
            
            /* interface */
            case 3 :
            {
                /* set the interface */
                if (strcmp("spi", optarg) == 0)
                {
                    interface = MFRC522_INTERFACE_SPI;
                }
                else if (strcmp("iic", optarg) == 0)
                {
                    interface = MFRC522_INTERFACE_IIC;
                }
                else if (strcmp("uart", optarg) == 0)
                {
                    interface = MFRC522_INTERFACE_UART;
                }
                else
                {
                    return 5;
                }
                
                break;
            }
            
            /* str */
            case 4 :
            {
                /* copy the string */
                memset(str, 0, sizeof(char) * 49);
                snprintf(str, 48, "%s", optarg);
                str_flag = 1;
            }

            /* the end */
            case -1 :
            {
                break;
            }
            
            /* others */
            default :
            {
                return 5;
            }
        }
    } while (c != -1);

    /* run the function */
    if (strcmp("t_reg", type) == 0)
    {
        /* run reg test */
        if (mfrc522_register_test(interface, addr) != 0)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else if (strcmp("t_mifare", type) == 0)
    {
        uint8_t res;

        /* set gpio irq */
        g_gpio_irq = mfrc522_mifare_test_irq_handler;
        
        /* gpio init */
        res = gpio_interrupt_init();
        if (res != 0)
        {
            return 1;
        }
        
        /* run mifare test */
        if (mfrc522_mifare_test(interface, addr) != 0)
        {
            (void)gpio_interrupt_deinit();
            g_gpio_irq = NULL;
            
            return 1;
        }
        
        /* gpio deinit */
        (void)gpio_interrupt_deinit();
        g_gpio_irq = NULL;
        
        return 0;
    }
    else if (strcmp("e_mifare", type) == 0)
    {
        uint8_t res;
        uint16_t i;
        uint8_t in_buf[64];
        uint8_t in_len;
        uint8_t out_len;
        uint8_t out_buf[64];
        uint16_t l;
        
        /* check the flag */
        if (data_flag != 1)
        {
            return 5;
        }

        /* set gpio irq */
        g_gpio_irq = mfrc522_interrupt_irq_handler;
        
        /* gpio init */
        res = gpio_interrupt_init();
        if (res != 0)
        {
            return 1;
        }
        
        /* basic init */
        res = mfrc522_basic_init(interface, addr, a_callback);
        if (res != 0)
        {
            (void)gpio_interrupt_deinit();
            g_gpio_irq = NULL;
            
            return 1;
        }
        
        /* copy data */
        memset(in_buf, 0, sizeof(char) * 64);
        snprintf((char *)in_buf, 63, "%llx", data);
        l = strlen((char *)in_buf) / 2;
        for (i = 0; i < l; i++)
        {
            in_buf[i] = (data >> (8 * (l - i - 1))) & 0xFF;
        }
        in_len = l;
        out_len = 64;
        
        /* transceiver */
        res = mfrc522_basic_transceiver(in_buf, in_len, out_buf, &out_len);
        if (res != 0)
        {
            (void)gpio_interrupt_deinit();
            g_gpio_irq = NULL;
            
            return 1;
        }
        
        /* output */
        for (i = 0; i < out_len; i++)
        {
            mfrc522_interface_debug_print("0x%02X ", out_buf[i]);
        }
        mfrc522_interface_debug_print("\n");
        
        /* basic deint */
        (void)mfrc522_basic_deinit();
        (void)gpio_interrupt_deinit();
        g_gpio_irq = NULL;
        
        return 0;
    }
    else if (strcmp("e_crc", type) == 0)
    {
        uint8_t res;
        uint16_t crc;
        
        /* check the flag */
        if (str_flag != 1)
        {
            return 5;
        }

        /* set gpio irq */
        g_gpio_irq = mfrc522_interrupt_irq_handler;
        
        /* gpio init */
        res = gpio_interrupt_init();
        if (res != 0)
        {
            return 1;
        }
        
        /* basic init */
        res = mfrc522_basic_init(interface, addr, a_callback);
        if (res != 0)
        {
            (void)gpio_interrupt_deinit();
            g_gpio_irq = NULL;
            
            return 1;
        }
        
        /* calculate crc */
        res = mfrc522_basic_calculate_crc((uint8_t *)str, (uint8_t)strlen(str), &crc);
        if (res != 0)
        {
            (void)gpio_interrupt_deinit();
            g_gpio_irq = NULL;
            
            return 1;
        }
        
        /* output */
        mfrc522_interface_debug_print("%s crc is 0x%04X.\n", str, crc);
        
        /* basic deinit */
        (void)mfrc522_basic_deinit();
        (void)gpio_interrupt_deinit();
        g_gpio_irq = NULL;
        
        return 0;
    }
    else if (strcmp("e_random", type) == 0)
    {
        uint8_t res;
        uint8_t i;
        uint8_t buf[25];
        
        /* set gpio irq */
        g_gpio_irq = mfrc522_interrupt_irq_handler;
        
        /* gpio init */
        res = gpio_interrupt_init();
        if (res != 0)
        {
            return 1;
        }
        
        /* basic int */
        res = mfrc522_basic_init(interface, addr, a_callback);
        if (res != 0)
        {
            (void)gpio_interrupt_deinit();
            g_gpio_irq = NULL;
            
            return 1;
        }
        
        /* get the random */
        res = mfrc522_basic_generate_random(buf);
        if (res != 0)
        {
            (void)gpio_interrupt_deinit();
            g_gpio_irq = NULL;
            
            return 1;
        }
        
        /* output */
        for (i = 0; i < 25; i++)
        {
            mfrc522_interface_debug_print("0x%02X ", buf[i]);
        }
        mfrc522_interface_debug_print("\n");
        
        /* basic deinit */
        (void)mfrc522_basic_deinit();
        (void)gpio_interrupt_deinit();
        g_gpio_irq = NULL;
        
        return 0;
    }
    else if (strcmp("h", type) == 0)
    {
        help:
        mfrc522_interface_debug_print("Usage:\n");
        mfrc522_interface_debug_print("  mfrc522 (-i | --information)\n");
        mfrc522_interface_debug_print("  mfrc522 (-h | --help)\n");
        mfrc522_interface_debug_print("  mfrc522 (-p | --port)\n");
        mfrc522_interface_debug_print("  mfrc522 (-t reg | --test=reg) [--interface=<spi | iic | uart>] [--addr=<address>]\n");
        mfrc522_interface_debug_print("  mfrc522 (-t mifare | --test=mifare) [--interface=<spi | iic | uart>] [--addr=<address>]\n");
        mfrc522_interface_debug_print("  mfrc522 (-e mifare | --example=mifare) [--interface=<spi | iic | uart>] [--addr=<address>] --data=<hex>\n");
        mfrc522_interface_debug_print("  mfrc522 (-e crc | --example=crc) [--interface=<spi | iic | uart>] [--addr=<address>] --str=<string>\n");
        mfrc522_interface_debug_print("  mfrc522 (-e random | --example=random) [--interface=<spi | iic | uart>] [--addr=<address>]\n");
        mfrc522_interface_debug_print("\n");
        mfrc522_interface_debug_print("Options:\n");
        mfrc522_interface_debug_print("      --addr=<address>    Set the addr pin.([default: 0])\n");
        mfrc522_interface_debug_print("      --data=<hex>        Set the send data and it is hexadecimal.\n");
        mfrc522_interface_debug_print("  -e <mifare | crc | random>, --example=<mifare | crc | random>\n");
        mfrc522_interface_debug_print("                          Run the driver example.\n");
        mfrc522_interface_debug_print("  -h, --help              Show the help.\n");
        mfrc522_interface_debug_print("  -i, --information       Show the chip information.\n");
        mfrc522_interface_debug_print("      --interface=<spi | iic | uart>\n");
        mfrc522_interface_debug_print("                          Set the chip interface.([default: spi])\n");
        mfrc522_interface_debug_print("  -p, --port              Display the pin connections of the current board.\n");
        mfrc522_interface_debug_print("      --str=<string>      Set the crc string.\n");
        mfrc522_interface_debug_print("  -t <reg | mifare>, --test=<reg | mifare>\n");
        mfrc522_interface_debug_print("                          Run the driver test.\n");
        
        return 0;
    }
    else if (strcmp("i", type) == 0)
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
    else if (strcmp("p", type) == 0)
    {
        /* print pin connection */
        mfrc522_interface_debug_print("mfrc522: SPI interface SCK connected to GPIO11(BCM).\n");
        mfrc522_interface_debug_print("mfrc522: SPI interface MISO connected to GPIO9(BCM).\n");
        mfrc522_interface_debug_print("mfrc522: SPI interface MOSI connected to GPIO10(BCM).\n");
        mfrc522_interface_debug_print("mfrc522: SPI interface CS connected to GPIO8(BCM).\n");
        mfrc522_interface_debug_print("mfrc522: IIC interface SCL connected to GPIO3(BCM).\n");
        mfrc522_interface_debug_print("mfrc522: IIC interface SDA connected to GPIO2(BCM).\n");
        mfrc522_interface_debug_print("mfrc522: UART interface TX connected to GPIO15(BCM).\n");
        mfrc522_interface_debug_print("mfrc522: UART interface RX connected to GPIO14(BCM).\n");
        mfrc522_interface_debug_print("mfrc522: INT connected to GPIO17(BCM).\n");
        mfrc522_interface_debug_print("mfrc522: RESET connected to GPIO27(BCM).\n");
        
        return 0;
    }
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
        mfrc522_interface_debug_print("mfrc522: unknown status code.\n");
    }

    return 0;
}
