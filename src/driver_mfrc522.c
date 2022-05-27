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
 * @file      driver_mfrc522.c
 * @brief     driver mfrc522 source file
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

#include "driver_mfrc522.h"

/**
 * @brief chip information definition
 */
#define CHIP_NAME                 "NXP MFRC522"        /**< chip name */
#define MANUFACTURER_NAME         "NXP"                /**< manufacturer name */
#define SUPPLY_VOLTAGE_MIN        2.5f                 /**< chip min supply voltage */
#define SUPPLY_VOLTAGE_MAX        3.6f                 /**< chip max supply voltage */
#define MAX_CURRENT               100.0f               /**< chip max current */
#define TEMPERATURE_MIN           -25.0f               /**< chip min operating temperature */
#define TEMPERATURE_MAX           85.0f                /**< chip max operating temperature */
#define DRIVER_VERSION            1000                 /**< driver version */

/**
 * @brief chip register definition
 */
#define MFRC522_REG_COMMAND               0x01        /**< starts and stops command execution register */
#define MFRC522_REG_COMIEN                0x02        /**< enable and disable interrupt request control bits register */
#define MFRC522_REG_DIVIEN                0x03        /**< enable and disable interrupt request control bits register */
#define MFRC522_REG_COMIRQ                0x04        /**< interrupt request bits register */
#define MFRC522_REG_DIVIRQ                0x05        /**< interrupt request bits register */
#define MFRC522_REG_ERROR                 0x06        /**< error bits showing the error status of the last command execued register */
#define MFRC522_REG_STATUS1               0x07        /**< communication status bits register */
#define MFRC522_REG_STATUS2               0x08        /**< receiver and transmitter status bits register */
#define MFRC522_REG_FIFO_DATA             0x09        /**< input and output of 64 byte FIFO buffer register */
#define MFRC522_REG_FIFO_LEVEL            0x0A        /**< number of bytes stored in the FIFO buffer register */
#define MFRC522_REG_WATER_LEVEL           0x0B        /**< level for FIFO underflow and overflow warning register */
#define MFRC522_REG_CONTROL               0x0C        /**< miscellaneous control registers register */
#define MFRC522_REG_BIT_FRAMING           0x0D        /**< adjustments for bit-oriented frames register */
#define MFRC522_REG_COLL                  0x0E        /**< bit position of the first bit-collision detected on the RF interface register */
#define MFRC522_REG_MODE                  0x11        /**< defines general modes for transmitting and receiving register */
#define MFRC522_REG_TX_MODE               0x12        /**< defines transmission data rate and framing register */
#define MFRC522_REG_RX_MODE               0x13        /**< defines reception data rate and framing register */
#define MFRC522_REG_TX_CONTROL            0x14        /**< controls the logical behavior of the antenna driver pins TX1 and TX 2 register */
#define MFRC522_REG_TX_ASK                0x15        /**< controls the setting of the transmission modulation register */
#define MFRC522_REG_TX_SEL                0x16        /**< selects the internal sources for the antenna driver register */
#define MFRC522_REG_RX_SEL                0x17        /**< selects internal receiver settings register */
#define MFRC522_REG_RX_THRESHOLD          0x18        /**< selects thresholds for the bit decoder register */
#define MFRC522_REG_DEMOD                 0x19        /**< defines demodulator settings register */
#define MFRC522_REG_MFTX                  0x1C        /**< controls some MIFARE communication transmit parameters register */
#define MFRC522_REG_MFRX                  0x1D        /**< controls some MIFARE communication receive parameters register */
#define MFRC522_REG_SERIAL_SPEED          0x1F        /**< selects the speed of the serial UART interface register */
#define MFRC522_REG_CRC_RESULT_H          0x21        /**< shows the MSB and LSB values of the CRC calculation high register */
#define MFRC522_REG_CRC_RESULT_L          0x22        /**< shows the MSB and LSB values of the CRC calculation low register */
#define MFRC522_REG_MOD_WIDTH             0x24        /**< controls the ModWidth setting register */
#define MFRC522_REG_RFCFG                 0x26        /**< configures the receiver gain register */
#define MFRC522_REG_GSN                   0x27        /**< selects the conductance of the antenna driver pins TX1 and TX2 for modulation register */
#define MFRC522_REG_CWGSP                 0x28        /**< defines the conductance of the p-driver output during periods of no modulation register */
#define MFRC522_REG_MODGSP                0x29        /**< defines the conductance of the p-driver output during periods of modulation register */
#define MFRC522_REG_TMODE                 0x2A        /**< defines settings for the internal timer register */
#define MFRC522_REG_TPRESCALER            0x2B        /**< defines settings for the internal timer register */
#define MFRC522_REG_TRELOAD_H             0x2C        /**< defines the 16-bit timer reload value high register */
#define MFRC522_REG_TRELOAD_L             0x2D        /**< defines the 16-bit timer reload value low register */
#define MFRC522_REG_TCOUNTER_VAL_H        0x2E        /**< shows the 16-bit timer value high register */
#define MFRC522_REG_TCOUNTER_VAL_L        0x2F        /**< shows the 16-bit timer value low register */
#define MFRC522_REG_TEST_SEL1             0x31        /**< general test signal configuration register */
#define MFRC522_REG_TEST_SEL2             0x32        /**< general test signal configuration and PRBS control register */
#define MFRC522_REG_TEST_PIN_EN           0x33        /**< enables pin output driver on pins D1 to D7 register */
#define MFRC522_REG_TEST_PIN_VALUE        0x34        /**< defines the values for D1 to D7 when it is used as an I/O bus register */
#define MFRC522_REG_TEST_BUS              0x35        /**< shows the status of the internal test bus register */
#define MFRC522_REG_AUTO_TEST             0x36        /**< controls the digital self test register */
#define MFRC522_REG_VERSION               0x37        /**< shows the software version register */
#define MFRC522_REG_ANALOG_TEST           0x38        /**< controls the pins AUX1 and AUX2 register */
#define MFRC522_REG_TEST_DAC1             0x39        /**< defines the test value for TestDAC1 register */
#define MFRC522_REG_TEST_DAC2             0x3A        /**< defines the test value for TestDAC2 register */
#define MFRC522_REG_TEST_ADC              0x3B        /**< shows the value of ADC I and Q channels register */

/**
 * @brief      read bytes
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[in]  reg is the iic register address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the data length
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
static uint8_t a_mfrc522_read(mfrc522_handle_t *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;
    
    if (handle->iic_spi_uart == MFRC522_INTERFACE_IIC)                     /* if iic interface */
    {
        if (handle->iic_read(handle->iic_addr, reg, buf, len) != 0)        /* iic read */
        {
            return 1;                                                      /* return error */
        }
        else
        {
            return 0;                                                      /* success return 0 */
        }
    }
    else if (handle->iic_spi_uart == MFRC522_INTERFACE_SPI)                /* if spi interface */
    {
        uint8_t addr;
        
        for (i = 0; i< len; i++)                                           /* len times */
        {
            addr = (uint8_t)((1 << 7) | (((reg + i) & 0x3F) << 1));        /* set the address */
            if (handle->spi_read(addr, buf + i, 1) != 0)                   /* check the result */
            {
                return 1;                                                  /* return error */
            }
        }
        
        return 0;                                                          /* success return 0 */
    }
    else if (handle->iic_spi_uart == MFRC522_INTERFACE_UART)               /* if uart interface */
    {
        uint8_t addr;
        
        for (i = 0; i< len; i++)                                           /* len times */
        {
            addr = (uint8_t)((1 << 7) | (((reg + i) & 0x3F) << 0));        /* set the address */
            if (handle->uart_flush() != 0)                                 /* uart flush */
            {
                return 1;                                                  /* return error */
            }
            if (handle->uart_write(&addr, 1) != 0)                         /* uart write */
            {
                return 1;                                                  /* return error */
            }
            if (handle->uart_read(buf + i, 1) != 1)                        /* uart read */
            {
                return 1;                                                  /* return error */
            }
        }
        
        return 0;                                                          /* success return 0 */
    }
    else
    {                                                                      /* return error */
        return 1;
    }
}

/**
 * @brief     write bytes
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] reg is the iic register address
 * @param[in] *buf points to a data buffer
 * @param[in] len is the data length
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
static uint8_t a_mfrc522_write(mfrc522_handle_t *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;
    
    if (handle->iic_spi_uart == MFRC522_INTERFACE_IIC)                     /* if iic interface */
    {
        if (handle->iic_write(handle->iic_addr, reg, buf, len) != 0)       /* iic write */
        {
            return 1;                                                      /* return error */
        }
        else
        {
            return 0;                                                      /* success return 0 */
        }
    }
    else if (handle->iic_spi_uart == MFRC522_INTERFACE_SPI)                /* if spi interface */
    {
        uint8_t addr;
        
        for (i = 0; i< len; i++)                                           /* len times */
        {
            addr = (uint8_t)((0 << 7) | (((reg + i) & 0x3F) << 1));        /* set the address */
            if (handle->spi_write(addr, buf + i, 1) != 0)                  /* check the result */
            {
                return 1;                                                  /* return error */
            }
        }
        
        return 0;                                                          /* success return 0 */
    }
    else if (handle->iic_spi_uart == MFRC522_INTERFACE_UART)               /* if uart interface */
    {
        uint8_t addr;
        uint8_t b[2];
        
        for (i = 0; i< len; i++)                                           /* len times */
        {
            addr = ((reg + i) & 0x3F) << 0;                                /* set the addr */
            b[0] = addr;                                                   /* set to buffer */
            b[1] = buf[i];                                                 /* set buffer */
            if (handle->uart_write(b, 2) != 0)                             /* uart write */
            {
                return 1;                                                  /* return error */
            }
        }
        
        return 0;                                                          /* success return 0 */
    }
    else
    {
        return 1;                                                          /* return error */
    }
}

/**
 * @brief     set the chip interface
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] interface is the chip interface
 * @return    status code
 *            - 0 success
 *            - 2 handle is NULL
 * @note      none
 */
uint8_t mfrc522_set_interface(mfrc522_handle_t *handle, mfrc522_interface_t interface) 
{
    if (handle == NULL)                               /* check handle */
    {
        return 2;                                     /* return error */
    }
    
    handle->iic_spi_uart = (uint8_t)interface;        /* set interface */
    
    return 0;                                         /* success return 0 */
}

/**
 * @brief      get the chip interface
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *interface points to a chip interface buffer
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 * @note       none
 */
uint8_t mfrc522_get_interface(mfrc522_handle_t *handle, mfrc522_interface_t *interface) 
{
    if (handle == NULL)                                              /* check handle */
    {
        return 2;                                                    /* return error */
    }
    
    *interface = (mfrc522_interface_t)(handle->iic_spi_uart);        /* get interface */
    
    return 0;                                                        /* success return 0 */
}

/**
 * @brief     set the iic address pin
 * @param[in] *handle points to a mfrc522 handle structure
 * @param[in] addr_pin is the address pin
 * @return    status code
 *            - 0 success
 *            - 2 handle is NULL
 * @note      none
 */
uint8_t mfrc522_set_addr_pin(mfrc522_handle_t *handle, uint8_t addr_pin)
{
    if (handle == NULL)                        /* check handle */
    {
        return 2;                              /* return error */
    }
    
    handle->iic_addr = (uint8_t)addr_pin;      /* set pin */
    
    return 0;                                  /* success return 0 */
}

/**
 * @brief      get the iic address pin
 * @param[in]  *handle points to a mfrc522 handle structure
 * @param[out] *addr_pin points to a address pin buffer
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 * @note       none
 */
uint8_t mfrc522_get_addr_pin(mfrc522_handle_t *handle, uint8_t *addr_pin)
{
    if (handle == NULL)                             /* check handle */
    {
        return 2;                                   /* return error */
    }
    
    *addr_pin = (uint8_t)(handle->iic_addr);        /* get pin */
    
    return 0;                                       /* success return 0 */
}

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
uint8_t mfrc522_init(mfrc522_handle_t *handle)
{
    uint8_t res;
    uint8_t prev;
    uint8_t id;
    uint32_t timeout;
    
    if (handle == NULL)                                                      /* check handle */
    {
        return 2;                                                            /* return error */
    }
    if (handle->debug_print == NULL)                                         /* check debug_print */
    {
        return 3;                                                            /* return error */
    }
    if (handle->reset_gpio_init == NULL)                                     /* check reset_gpio_init */
    {
        handle->debug_print("mfrc522: reset_gpio_init is null.\n");          /* reset_gpio_init is null */
        
        return 3;                                                            /* return error */
    }
    if (handle->reset_gpio_deinit == NULL)                                   /* check reset_gpio_deinit */
    {
        handle->debug_print("mfrc522: reset_gpio_deinit is null.\n");        /* reset_gpio_deinit is null */
        
        return 3;                                                            /* return error */
    }
    if (handle->reset_gpio_write == NULL)                                    /* check reset_gpio_write */
    {
        handle->debug_print("mfrc522: reset_gpio_write is null.\n");         /* reset_gpio_write is null */
        
        return 3;                                                            /* return error */
    }
    if (handle->iic_init == NULL)                                            /* check iic_init */
    {
        handle->debug_print("mfrc522: iic_init is null.\n");                 /* iic_init is null */
        
        return 3;                                                            /* return error */
    }
    if (handle->iic_deinit == NULL)                                          /* check iic_deinit */
    {
        handle->debug_print("mfrc522: iic_deinit is null.\n");               /* iic_deinit is null */
        
        return 3;                                                            /* return error */
    }
    if (handle->iic_read == NULL)                                            /* check iic_read */
    {
        handle->debug_print("mfrc522: iic_read is null.\n");                 /* iic_read is null */
        
        return 3;                                                            /* return error */
    }
    if (handle->iic_write == NULL)                                           /* check iic_write */
    {
        handle->debug_print("mfrc522: iic_write is null.\n");                /* iic_write is null */
        
        return 3;                                                            /* return error */
    }
    if (handle->uart_init == NULL)                                           /* check uart_init */
    {
        handle->debug_print("mfrc522: uart_init is null.\n");                /* uart_init is null */
        
        return 3;                                                            /* return error */
    }
    if (handle->uart_deinit == NULL)                                         /* check uart_deinit */
    {
        handle->debug_print("mfrc522: uart_deinit is null.\n");              /* uart_deinit is null */
        
        return 3;                                                            /* return error */
    }
    if (handle->uart_read == NULL)                                           /* check uart_read */
    {
        handle->debug_print("mfrc522: uart_read is null.\n");                /* uart_read is null */
        
        return 3;                                                            /* return error */
    }
    if (handle->uart_write == NULL)                                          /* check uart_write */
    {
        handle->debug_print("mfrc522: uart_write is null.\n");               /* uart_write is null */
        
        return 3;                                                            /* return error */
    }
    if (handle->uart_flush == NULL)                                          /* check uart_flush */
    {
        handle->debug_print("mfrc522: uart_flush is null.\n");               /* uart_flush is null */
        
        return 3;                                                            /* return error */
    }
    if (handle->spi_init == NULL)                                            /* check spi_init */
    {
        handle->debug_print("mfrc522: spi_init is null.\n");                 /* spi_init is null */
        
        return 3;                                                            /* return error */
    }
    if (handle->spi_deinit == NULL)                                          /* check spi_deinit */
    {
        handle->debug_print("mfrc522: spi_deinit is null.\n");               /* spi_deinit is null */
        
        return 3;                                                            /* return error */
    }
    if (handle->spi_read == NULL)                                            /* check spi_read */
    {
        handle->debug_print("mfrc522: spi_read is null.\n");                 /* spi_read is null */
        
        return 3;                                                            /* return error */
    }
    if (handle->spi_write == NULL)                                           /* check spi_write */
    {
        handle->debug_print("mfrc522: spi_write is null.\n");                /* spi_readspi_writeis null */
        
        return 3;                                                            /* return error */
    }
    if (handle->delay_ms == NULL)                                            /* check delay_ms */
    {
        handle->debug_print("mfrc522: delay_ms is null.\n");                 /* delay_ms is null */
        
        return 3;                                                            /* return error */
    }
    if (handle->receive_callback == NULL)                                    /* check receive_callback */
    {
        handle->debug_print("mfrc522: receive_callback is null.\n");         /* receive_callback is null */
        
        return 3;                                                            /* return error */
    }
    
    if (handle->reset_gpio_init() != 0)                                      /* reset gpio init */
    {
        handle->debug_print("mfrc522: reset gpio init failed.\n");           /* reset gpio init failed */
        
        return 1;                                                            /* return error */
    }
    if (handle->iic_spi_uart == MFRC522_INTERFACE_IIC)                       /* if iic interface */
    {
        if (handle->iic_init() != 0)                                         /* iic init */
        {
            handle->debug_print("mfrc522: iic init failed.\n");              /* iic init failed */
            (void)handle->reset_gpio_deinit();                               /* reset gpio deinit */
            
            return 1;                                                        /* return error */
        }
    }
    else if (handle->iic_spi_uart == MFRC522_INTERFACE_SPI)                  /* if spi interface */
    {
        if (handle->spi_init() != 0)                                         /* spi init */
        {
            handle->debug_print("mfrc522: spi init failed.\n");              /* spi init failed */
            (void)handle->reset_gpio_deinit();                               /* reset gpio deinit */
            
            return 1;                                                        /* return error */
        }
    }
    else if (handle->iic_spi_uart == MFRC522_INTERFACE_UART)                 /* if uart interface */
    {
        if (handle->uart_init() != 0)                                        /* uart init */
        {
            handle->debug_print("mfrc522: uart init failed.\n");             /* uart init failed */
            (void)handle->reset_gpio_deinit();                               /* reset gpio deinit */
            
            return 1;                                                        /* return error */
        }
    }
    else
    {
        handle->debug_print("mfrc522: interface is invalid.\n");             /* interface is invalid */
        
        return 4;                                                            /* return error */
    }

    if (handle->reset_gpio_write(0) != 0)                                    /* set 0 */
    {
        handle->debug_print("mfrc522: reset gpio write failed.\n");          /* reset gpio write failed */
        res = 1;                                                             /* set the exit code */
        
        goto exit_code;                                                      /* goto the exit code */
    }
    handle->delay_ms(10);                                                    /* delay 10 ms */
    if (handle->reset_gpio_write(1) != 0)                                    /* set 1 */
    {
        handle->debug_print("mfrc522: reset gpio write failed.\n");          /* reset gpio write failed */
        res = 1;                                                             /* set the exit code */
        
        goto exit_code;                                                      /* goto the exit code */
    }

    res = a_mfrc522_read(handle, MFRC522_REG_COMMAND, &prev, 1);             /* read config */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: read command failed.\n");              /* read command failed */
        res = 1;                                                             /* set the exit code */
        
        goto exit_code;                                                      /* goto the exit code */
    }
    prev &= ~(0xF << 0);                                                     /* clear the settings */
    prev |= MFRC522_COMMAND_SOFT_RESET;                                      /* set the idle */
    res = a_mfrc522_write(handle, MFRC522_REG_COMMAND, &prev, 1);            /* write config */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: write command failed.\n");             /* write command failed */
        res = 1;                                                             /* set the exit code */
        
        goto exit_code;                                                      /* goto the exit code */
    }
    handle->delay_ms(1);                                                     /* delay 1 ms */
    timeout = 1000;                                                          /* set 1000 ms */
    while (timeout != 0)                                                     /* check the timeout */
    {
        res = a_mfrc522_read(handle, MFRC522_REG_COMMAND, &prev, 1);         /* read config */
        if (res != 0)                                                        /* check the result */
        {
            handle->debug_print("mfrc522: read command failed.\n");          /* read command failed */
            res = 1;                                                         /* set the exit code */
            
            goto exit_code;                                                  /* goto the exit code */
        }
        timeout--;                                                           /* timeout-- */
        handle->delay_ms(1);                                                 /* delay 1 ms */
        if ((prev & (1 << 4)) == 0)                                          /* check the power on bit */
        {
            break;                                                           /* break */
        }
        if (timeout == 0)                                                    /* check the timeout */
        {
            handle->debug_print("mfrc522: read timeout.\n");                 /* read timeout */
            res = 1;                                                         /* set the exit code */
            
            goto exit_code;                                                  /* goto the exit code */
        }
    }

    if (a_mfrc522_read(handle, MFRC522_REG_VERSION, &id, 1) != 0)            /* get the id */
    {
        handle->debug_print("mfrc522: get id failed.\n");                    /* get id failed */
        res = 5;                                                             /* set the exit code */
        
        goto exit_code;                                                      /* goto the exit code */
    }
    if (((id >> 4) & 0xF) != 9)                                              /* check the id */
    {
        handle->debug_print("mfrc522: check id failed.\n");                  /* check id failed */
        res = 6;                                                             /* set the exit code */
        
        goto exit_code;                                                      /* goto the exit code */
    }
    
    handle->inited = 1;                                                      /* flag inited */
    handle->irq_flag = 0x0000;                                               /* set 0x0000 */
    
    return 0;                                                                /* success return 0 */
    
    exit_code:
    
    if (handle->iic_spi_uart == MFRC522_INTERFACE_IIC)                       /* if iic interface */
    {
        (void)handle->iic_deinit();                                          /* iic deinit */
    }
    else if (handle->iic_spi_uart == MFRC522_INTERFACE_SPI)                  /* if spi interface */
    {
        (void)handle->spi_deinit();                                          /* spi deinit */
    }
    else                                                                     /* if uart interface */
    {
        (void)handle->uart_deinit();                                         /* uart deinit */
    }
    (void)handle->reset_gpio_deinit();                                       /* reset gpio deinit */
    
    return res;                                                              /* return error */
}

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
uint8_t mfrc522_deinit(mfrc522_handle_t *handle)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                  /* check handle */
    {
        return 2;                                                        /* return error */
    }
    if (handle->inited != 1)                                             /* check handle initialization */
    {
        return 3;                                                        /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_COMMAND, &prev, 1);         /* read config */
    if (res != 0)                                                        /* check the result */
    {
        handle->debug_print("mfrc522: read command failed.\n");          /* read command failed */
        
        return 1;                                                        /* return error */
    }
    prev &= ~(0xF << 0);                                                 /* clear the settings */
    prev |= MFRC522_COMMAND_SOFT_RESET;                                  /* set the idle */
    res = a_mfrc522_write(handle, MFRC522_REG_COMMAND, &prev, 1);        /* write config */
    if (res != 0)                                                        /* check the result */
    {
        handle->debug_print("mfrc522: write command failed.\n");         /* write command failed */
        
        return 1;                                                        /* return error */
    }
    handle->delay_ms(10);                                                /* delay 10 ms */
    if (handle->reset_gpio_write(0) != 0)                                /* power down */
    {
        handle->debug_print("mfrc522: reset gpio write failed.\n");      /* reset gpio write failed */
        
        return 5;                                                        /* return error */
    }
    if (handle->iic_spi_uart == MFRC522_INTERFACE_IIC)                   /* if iic interface */
    {
        if (handle->iic_deinit() != 0)                                   /* iic deinit */
        {
            handle->debug_print("mfrc522: iic deinit failed.\n");        /* iic deinit failed */
            
            return 1;                                                    /* return error */
        }
    }
    else if (handle->iic_spi_uart == MFRC522_INTERFACE_SPI)              /* if spi interface */
    {
        if (handle->spi_deinit() != 0)                                   /* spi deinit */
        {
            handle->debug_print("mfrc522: spi deinit failed.\n");        /* spi deinit failed */
            
            return 1;                                                    /* return error */
        }
    }
    else if (handle->iic_spi_uart == MFRC522_INTERFACE_UART)             /* if uart interface */
    {
        if (handle->uart_deinit() != 0)                                  /* uart deinit */
        {
            handle->debug_print("mfrc522: uart deinit failed.\n");       /* uart deinit failed */
            
            return 1;                                                    /* return error */
        }
    }
    else
    {
        handle->debug_print("mfrc522: interface is invalid.\n");         /* interface is invalid */
        
        return 4;                                                        /* return error */
    }
    if (handle->reset_gpio_deinit() != 0)                                /* reset gpio deinit */
    {
        handle->debug_print("mfrc522: reset gpio deinit failed.\n");     /* reset gpio deinit failed */
        
        return 1;                                                        /* return error */
    }
    
    handle->inited = 0;                                                  /* flag closed */
    
    return 0;                                                            /* success return 0 */
}

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
uint8_t mfrc522_irq_handler(mfrc522_handle_t *handle)
{
    uint8_t res;
    uint8_t prev;
    uint8_t prev1;
    uint8_t prev2;
    
    if (handle == NULL)                                                      /* check handle */
    {
        return 2;                                                            /* return error */
    }
    if (handle->inited != 1)                                                 /* check handle initialization */
    {
        return 3;                                                            /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_STATUS1, &prev, 1);             /* read status1 */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: read status1 failed.\n");              /* read status1 failed */
        
        return 1;                                                            /* return error */
    }
    res = a_mfrc522_read(handle, MFRC522_REG_COMIRQ, &prev1, 1);             /* read comirq */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: read comirq failed.\n");               /* read comirq failed */
        
        return 1;                                                            /* return error */
    }
    res = a_mfrc522_read(handle, MFRC522_REG_DIVIRQ, &prev2, 1);             /* read divirq */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: read divirq failed.\n");               /* read divirq failed */
        
        return 1;                                                            /* return error */
    }
    
    if ((prev & (1 << 4)) != 0)                                              /* if set */
    {
        res = a_mfrc522_read(handle, MFRC522_REG_COMIRQ, &prev, 1);          /* read config */
        if (res != 0)                                                        /* check the result */
        {
            handle->debug_print("mfrc522: read comirq failed.\n");           /* read comirq failed */
            
            return 1;                                                        /* return error */
        }
        prev &= ~(1 << 7);                                                   /* clear the settings */
        res = a_mfrc522_write(handle, MFRC522_REG_COMIRQ, &prev, 1);         /* write config */
        if (res != 0)                                                        /* check the result */
        {
            handle->debug_print("mfrc522: write comirq failed.\n");          /* write comirq failed */
            
            return 1;                                                        /* return error */
        }
        res = a_mfrc522_read(handle, MFRC522_REG_DIVIRQ, &prev, 1);          /* read config */
        if (res != 0)                                                        /* check the result */
        {
            handle->debug_print("mfrc522: read divirq failed.\n");           /* read divirq failed */
            
            return 1;                                                        /* return error */
        }
        prev &= ~(1 << 7);                                                   /* clear the settings */
        res = a_mfrc522_write(handle, MFRC522_REG_DIVIRQ, &prev, 1);         /* write config */
        if (res != 0)                                                        /* check the result */
        {
            handle->debug_print("mfrc522: write divirq failed.\n");          /* write divirq failed */
            
            return 1;                                                        /* return error */
        }
        
        if ((prev1 & (1 << MFRC522_INTERRUPT_TIMER)) != 0)                   /* timer */
        {
            handle->irq_flag |= 1 << MFRC522_INTERRUPT_TIMER;                /* set the irq flag */
            if (handle->receive_callback != NULL)                            /* if receive callback */
            {
                handle->receive_callback(MFRC522_INTERRUPT_TIMER);           /* run callback */
            }
        }
        if ((prev1 & (1 << MFRC522_INTERRUPT_ERR)) != 0)                     /* err */
        {
            handle->irq_flag |= 1 << MFRC522_INTERRUPT_ERR;                  /* set the irq flag */
            if (handle->receive_callback != NULL)                            /* if receive callback */
            {
                handle->receive_callback(MFRC522_INTERRUPT_ERR);             /* run callback */
            }
        }
        if ((prev1 & (1 << MFRC522_INTERRUPT_LO_ALERT)) != 0)                /* lo alert */
        {
            handle->irq_flag |= 1 << MFRC522_INTERRUPT_LO_ALERT;             /* set the irq flag */
            if (handle->receive_callback != NULL)                            /* if receive callback */
            {
                handle->receive_callback(MFRC522_INTERRUPT_LO_ALERT);        /* run callback */
            }
        }
        if ((prev1 & (1 << MFRC522_INTERRUPT_HI_ALERT)) != 0)                /* hi alert */
        {
            handle->irq_flag |= 1 << MFRC522_INTERRUPT_HI_ALERT;             /* set the irq flag */
            if (handle->receive_callback != NULL)                            /* if receive callback */
            {
                handle->receive_callback(MFRC522_INTERRUPT_HI_ALERT);        /* run callback */
            }
        }
        if ((prev1 & (1 << MFRC522_INTERRUPT_IDLE)) != 0)                    /* idle */
        {
            handle->irq_flag |= 1 << MFRC522_INTERRUPT_IDLE;                 /* set the irq flag */
            if (handle->receive_callback != NULL)                            /* if receive callback */
            {
                handle->receive_callback(MFRC522_INTERRUPT_IDLE);            /* run callback */
            }
        }
        if ((prev1 & (1 << MFRC522_INTERRUPT_RX)) != 0)                      /* rx */
        {
            handle->irq_flag |= 1 << MFRC522_INTERRUPT_RX;                   /* set the irq flag */
            if (handle->receive_callback != NULL)                            /* if receive callback */
            {
                handle->receive_callback(MFRC522_INTERRUPT_RX);              /* run callback */
            }
        }
        if ((prev1 & (1 << MFRC522_INTERRUPT_TX)) != 0)                      /* tx */
        {
            handle->irq_flag |= 1 << MFRC522_INTERRUPT_TX;                   /* set the irq flag */
            if (handle->receive_callback != NULL)                            /* if receive callback */
            {
                handle->receive_callback(MFRC522_INTERRUPT_TX);              /* run callback */
            }
        }
        if ((prev2 & (1 << MFRC522_INTERRUPT2_CRC)) != 0)                    /* crc */
        {
            handle->irq_flag |= 1 << MFRC522_INTERRUPT_CRC;                  /* set the irq flag */
            if (handle->receive_callback != NULL)                            /* if receive callback */
            {
                handle->receive_callback(MFRC522_INTERRUPT_CRC);             /* run callback */
            }
        }
        if ((prev2 & (1 << MFRC522_INTERRUPT2_MFIN_ACT)) != 0)               /* mfin act */
        {
            handle->irq_flag |= 1 << MFRC522_INTERRUPT_MFIN_ACT;             /* set the irq flag */
            if (handle->receive_callback != NULL)                            /* if receive callback */
            {
                handle->receive_callback(MFRC522_INTERRUPT_MFIN_ACT);        /* run callback */
            }
        }
    }
    
    return 0;                                                                /* success return 0 */
}

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
                            uint8_t *err, uint32_t ms)
{
    uint8_t res;
    uint8_t prev;
    uint8_t i;
    uint16_t wait_for;
    uint32_t timeout;
    
    if (handle == NULL)                                                             /* check handle */
    {
        return 2;                                                                   /* return error */
    }
    if (handle->inited != 1)                                                        /* check handle initialization */
    {
        return 3;                                                                   /* return error */
    }
    if ((in_buf == NULL) || (out_buf == NULL))                                      /* check the buffer */
    {
        handle->debug_print("mfrc522: buffer is NULL.\n");                          /* buffer is NULL */
        
        return 4;                                                                   /* return error */
    }
    if (in_len > 64)                                                                /* check the length */
    {
        handle->debug_print("mfrc522: in_len is over 64.\n");                       /* in_len is over 64 */
        
        return 5;                                                                   /* return error */
    }
    
    /* set idle */
    res = a_mfrc522_read(handle, MFRC522_REG_COMMAND, &prev, 1);                    /* read config */
    if (res != 0)                                                                   /* check the result */
    {
        handle->debug_print("mfrc522: read command failed.\n");                     /* read command failed */
        
        return 1;                                                                   /* return error */
    }
    prev &= ~(0xF << 0);                                                            /* clear the settings */
    prev |= MFRC522_COMMAND_IDLE;                                                   /* set the idle */
    res = a_mfrc522_write(handle, MFRC522_REG_COMMAND, &prev, 1);                   /* write config */
    if (res != 0)                                                                   /* check the result */
    {
        handle->debug_print("mfrc522: write command failed.\n");                    /* write command failed */
        
        return 1;                                                                   /* return error */
    }
    
    /* flush the fifo */
    res = a_mfrc522_read(handle, MFRC522_REG_FIFO_LEVEL, &prev, 1);                 /* read level */
    if (res != 0)                                                                   /* check the result */
    {
        handle->debug_print("mfrc522: read fifo level failed.\n");                  /* read fifo level failed */
        
        return 1;                                                                   /* return error */
    }
    prev |= 1 << 7;                                                                 /* set the flush bit */
    res = a_mfrc522_write(handle, MFRC522_REG_FIFO_LEVEL, &prev, 1);                /* write level */
    if (res != 0)                                                                   /* check the result */
    {
        handle->debug_print("mfrc522: write fifo level failed.\n");                 /* write fifo level failed */
        
        return 1;                                                                   /* return error */
    }
    
    /* write fifo */
    for (i = 0; i < in_len; i++)                                                    /* loop */
    {
        res = a_mfrc522_write(handle, MFRC522_REG_FIFO_DATA, in_buf + i, 1);        /* write data */
        if (res != 0)                                                               /* check the result */
        {
            handle->debug_print("mfrc522: write fifo data failed.\n");              /* write fifo data failed */
            
            return 1;                                                               /* return error */
        }
    }
    
    /* clear the flag */
    handle->irq_flag = 0;                                                           /* clear the irq flag */
    res = a_mfrc522_read(handle, MFRC522_REG_COMIRQ, &prev, 1);                     /* read config */
    if (res != 0)                                                                   /* check the result */
    {
        handle->debug_print("mfrc522: read comirq failed.\n");                      /* read comirq failed */
        
        return 1;                                                                   /* return error */
    }
    prev &= ~(1 << 7);                                                              /* clear the settings */
    res = a_mfrc522_write(handle, MFRC522_REG_COMIRQ, &prev, 1);                    /* write config */
    if (res != 0)                                                                   /* check the result */
    {
        handle->debug_print("mfrc522: write comirq failed.\n");                     /* write comirq failed */
        
        return 1;                                                                   /* return error */
    }
    res = a_mfrc522_read(handle, MFRC522_REG_DIVIRQ, &prev, 1);                     /* read config */
    if (res != 0)                                                                   /* check the result */
    {
        handle->debug_print("mfrc522: read divirq failed.\n");                      /* read divirq failed */
        
        return 1;                                                                   /* return error */
    }
    prev &= ~(1 << 7);                                                              /* clear the settings */
    res = a_mfrc522_write(handle, MFRC522_REG_DIVIRQ, &prev, 1);                    /* write config */
    if (res != 0)                                                                   /* check the result */
    {
        handle->debug_print("mfrc522: write divirq failed.\n");                     /* write divirq failed */
        
        return 1;                                                                   /* return error */
    }
    
    /* set the command */
    res = a_mfrc522_read(handle, MFRC522_REG_COMMAND, &prev, 1);                    /* read config */
    if (res != 0)                                                                   /* check the result */
    {
        handle->debug_print("mfrc522: read command failed.\n");                     /* read command failed */
        
        return 1;                                                                   /* return error */
    }
    prev &= ~(0xF << 0);                                                            /* clear the settings */
    prev |= command;                                                                /* set the command */
    res = a_mfrc522_write(handle, MFRC522_REG_COMMAND, &prev, 1);                   /* write config */
    if (res != 0)                                                                   /* check the result */
    {
        handle->debug_print("mfrc522: write command failed.\n");                    /* write command failed */
        
        return 1;                                                                   /* return error */
    }
    
    /* set bit framing */
    if (command == MFRC522_COMMAND_TRANSCEIVE)                                      /* if transceive*/
    {
        res = a_mfrc522_read(handle, MFRC522_REG_BIT_FRAMING, &prev, 1);            /* read bit framing */
        if (res != 0)                                                               /* check the result */
        {
            handle->debug_print("mfrc522: read bit framing failed.\n");             /* read bit framing failed */
            
            return 1;                                                               /* return error */
        }
        prev |= 1 << 7;                                                             /* set the start bit */
        res = a_mfrc522_write(handle, MFRC522_REG_BIT_FRAMING, &prev, 1);           /* write bit framing */
        if (res != 0)                                                               /* check the result */
        {
            handle->debug_print("mfrc522: write bit framing failed.\n");            /* write bit framing failed */
            
            return 1;                                                               /* return error */
        }
    }
    
    /* set the wait bit */
    if (command == MFRC522_COMMAND_MF_AUTHENT)                                      /* if authent */
    {
        wait_for = (1 << MFRC522_INTERRUPT_IDLE) | 
                   (1 << MFRC522_INTERRUPT_TIMER);                                  /* if idle && timer */
    }
    else if (command == MFRC522_COMMAND_TRANSCEIVE)                                 /* if transceive */
    {
        wait_for = (1 << MFRC522_INTERRUPT_RX) | 
                   (1 << MFRC522_INTERRUPT_TIMER);                                  /* if rx && timer */
    }
    else if (command == MFRC522_COMMAND_CALC_CRC)                                   /* if crc */
    {
        wait_for = (1 << MFRC522_INTERRUPT_CRC) | 
                   (1 << MFRC522_INTERRUPT_TIMER);                                  /* if crc && timer */
    }
    else
    {
        wait_for = (1 << MFRC522_INTERRUPT_IDLE) | 
                   (1 << MFRC522_INTERRUPT_TIMER);                                  /* if idle && timer */
    }
    timeout = ms;                                                                   /* set timeout */
    while (timeout != 0)                                                            /* check the timeout */
    {
        handle->delay_ms(1);                                                        /* delay 1ms */
        timeout--;                                                                  /* timeout-- */
        if ((handle->irq_flag & wait_for) != 0)                                     /* check the wait for */
        {
            break;                                                                  /* break */
        }
        if (timeout == 0)                                                           /* if timeout == 0 */
        {
            handle->debug_print("mfrc522: read timeout.\n");                        /* read timeout */
            
            return 6;                                                               /* return error */
        }
    }
    
    /* end */
    if (command == MFRC522_COMMAND_TRANSCEIVE)                                      /* if transceive*/
    {
        res = a_mfrc522_read(handle, MFRC522_REG_BIT_FRAMING, &prev, 1);            /* read bit framing */
        if (res != 0)                                                               /* check the result */
        {
            handle->debug_print("mfrc522: read bit framing failed.\n");             /* read bit framing failed */
            
            return 1;                                                               /* return error */
        }
        prev &= ~(1 << 7);                                                          /* clear the settings */
        res = a_mfrc522_write(handle, MFRC522_REG_BIT_FRAMING, &prev, 1);           /* write bit framing */
        if (res != 0)                                                               /* check the result */
        {
            handle->debug_print("mfrc522: write bit framing failed.\n");            /* write bit framing failed */
            
            return 1;                                                               /* return error */
        }
    }
    
    /* check timer timeout */
    if ((handle->irq_flag & ((1 << MFRC522_INTERRUPT_TIMER))) != 0)                 /* check the timer */
    {
        handle->debug_print("mfrc522: timer timeout.\n");                           /* timer timeout */
        
        return 7;                                                                   /* return error */
    }
    
    /* check error */
    if ((handle->irq_flag & ((1 << MFRC522_INTERRUPT_ERR))) != 0)                   /* check the error */
    {
        res = a_mfrc522_read(handle, MFRC522_REG_ERROR, err, 1);                    /* read config */
        if (res != 0)                                                               /* check the result */
        {
            handle->debug_print("mfrc522: read error failed.\n");                   /* read error failed */
            
            return 1;                                                               /* return error */
        }
        handle->debug_print("mfrc522: find error.\n");                              /* find error */
        
        return 8;                                                                   /* return error */
    }
    
    /* get the fifo */
    if ((command == MFRC522_COMMAND_TRANSCEIVE) && ((*out_len) != 0))               /* if transceive and need get from fifo */
    {
        uint8_t level;
        
        res = a_mfrc522_read(handle, MFRC522_REG_FIFO_LEVEL, &level, 1);            /* read level */
        if (res != 0)                                                               /* check the result */
        {
            handle->debug_print("mfrc522: read fifo level failed.\n");              /* read fifo level failed */
            
            return 1;                                                               /* return error */
        }

        *out_len = level > (*out_len) ? (*out_len) : level;                         /* set the output length */
        for (i = 0; i < (*out_len); i++)                                            /* loop */
        {
            res = a_mfrc522_read(handle, MFRC522_REG_FIFO_DATA, out_buf + i, 1);    /* read data */
            if (res != 0)                                                           /* check the result */
            {
                handle->debug_print("mfrc522: read fifo data failed.\n");           /* read fifo data failed */
                
                return 1;                                                           /* return error */
            }
        }
    }
    if ((command == MFRC522_COMMAND_MEM) && ((*out_len) != 0))                      /* if mem and need get from fifo */
    {
        uint8_t level; 
        
        res = a_mfrc522_read(handle, MFRC522_REG_FIFO_LEVEL, &level, 1);            /* read level */
        if (res != 0)                                                               /* check the result */
        {
            handle->debug_print("mfrc522: read fifo level failed.\n");              /* read fifo level failed */
            
            return 1;                                                               /* return error */
        }
        *out_len = level > (*out_len) ? (*out_len) : level;                         /* set the output length */
        for (i = 0; i < (*out_len); i++)                                            /* loop */
        {
            res = a_mfrc522_read(handle, MFRC522_REG_FIFO_DATA, out_buf + i, 1);    /* read data */
            if (res != 0)                                                           /* check the result */
            {
                handle->debug_print("mfrc522: read fifo data failed.\n");           /* read fifo data failed */
                
                return 1;                                                           /* return error */
            }
        }
    }
    
    /* stop the timer */
    res = a_mfrc522_read(handle, MFRC522_REG_CONTROL, &prev, 1);                    /* read control */
    if (res != 0)                                                                   /* check the result */
    {
        handle->debug_print("mfrc522: read control failed.\n");                     /* read control failed */
        
        return 1;                                                                   /* return error */
    }
    prev |= 1 << 7;                                                                 /* set the stop bit */
    res = a_mfrc522_write(handle, MFRC522_REG_CONTROL, &prev, 1);                   /* write control */
    if (res != 0)                                                                   /* check the result */
    {
        handle->debug_print("mfrc522: write control failed.\n");                    /* write control failed */
        
        return 1;                                                                   /* return error */
    }
    
    /* set the idle */
    res = a_mfrc522_read(handle, MFRC522_REG_COMMAND, &prev, 1);                    /* read config */
    if (res != 0)                                                                   /* check the result */
    {
        handle->debug_print("mfrc522: read command failed.\n");                     /* read command failed */
        
        return 1;                                                                   /* return error */
    }
    prev &= ~(0xF << 0);                                                            /* clear the settings */
    prev |= MFRC522_COMMAND_IDLE;                                                   /* set the idle */
    res = a_mfrc522_write(handle, MFRC522_REG_COMMAND, &prev, 1);                   /* write config */
    if (res != 0)                                                                   /* check the result */
    {
        handle->debug_print("mfrc522: write command failed.\n");                    /* write command failed */
        
        return 1;                                                                   /* return error */
    }
    *err = prev;                                                                    /* set the 0 */
    
    return 0;                                                                       /* success return 0 */
}

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
uint8_t mfrc522_set_receiver_analog(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                  /* check handle */
    {
        return 2;                                                        /* return error */
    }
    if (handle->inited != 1)                                             /* check handle initialization */
    {
        return 3;                                                        /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_COMMAND, &prev, 1);         /* read config */
    if (res != 0)                                                        /* check the result */
    {
        handle->debug_print("mfrc522: read command failed.\n");          /* read command failed */
        
        return 1;                                                        /* return error */
    }
    prev &= ~(1 << 5);                                                   /* clear the settings */
    prev |= ((!enable) << 5) | MFRC522_COMMAND_NO_CHANGE;                /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_COMMAND, &prev, 1);        /* write config */
    if (res != 0)                                                        /* check the result */
    {
        handle->debug_print("mfrc522: write command failed.\n");         /* write command failed */
        
        return 1;                                                        /* return error */
    }
    
    return 0;                                                            /* success return 0 */
}

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
uint8_t mfrc522_get_receiver_analog(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                  /* check handle */
    {
        return 2;                                                        /* return error */
    }
    if (handle->inited != 1)                                             /* check handle initialization */
    {
        return 3;                                                        /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_COMMAND, &prev, 1);         /* read config */
    if (res != 0)                                                        /* check the result */
    {
        handle->debug_print("mfrc522: read command failed.\n");          /* read command failed */
        
        return 1;                                                        /* return error */
    }
    *enable = (mfrc522_bool_t)(!((prev >> 5) & 0x01));                   /* get the bool */
    
    return 0;                                                            /* success return 0 */
}

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
uint8_t mfrc522_set_power_down(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                  /* check handle */
    {
        return 2;                                                        /* return error */
    }
    if (handle->inited != 1)                                             /* check handle initialization */
    {
        return 3;                                                        /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_COMMAND, &prev, 1);         /* read config */
    if (res != 0)                                                        /* check the result */
    {
        handle->debug_print("mfrc522: read command failed.\n");          /* read command failed */
        
        return 1;                                                        /* return error */
    }
    prev &= ~(1 << 4);                                                   /* clear the settings */
    prev |= (enable << 4) | MFRC522_COMMAND_NO_CHANGE;                   /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_COMMAND, &prev, 1);        /* write config */
    if (res != 0)                                                        /* check the result */
    {
        handle->debug_print("mfrc522: write command failed.\n");         /* write command failed */
        
        return 1;                                                        /* return error */
    }
    
    return 0;                                                            /* success return 0 */
}

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
uint8_t mfrc522_get_power_down(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                  /* check handle */
    {
        return 2;                                                        /* return error */
    }
    if (handle->inited != 1)                                             /* check handle initialization */
    {
        return 3;                                                        /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_COMMAND, &prev, 1);         /* read config */
    if (res != 0)                                                        /* check the result */
    {
        handle->debug_print("mfrc522: read command failed.\n");          /* read command failed */
        
        return 1;                                                        /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 4) & 0x01);                      /* get the bool */
    
    return 0;                                                            /* success return 0 */
}

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
uint8_t mfrc522_set_command(mfrc522_handle_t *handle, mfrc522_command_t command)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                  /* check handle */
    {
        return 2;                                                        /* return error */
    }
    if (handle->inited != 1)                                             /* check handle initialization */
    {
        return 3;                                                        /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_COMMAND, &prev, 1);         /* read config */
    if (res != 0)                                                        /* check the result */
    {
        handle->debug_print("mfrc522: read command failed.\n");          /* read command failed */
        
        return 1;                                                        /* return error */
    }
    prev &= ~(0xF << 0);                                                 /* clear the settings */
    prev |= command;                                                     /* set the command */
    res = a_mfrc522_write(handle, MFRC522_REG_COMMAND, &prev, 1);        /* write config */
    if (res != 0)                                                        /* check the result */
    {
        handle->debug_print("mfrc522: write command failed.\n");         /* write command failed */
        
        return 1;                                                        /* return error */
    }
    
    return 0;                                                            /* success return 0 */
}

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
uint8_t mfrc522_get_command(mfrc522_handle_t *handle, mfrc522_command_t *command)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                  /* check handle */
    {
        return 2;                                                        /* return error */
    }
    if (handle->inited != 1)                                             /* check handle initialization */
    {
        return 3;                                                        /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_COMMAND, &prev, 1);         /* read config */
    if (res != 0)                                                        /* check the result */
    {
        handle->debug_print("mfrc522: read command failed.\n");          /* read command failed */
        
        return 1;                                                        /* return error */
    }
    *command = (mfrc522_command_t)((prev >> 0) & 0x0F);                  /* get the command */
    
    return 0;                                                            /* success return 0 */
}

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
uint8_t mfrc522_set_interrupt1(mfrc522_handle_t *handle, mfrc522_interrupt1_t type, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_COMIEN, &prev, 1);         /* read config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read comien failed.\n");          /* read comien failed */
        
        return 1;                                                       /* return error */
    }
    prev &= ~(1 << type);                                               /* clear the settings */
    prev |= (enable << type);                                           /* set the type */
    res = a_mfrc522_write(handle, MFRC522_REG_COMIEN, &prev, 1);        /* write config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: write comien failed.\n");         /* write comien failed */
        
        return 1;                                                       /* return error */
    }
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_get_interrupt1(mfrc522_handle_t *handle, mfrc522_interrupt1_t type, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_COMIEN, &prev, 1);         /* read config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read comien failed.\n");          /* read comien failed */
        
        return 1;                                                       /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> type) & 0x01);                  /* set the bool */
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_set_interrupt1_pin_invert(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_COMIEN, &prev, 1);         /* read config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read comien failed.\n");          /* read comien failed */
        
        return 1;                                                       /* return error */
    }
    prev &= ~(1 << 7);                                                  /* clear the settings */
    prev |= (enable << 7);                                              /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_COMIEN, &prev, 1);        /* write config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: write comien failed.\n");         /* write comien failed */
        
        return 1;                                                       /* return error */
    }
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_get_interrupt1_pin_invert(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_COMIEN, &prev, 1);         /* read config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read comien failed.\n");          /* read comien failed */
        
        return 1;                                                       /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 7) & 0x01);                     /* get the bool */
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_set_interrupt1_mark(mfrc522_handle_t *handle, mfrc522_interrupt_mark_t mark)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_COMIRQ, &prev, 1);         /* read config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read comirq failed.\n");          /* read comirq failed */
        
        return 1;                                                       /* return error */
    }
    prev &= ~(1 << 7);                                                  /* clear the settings */
    prev |= (mark << 7);                                                /* set the mark */
    res = a_mfrc522_write(handle, MFRC522_REG_COMIRQ, &prev, 1);        /* write config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: write comirq failed.\n");         /* write comirq failed */
        
        return 1;                                                       /* return error */
    }
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_set_interrupt2(mfrc522_handle_t *handle, mfrc522_interrupt2_t type, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_DIVIEN, &prev, 1);         /* read config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read divien failed.\n");          /* read divien failed */
        
        return 1;                                                       /* return error */
    }
    prev &= ~(1 << type);                                               /* clear the settings */
    prev |= (enable << type);                                           /* set the type */
    res = a_mfrc522_write(handle, MFRC522_REG_DIVIEN, &prev, 1);        /* write config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: write divien failed.\n");         /* write divien failed */
        
        return 1;                                                       /* return error */
    }
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_get_interrupt2(mfrc522_handle_t *handle, mfrc522_interrupt2_t type, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_DIVIEN, &prev, 1);         /* read config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read divien failed.\n");          /* read divien failed */
        
        return 1;                                                       /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> type) & 0x01);                  /* get the bool */
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_set_interrupt_pin_type(mfrc522_handle_t *handle, mfrc522_interrupt_pin_type_t type)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_DIVIEN, &prev, 1);         /* read config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read divien failed.\n");          /* read divien failed */
        
        return 1;                                                       /* return error */
    }
    prev &= ~(1 << 7);                                                  /* clear the settings */
    prev |= (type << 7);                                                /* set the type */
    res = a_mfrc522_write(handle, MFRC522_REG_DIVIEN, &prev, 1);        /* write config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: write divien failed.\n");         /* write divien failed */
        
        return 1;                                                       /* return error */
    }
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_get_interrupt_pin_type(mfrc522_handle_t *handle, mfrc522_interrupt_pin_type_t *type)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_DIVIEN, &prev, 1);         /* read config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read divien failed.\n");          /* read divien failed */
        
        return 1;                                                       /* return error */
    }
    *type = (mfrc522_interrupt_pin_type_t)((prev >> 7) & 0x01);         /* get the type */
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_set_interrupt2_mark(mfrc522_handle_t *handle, mfrc522_interrupt_mark_t mark)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_DIVIRQ, &prev, 1);         /* read config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read divirq failed.\n");          /* read divirq failed */
        
        return 1;                                                       /* return error */
    }
    prev &= ~(1 << 7);                                                  /* clear the settings */
    prev |= (mark << 7);                                                /* set the mark */
    res = a_mfrc522_write(handle, MFRC522_REG_DIVIRQ, &prev, 1);        /* write config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: write divirq failed.\n");         /* write divirq failed */
        
        return 1;                                                       /* return error */
    }
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_get_interrupt1_status(mfrc522_handle_t *handle, uint8_t *status)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_COMIRQ, &prev, 1);         /* read config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read comirq failed.\n");          /* read comirq failed */
        
        return 1;                                                       /* return error */
    }
    *status = prev & (~(1 << 7));                                       /* get the status */
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_get_interrupt2_status(mfrc522_handle_t *handle, uint8_t *status)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_DIVIRQ, &prev, 1);         /* read config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read divirq failed.\n");          /* read divirq failed */
        
        return 1;                                                       /* return error */
    }
    *status = prev & (~(1 << 7));                                       /* get the status */
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_get_error(mfrc522_handle_t *handle, uint8_t *err)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_ERROR, &prev, 1);         /* read config */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read error failed.\n");          /* read error failed */
        
        return 1;                                                      /* return error */
    }
    *err = prev;                                                       /* set the error */

    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_get_status1(mfrc522_handle_t *handle, uint8_t *status)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_STATUS1, &prev, 1);       /* read config */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read status1 failed.\n");        /* read status1 failed */
        
        return 1;                                                      /* return error */
    }
    *status = prev;                                                    /* set the status */

    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_get_status2(mfrc522_handle_t *handle, uint8_t *status)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_STATUS2, &prev, 1);       /* read config */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read status2 failed.\n");        /* read status2 failed */
        
        return 1;                                                      /* return error */
    }
    *status = prev & (1 << 3);                                         /* set the status */

    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_get_modem_state(mfrc522_handle_t *handle, mfrc522_modem_state_t *state)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_STATUS2, &prev, 1);       /* read config */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read status2 failed.\n");        /* read status2 failed */
        
        return 1;                                                      /* return error */
    }
    *state = (mfrc522_modem_state_t)(prev & 0x7);                      /* set the modem state */

    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_set_mifare_crypto1_on(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_STATUS2, &prev, 1);        /* read config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read status2 failed.\n");         /* read status2 failed */
        
        return 1;                                                       /* return error */
    }
    prev &= ~(1 << 3);                                                  /* clear the settings */
    prev |= (enable << 3);                                              /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_STATUS2, &prev, 1);       /* write config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: write status2 failed.\n");        /* write status2 failed */
        
        return 1;                                                       /* return error */
    }
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_set_force_iic_high_speed(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_STATUS2, &prev, 1);        /* read config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read status2 failed.\n");         /* read status2 failed */
        
        return 1;                                                       /* return error */
    }
    prev &= ~(1 << 6);                                                  /* clear the settings */
    prev |= (enable << 6);                                              /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_STATUS2, &prev, 1);       /* write config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: write status2 failed.\n");        /* write status2 failed */
        
        return 1;                                                       /* return error */
    }
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_get_force_iic_high_speed(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_STATUS2, &prev, 1);        /* read config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read status2 failed.\n");         /* read status2 failed */
        
        return 1;                                                       /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 6) & 0x01);                     /* get the bool */
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_set_clear_temperature_error(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_STATUS2, &prev, 1);        /* read config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read status2 failed.\n");         /* read status2 failed */
        
        return 1;                                                       /* return error */
    }
    prev &= ~(1 << 7);                                                  /* clear the settings */
    prev |= (enable << 7);                                              /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_STATUS2, &prev, 1);       /* write config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: write status2 failed.\n");        /* write status2 failed */
        
        return 1;                                                       /* return error */
    }
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_get_clear_temperature_error(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_STATUS2, &prev, 1);        /* read config */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read status2 failed.\n");         /* read status2 failed */
        
        return 1;                                                       /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 7) & 0x01);                     /* get the bool */
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_set_fifo_data(mfrc522_handle_t *handle, uint8_t *data, uint8_t len)
{
    uint8_t res;
    uint8_t i;
    
    if (handle == NULL)                                                           /* check handle */
    {
        return 2;                                                                 /* return error */
    }
    if (handle->inited != 1)                                                      /* check handle initialization */
    {
        return 3;                                                                 /* return error */
    }
    if (len > 64)                                                                 /* check the length */
    {
        handle->debug_print("mfrc522: len is over 64.\n");                        /* len is over 64 */
        
        return 4;                                                                 /* return error */
    }
    
    for (i = 0; i < len; i++)                                                     /* loop */
    {
        res = a_mfrc522_write(handle, MFRC522_REG_FIFO_DATA, data + i, 1);        /* write data */
        if (res != 0)                                                             /* check the result */
        {
            handle->debug_print("mfrc522: write fifo data failed.\n");            /* write fifo data failed */
            
            return 1;                                                             /* return error */
        }
    }
    
    return 0;                                                                     /* success return 0 */
}

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
uint8_t mfrc522_get_fifo_data(mfrc522_handle_t *handle, uint8_t *data, uint8_t len)
{
    uint8_t res;
    uint8_t i;
    
    if (handle == NULL)                                                          /* check handle */
    {
        return 2;                                                                /* return error */
    }
    if (handle->inited != 1)                                                     /* check handle initialization */
    {
        return 3;                                                                /* return error */
    }
    if (len > 64)                                                                /* check the length */
    {
        handle->debug_print("mfrc522: len is over 64.\n");                       /* len is over 64 */
        
        return 4;                                                                /* return error */
    }
    
    for (i = 0; i < len; i++)                                                    /* loop */
    {
        res = a_mfrc522_read(handle, MFRC522_REG_FIFO_DATA, data + i, 1);        /* read data */
        if (res != 0)                                                            /* check the result */
        {
            handle->debug_print("mfrc522: read fifo data failed.\n");            /* read fifo data failed */
            
            return 1;                                                            /* return error */
        }
    }
    
    return 0;                                                                    /* success return 0 */
}

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
uint8_t mfrc522_get_fifo_level(mfrc522_handle_t *handle, uint8_t *level)
{
    uint8_t res;
    
    if (handle == NULL)                                                  /* check handle */
    {
        return 2;                                                        /* return error */
    }
    if (handle->inited != 1)                                             /* check handle initialization */
    {
        return 3;                                                        /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_FIFO_LEVEL, level, 1);      /* read level */
    if (res != 0)                                                        /* check the result */
    {
        handle->debug_print("mfrc522: read fifo level failed.\n");       /* read fifo level failed */
        
        return 1;                                                        /* return error */
    }
    
    return 0;                                                            /* success return 0 */
}

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
uint8_t mfrc522_flush_fifo(mfrc522_handle_t *handle)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                  /* check handle */
    {
        return 2;                                                        /* return error */
    }
    if (handle->inited != 1)                                             /* check handle initialization */
    {
        return 3;                                                        /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_FIFO_LEVEL, &prev, 1);      /* read level */
    if (res != 0)                                                        /* check the result */
    {
        handle->debug_print("mfrc522: read fifo level failed.\n");       /* read fifo level failed */
        
        return 1;                                                        /* return error */
    }
    prev |= 1 << 7;                                                      /* set the flush bit */
    res = a_mfrc522_write(handle, MFRC522_REG_FIFO_LEVEL, &prev, 1);     /* write level */
    if (res != 0)                                                        /* check the result */
    {
        handle->debug_print("mfrc522: write fifo level failed.\n");      /* write fifo level failed */
        
        return 1;                                                        /* return error */
    }
    
    return 0;                                                            /* success return 0 */
}

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
uint8_t mfrc522_get_water_level(mfrc522_handle_t *handle, uint8_t *level)
{
    uint8_t res;
    
    if (handle == NULL)                                                  /* check handle */
    {
        return 2;                                                        /* return error */
    }
    if (handle->inited != 1)                                             /* check handle initialization */
    {
        return 3;                                                        /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_WATER_LEVEL, level, 1);     /* read level */
    if (res != 0)                                                        /* check the result */
    {
        handle->debug_print("mfrc522: read water level failed.\n");      /* read water level failed */
        
        return 1;                                                        /* return error */
    }
    *level &= ~(3 << 6);                                                 /* clear the bits */
    
    return 0;                                                            /* success return 0 */
}

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
uint8_t mfrc522_set_water_level(mfrc522_handle_t *handle, uint8_t level)
{
    uint8_t res;
    
    if (handle == NULL)                                                  /* check handle */
    {
        return 2;                                                        /* return error */
    }
    if (handle->inited != 1)                                             /* check handle initialization */
    {
        return 3;                                                        /* return error */
    }
    if (level > 0x3F)                                                    /* check the level */
    {
        handle->debug_print("mfrc522: level is over 0x3F.\n");           /* level is over 0x3F */
        
        return 4;                                                        /* return error */
    }
    
    res = a_mfrc522_write(handle, MFRC522_REG_WATER_LEVEL, &level, 1);   /* write level */
    if (res != 0)                                                        /* check the result */
    {
        handle->debug_print("mfrc522: write water level failed.\n");     /* write water level failed */
        
        return 1;                                                        /* return error */
    }
    
    return 0;                                                            /* success return 0 */
}

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
uint8_t mfrc522_stop_timer(mfrc522_handle_t *handle)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_CONTROL, &prev, 1);        /* read control */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read control failed.\n");         /* read control failed */
        
        return 1;                                                       /* return error */
    }
    prev |= 1 << 7;                                                     /* set the stop bit */
    res = a_mfrc522_write(handle, MFRC522_REG_CONTROL, &prev, 1);       /* write control */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: write control failed.\n");        /* write control failed */
        
        return 1;                                                       /* return error */
    }
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_start_timer(mfrc522_handle_t *handle)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_CONTROL, &prev, 1);        /* read control */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read control failed.\n");         /* read control failed */
        
        return 1;                                                       /* return error */
    }
    prev |= 1 << 6;                                                     /* set the start bit */
    res = a_mfrc522_write(handle, MFRC522_REG_CONTROL, &prev, 1);       /* write control */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: write control failed.\n");        /* write control failed */
        
        return 1;                                                       /* return error */
    }
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_get_rx_last_bits(mfrc522_handle_t *handle, uint8_t *bits)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_CONTROL, &prev, 1);        /* read control */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read control failed.\n");         /* read control failed */
        
        return 1;                                                       /* return error */
    }
    *bits = prev & (0x7 << 0);                                          /* set bits */
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_start_send(mfrc522_handle_t *handle)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                      /* check handle */
    {
        return 2;                                                            /* return error */
    }
    if (handle->inited != 1)                                                 /* check handle initialization */
    {
        return 3;                                                            /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_BIT_FRAMING, &prev, 1);         /* read bit framing */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: read bit framing failed.\n");          /* read bit framing failed */
        
        return 1;                                                            /* return error */
    }
    prev &= ~(1 << 7);                                                       /* clear the settings */
    prev |= 1 << 7;                                                          /* set the start bit */
    res = a_mfrc522_write(handle, MFRC522_REG_BIT_FRAMING, &prev, 1);        /* write bit framing */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: write bit framing failed.\n");         /* write bit framing failed */
        
        return 1;                                                            /* return error */
    }
    
    return 0;                                                                /* success return 0 */
}

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
uint8_t mfrc522_stop_send(mfrc522_handle_t *handle)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                      /* check handle */
    {
        return 2;                                                            /* return error */
    }
    if (handle->inited != 1)                                                 /* check handle initialization */
    {
        return 3;                                                            /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_BIT_FRAMING, &prev, 1);         /* read bit framing */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: read bit framing failed.\n");          /* read bit framing failed */
        
        return 1;                                                            /* return error */
    }
    prev &= ~(1 << 7);                                                       /* clear the settings */
    res = a_mfrc522_write(handle, MFRC522_REG_BIT_FRAMING, &prev, 1);        /* write bit framing */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: write bit framing failed.\n");         /* write bit framing failed */
        
        return 1;                                                            /* return error */
    }
    
    return 0;                                                                /* success return 0 */
}

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
uint8_t mfrc522_get_tx_last_bits(mfrc522_handle_t *handle, uint8_t *bits)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_BIT_FRAMING, &prev, 1);    /* read bit framing */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read bit framing failed.\n");     /* read bit framing failed */
        
        return 1;                                                       /* return error */
    }
    *bits = prev & (0x7 << 0);                                          /* set bits */
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_set_tx_last_bits(mfrc522_handle_t *handle, uint8_t bits)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                      /* check handle */
    {
        return 2;                                                            /* return error */
    }
    if (handle->inited != 1)                                                 /* check handle initialization */
    {
        return 3;                                                            /* return error */
    }
    if (bits > 7)                                                            /* check the length */
    {
        handle->debug_print("mfrc522: bits is over 7.\n");                   /* bits is over 7 */
        
        return 4;                                                            /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_BIT_FRAMING, &prev, 1);         /* read bit framing */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: read bit framing failed.\n");          /* read bit framing failed */
        
        return 1;                                                            /* return error */
    }
    prev &= ~(7 << 0);                                                       /* clear the settings */
    prev |= bits << 0;                                                       /* set the bits */
    res = a_mfrc522_write(handle, MFRC522_REG_BIT_FRAMING, &prev, 1);        /* write bit framing */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: write bit framing failed.\n");         /* write bit framing failed */
        
        return 1;                                                            /* return error */
    }
    
    return 0;                                                                /* success return 0 */
}

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
uint8_t mfrc522_set_rx_align(mfrc522_handle_t *handle, mfrc522_rx_align_t align)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                      /* check handle */
    {
        return 2;                                                            /* return error */
    }
    if (handle->inited != 1)                                                 /* check handle initialization */
    {
        return 3;                                                            /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_BIT_FRAMING, &prev, 1);         /* read bit framing */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: read bit framing failed.\n");          /* read bit framing failed */
        
        return 1;                                                            /* return error */
    }
    prev &= ~(7 << 4);                                                       /* clear the settings */
    prev |= align << 4;                                                      /* set the align */
    res = a_mfrc522_write(handle, MFRC522_REG_BIT_FRAMING, &prev, 1);        /* write bit framing */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: write bit framing failed.\n");         /* write bit framing failed */
        
        return 1;                                                            /* return error */
    }
    
    return 0;                                                                /* success return 0 */
}

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
uint8_t mfrc522_get_rx_align(mfrc522_handle_t *handle, mfrc522_rx_align_t *align)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                      /* check handle */
    {
        return 2;                                                            /* return error */
    }
    if (handle->inited != 1)                                                 /* check handle initialization */
    {
        return 3;                                                            /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_BIT_FRAMING, &prev, 1);         /* read bit framing */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: read bit framing failed.\n");          /* read bit framing failed */
        
        return 1;                                                            /* return error */
    }
    *align = (mfrc522_rx_align_t)((prev >> 4) & 0x07);                       /* get the align */
    
    return 0;                                                                /* success return 0 */
}

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
uint8_t mfrc522_set_value_clear_after_coll(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_COLL, &prev, 1);         /* read coll */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read coll failed.\n");          /* read coll failed */
        
        return 1;                                                     /* return error */
    }
    prev &= ~(1 << 7);                                                /* clear the settings */
    prev |= (!enable) << 7;                                           /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_COLL, &prev, 1);        /* write coll */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: write coll failed.\n");         /* write coll failed */
        
        return 1;                                                     /* return error */
    }
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_get_value_clear_after_coll(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_COLL, &prev, 1);         /* read coll */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read coll failed.\n");          /* read coll failed */
        
        return 1;                                                     /* return error */
    }
    *enable = (mfrc522_bool_t)(!((prev >> 7) & 0x01));                /* get the bool */
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_get_collision_position_not_valid(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_COLL, &prev, 1);         /* read coll */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read coll failed.\n");          /* read coll failed */
        
        return 1;                                                     /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 5) & 0x01);                   /* get the bool */
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_get_collision_position(mfrc522_handle_t *handle, uint8_t *pos)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_COLL, &prev, 1);         /* read coll */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read coll failed.\n");          /* read coll failed */
        
        return 1;                                                     /* return error */
    }
    *pos = prev & 0x1F;                                               /* get the pos */
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_set_crc_msb_first(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_MODE, &prev, 1);         /* read mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read mode failed.\n");          /* read mode failed */
        
        return 1;                                                     /* return error */
    }
    prev &= ~(1 << 7);                                                /* clear the settings */
    prev |= enable << 7;                                              /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_MODE, &prev, 1);        /* write mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: write mode failed.\n");         /* write mode failed */
        
        return 1;                                                     /* return error */
    }
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_get_crc_msb_first(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_MODE, &prev, 1);         /* read mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read mode failed.\n");          /* read mode failed */
        
        return 1;                                                     /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 7) & 0x01);                   /* get the bool */
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_set_tx_wait_rf(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_MODE, &prev, 1);         /* read mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read mode failed.\n");          /* read mode failed */
        
        return 1;                                                     /* return error */
    }
    prev &= ~(1 << 5);                                                /* clear the settings */
    prev |= enable << 5;                                              /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_MODE, &prev, 1);        /* write mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: write mode failed.\n");         /* write mode failed */
        
        return 1;                                                     /* return error */
    }
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_get_tx_wait_rf(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_MODE, &prev, 1);         /* read mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read mode failed.\n");          /* read mode failed */
        
        return 1;                                                     /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 5) & 0x01);                   /* get the bool */
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_set_mfin_polarity(mfrc522_handle_t *handle, mfrc522_mfin_polarity_t polarity)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_MODE, &prev, 1);         /* read mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read mode failed.\n");          /* read mode failed */
        
        return 1;                                                     /* return error */
    }
    prev &= ~(1 << 3);                                                /* clear the settings */
    prev |= polarity << 3;                                            /* set the polarity */
    res = a_mfrc522_write(handle, MFRC522_REG_MODE, &prev, 1);        /* write mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: write mode failed.\n");         /* write mode failed */
        
        return 1;                                                     /* return error */
    }
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_get_mfin_polarity(mfrc522_handle_t *handle, mfrc522_mfin_polarity_t *polarity)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_MODE, &prev, 1);         /* read mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read mode failed.\n");          /* read mode failed */
        
        return 1;                                                     /* return error */
    }
    *polarity = (mfrc522_mfin_polarity_t)((prev >> 3) & 0x01);        /* get the polarity */
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_set_crc_preset(mfrc522_handle_t *handle, mfrc522_crc_preset_t preset)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_MODE, &prev, 1);         /* read mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read mode failed.\n");          /* read mode failed */
        
        return 1;                                                     /* return error */
    }
    prev &= ~(3 << 0);                                                /* clear the settings */
    prev |= preset << 0;                                              /* set the preset */
    res = a_mfrc522_write(handle, MFRC522_REG_MODE, &prev, 1);        /* write mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: write mode failed.\n");         /* write mode failed */
        
        return 1;                                                     /* return error */
    }
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_get_crc_preset(mfrc522_handle_t *handle, mfrc522_crc_preset_t *preset)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_MODE, &prev, 1);         /* read mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read mode failed.\n");          /* read mode failed */
        
        return 1;                                                     /* return error */
    }
    *preset = (mfrc522_crc_preset_t)(prev & 0x3);                     /* get the preset */
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_set_tx_crc_generation(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TX_MODE, &prev, 1);      /* read tx mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read tx mode failed.\n");       /* read tx mode failed */
        
        return 1;                                                     /* return error */
    }
    prev &= ~(1 << 7);                                                /* clear the settings */
    prev |= enable << 7;                                              /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_TX_MODE, &prev, 1);     /* write tx mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: write tx mode failed.\n");      /* write tx mode failed */
        
        return 1;                                                     /* return error */
    }
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_get_tx_crc_generation(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TX_MODE, &prev, 1);      /* read tx mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read tx mode failed.\n");       /* read tx mode failed */
        
        return 1;                                                     /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 7) & 0x01);                   /* get the bool */
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_set_tx_speed(mfrc522_handle_t *handle, mfrc522_speed_t speed)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TX_MODE, &prev, 1);      /* read tx mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read tx mode failed.\n");       /* read tx mode failed */
        
        return 1;                                                     /* return error */
    }
    prev &= ~(7 << 4);                                                /* clear the settings */
    prev |= speed << 4;                                               /* set the speed */
    res = a_mfrc522_write(handle, MFRC522_REG_TX_MODE, &prev, 1);     /* write tx mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: write tx mode failed.\n");      /* write tx mode failed */
        
        return 1;                                                     /* return error */
    }
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_get_tx_speed(mfrc522_handle_t *handle, mfrc522_speed_t *speed)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TX_MODE, &prev, 1);      /* read tx mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read tx mode failed.\n");       /* read tx mode failed */
        
        return 1;                                                     /* return error */
    }
    *speed = (mfrc522_speed_t)((prev >> 4) & 0x7);                    /* get the bool */
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_set_modulation_invert(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TX_MODE, &prev, 1);      /* read tx mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read tx mode failed.\n");       /* read tx mode failed */
        
        return 1;                                                     /* return error */
    }
    prev &= ~(1 << 3);                                                /* clear the settings */
    prev |= enable << 3;                                              /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_TX_MODE, &prev, 1);     /* write tx mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: write tx mode failed.\n");      /* write tx mode failed */
        
        return 1;                                                     /* return error */
    }
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_get_modulation_invert(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TX_MODE, &prev, 1);      /* read tx mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read tx mode failed.\n");       /* read tx mode failed */
        
        return 1;                                                     /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 3) & 0x01);                   /* get the bool */
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_set_rx_crc_generation(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_RX_MODE, &prev, 1);      /* read rx mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read rx mode failed.\n");       /* read rx mode failed */
        
        return 1;                                                     /* return error */
    }
    prev &= ~(1 << 7);                                                /* clear the settings */
    prev |= enable << 7;                                              /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_RX_MODE, &prev, 1);     /* write rx mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: write rx mode failed.\n");      /* write rx mode failed */
        
        return 1;                                                     /* return error */
    }
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_get_rx_crc_generation(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_RX_MODE, &prev, 1);      /* read rx mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read rx mode failed.\n");       /* read rx mode failed */
        
        return 1;                                                     /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 7) & 0x01);                   /* get the bool */
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_set_rx_speed(mfrc522_handle_t *handle, mfrc522_speed_t speed)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_RX_MODE, &prev, 1);      /* read rx mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read rx mode failed.\n");       /* read rx mode failed */
        
        return 1;                                                     /* return error */
    }
    prev &= ~(7 << 4);                                                /* clear the settings */
    prev |= speed << 4;                                               /* set the speed */
    res = a_mfrc522_write(handle, MFRC522_REG_RX_MODE, &prev, 1);     /* write rx mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: write rx mode failed.\n");      /* write rx mode failed */
        
        return 1;                                                     /* return error */
    }
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_get_rx_speed(mfrc522_handle_t *handle, mfrc522_speed_t *speed)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_RX_MODE, &prev, 1);      /* read rx mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read rx mode failed.\n");       /* read rx mode failed */
        
        return 1;                                                     /* return error */
    }
    *speed = (mfrc522_speed_t)((prev >> 4) & 0x07);                   /* get the speed */
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_set_rx_no_error(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_RX_MODE, &prev, 1);      /* read rx mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read rx mode failed.\n");       /* read rx mode failed */
        
        return 1;                                                     /* return error */
    }
    prev &= ~(1 << 3);                                                /* clear the settings */
    prev |= enable << 3;                                              /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_RX_MODE, &prev, 1);     /* write rx mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: write rx mode failed.\n");      /* write rx mode failed */
        
        return 1;                                                     /* return error */
    }
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_get_rx_no_error(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_RX_MODE, &prev, 1);      /* read rx mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read rx mode failed.\n");       /* read rx mode failed */
        
        return 1;                                                     /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 3) & 0x01);                   /* get the bool */
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_set_rx_multiple(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_RX_MODE, &prev, 1);      /* read rx mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read rx mode failed.\n");       /* read rx mode failed */
        
        return 1;                                                     /* return error */
    }
    prev &= ~(1 << 2);                                                /* clear the settings */
    prev |= enable << 2;                                              /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_RX_MODE, &prev, 1);     /* write rx mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: write rx mode failed.\n");      /* write rx mode failed */
        
        return 1;                                                     /* return error */
    }
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_get_rx_multiple(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_RX_MODE, &prev, 1);      /* read rx mode */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read rx mode failed.\n");       /* read rx mode failed */
        
        return 1;                                                     /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 2) & 0x01);                   /* get the bool */
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_set_antenna_driver(mfrc522_handle_t *handle, mfrc522_antenna_driver_t driver, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                     /* check handle */
    {
        return 2;                                                           /* return error */
    }
    if (handle->inited != 1)                                                /* check handle initialization */
    {
        return 3;                                                           /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TX_CONTROL, &prev, 1);         /* read tx control */
    if (res != 0)                                                           /* check the result */
    {
        handle->debug_print("mfrc522: read tx control failed.\n");          /* read tx control failed */
        
        return 1;                                                           /* return error */
    }
    prev &= ~(1 << driver);                                                 /* clear the settings */
    prev |= enable << driver;                                               /* set the driver */
    res = a_mfrc522_write(handle, MFRC522_REG_TX_CONTROL, &prev, 1);        /* write tx control */
    if (res != 0)                                                           /* check the result */
    {
        handle->debug_print("mfrc522: write tx control failed.\n");         /* write tx control failed */
        
        return 1;                                                           /* return error */
    }
    
    return 0;                                                               /* success return 0 */
}

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
uint8_t mfrc522_get_antenna_driver(mfrc522_handle_t *handle, mfrc522_antenna_driver_t driver, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                     /* check handle */
    {
        return 2;                                                           /* return error */
    }
    if (handle->inited != 1)                                                /* check handle initialization */
    {
        return 3;                                                           /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TX_CONTROL, &prev, 1);         /* read tx control */
    if (res != 0)                                                           /* check the result */
    {
        handle->debug_print("mfrc522: read tx control failed.\n");          /* read tx control failed */
        
        return 1;                                                           /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> driver) & 0x01);                    /* get the bool */
    
    return 0;                                                               /* success return 0 */
}

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
uint8_t mfrc522_set_force_100_ask(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TX_ASK, &prev, 1);        /* read tx ask */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read tx ask failed.\n");         /* read tx ask failed */
        
        return 1;                                                      /* return error */
    }
    prev &= ~(1 << 6);                                                 /* clear the settings */
    prev |= enable << 6;                                               /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_TX_ASK, &prev, 1);       /* write tx ask */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: write tx ask failed.\n");        /* write tx ask failed */
        
        return 1;                                                      /* return error */
    }
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_get_force_100_ask(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TX_ASK, &prev, 1);        /* read tx ask */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read tx ask failed.\n");         /* read tx ask failed */
        
        return 1;                                                      /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 6) & 0x1);                     /* get the bool */
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_set_tx_input(mfrc522_handle_t *handle, mfrc522_tx_input_t input)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TX_SEL, &prev, 1);        /* read tx sel */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read tx sel failed.\n");         /* read tx sel failed */
        
        return 1;                                                      /* return error */
    }
    prev &= ~(3 << 4);                                                 /* clear the settings */
    prev |= input << 4;                                                /* set the input */
    res = a_mfrc522_write(handle, MFRC522_REG_TX_SEL, &prev, 1);       /* write tx sel */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: write tx sel failed.\n");        /* write tx sel failed */
        
        return 1;                                                      /* return error */
    }
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_get_tx_input(mfrc522_handle_t *handle, mfrc522_tx_input_t *input)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TX_SEL, &prev, 1);        /* read tx sel */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read tx sel failed.\n");         /* read tx sel failed */
        
        return 1;                                                      /* return error */
    }
    *input = (mfrc522_tx_input_t)((prev >> 4) & 0x3);                  /* get the input */
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_set_mfout_input(mfrc522_handle_t *handle, mfrc522_mfout_input_t input)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TX_SEL, &prev, 1);        /* read tx sel */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read tx sel failed.\n");         /* read tx sel failed */
        
        return 1;                                                      /* return error */
    }
    prev &= ~(0xF << 0);                                               /* clear the settings */
    prev |= input << 0;                                                /* set the input */
    res = a_mfrc522_write(handle, MFRC522_REG_TX_SEL, &prev, 1);       /* write tx sel */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: write tx sel failed.\n");        /* write tx sel failed */
        
        return 1;                                                      /* return error */
    }
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_get_mfout_input(mfrc522_handle_t *handle, mfrc522_mfout_input_t *input)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TX_SEL, &prev, 1);        /* read tx sel */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read tx sel failed.\n");         /* read tx sel failed */
        
        return 1;                                                      /* return error */
    }
    *input = (mfrc522_mfout_input_t)(prev & 0x0F);                     /* get the input */
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_set_contactless_uart_input(mfrc522_handle_t *handle, mfrc522_contactless_uart_input_t input)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_RX_SEL, &prev, 1);        /* read rx sel */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read rx sel failed.\n");         /* read rx sel failed */
        
        return 1;                                                      /* return error */
    }
    prev &= ~(3 << 6);                                                 /* clear the settings */
    prev |= input << 6;                                                /* set the input */
    res = a_mfrc522_write(handle, MFRC522_REG_RX_SEL, &prev, 1);       /* write rx sel */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: write rx sel failed.\n");        /* write rx sel failed */
        
        return 1;                                                      /* return error */
    }
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_get_contactless_uart_input(mfrc522_handle_t *handle, mfrc522_contactless_uart_input_t *input)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_RX_SEL, &prev, 1);        /* read rx sel */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read rx sel failed.\n");         /* read rx sel failed */
        
        return 1;                                                      /* return error */
    }
    *input = (mfrc522_contactless_uart_input_t)((prev >> 6) & 0x3);    /* get th input */
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_set_rx_wait(mfrc522_handle_t *handle, uint8_t t)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    if (t > 0x3F)                                                      /* check t */
    {
        handle->debug_print("mfrc522: t is over 0x3F.\n");             /* t is over 0x3F */
        
        return 4;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_RX_SEL, &prev, 1);        /* read rx sel */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read rx sel failed.\n");         /* read rx sel failed */
        
        return 1;                                                      /* return error */
    }
    prev &= ~(0x3F << 0);                                              /* clear the settings */
    prev |= t << 0;                                                    /* set the input */
    res = a_mfrc522_write(handle, MFRC522_REG_RX_SEL, &prev, 1);       /* write rx sel */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: write rx sel failed.\n");        /* write rx sel failed */
        
        return 1;                                                      /* return error */
    }
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_get_rx_wait(mfrc522_handle_t *handle, uint8_t *t)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_RX_SEL, &prev, 1);        /* read rx sel */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read rx sel failed.\n");         /* read rx sel failed */
        
        return 1;                                                      /* return error */
    }
    *t = prev & 0x3F;                                                  /* get the rx wait */
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_set_min_level(mfrc522_handle_t *handle, uint8_t level)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                      /* check handle */
    {
        return 2;                                                            /* return error */
    }
    if (handle->inited != 1)                                                 /* check handle initialization */
    {
        return 3;                                                            /* return error */
    }
    if (level > 0xF)                                                         /* check the level */
    {
        handle->debug_print("mfrc522: level is over 0xF.\n");                /* level is over 0xF */
        
        return 4;                                                            /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_RX_THRESHOLD, &prev, 1);        /* read rx threshold */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: read rx threshold failed.\n");         /* read rx threshold failed */
        
        return 1;                                                            /* return error */
    }
    prev &= ~(0xF << 4);                                                     /* clear the settings */
    prev |= level << 4;                                                      /* set the level */
    res = a_mfrc522_write(handle, MFRC522_REG_RX_THRESHOLD, &prev, 1);       /* write rx threshold */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: write rx threshold failed.\n");        /* write rx threshold failed */
        
        return 1;                                                            /* return error */
    }
    
    return 0;                                                                /* success return 0 */
}

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
uint8_t mfrc522_get_min_level(mfrc522_handle_t *handle, uint8_t *level)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                      /* check handle */
    {
        return 2;                                                            /* return error */
    }
    if (handle->inited != 1)                                                 /* check handle initialization */
    {
        return 3;                                                            /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_RX_THRESHOLD, &prev, 1);        /* read rx threshold */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: read rx threshold failed.\n");         /* read rx threshold failed */
        
        return 1;                                                            /* return error */
    }
    *level = (prev >> 4) & 0xF;                                              /* get the level */
    
    return 0;                                                                /* success return 0 */
}

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
uint8_t mfrc522_set_collision_level(mfrc522_handle_t *handle, uint8_t level)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                      /* check handle */
    {
        return 2;                                                            /* return error */
    }
    if (handle->inited != 1)                                                 /* check handle initialization */
    {
        return 3;                                                            /* return error */
    }
    if (level > 7)                                                           /* check the level */
    {
        handle->debug_print("mfrc522: level is over 7.\n");                  /* level is over 7 */
        
        return 4;                                                            /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_RX_THRESHOLD, &prev, 1);        /* read rx threshold */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: read rx threshold failed.\n");         /* read rx threshold failed */
        
        return 1;                                                            /* return error */
    }
    prev &= ~(0x7 << 0);                                                     /* clear the settings */
    prev |= level << 0;                                                      /* set the level */
    res = a_mfrc522_write(handle, MFRC522_REG_RX_THRESHOLD, &prev, 1);       /* write rx threshold */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: write rx threshold failed.\n");        /* write rx threshold failed */
        
        return 1;                                                            /* return error */
    }
    
    return 0;                                                                /* success return 0 */
}

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
uint8_t mfrc522_get_collision_level(mfrc522_handle_t *handle, uint8_t *level)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                      /* check handle */
    {
        return 2;                                                            /* return error */
    }
    if (handle->inited != 1)                                                 /* check handle initialization */
    {
        return 3;                                                            /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_RX_THRESHOLD, &prev, 1);        /* read rx threshold */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: read rx threshold failed.\n");         /* read rx threshold failed */
        
        return 1;                                                            /* return error */
    }
    *level = prev & 0x07;                                                    /* get the level */
    
    return 0;                                                                /* success return 0 */
}

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
uint8_t mfrc522_set_channel_reception(mfrc522_handle_t *handle, mfrc522_channel_reception_t reception)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_DEMOD, &prev, 1);        /* read demod */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read demod failed.\n");         /* read demod failed */
        
        return 1;                                                     /* return error */
    }
    prev &= ~(0x3 << 6);                                              /* clear the settings */
    prev |= reception << 6;                                           /* set the reception */
    res = a_mfrc522_write(handle, MFRC522_REG_DEMOD, &prev, 1);       /* write demod */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: write demod failed.\n");        /* write demod failed */
        
        return 1;                                                     /* return error */
    }
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_get_channel_reception(mfrc522_handle_t *handle, mfrc522_channel_reception_t *reception)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_DEMOD, &prev, 1);        /* read demod */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read demod failed.\n");         /* read demod failed */
        
        return 1;                                                     /* return error */
    }
    *reception = (mfrc522_channel_reception_t)((prev >> 6) & 0x03);   /* get the reception */
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_set_fix_iq(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_DEMOD, &prev, 1);        /* read demod */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read demod failed.\n");         /* read demod failed */
        
        return 1;                                                     /* return error */
    }
    prev &= ~(1 << 5);                                                /* clear the settings */
    prev |= enable << 5;                                              /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_DEMOD, &prev, 1);       /* write demod */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: write demod failed.\n");        /* write demod failed */
        
        return 1;                                                     /* return error */
    }
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_get_fix_iq(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_DEMOD, &prev, 1);        /* read demod */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read demod failed.\n");         /* read demod failed */
        
        return 1;                                                     /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 5) & 0x01);                   /* set the bool */
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_set_timer_prescal_even(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_DEMOD, &prev, 1);        /* read demod */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read demod failed.\n");         /* read demod failed */
        
        return 1;                                                     /* return error */
    }
    prev &= ~(1 << 4);                                                /* clear the settings */
    prev |= enable << 4;                                              /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_DEMOD, &prev, 1);       /* write demod */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: write demod failed.\n");        /* write demod failed */
        
        return 1;                                                     /* return error */
    }
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_get_timer_prescal_even(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_DEMOD, &prev, 1);        /* read demod */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read demod failed.\n");         /* read demod failed */
        
        return 1;                                                     /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 4) & 0x01);                   /* get the bool */
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_set_timer_constant_reception(mfrc522_handle_t *handle, uint8_t t)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    if (t > 3)                                                        /* check the t */
    {
        handle->debug_print("mfrc522: t is over 3.\n");               /* t is over 3 */
        
        return 4;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_DEMOD, &prev, 1);        /* read demod */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read demod failed.\n");         /* read demod failed */
        
        return 1;                                                     /* return error */
    }
    prev &= ~(3 << 2);                                                /* clear the settings */
    prev |= t << 2;                                                   /* set the pll */
    res = a_mfrc522_write(handle, MFRC522_REG_DEMOD, &prev, 1);       /* write demod */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: write demod failed.\n");        /* write demod failed */
        
        return 1;                                                     /* return error */
    }
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_get_timer_constant_reception(mfrc522_handle_t *handle, uint8_t *t)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_DEMOD, &prev, 1);        /* read demod */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read demod failed.\n");         /* read demod failed */
        
        return 1;                                                     /* return error */
    }
    *t = (prev >> 2) & 0x3;                                           /* get the pll */
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_set_timer_constant_sync(mfrc522_handle_t *handle, uint8_t t)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    if (t > 3)                                                        /* check the t */
    {
        handle->debug_print("mfrc522: t is over 3.\n");               /* t is over 3 */
        
        return 4;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_DEMOD, &prev, 1);        /* read demod */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read demod failed.\n");         /* read demod failed */
        
        return 1;                                                     /* return error */
    }
    prev &= ~(3 << 0);                                                /* clear the settings */
    prev |= t << 0;                                                   /* set the pll */
    res = a_mfrc522_write(handle, MFRC522_REG_DEMOD, &prev, 1);       /* write demod */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: write demod failed.\n");        /* write demod failed */
        
        return 1;                                                     /* return error */
    }
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_get_timer_constant_sync(mfrc522_handle_t *handle, uint8_t *t)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_DEMOD, &prev, 1);        /* read demod */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read demod failed.\n");         /* read demod failed */
        
        return 1;                                                     /* return error */
    }
    *t = (prev >> 0) & 0x3;                                           /* get the pll */
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_set_tx_wait(mfrc522_handle_t *handle, uint8_t t)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                              /* check handle */
    {
        return 2;                                                    /* return error */
    }
    if (handle->inited != 1)                                         /* check handle initialization */
    {
        return 3;                                                    /* return error */
    }
    if (t > 3)                                                       /* check the t */
    {
        handle->debug_print("mfrc522: t is over 3.\n");              /* t is over 3 */
        
        return 4;                                                    /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_MFTX, &prev, 1);        /* read mftx */
    if (res != 0)                                                    /* check the result */
    {
        handle->debug_print("mfrc522: read mftx failed.\n");         /* read mftx failed */
        
        return 1;                                                    /* return error */
    }
    prev &= ~(3 << 0);                                               /* clear the settings */
    prev |= t << 0;                                                  /* set the wait */
    res = a_mfrc522_write(handle, MFRC522_REG_MFTX, &prev, 1);       /* write mftx */
    if (res != 0)                                                    /* check the result */
    {
        handle->debug_print("mfrc522: write mftx failed.\n");        /* write mftx failed */
        
        return 1;                                                    /* return error */
    }
    
    return 0;                                                        /* success return 0 */
}

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
uint8_t mfrc522_get_tx_wait(mfrc522_handle_t *handle, uint8_t *t)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                              /* check handle */
    {
        return 2;                                                    /* return error */
    }
    if (handle->inited != 1)                                         /* check handle initialization */
    {
        return 3;                                                    /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_MFTX, &prev, 1);        /* read mftx */
    if (res != 0)                                                    /* check the result */
    {
        handle->debug_print("mfrc522: read mftx failed.\n");         /* read mftx failed */
        
        return 1;                                                    /* return error */
    }
    *t = prev & 0x03;                                                /* set the wait */
    
    return 0;                                                        /* success return 0 */
}

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
uint8_t mfrc522_set_parity_disable(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                              /* check handle */
    {
        return 2;                                                    /* return error */
    }
    if (handle->inited != 1)                                         /* check handle initialization */
    {
        return 3;                                                    /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_MFRX, &prev, 1);        /* read mfrx */
    if (res != 0)                                                    /* check the result */
    {
        handle->debug_print("mfrc522: read mfrx failed.\n");         /* read mfrx failed */
        
        return 1;                                                    /* return error */
    }
    prev &= ~(1 << 4);                                               /* clear the settings */
    prev |= enable << 4;                                             /* set the enable */
    res = a_mfrc522_write(handle, MFRC522_REG_MFRX, &prev, 1);       /* write mfrx */
    if (res != 0)                                                    /* check the result */
    {
        handle->debug_print("mfrc522: write mfrx failed.\n");        /* write mfrx failed */
        
        return 1;                                                    /* return error */
    }
    
    return 0;                                                        /* success return 0 */
}

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
uint8_t mfrc522_get_parity_disable(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                              /* check handle */
    {
        return 2;                                                    /* return error */
    }
    if (handle->inited != 1)                                         /* check handle initialization */
    {
        return 3;                                                    /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_MFRX, &prev, 1);        /* read mfrx */
    if (res != 0)                                                    /* check the result */
    {
        handle->debug_print("mfrc522: read mfrx failed.\n");         /* read mfrx failed */
        
        return 1;                                                    /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 4) & 0x01);                  /* get the bool */
    
    return 0;                                                        /* success return 0 */
}

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
uint8_t mfrc522_set_serial_speed(mfrc522_handle_t *handle, uint8_t t0, uint8_t t1)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                      /* check handle */
    {
        return 2;                                                            /* return error */
    }
    if (handle->inited != 1)                                                 /* check handle initialization */
    {
        return 3;                                                            /* return error */
    }
    if (t0 > 0x7)                                                            /* check the t0 */
    {
        handle->debug_print("mfrc522: t0 is over 0x7.\n");                   /* t0 is over 0x7 */
        
        return 4;                                                            /* return error */
    }
    if (t1 > 0x1F)                                                           /* check the t1 */
    {
        handle->debug_print("mfrc522: t1 is over 0x1F.\n");                  /* t1 is over 0x1F */
        
        return 5;                                                            /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_SERIAL_SPEED, &prev, 1);        /* read serial speed  */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: read serial speed failed.\n");         /* read serial speed failed */
        
        return 1;                                                            /* return error */
    }
    prev = ((t0 & 0x7) << 5) | (t1 & 0x1F);                                  /* set the speed */
    res = a_mfrc522_write(handle, MFRC522_REG_SERIAL_SPEED, &prev, 1);       /* write serial speed */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: write serial speed failed.\n");        /* write serial speed failed */
        
        return 1;                                                            /* return error */
    }
    
    return 0;                                                                /* success return 0 */
}

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
uint8_t mfrc522_get_serial_speed(mfrc522_handle_t *handle, uint8_t *t0, uint8_t *t1)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                      /* check handle */
    {
        return 2;                                                            /* return error */
    }
    if (handle->inited != 1)                                                 /* check handle initialization */
    {
        return 3;                                                            /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_SERIAL_SPEED, &prev, 1);        /* read serial speed */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: read serial speed failed.\n");         /* read serial speed failed */
        
        return 1;                                                            /* return error */
    }
    *t0 = (prev >> 5) & 0xF;                                                 /* set the t0 */
    *t1 = (prev >> 0) & 0x1F;                                                /* set the t1 */
    
    return 0;                                                                /* success return 0 */
}

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
uint8_t mfrc522_get_crc(mfrc522_handle_t *handle, uint16_t *crc)
{
    uint8_t res;
    uint8_t buf[2];
    
    if (handle == NULL)                                                    /* check handle */
    {
        return 2;                                                          /* return error */
    }
    if (handle->inited != 1)                                               /* check handle initialization */
    {
        return 3;                                                          /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_CRC_RESULT_H, buf, 2);        /* read crc result */
    if (res != 0)                                                          /* check the result */
    {
        handle->debug_print("mfrc522: read crc result failed.\n");         /* read crc result failed */
        
        return 1;                                                          /* return error */
    }
    *crc = ((uint16_t)buf[0] << 8) | buf[1];                               /* get the result */
    
    return 0;                                                              /* success return 0 */
}

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
uint8_t mfrc522_set_modulation_width(mfrc522_handle_t *handle, uint8_t width)
{
    uint8_t res;
    
    if (handle == NULL)                                                     /* check handle */
    {
        return 2;                                                           /* return error */
    }
    if (handle->inited != 1)                                                /* check handle initialization */
    {
        return 3;                                                           /* return error */
    }
    
    res = a_mfrc522_write(handle, MFRC522_REG_MOD_WIDTH, &width, 1);        /* write mod width */
    if (res != 0)                                                           /* check the result */
    {
        handle->debug_print("mfrc522: write mod width failed.\n");          /* write mod width failed */
        
        return 1;                                                           /* return error */
    }
    
    return 0;                                                               /* success return 0 */
}

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
uint8_t mfrc522_get_modulation_width(mfrc522_handle_t *handle, uint8_t *width)
{
    uint8_t res;
    
    if (handle == NULL)                                                   /* check handle */
    {
        return 2;                                                         /* return error */
    }
    if (handle->inited != 1)                                              /* check handle initialization */
    {
        return 3;                                                         /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_MOD_WIDTH, width, 1);        /* read mod width */
    if (res != 0)                                                         /* check the result */
    {
        handle->debug_print("mfrc522: read mod width failed.\n");         /* read mod width failed */
        
        return 1;                                                         /* return error */
    }
    
    return 0;                                                             /* success return 0 */
}

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
uint8_t mfrc522_set_rx_gain(mfrc522_handle_t *handle, mfrc522_rx_gain_t gain)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_RFCFG, &prev, 1);        /* read rf cfg */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read rf cfg failed.\n");        /* read rf cfg failed */
        
        return 1;                                                     /* return error */
    }
    prev &= ~(7 << 4);                                                /* clear the settings */
    prev |= gain <<4;                                                 /* set the gain */
    res = a_mfrc522_write(handle, MFRC522_REG_RFCFG, &prev, 1);       /* write rf cfg */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: write rf cfg failed.\n");       /* write rf cfg failed */
        
        return 1;                                                     /* return error */
    }
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_get_rx_gain(mfrc522_handle_t *handle, mfrc522_rx_gain_t *gain)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                               /* check handle */
    {
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1)                                          /* check handle initialization */
    {
        return 3;                                                     /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_RFCFG, &prev, 1);        /* read rf cfg */
    if (res != 0)                                                     /* check the result */
    {
        handle->debug_print("mfrc522: read rf cfg failed.\n");        /* read rf cfg failed */
        
        return 1;                                                     /* return error */
    }
    *gain = (mfrc522_rx_gain_t)((prev >> 4) & 0x7);                   /* get the gain */
    
    return 0;                                                         /* success return 0 */
}

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
uint8_t mfrc522_set_cwgsn(mfrc522_handle_t *handle, uint8_t n)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                             /* check handle */
    {
        return 2;                                                   /* return error */
    }
    if (handle->inited != 1)                                        /* check handle initialization */
    {
        return 3;                                                   /* return error */
    }
    if (n > 0xF)                                                    /* check n */
    {
        handle->debug_print("mfrc522: n is over 0xF.\n");           /* n is over 0xF */
        
        return 4;                                                   /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_GSN, &prev, 1);        /* read gsn */
    if (res != 0)                                                   /* check the result */
    {
        handle->debug_print("mfrc522: read gsn failed.\n");         /* read gsn failed */
        
        return 1;                                                   /* return error */
    }
    prev &= ~(0xF << 4);                                            /* clear the settings */
    prev |= n <<4;                                                  /* set the param */
    res = a_mfrc522_write(handle, MFRC522_REG_GSN, &prev, 1);       /* write gsn */
    if (res != 0)                                                   /* check the result */
    {
        handle->debug_print("mfrc522: write gsn failed.\n");        /* write gsn failed */
        
        return 1;                                                   /* return error */
    }
    
    return 0;                                                       /* success return 0 */
}

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
uint8_t mfrc522_get_cwgsn(mfrc522_handle_t *handle, uint8_t *n)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                             /* check handle */
    {
        return 2;                                                   /* return error */
    }
    if (handle->inited != 1)                                        /* check handle initialization */
    {
        return 3;                                                   /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_GSN, &prev, 1);        /* read gsn */
    if (res != 0)                                                   /* check the result */
    {
        handle->debug_print("mfrc522: read gsn failed.\n");         /* read gsn failed */
        
        return 1;                                                   /* return error */
    }
    *n = (prev >> 4) & 0xF;                                         /* get the param */
    
    return 0;                                                       /* success return 0 */
}

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
uint8_t mfrc522_set_modgsn(mfrc522_handle_t *handle, uint8_t n)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                             /* check handle */
    {
        return 2;                                                   /* return error */
    }
    if (handle->inited != 1)                                        /* check handle initialization */
    {
        return 3;                                                   /* return error */
    }
    if (n > 0xF)                                                    /* check n */
    {
        handle->debug_print("mfrc522: n is over 0xF.\n");           /* n is over 0xF */
        
        return 4;                                                   /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_GSN, &prev, 1);        /* read gsn */
    if (res != 0)                                                   /* check the result */
    {
        handle->debug_print("mfrc522: read gsn failed.\n");         /* read gsn failed */
        
        return 1;                                                   /* return error */
    }
    prev &= ~(0xF << 0);                                            /* clear the settings */
    prev |= n <<0;                                                  /* set the param */
    res = a_mfrc522_write(handle, MFRC522_REG_GSN, &prev, 1);       /* write gsn */
    if (res != 0)                                                   /* check the result */
    {
        handle->debug_print("mfrc522: write gsn failed.\n");        /* write gsn failed */
        
        return 1;                                                   /* return error */
    }
    
    return 0;                                                       /* success return 0 */
}

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
uint8_t mfrc522_get_modgsn(mfrc522_handle_t *handle, uint8_t *n)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                             /* check handle */
    {
        return 2;                                                   /* return error */
    }
    if (handle->inited != 1)                                        /* check handle initialization */
    {
        return 3;                                                   /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_GSN, &prev, 1);        /* read gsn */
    if (res != 0)                                                   /* check the result */
    {
        handle->debug_print("mfrc522: read gsn failed.\n");         /* read gsn failed */
        
        return 1;                                                   /* return error */
    }
    *n = (prev >> 0) & 0xF;                                         /* get the param */
    
    return 0;                                                       /* success return 0 */
}

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
uint8_t mfrc522_set_cwgsp(mfrc522_handle_t *handle, uint8_t n)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                             /* check handle */
    {
        return 2;                                                   /* return error */
    }
    if (handle->inited != 1)                                        /* check handle initialization */
    {
        return 3;                                                   /* return error */
    }
    if (n > 0x3F)                                                   /* check n */
    {
        handle->debug_print("mfrc522: n is over 0x3F.\n");          /* n is over 0xF3 */
        
        return 4;                                                   /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_CWGSP, &prev, 1);      /* read cwgsp */
    if (res != 0)                                                   /* check the result */
    {
        handle->debug_print("mfrc522: read cwgsp failed.\n");       /* read cwgsp failed */
        
        return 1;                                                   /* return error */
    }
    prev &= ~(0x3F << 0);                                           /* clear the settings */
    prev |= n <<0;                                                  /* set the param */
    res = a_mfrc522_write(handle, MFRC522_REG_CWGSP, &prev, 1);     /* write cwgsp */
    if (res != 0)                                                   /* check the result */
    {
        handle->debug_print("mfrc522: write cwgsp failed.\n");      /* write cwgsp failed */
        
        return 1;                                                   /* return error */
    }
    
    return 0;                                                       /* success return 0 */
}

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
uint8_t mfrc522_get_cwgsp(mfrc522_handle_t *handle, uint8_t *n)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                             /* check handle */
    {
        return 2;                                                   /* return error */
    }
    if (handle->inited != 1)                                        /* check handle initialization */
    {
        return 3;                                                   /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_CWGSP, &prev, 1);      /* read cwgsp */
    if (res != 0)                                                   /* check the result */
    {
        handle->debug_print("mfrc522: read cwgsp failed.\n");       /* read cwgsp failed */
        
        return 1;                                                   /* return error */
    }
    *n = prev & 0x3F;                                               /* get the cwgsp */
    
    return 0;                                                       /* success return 0 */
}

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
uint8_t mfrc522_set_modgsp(mfrc522_handle_t *handle, uint8_t n)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                              /* check handle */
    {
        return 2;                                                    /* return error */
    }
    if (handle->inited != 1)                                         /* check handle initialization */
    {
        return 3;                                                    /* return error */
    }
    if (n > 0x3F)                                                    /* check n */
    {
        handle->debug_print("mfrc522: n is over 0x3F.\n");           /* n is over 0xF3 */
        
        return 4;                                                    /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_MODGSP, &prev, 1);      /* read modgsp */
    if (res != 0)                                                    /* check the result */
    {
        handle->debug_print("mfrc522: read modgsp failed.\n");       /* read modgsp failed */
        
        return 1;                                                    /* return error */
    }
    prev &= ~(0x3F << 0);                                            /* clear the settings */
    prev |= n <<0;                                                   /* set the param */
    res = a_mfrc522_write(handle, MFRC522_REG_MODGSP, &prev, 1);     /* write modgsp */
    if (res != 0)                                                    /* check the result */
    {
        handle->debug_print("mfrc522: write modgsp failed.\n");      /* write modgsp failed */
        
        return 1;                                                    /* return error */
    }
    
    return 0;                                                        /* success return 0 */
}

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
uint8_t mfrc522_get_modgsp(mfrc522_handle_t *handle, uint8_t *n)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                              /* check handle */
    {
        return 2;                                                    /* return error */
    }
    if (handle->inited != 1)                                         /* check handle initialization */
    {
        return 3;                                                    /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_MODGSP, &prev, 1);      /* read modgsp */
    if (res != 0)                                                    /* check the result */
    {
        handle->debug_print("mfrc522: read modgsp failed.\n");       /* read modgsp failed */
        
        return 1;                                                    /* return error */
    }
    *n = prev & 0x3F;                                                /* get the modgsp */
    
    return 0;                                                        /* success return 0 */
}

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
uint8_t mfrc522_set_timer_auto(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TMODE, &prev, 1);         /* read tmode */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read tmode failed.\n");          /* read tmode failed */
        
        return 1;                                                      /* return error */
    }
    prev &= ~(1 << 7);                                                 /* clear the settings */
    prev |= enable << 7;                                               /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_TMODE, &prev, 1);        /* write tmode */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: write tmode failed.\n");         /* write tmode failed */
        
        return 1;                                                      /* return error */
    }
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_get_timer_auto(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TMODE, &prev, 1);         /* read tmode */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read tmode failed.\n");          /* read tmode failed */
        
        return 1;                                                      /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 7) & 0x01);                    /* get the bool */
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_set_timer_gated_mode(mfrc522_handle_t *handle, mfrc522_timer_gated_mode_t mode)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TMODE, &prev, 1);         /* read tmode */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read tmode failed.\n");          /* read tmode failed */
        
        return 1;                                                      /* return error */
    }
    prev &= ~(3 << 5);                                                 /* clear the settings */
    prev |= mode << 5;                                                 /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_TMODE, &prev, 1);        /* write tmode */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: write tmode failed.\n");         /* write tmode failed */
        
        return 1;                                                      /* return error */
    }
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_get_timer_gated_mode(mfrc522_handle_t *handle, mfrc522_timer_gated_mode_t *mode)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TMODE, &prev, 1);         /* read tmode */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read tmode failed.\n");          /* read tmode failed */
        
        return 1;                                                      /* return error */
    }
    *mode = (mfrc522_timer_gated_mode_t)((prev >> 5) & 0x03);          /* get the mode */
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_set_timer_auto_restart(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TMODE, &prev, 1);         /* read tmode */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read tmode failed.\n");          /* read tmode failed */
        
        return 1;                                                      /* return error */
    }
    prev &= ~(1 << 4);                                                 /* clear the settings */
    prev |= enable << 4;                                               /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_TMODE, &prev, 1);        /* write tmode */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: write tmode failed.\n");         /* write tmode failed */
        
        return 1;                                                      /* return error */
    }
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_get_timer_auto_restart(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TMODE, &prev, 1);         /* read tmode */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read tmode failed.\n");          /* read tmode failed */
        
        return 1;                                                      /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 4) & 0x01);                    /* get the bool */
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_set_timer_prescaler(mfrc522_handle_t *handle, uint16_t t)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    if (t > 0xFFF)                                                     /* check t */
    {
        handle->debug_print("mfrc522: t is over 0xFFF.\n");            /* t is over 0xFFF */
        
        return 4;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TMODE, &prev, 1);         /* read tmode */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read tmode failed.\n");          /* read tmode failed */
        
        return 1;                                                      /* return error */
    }
    prev &= ~(0xF << 0);                                               /* clear the settings */
    prev |= ((t >> 8) & 0xF) << 0;                                     /* set the param */
    res = a_mfrc522_write(handle, MFRC522_REG_TMODE, &prev, 1);        /* write tmode */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: write tmode failed.\n");         /* write tmode failed */
        
        return 1;                                                      /* return error */
    }
    prev = t & 0xFF;                                                   /* set the t */
    res = a_mfrc522_write(handle, MFRC522_REG_TPRESCALER, &prev, 1);   /* write tprescaler */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: write tprescaler failed.\n");    /* write tprescaler failed */
        
        return 1;                                                      /* return error */
    }
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_get_timer_prescaler(mfrc522_handle_t *handle, uint16_t *t)
{
    uint8_t res;
    uint8_t prev1, prev2;
    
    if (handle == NULL)                                                     /* check handle */
    {
        return 2;                                                           /* return error */
    }
    if (handle->inited != 1)                                                /* check handle initialization */
    {
        return 3;                                                           /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TMODE, &prev1, 1);             /* read tmode */
    if (res != 0)                                                           /* check the result */
    {
        handle->debug_print("mfrc522: read tmode failed.\n");               /* read tmode failed */
        
        return 1;                                                           /* return error */
    }
    res = a_mfrc522_read(handle, MFRC522_REG_TPRESCALER, &prev2, 1);        /* read tprescaler */
    if (res != 0)                                                           /* check the result */
    {
        handle->debug_print("mfrc522: read tprescaler failed.\n");          /* read tprescaler failed */
        
        return 1;                                                           /* return error */
    }
    *t = (uint16_t)((prev1 >> 0) & 0xF) << 8 | prev2;                       /* set the param */
    
    return 0;                                                               /* success return 0 */
}

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
uint8_t mfrc522_set_timer_reload(mfrc522_handle_t *handle, uint16_t reload)
{
    uint8_t res;
    uint8_t buf[2];
    
    if (handle == NULL)                                                  /* check handle */
    {
        return 2;                                                        /* return error */
    }
    if (handle->inited != 1)                                             /* check handle initialization */
    {
        return 3;                                                        /* return error */
    }
    
    buf[0] = (reload >> 8) & 0xFF;                                       /* set high */
    buf[1] = (reload >> 0) & 0xFF;                                       /* set low */
    res = a_mfrc522_write(handle, MFRC522_REG_TRELOAD_H, buf, 2);        /* write treload */
    if (res != 0)                                                        /* check the result */
    {
        handle->debug_print("mfrc522: write treload failed.\n");         /* write treload failed */
        
        return 1;                                                        /* return error */
    }
    
    return 0;                                                            /* success return 0 */
}

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
uint8_t mfrc522_get_timer_reload(mfrc522_handle_t *handle, uint16_t *reload)
{
    uint8_t res;
    uint8_t buf[2];
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TRELOAD_H, buf, 2);        /* read treload */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read treload failed.\n");         /* read treload failed */
        
        return 1;                                                       /* return error */
    }
    *reload = (uint16_t)buf[0] << 8 | buf[1];                           /* set the reload */
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_get_timer_counter(mfrc522_handle_t *handle, uint16_t *cnt)
{
    uint8_t res;
    uint8_t buf[2];
    
    if (handle == NULL)                                                      /* check handle */
    {
        return 2;                                                            /* return error */
    }
    if (handle->inited != 1)                                                 /* check handle initialization */
    {
        return 3;                                                            /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TCOUNTER_VAL_H, buf, 2);        /* read tcounter */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: read tcounter failed.\n");             /* read tcounter failed */
        
        return 1;                                                            /* return error */
    }
    *cnt = (uint16_t)buf[0] << 8 | buf[1];                                   /* set the reload */
    
    return 0;                                                                /* success return 0 */
}

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
uint8_t mfrc522_set_test_bus_signal_1(mfrc522_handle_t *handle, uint8_t s)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    if (s > 7)                                                         /* check s */
    {
        handle->debug_print("mfrc522: s is over 7.\n");                /* s is over 7 */
        
        return 4;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TEST_SEL1, &prev, 1);     /* read test sel 1 */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read test sel 1 failed.\n");     /* read test sel 1 failed */
        
        return 1;                                                      /* return error */
    }
    prev &= ~(0x7 << 0);                                               /* clear the settings */
    prev |= s << 0;                                                    /* set the signal */
    res = a_mfrc522_write(handle, MFRC522_REG_TEST_SEL1, &prev, 1);    /* write test sel 1 */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: write test sel 1 failed.\n");    /* write test sel 1 failed */
        
        return 1;                                                      /* return error */
    }
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_get_test_bus_signal_1(mfrc522_handle_t *handle, uint8_t *s)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TEST_SEL1, &prev, 1);     /* read test sel 1 */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read test sel 1 failed.\n");     /* read test sel 1 failed */
        
        return 1;                                                      /* return error */
    }
    *s = (prev >> 0) & 0x7;                                            /* get the signal */
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_set_test_bus_signal_2(mfrc522_handle_t *handle, uint8_t s)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    if (s > 0x1F)                                                      /* check s */
    {
        handle->debug_print("mfrc522: s is over 0x1F.\n");             /* s is over 0x1F */
        
        return 4;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TEST_SEL2, &prev, 1);     /* read test sel 2 */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read test sel 2 failed.\n");     /* read test sel 2 failed */
        
        return 1;                                                      /* return error */
    }
    prev &= ~(0x1F << 0);                                              /* clear the settings */
    prev |= s << 0;                                                    /* set the signal */
    res = a_mfrc522_write(handle, MFRC522_REG_TEST_SEL2, &prev, 1);    /* write test sel 2 */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: write test sel 2 failed.\n");    /* write test sel 2 failed */
        
        return 1;                                                      /* return error */
    }
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_get_test_bus_signal_2(mfrc522_handle_t *handle, uint8_t *s)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }

    res = a_mfrc522_read(handle, MFRC522_REG_TEST_SEL2, &prev, 1);     /* read test sel 2 */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read test sel 2 failed.\n");     /* read test sel 2 failed */
        
        return 1;                                                      /* return error */
    }
    *s = (prev >> 0) & 0x1F;                                           /* set the signal */
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_set_test_bus_flip(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TEST_SEL2, &prev, 1);     /* read test sel 2 */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read test sel 2 failed.\n");     /* read test sel 2 failed */
        
        return 1;                                                      /* return error */
    }
    prev &= ~(1 << 7);                                                 /* clear the settings */
    prev |= enable << 7;                                               /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_TEST_SEL2, &prev, 1);    /* write test sel 2 */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: write test sel 2 failed.\n");    /* write test sel 2 failed */
        
        return 1;                                                      /* return error */
    }
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_get_test_bus_flip(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TEST_SEL2, &prev, 1);     /* read test sel 2 */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read test sel 2 failed.\n");     /* read test sel 2 failed */
        
        return 1;                                                      /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 7) & 0x01);                    /* get the bool */
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_set_test_prbs9(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TEST_SEL2, &prev, 1);     /* read test sel 2 */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read test sel 2 failed.\n");     /* read test sel 2 failed */
        
        return 1;                                                      /* return error */
    }
    prev &= ~(1 << 6);                                                 /* clear the settings */
    prev |= enable << 6;                                               /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_TEST_SEL2, &prev, 1);    /* write test sel 2 */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: write test sel 2 failed.\n");    /* write test sel 2 failed */
        
        return 1;                                                      /* return error */
    }
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_get_test_prbs9(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TEST_SEL2, &prev, 1);     /* read test sel 2 */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read test sel 2 failed.\n");     /* read test sel 2 failed */
        
        return 1;                                                      /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 6) & 0x01);                    /* get the bool */
    
    return 0;                                                          /* success return 0 */
}
 
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
uint8_t mfrc522_set_test_prbs15(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TEST_SEL2, &prev, 1);     /* read test sel 2 */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read test sel 2 failed.\n");     /* read test sel 2 failed */
        
        return 1;                                                      /* return error */
    }
    prev &= ~(1 << 5);                                                 /* clear the settings */
    prev |= enable << 5;                                               /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_TEST_SEL2, &prev, 1);    /* write test sel 2 */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: write test sel 2 failed.\n");    /* write test sel 2 failed */
        
        return 1;                                                      /* return error */
    }
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_get_test_prbs15(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                /* check handle */
    {
        return 2;                                                      /* return error */
    }
    if (handle->inited != 1)                                           /* check handle initialization */
    {
        return 3;                                                      /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TEST_SEL2, &prev, 1);     /* read test sel 2 */
    if (res != 0)                                                      /* check the result */
    {
        handle->debug_print("mfrc522: read test sel 2 failed.\n");     /* read test sel 2 failed */
        
        return 1;                                                      /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 5) & 0x01);                    /* get the bool */
    
    return 0;                                                          /* success return 0 */
}

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
uint8_t mfrc522_set_test_rs232_line(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                      /* check handle */
    {
        return 2;                                                            /* return error */
    }
    if (handle->inited != 1)                                                 /* check handle initialization */
    {
        return 3;                                                            /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TEST_PIN_EN, &prev, 1);         /* read test pin en */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: read test pin en failed.\n");          /* read test pin en failed */
        
        return 1;                                                            /* return error */
    }
    prev &= ~(1 << 7);                                                       /* clear the settings */
    prev |= enable << 7;                                                     /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_TEST_PIN_EN, &prev, 1);        /* write test pin en */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: write test pin en failed.\n");         /* write test pin en failed */
        
        return 1;                                                            /* return error */
    }
    
    return 0;                                                                /* success return 0 */
}

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
uint8_t mfrc522_get_test_rs232_line(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                      /* check handle */
    {
        return 2;                                                            /* return error */
    }
    if (handle->inited != 1)                                                 /* check handle initialization */
    {
        return 3;                                                            /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TEST_PIN_EN, &prev, 1);         /* read test pin en */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: read test pin en failed.\n");          /* read test pin en failed */
        
        return 1;                                                            /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 7) & 0x01);                          /* get the bool */
    
    return 0;                                                                /* success return 0 */
}

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
uint8_t mfrc522_set_test_pin_enable(mfrc522_handle_t *handle, uint8_t pin)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                      /* check handle */
    {
        return 2;                                                            /* return error */
    }
    if (handle->inited != 1)                                                 /* check handle initialization */
    {
        return 3;                                                            /* return error */
    }
    if (pin > 0x3F)                                                          /* check pin */
    {
        handle->debug_print("mfrc522: pin is over 0x3F.\n");                 /* pin is over 0x3F */
        
        return 4;                                                            /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TEST_PIN_EN, &prev, 1);         /* read test pin en */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: read test pin en failed.\n");          /* read test pin en failed */
        
        return 1;                                                            /* return error */
    }
    prev &= ~(0x3F << 1);                                                    /* clear the settings */
    prev |= pin << 1;                                                        /* set the pin */
    res = a_mfrc522_write(handle, MFRC522_REG_TEST_PIN_EN, &prev, 1);        /* write test pin en */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: write test pin en failed.\n");         /* write test pin en failed */
        
        return 1;                                                            /* return error */
    }
    
    return 0;                                                                /* success return 0 */
}

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
uint8_t mfrc522_get_test_pin_enable(mfrc522_handle_t *handle, uint8_t *pin)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                      /* check handle */
    {
        return 2;                                                            /* return error */
    }
    if (handle->inited != 1)                                                 /* check handle initialization */
    {
        return 3;                                                            /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TEST_PIN_EN, &prev, 1);         /* read test pin en */
    if (res != 0)                                                            /* check the result */
    {
        handle->debug_print("mfrc522: read test pin en failed.\n");          /* read test pin en failed */
        
        return 1;                                                            /* return error */
    }
    *pin = (prev >> 1) & 0x3F;                                               /* get the pin */
    
    return 0;                                                                /* success return 0 */
}

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
uint8_t mfrc522_set_test_port_io(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                        /* check handle */
    {
        return 2;                                                              /* return error */
    }
    if (handle->inited != 1)                                                   /* check handle initialization */
    {
        return 3;                                                              /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TEST_PIN_VALUE, &prev, 1);        /* read test pin value */
    if (res != 0)                                                              /* check the result */
    {
        handle->debug_print("mfrc522: read test pin value failed.\n");         /* read test pin value failed */
        
        return 1;                                                              /* return error */
    }
    prev &= ~(1 << 7);                                                         /* clear the settings */
    prev |= enable << 7;                                                       /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_TEST_PIN_VALUE, &prev, 1);       /* write test pin value */
    if (res != 0)                                                              /* check the result */
    {
        handle->debug_print("mfrc522: write test pin value failed.\n");        /* write test pin value failed */
        
        return 1;                                                              /* return error */
    }
    
    return 0;                                                                  /* success return 0 */
}

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
uint8_t mfrc522_get_test_port_io(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                        /* check handle */
    {
        return 2;                                                              /* return error */
    }
    if (handle->inited != 1)                                                   /* check handle initialization */
    {
        return 3;                                                              /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TEST_PIN_VALUE, &prev, 1);        /* read test pin value */
    if (res != 0)                                                              /* check the result */
    {
        handle->debug_print("mfrc522: read test pin value failed.\n");         /* read test pin value failed */
        
        return 1;                                                              /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 7) & 0x01);                            /* get the bool */
    
    return 0;                                                                  /* success return 0 */
}

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
uint8_t mfrc522_set_test_pin_value(mfrc522_handle_t *handle, uint8_t value)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                        /* check handle */
    {
        return 2;                                                              /* return error */
    }
    if (handle->inited != 1)                                                   /* check handle initialization */
    {
        return 3;                                                              /* return error */
    }
    if (value > 0x3F)                                                          /* check the value */
    {
        handle->debug_print("mfrc522: value is over 0x3F.\n");                 /* value is over 0x3F */
        
        return 4;                                                              /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TEST_PIN_VALUE, &prev, 1);        /* read test pin value */
    if (res != 0)                                                              /* check the result */
    {
        handle->debug_print("mfrc522: read test pin value failed.\n");         /* read test pin value failed */
        
        return 1;                                                              /* return error */
    }
    prev &= ~(0x3F << 1);                                                      /* clear the settings */
    prev |= value << 1;                                                        /* set the value */
    res = a_mfrc522_write(handle, MFRC522_REG_TEST_PIN_VALUE, &prev, 1);       /* write test pin value */
    if (res != 0)                                                              /* check the result */
    {
        handle->debug_print("mfrc522: write test pin value failed.\n");        /* write test pin value failed */
        
        return 1;                                                              /* return error */
    }
    
    return 0;                                                                  /* success return 0 */
}

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
uint8_t mfrc522_get_test_pin_value(mfrc522_handle_t *handle, uint8_t *value)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                        /* check handle */
    {
        return 2;                                                              /* return error */
    }
    if (handle->inited != 1)                                                   /* check handle initialization */
    {
        return 3;                                                              /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TEST_PIN_VALUE, &prev, 1);        /* read test pin value */
    if (res != 0)                                                              /* check the result */
    {
        handle->debug_print("mfrc522: read test pin value failed.\n");         /* read test pin value failed */
        
        return 1;                                                              /* return error */
    }
    *value = (prev >> 1) & 0x3F;                                               /* get the value */
    
    return 0;                                                                  /* success return 0 */
}

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
uint8_t mfrc522_get_test_bus(mfrc522_handle_t *handle, uint8_t *bus)
{
    uint8_t res;
    
    if (handle == NULL)                                                  /* check handle */
    {
        return 2;                                                        /* return error */
    }
    if (handle->inited != 1)                                             /* check handle initialization */
    {
        return 3;                                                        /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_TEST_BUS, bus, 1);          /* read test bus */
    if (res != 0)                                                        /* check the result */
    {
        handle->debug_print("mfrc522: read test bus failed.\n");         /* read test bus failed */
        
        return 1;                                                        /* return error */
    }
    
    return 0;                                                            /* success return 0 */
}

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
uint8_t mfrc522_set_test_amp_rcv(mfrc522_handle_t *handle, mfrc522_bool_t enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                    /* check handle */
    {
        return 2;                                                          /* return error */
    }
    if (handle->inited != 1)                                               /* check handle initialization */
    {
        return 3;                                                          /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_AUTO_TEST, &prev, 1);         /* read auto test */
    if (res != 0)                                                          /* check the result */
    {
        handle->debug_print("mfrc522: read auto test failed.\n");          /* read auto test failed */
        
        return 1;                                                          /* return error */
    }
    prev &= ~(1 << 6);                                                     /* clear the settings */
    prev |= enable << 6;                                                   /* set the bool */
    res = a_mfrc522_write(handle, MFRC522_REG_AUTO_TEST, &prev, 1);        /* write auto test */
    if (res != 0)                                                          /* check the result */
    {
        handle->debug_print("mfrc522: write auto test failed.\n");         /* write auto test failed */
        
        return 1;                                                          /* return error */
    }
    
    return 0;                                                              /* success return 0 */
}

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
uint8_t mfrc522_get_test_amp_rcv(mfrc522_handle_t *handle, mfrc522_bool_t *enable)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                    /* check handle */
    {
        return 2;                                                          /* return error */
    }
    if (handle->inited != 1)                                               /* check handle initialization */
    {
        return 3;                                                          /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_AUTO_TEST, &prev, 1);         /* read auto test */
    if (res != 0)                                                          /* check the result */
    {
        handle->debug_print("mfrc522: read auto test failed.\n");          /* read auto test failed */
        
        return 1;                                                          /* return error */
    }
    *enable = (mfrc522_bool_t)((prev >> 6) & 0x01);                        /* get the bool */
    
    return 0;                                                              /* success return 0 */
}

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
uint8_t mfrc522_set_self_test(mfrc522_handle_t *handle, uint8_t test)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                    /* check handle */
    {
        return 2;                                                          /* return error */
    }
    if (handle->inited != 1)                                               /* check handle initialization */
    {
        return 3;                                                          /* return error */
    }
    if (test > 0xF)                                                        /* check the test */
    {
        handle->debug_print("mfrc522: test is over 0xF.\n");               /* test is over 0xF */
        
        return 4;                                                          /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_AUTO_TEST, &prev, 1);         /* read auto test */
    if (res != 0)                                                          /* check the result */
    {
        handle->debug_print("mfrc522: read auto test failed.\n");          /* read auto test failed */
        
        return 1;                                                          /* return error */
    }
    prev &= ~(0xF << 0);                                                   /* clear the settings */
    prev |= test << 0;                                                     /* set the test */
    res = a_mfrc522_write(handle, MFRC522_REG_AUTO_TEST, &prev, 1);        /* write auto test */
    if (res != 0)                                                          /* check the result */
    {
        handle->debug_print("mfrc522: write auto test failed.\n");         /* write auto test failed */
        
        return 1;                                                          /* return error */
    }
    
    return 0;                                                              /* success return 0 */
}

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
uint8_t mfrc522_get_self_test(mfrc522_handle_t *handle, uint8_t *test)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                    /* check handle */
    {
        return 2;                                                          /* return error */
    }
    if (handle->inited != 1)                                               /* check handle initialization */
    {
        return 3;                                                          /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_AUTO_TEST, &prev, 1);         /* read auto test */
    if (res != 0)                                                          /* check the result */
    {
        handle->debug_print("mfrc522: read auto test failed.\n");          /* read auto test failed */
        
        return 1;                                                          /* return error */
    }
    *test = prev & 0xF;                                                    /* get the test */
    
    return 0;                                                              /* success return 0 */
}

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
uint8_t mfrc522_get_version(mfrc522_handle_t *handle, uint8_t *id, uint8_t *version)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_VERSION, &prev, 1);        /* read version */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read version failed.\n");         /* read version failed */
        
        return 1;                                                       /* return error */
    }
    *id = (prev >> 4) & 0xF;                                            /* get the id */
    *version= (prev >> 0) & 0xF;                                        /* get the version */
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_set_test_analog_control_aux_1(mfrc522_handle_t *handle, mfrc522_test_analog_control_t control)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                    /* check handle */
    {
        return 2;                                                          /* return error */
    }
    if (handle->inited != 1)                                               /* check handle initialization */
    {
        return 3;                                                          /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_ANALOG_TEST, &prev, 1);       /* read analog test */
    if (res != 0)                                                          /* check the result */
    {
        handle->debug_print("mfrc522: read analog test failed.\n");        /* read analog test failed */
        
        return 1;                                                          /* return error */
    }
    prev &= ~(0xF << 4);                                                   /* clear the settings */
    prev |= control << 4;                                                  /* set the control */
    res = a_mfrc522_write(handle, MFRC522_REG_ANALOG_TEST, &prev, 1);      /* write analog test */
    if (res != 0)                                                          /* check the result */
    {
        handle->debug_print("mfrc522: write analog test failed.\n");       /* write analog test failed */
        
        return 1;                                                          /* return error */
    }
    
    return 0;                                                              /* success return 0 */
}

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
uint8_t mfrc522_get_test_analog_control_aux_1(mfrc522_handle_t *handle, mfrc522_test_analog_control_t *control)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                    /* check handle */
    {
        return 2;                                                          /* return error */
    }
    if (handle->inited != 1)                                               /* check handle initialization */
    {
        return 3;                                                          /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_ANALOG_TEST, &prev, 1);       /* read analog test */
    if (res != 0)                                                          /* check the result */
    {
        handle->debug_print("mfrc522: read analog test failed.\n");        /* read analog test failed */
        
        return 1;                                                          /* return error */
    }
    *control = (mfrc522_test_analog_control_t)((prev >> 4) & 0xF);         /* get the control */
    
    return 0;                                                              /* success return 0 */
}

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
uint8_t mfrc522_set_test_analog_control_aux_2(mfrc522_handle_t *handle, mfrc522_test_analog_control_t control)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                    /* check handle */
    {
        return 2;                                                          /* return error */
    }
    if (handle->inited != 1)                                               /* check handle initialization */
    {
        return 3;                                                          /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_ANALOG_TEST, &prev, 1);       /* read analog test */
    if (res != 0)                                                          /* check the result */
    {
        handle->debug_print("mfrc522: read analog test failed.\n");        /* read analog test failed */
        
        return 1;                                                          /* return error */
    }
    prev &= ~(0xF << 0);                                                   /* clear the settings */
    prev |= control << 0;                                                  /* set the control */
    res = a_mfrc522_write(handle, MFRC522_REG_ANALOG_TEST, &prev, 1);      /* write analog test */
    if (res != 0)                                                          /* check the result */
    {
        handle->debug_print("mfrc522: write analog test failed.\n");       /* write analog test failed */
        
        return 1;                                                          /* return error */
    }
    
    return 0;                                                              /* success return 0 */
}

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
uint8_t mfrc522_get_test_analog_control_aux_2(mfrc522_handle_t *handle, mfrc522_test_analog_control_t *control)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                    /* check handle */
    {
        return 2;                                                          /* return error */
    }
    if (handle->inited != 1)                                               /* check handle initialization */
    {
        return 3;                                                          /* return error */
    }
    
    res = a_mfrc522_read(handle, MFRC522_REG_ANALOG_TEST, &prev, 1);       /* read analog test */
    if (res != 0)                                                          /* check the result */
    {
        handle->debug_print("mfrc522: read analog test failed.\n");        /* read analog test failed */
        
        return 1;                                                          /* return error */
    }
    *control = (mfrc522_test_analog_control_t)((prev >> 0) & 0xF);         /* get the control */
    
    return 0;                                                              /* success return 0 */
}

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
uint8_t mfrc522_set_test_dac_1(mfrc522_handle_t *handle, uint8_t dac)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                  /* check handle */
    {
        return 2;                                                        /* return error */
    }
    if (handle->inited != 1)                                             /* check handle initialization */
    {
        return 3;                                                        /* return error */
    }
    if (dac > 0x3F)                                                      /* check the dac */
    {
        handle->debug_print("mfrc522: dac is over 0x3F.\n");             /* dac is over 0x3F */
        
        return 4;                                                        /* return error */
    }
    
    prev = dac & 0x3F;                                                   /* set the dac */
    res = a_mfrc522_write(handle, MFRC522_REG_TEST_DAC1, &prev, 1);      /* write test dac1 */
    if (res != 0)                                                        /* check the result */
    {
        handle->debug_print("mfrc522: write test dac1 failed.\n");       /* write test dac1 failed */
        
        return 1;                                                        /* return error */
    }
    
    return 0;                                                            /* success return 0 */
}

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
uint8_t mfrc522_get_test_dac_1(mfrc522_handle_t *handle, uint8_t *dac)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }

    res = a_mfrc522_read(handle, MFRC522_REG_TEST_DAC1, &prev, 1);      /* read test dac1 */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read test dac1 failed.\n");       /* read test dac1 failed */
        
        return 1;                                                       /* return error */
    }
    *dac = prev & 0x3F;                                                 /* get the dac */
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_set_test_dac_2(mfrc522_handle_t *handle, uint8_t dac)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                  /* check handle */
    {
        return 2;                                                        /* return error */
    }
    if (handle->inited != 1)                                             /* check handle initialization */
    {
        return 3;                                                        /* return error */
    }
    if (dac > 0x3F)                                                      /* check the dac */
    {
        handle->debug_print("mfrc522: dac is over 0x3F.\n");             /* dac is over 0x3F */
        
        return 4;                                                        /* return error */
    }
    
    prev = dac & 0x3F;                                                   /* set the dac */
    res = a_mfrc522_write(handle, MFRC522_REG_TEST_DAC2, &prev, 1);      /* write test dac2 */
    if (res != 0)                                                        /* check the result */
    {
        handle->debug_print("mfrc522: write test dac2 failed.\n");       /* write test dac2 failed */
        
        return 1;                                                        /* return error */
    }
    
    return 0;                                                            /* success return 0 */
}

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
uint8_t mfrc522_get_test_dac_2(mfrc522_handle_t *handle, uint8_t *dac)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }

    res = a_mfrc522_read(handle, MFRC522_REG_TEST_DAC2, &prev, 1);      /* read test dac2 */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read test dac2 failed.\n");       /* read test dac2 failed */
        
        return 1;                                                       /* return error */
    }
    *dac = prev & 0x3F;                                                 /* get the dac */
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_get_test_adc(mfrc522_handle_t *handle, uint8_t *adc_i, uint8_t *adc_q)
{
    uint8_t res;
    uint8_t prev;
    
    if (handle == NULL)                                                 /* check handle */
    {
        return 2;                                                       /* return error */
    }
    if (handle->inited != 1)                                            /* check handle initialization */
    {
        return 3;                                                       /* return error */
    }

    res = a_mfrc522_read(handle, MFRC522_REG_TEST_ADC, &prev, 1);      /* read test dac2 */
    if (res != 0)                                                       /* check the result */
    {
        handle->debug_print("mfrc522: read test dac2 failed.\n");       /* read test dac2 failed */
        
        return 1;                                                       /* return error */
    }
    *adc_i = (prev >> 4) & 0xF;                                         /* get adc i */
    *adc_q = (prev >> 0) & 0xF;                                         /* get adc q */
    
    return 0;                                                           /* success return 0 */
}

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
uint8_t mfrc522_set_reg(mfrc522_handle_t *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (handle == NULL)                                  /* check handle */
    {
        return 2;                                        /* return error */
    }
    if (handle->inited != 1)                             /* check handle initialization */
    {
        return 3;                                        /* return error */
    }
    
    return a_mfrc522_write(handle, reg, buf, len);       /* write data */
}

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
uint8_t mfrc522_get_reg(mfrc522_handle_t *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (handle == NULL)                                 /* check handle */
    {
        return 2;                                       /* return error */
    }
    if (handle->inited != 1)                            /* check handle initialization */
    {
        return 3;                                       /* return error */
    }
    
    return a_mfrc522_read(handle, reg, buf, len);       /* read data */
}

/**
 * @brief      get chip information
 * @param[out] *info points to a mfrc522 info structure
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 * @note       none
 */
uint8_t mfrc522_info(mfrc522_info_t *info)
{
    if (info == NULL)                                               /* check handle */
    {
        return 2;                                                   /* return error */
    }
    
    memset(info, 0, sizeof(mfrc522_info_t));                        /* initialize mfrc522 info structure */
    strncpy(info->chip_name, CHIP_NAME, 32);                        /* copy chip name */
    strncpy(info->manufacturer_name, MANUFACTURER_NAME, 32);        /* copy manufacturer name */
    strncpy(info->interface, "IIC SPI UART", 32);                   /* copy interface name */
    info->supply_voltage_min_v = SUPPLY_VOLTAGE_MIN;                /* set minimal supply voltage */
    info->supply_voltage_max_v = SUPPLY_VOLTAGE_MAX;                /* set maximum supply voltage */
    info->max_current_ma = MAX_CURRENT;                             /* set maximum current */
    info->temperature_max = TEMPERATURE_MAX;                        /* set minimal temperature */
    info->temperature_min = TEMPERATURE_MIN;                        /* set maximum temperature */
    info->driver_version = DRIVER_VERSION;                          /* set driver verison */
    
    return 0;                                                       /* success return 0 */
}
