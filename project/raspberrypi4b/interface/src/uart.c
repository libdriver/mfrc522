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
 * @file      uart.c
 * @brief     uart source file
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

#include "uart.h"
#include <fcntl.h>
#include <string.h>
#include <termios.h>

/**
 * @brief     uart config
 * @param[in] fd is the uart handler
 * @param[in] baud_rate is the baud rate
 * @param[in] data_bits is the data bits
 * @param[in] parity is the data parity
 * @param[in] stop_bits is the stop bits
 * @return    status code
 *            - 0 success
 *            - 1 set config failed
 * @note      none
 */
static uint8_t a_uart_config(int fd, uint32_t baud_rate, uint8_t data_bits, char parity, uint8_t stop_bits)
{
    struct termios cfg;
    int speed;
    
    /* get cfg */
    if (tcgetattr(fd, &cfg) != 0)
    {
        perror("uart: get cfg failed.\n");
        
        return 1;
    }
    
    /* set raw mode */
    cfmakeraw(&cfg);
    
    /* set the baud rate */
    switch (baud_rate)
    {
        /* 2400bps */
        case 2400 :
        {
            speed = B2400;
            
            break;
        }
        
        /* 4800bps */
        case 4800 :
        {
            speed = B4800;
            
            break;
        }
        
        /* 9600bps */
        case 9600 :
        {
            speed = B9600;
            
            break;
        }
        
        /* 19200bps */
        case 19200 :
        {
            speed = B19200;
            
            break;
        }
        
        /* 38400bps */
        case 38400 :
        {
            speed = B38400;
            
            break;
        }
        
        /* 115200bps */
        case 115200 :
        {
            speed = B115200;
            
            break;
        }
        
        /* invalid param */
        default :
        {
            perror("uart: baud rate is invalid.\n");
            
            return 1;
        }
    }
    
    /* set input speed */
    if (cfsetispeed(&cfg, speed) != 0)
    {
        perror("uart: set speed failed.\n");
        
        return 1;
    }
    
    /* set output speed */
    if (cfsetospeed(&cfg, speed) != 0)
    {
        perror("uart: set speed failed.\n");
        
        return 1;
    }
    
    /* set data bits */
    switch (data_bits)
    {
        /* 5 bit */
        case 5 :
        {
            cfg.c_cflag &= ~CSIZE;
            cfg.c_cflag |= CS5;
         
            break;
        }
        
        /* 6 bit */
        case 6 :
        {
            cfg.c_cflag &= ~CSIZE;
            cfg.c_cflag |= CS6;
            
            break;
        }
        
        /* 7 bit */
        case 7 :
        {
            cfg.c_cflag &= ~CSIZE;
            cfg.c_cflag |= CS7;
            
            break;
        }
        
        /* 8 bit */
        case 8 :
        {
            cfg.c_cflag &= ~CSIZE;
            cfg.c_cflag |= CS8;
            
            break;
        }
        
        /* invalid param */
        default :
        {
            perror("uart: data bits is invalid.\n");
            
            return 1;
        }
    }
    
    /* set parity */
    switch (parity)
    {
        /* parity none */
        case 'n' :
        case 'N' : 
        {
            cfg.c_cflag &= ~PARENB;
            cfg.c_iflag &= ~INPCK;
            
            break;
        }
        
        /* parity odd */
        case 'o' :
        case 'O' :
        {
            cfg.c_cflag |= (PARODD | PARENB);
            cfg.c_iflag |= INPCK;
            
            break;
        }
        
        /* parity even */
        case 'e' :
        case 'E' :
        {
            cfg.c_cflag |=  PARENB;
            cfg.c_cflag &= ~PARODD;
            cfg.c_iflag |= INPCK;
            
            break;
        }
        
        /* invalid param */
        default :
        {
            perror("uart: parity is invalid.\n");
            
            return 1;
        }
    }
    
    /* set stop bits */
    switch (stop_bits)
    {
        /* 1 stop bit */
        case 1 :
        {
            cfg.c_cflag &= ~CSTOPB;
            
            break;
        }
        
        /* 2 stop bits */
        case 2 :
        {
            cfg.c_cflag |= CSTOPB;
            
            break;
        }
        
        /* invalid param */
        default :
        {
            perror("uart: stop bits is invalid.\n");
            
            return 1;
        }
    }
    
    /* set min wait time 1 * (1 / 10)s */
    cfg.c_cc[VTIME] = 0;
    
    /* set min char 1 */
    cfg.c_cc[VMIN] = 1;
    
    /* flush data */
    if (tcflush(fd, TCIFLUSH) != 0)
    {
        perror("uart: uart flush failed.\n");
            
        return 1;
    }
    
    /* write cfg */
    if (tcsetattr(fd, TCSANOW, &cfg) != 0)
    {
        perror("uart: write cfg failed.\n");
        
        return 1;
    }
    
    return 0;
}

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
uint8_t uart_init(char *name, int *fd, uint32_t baud_rate, uint8_t data_bits, char parity, uint8_t stop_bits)
{
    /* open the device */
    *fd = open (name, O_RDWR | O_NOCTTY);
    if ((*fd) < 0)
    {
        perror("uart: open failed.\n");
        
        return 1;
    }
    else
    {
        /* default settings */
        return a_uart_config(*fd, baud_rate, data_bits, parity, stop_bits);
    }
}

/**
 * @brief     uart deinit
 * @param[in] fd is the uart handle
 * @return    status code
 *            - 0 success
 *            - 1 deinit failed
 * @note      none
 */
uint8_t uart_deinit(int fd)
{
    /* close the device */
    if (close(fd) < 0)
    {
        perror("uart: close failed.\n");
        
        return 1;
    }
    else
    {
        return 0;
    }
}

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
uint8_t uart_write(int fd, uint8_t *buf, uint32_t len)
{
    /* write data */
    if (write(fd, buf, len) < 0)
    {
        perror("uart: write failed.\n");
        
        return 1;
    }
    else
    {
        return 0;
    }
}

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
uint8_t uart_read(int fd, uint8_t *buf, uint32_t *len)
{
    ssize_t l;
    
    /* read data */
    l = read(fd, buf, *len);
    if (l < 0) 
    {
        perror("uart: read failed.\n");
        
        return 1;
    }
    else
    {
        /* set read data length */
        *len = l;
        
        return 0;
    }
}

/**
 * @brief     uart flush
 * @param[in] fd is the uart handle
 * @return    status code
 *            - 0 success
 *            - 1 flush failed
 * @note      none
 */
uint8_t uart_flush(int fd)
{
    /* flush data */
    if (tcflush(fd, TCIOFLUSH) < 0)
    {
        perror("uart: flush failed.\n");
        
        return 1;
    }
    else
    {
        return 0;
    }
}
