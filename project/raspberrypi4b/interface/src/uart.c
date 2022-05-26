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
 * @date      2021-02-12
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2021/02/12  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "uart.h"

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
static uint8_t _uart_config(int fd, uint32_t baud_rate, uint8_t data_bits, char parity, uint8_t stop_bits)
{
    struct termios cfg;
    int speed;
    
    if (tcgetattr(fd, &cfg) != 0)                       /* get cfg */
    {
        perror("uart: get cfg failed.\n");
     
        return 1;
    }
    cfmakeraw(&cfg);                                    /* set raw mode */
    switch (baud_rate)                                  /* set baud rate */
    {
        case 2400 :                                     /* 2400bps */
        {
            speed = B2400;
         
            break;
        }
        case 4800 :                                     /* 4800bps */
        {
            speed = B4800;
         
            break;
        }
        case 9600 :                                     /* 9600bps */
        {
            speed = B9600;
            
            break;
        }
        case 19200 :                                    /* 19200bps */
        {
            speed = B19200;
            
            break;
        }
        case 38400 :                                    /* 38400bps */
        {
            speed = B38400;
            
            break;
        }
        case 115200 :                                   /* 115200bps */
        {
            speed = B115200;
            
            break;
        }
        default :                                       /* invalid param */
        {
            perror("uart: baud rate is invalid.\n");
         
            return 1;
        }
    }
    cfsetispeed(&cfg, speed);                           /* set input speed */
    cfsetospeed(&cfg, speed);                           /* set output speed */
    switch (data_bits)                                  /* set data bits */
    {
        case 5 :                                        /* 5 bit */
        {
            cfg.c_cflag &= ~CSIZE;                      /* mask flag */
            cfg.c_cflag |= CS5;
         
            break;
        }
        case 6 :                                        /* 6 bit */
        {
            cfg.c_cflag &= ~CSIZE;                      /* mask flag */
            cfg.c_cflag |= CS6;
            
            break;
        }
        case 7 :                                        /* 6 bit */
        {
            cfg.c_cflag &= ~CSIZE;                      /* mask flag */
            cfg.c_cflag |= CS7;
            
            break;
        }
        case 8 :                                        /* 8 bit */
        {
            cfg.c_cflag &= ~CSIZE;                      /* mask flag */
            cfg.c_cflag |= CS8;
            
            break;
        }
        default :                                       /* invalid param */
        {
            perror("uart: data bits is invalid.\n");
            
            return 1;
        }
    }
    switch (parity)                                     /* set parity */
    {
        case 'n' :                                      /* parity none */
        case 'N' : 
        {
            cfg.c_cflag &= ~PARENB;
            cfg.c_iflag &= ~INPCK;
            
            break;
        }
        case 'o' :                                      /* parity odd */
        case 'O' :
        {
            cfg.c_cflag |= (PARODD | PARENB);
            cfg.c_iflag |= INPCK;
            
            break;
        }
        case 'e' :
        case 'E' :                                      /* parity enb */
        {
         
            cfg.c_cflag |=  PARENB;
            cfg.c_cflag &= ~PARODD;
            cfg.c_iflag |= INPCK;
            
            break;
        }
        default :                                       /* invalid param */
        {
            perror("uart: parity is invalid.\n");
            
            return 1;
        }
    }
    switch (stop_bits)                                  /* set stop bits */
    {
        case 1 :                                        /* 1 stop bit */
        {
            cfg.c_cflag &= ~CSTOPB;
            
            break;
        }
        case 2 :                                        /* 2 stop bits */
        {
            cfg.c_cflag |= CSTOPB;
            
            break;
        }
        default :                                       /* invalid param */
        {
            perror("uart: stop bits is invalid.\n");
            
            return 1;
        }
    }
    cfg.c_cc[VTIME] = 0;                                /* set min wait time 1*(1/10)s */
    cfg.c_cc[VMIN] = 1;                                 /* set min char 1 */
    tcflush(fd, TCIFLUSH);                              /* flush data */
    if (tcsetattr(fd, TCSANOW, &cfg) != 0)              /* write cfg */
    {
        perror("uart: write cfg failed.\n");
        
        return 1;
    }
    
    return 0;                                           /* success return 0 */
}

/**
 * @brief     uart init
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
    *fd = open (name, O_RDWR | O_NOCTTY);               /* open uart */
    if ((*fd) < 0)                                      /* if open failed */
    {
        perror("uart: open failed.\n");
     
        return 1;                                       /* return error */
    }
    else
    {
        return _uart_config(*fd, baud_rate, data_bits, parity, stop_bits);
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
    if (close(fd) < 0)
    {
        perror("uart: close failed.\n");                /* close failed */
     
        return 1;                                       /* return error */
    }
    else
    {
        return 0;                                       /* success return 0 */
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
    if (write(fd, buf, len) < 0) 
    {
        perror("uart: write failed.\n");                /* write failed */
     
        return 1;                                       /* return error */
    }
    else
    {
        return 0;                                       /* success return 0 */
    }
}

/**
 * @brief           uart read data
 * @param[in]       fd is the uart handle
 * @param[out]      *buf points to a data buffer
 * @param[in, out]  *len points to a length of the data buffer
 * @return          status code
 *                  - 0 success
 *                  - 1 read failed
 * @note            none
 */
uint8_t uart_read(int fd, uint8_t *buf, uint32_t *len)
{
    ssize_t l;
    
    l = read(fd, buf, *len);                            /* read data */
    if (l < 0) 
    {
        perror("uart: read failed.\n");                 /* close failed */
     
        return 1;                                       /* read error */
    }
    else
    {
        *len = l;                                       /* set read data length */

        return 0;                                       /* success return 0 */
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
    if (tcflush(fd, TCIOFLUSH) < 0)                     /* flush data */
    {
        perror("uart: flush failed.\n");                /* flush failed */
     
        return 1;                                       /* return error */
    }
    else
    {
        return 0;                                       /* success return 0 */
    }
}
