/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */

//arm-linux-gcc spidev_test.c -fPIC -shared -o libesam.so

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "libDrive.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))


static const char* device = "/dev/spidev0.1";
static uint8_t mode = 3;
static uint8_t bits = 8;
static uint32_t speed = 5000000;
static uint16_t delay = 20000;

int esam_open()
{
    int fd, ret;
    fd = open(device, O_RDWR);

    if (fd < 0) {
        perror("can't open esam device");
        return -1;
    }

    ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);

    if (ret == -1) {
        perror("can't set spi mode");
        return ret;
    }

    ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);

    if (ret == -1) {
        perror("can't get spi mode");
        return ret;
    }

    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);

    if (ret == -1) {
        perror("can't set bits per word");
        return ret;
    }

    ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);

    if (ret == -1) {
        perror("can't get bits per word");
        return ret;
    }

    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    if (ret == -1) {
        perror("can't set max speed hz");
        return ret;
    }

    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);

    if (ret == -1) {
        perror("can't get max speed hz");
        return ret;
    }
//	printf("spi mode: %d\n", mode);
//	printf("bits per word: %d\n", bits);
//	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);
    return fd;
}

//uint8_t get_xl[] = {0x55,0x80,0x0e,0x00,0x02,0x00,0x00,0x00};
//int esam_read(uint8_t* in, uint8_t len_wr, uint8_t* out, uint8_t* len_rd)
int esam_read(uint32_t esamCMD,uint8_t *pInBuf,uint16_t inLen,uint8_t* out, uint16_t* len_rd)
{
    int fd, ret = 0, i;
    
    uint8_t LRC1 = 0x00;
    uint8_t LRC2 = 0x00;
    
    uint8_t tx_buff[2048];
    
    uint8_t buf_header[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    uint16_t field_len;
    
    if(inLen > 2048)
    {
    	ret = ERR_TX_BIG;
    	return ret;
    }
    
    tx_buff[0] = 0x55;
    tx_buff[1] = (esamCMD >> 24)&0xff;
    tx_buff[2] = (esamCMD >> 16)&0xff;
    tx_buff[3] = (esamCMD>>8)&0xff;
    tx_buff[4] = (esamCMD)&0xff;
    tx_buff[5] = (inLen>>8)&0xff;
    tx_buff[6] = (inLen)&0xff;

    if(inLen != 0)
    {
    	memcpy(&tx_buff[7],pInBuf,inLen);
    }
    
    for(i = 1; i < 7 + inLen ; i++) {
        LRC1 ^= tx_buff[i];
    }
	LRC1 = ~ LRC1;
	    
    tx_buff[7 + inLen  ] = LRC1;
    
	for (i = 0; i < 7 + inLen + 1; i++) {
		if (!(i % 8))
			puts("");
		printf("%.2X ", tx_buff[i]);
	}
    
    *len_rd = 0;
    struct spi_ioc_transfer tr[] = {
        {
            .tx_buf = (unsigned long)tx_buff,
            .len = 7 + inLen + 1,
            .cs_change = 1,
            .delay_usecs = delay,
        },
        {
            .rx_buf = (unsigned long)buf_header,
            .len = 7,
            .cs_change = 1,
        },
    };
    struct spi_ioc_transfer tr1[] = {
        {
            .rx_buf = (unsigned long)out,
            .len = 1,
        },
    };

    fd = esam_open();

    if (fd < 0) {
        perror("can't perform esam_open");
        ret = ERR_DRV_OPEN;
        return ret;
    }

    ret = ioctl(fd, SPI_IOC_MESSAGE(ARRAY_SIZE(tr)), &tr); //1. 0xFF,0xFF,0x55,0x90,0x00,Len1,Len2

    if (ret < 0) {
        perror("can't perform ioctl,1");
        ret = ERR_DRV_IOCTL;
        goto ERR;
    }

	if( (buf_header[0] != 0xFF) || (buf_header[1] != 0xFF) || (buf_header[2] != 0x55) )
	{
		printf("read first three byte err the 0=%x,1=%x,2=%x\n", buf_header[0],buf_header[1], buf_header[2]);
		ret = ERR_SYNC;
		goto ERR;
	}
    
    if( (buf_header[3] != 0x90) || (buf_header[4] != 0x00) ) 
    {
        printf("read second four byte err the 3=%x,4=%x,5=%x,6=%x\n",
               buf_header[3], buf_header[4], buf_header[5], buf_header[6]);
        ret = ( buf_header[3] << 8 | buf_header[4] );
        goto ERR;
    }
    

    field_len = (buf_header[5] << 8 | buf_header[6] );
    tr1[0].len = field_len + 1;
    printf("the tr1[0].len =%d\n", field_len);
    
    ret = ioctl(fd, SPI_IOC_MESSAGE(ARRAY_SIZE(tr1)), &tr1); //2.

    if (ret < 0) {
        perror("can't perform ioctl,2");
        ret = ERR_DRV_IOCTL;
        goto ERR;
    }

    for(i = 3; i < 7; i++) {
        LRC2 ^= buf_header[i];
    }

    for(i = 0; i < field_len; i++) {
        LRC2 ^= out[i];
    }

    LRC2 = ~LRC2;

	printf("LRC2=%x,actual=%x\n", LRC2, out[field_len]);
	
    if( LRC2 != out[field_len] ) {
    	ret = ERR_LRC_CHECK;
    	goto ERR;
    }
	ret = ( buf_header[3] << 8 | buf_header[4] );
    *len_rd = field_len;
ERR:
    close(fd);
    return ret;
}


