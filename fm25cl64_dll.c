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
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define fm25cl64_CMD_WRSR		0x01	/* Write status register */
#define fm25cl64_CMD_WRDI		0x04	/* Write disable */
#define fm25cl64_CMD_RDSR		0x05	/* Read status register */
#define fm25cl64_CMD_WREN		0x06	/* Write enable */
#define fm25cl64_CMD_READ		0x03	/* High speed read */
#define fm25cl64_CMD_WRITE		0x02	/* High speed write */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static const char *device = "/dev/spidev0.0";
static uint8_t mode = 3;
static uint8_t bits = 8;
static uint32_t speed = 5000000;

int16_t fm25cl64_open()
{
	int16_t fd,ret;
	
	fd = open(device, O_RDWR);
	if (fd < 0)
	{
		perror("can't open esam device");
		return fd;
	}
	
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
	{
		perror("can't set spi mode");
		return ret;
	}

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
	{
		perror("can't get spi mode");
		return ret;
	}

	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
	{
		perror("can't set bits per word");
		return ret;
	}

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
	{
		perror("can't get bits per word");
		return ret;
	}


	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
	{
		perror("can't set max speed hz");
		return ret;
	}

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
	{
		perror("can't get max speed hz");
		return ret;
	}

//	printf("spi mode: %d\n", mode);
//	printf("bits per word: %d\n", bits);
//	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);
	return fd;
}

int16_t fm25cl64_write(int16_t addr,uint8_t *data,uint16_t len)
{
	int16_t ret,fd;
		
	uint8_t write_en[1] = {fm25cl64_CMD_WREN,};
	
	uint8_t command[3];
	if( (addr + len) > 0xFFFF)
		return -EFAULT;
	command[0] = fm25cl64_CMD_WRITE;
	command[1] = addr >> 8;
	command[2] = addr;

	struct spi_ioc_transfer tr[] = {
		[0] = {
			.tx_buf = (unsigned long)write_en,
			.len = ARRAY_SIZE(write_en),
			.cs_change = 1,
		},
		[1] = {
			.tx_buf = (unsigned long)command,
			.len = ARRAY_SIZE(command),
		},
		[2] = {
			.tx_buf = (unsigned long)data,
			.len = len,
		},
	};
	fd = fm25cl64_open();
	if (fd < 0)
	{
		perror("can't perform fm25cl64_open");
		return fd;
	}
	
	ret = ioctl(fd, SPI_IOC_MESSAGE(ARRAY_SIZE(tr)), &tr);
	if (ret < 0)
	{
		perror("can't perform fm25cl64_write");
//		return ret;
	}
	close(fd);
	return ret;
}

int16_t fm25cl64_read(int16_t addr,uint8_t *data,uint16_t len)
{
	int16_t ret,fd;
			
	uint8_t command[3];
	if( (addr + len) > 0xFFFF)
		return -EFAULT;
	command[0] = fm25cl64_CMD_READ;
	command[1] = addr >> 8;
	command[2] = addr;

	struct spi_ioc_transfer tr[] = {
		[0] = {
			.tx_buf = (unsigned long)command,
			.len = ARRAY_SIZE(command),
		},
		[1] = {
			.rx_buf = (unsigned long)data,
			.len = len,
		},
	};
	
	fd = fm25cl64_open();
	if (fd < 0)
	{
		perror("can't perform fm25cl64_open");
		return fd;
	}
	
	ret = ioctl(fd, SPI_IOC_MESSAGE(ARRAY_SIZE(tr)), &tr);
	if (ret < 0)
	{
		perror("can't perform fm25cl64_read");
//		return ret;
	}
	close(fd);
	return ret;
}

//void fm25cl64_close(int16_t fd)
//{
//	close(fd);
//}
