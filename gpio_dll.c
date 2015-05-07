#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <getopt.h>
#include "libDrive.h"

#define DEV_NAME	"/dev/gpio_ctrl"

static const char *device = "/dev/spidev0.0";

int16_t gpio_open()
{
	int16_t fd;	
	fd = open(device, O_RDWR);
	if (fd < 0)
	{
		perror("can't open gpio device");
	}
	return fd;
}

void gpio_close(int fd)
{
	close(fd);
}

int gpio_get_or_set_value(int cmd_magic,char* value)
{	
	int16_t fd;
	fd = open(device, O_RDWR);
	if (fd < 0)
	{
		perror("can't open gpio device");
		return fd;
	}
	
	return ioctl(fd, cmd_magic, value);
}
