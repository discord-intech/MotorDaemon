#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <iostream>

using namespace std;


int i2c_write_value(int file, unsigned char addr, unsigned char reg, unsigned char value);

unsigned char i2c_get_register(int file, unsigned char addr, unsigned char reg);
