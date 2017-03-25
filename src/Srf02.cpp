#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include "Srf02.hpp"
#include <iostream>

using namespace std;

#define VALUE_LSB 0x03
#define VALUE_MSB 0x02
#define COMMAND_REG 0x00

#define MAX_BUF 64

Srf02::Srf02(int bus, int address){
	I2CBus = bus;
	I2CAddress = address;
	readValue();
}

int Srf02::writeValue(unsigned char value) {

	unsigned char addr = I2CAddress;
	unsigned char reg = COMMAND_REG;

	int file = I2CBus;

	//int file = open("/dev/i2c-2", O_RDWR);

	unsigned char outbuf[2];
	struct i2c_rdwr_ioctl_data packets;
	struct i2c_msg messages[1];

	messages[0].addr  = addr;
	messages[0].flags = 0;
	messages[0].len   = sizeof(outbuf);
	messages[0].buf   = outbuf;

	/* The first byte indicates which register we'll write */
	outbuf[0] = reg;

	/* 
	 *      * The second byte indicates the value to write.  Note that for many
	 *           * devices, we can write multiple, sequential registers at once by
	 *                * simply making outbuf bigger.
	 *                     */
	outbuf[1] = value;

	/* Transfer the i2c packets to the kernel and verify it worked */
	packets.msgs  = messages;
	packets.nmsgs = 1;
	if(ioctl(file, I2C_RDWR, &packets) < 0) {
		perror("Unable to send data");
		return 1;
	}

	return 0;
}


unsigned char Srf02::get_i2c_register(unsigned char reg) {
	
	
	int file = I2CBus;
	unsigned int addr = I2CAddress;

	unsigned char inbuf, outbuf;
	struct i2c_rdwr_ioctl_data packets;
	struct i2c_msg messages[2];

	/*
	 *      * In order to read a register, we first do a "dummy write" by writing
	 *           * 0 bytes to the register we want to read from.  This is similar to
	 *                * the packet in set_i2c_register, except it's 1 byte rather than 2.
	 *                     */
	outbuf = reg;
	messages[0].addr  = addr;
	messages[0].flags = 0;
	messages[0].len   = sizeof(outbuf);
	messages[0].buf   = &outbuf;

	/* The data will get returned in this structure */
	messages[1].addr  = addr;
	messages[1].flags = I2C_M_RD/* | I2C_M_NOSTART*/;
	messages[1].len   = sizeof(inbuf);
	messages[1].buf   = &inbuf;

	/* Send the request to the kernel and get the result back */
	packets.msgs      = messages;
	packets.nmsgs     = 2;
	if(ioctl(file, I2C_RDWR, &packets) < 0) {
		perror("Unable to send data");
		return 1;
	}
	
	return inbuf;
}

int Srf02::readValue(){




	this->value = convertValue(VALUE_MSB, VALUE_LSB);
	return 0;
}

int Srf02::convertValue(unsigned int MSB, unsigned int LSB){
	short temp = dataBuffer[MSB];
	temp = (temp<<8) | dataBuffer[LSB];
	return temp;
}

int Srf02::getValue(){
	return this->value;
}
