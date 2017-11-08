#include "Srf02.hpp"

using namespace std;

#define VALUE_LSB 0x03
#define VALUE_MSB 0x02
#define COMMAND_REG 0x00

#define MAX_BUF 64

Srf02::Srf02(int bus, unsigned char address){
	I2CBus = bus;
	I2CAddress = address;
	readValue();
}

int Srf02::writeValue(unsigned char value) {

	return i2c_write_value(I2CBus, I2CAddress, COMMAND_REG, value);
}


unsigned char Srf02::get_i2c_register(unsigned char reg) {

	return i2c_get_register(I2CBus, I2CAddress, reg);
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
