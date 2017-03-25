#ifndef SRF02_H_
#define SRF02_H_
#define SRF02_I2C_BUFFER 6


class Srf02 {
	private:
		int I2CBus, I2CAddress;
		char dataBuffer[SRF02_I2C_BUFFER];

		int value;

		int convertValue(unsigned int MSB, unsigned int LSB);
	
	public:
		Srf02(int bus, int address);
	
		unsigned char get_i2c_register(unsigned char reg);	
		int writeValue(unsigned char value);
		int readValue();
		int getValue();
	};

#endif
