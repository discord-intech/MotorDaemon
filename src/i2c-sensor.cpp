#include "i2c-sensor.hpp"

int i2c_write_value(int file, unsigned char addr, unsigned char reg, unsigned char value) {


	unsigned char outbuf[2];
	struct i2c_rdwr_ioctl_data packets;
	struct i2c_msg messages[1];

	messages[0].addr  = addr;
	messages[0].flags = 0;
	messages[0].len   = sizeof(outbuf);
	messages[0].buf   = outbuf;

	// Registre dans lequel on ecrit
	outbuf[0] = reg;
  // Valeur à ecrire (on doit pouvoir ecrire des mots en ajoutant des adresses)
	outbuf[1] = value;

	// Envoi et verification
	packets.msgs  = messages;
	packets.nmsgs = 1;
	if(ioctl(file, I2C_RDWR, &packets) < 0) {
		perror("Unable to send data");
		return 1;
	}

	return 0;
}


unsigned char i2c_get_register(int file, unsigned char addr, unsigned char reg) {


	unsigned char inbuf, outbuf;
	struct i2c_rdwr_ioctl_data packets;
	struct i2c_msg messages[2];

	outbuf = reg;
	messages[0].addr  = addr;
	messages[0].flags = 0;
	messages[0].len   = sizeof(outbuf);
	messages[0].buf   = &outbuf;

	// Structure dans laquelle on récupère les données
	messages[1].addr  = addr;
	messages[1].flags = I2C_M_RD/* | I2C_M_NOSTART*/;
	messages[1].len   = sizeof(inbuf);
	messages[1].buf   = &inbuf;

  //TODO: essayer de lire des mots d'un seul coup

	// Envoi et récupération des données
	packets.msgs      = messages;
	packets.nmsgs     = 2;
	if(ioctl(file, I2C_RDWR, &packets) < 0) {
		perror("Unable to send data");
		return 1;
	}

	return inbuf;
}
