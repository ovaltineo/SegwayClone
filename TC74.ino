#ifdef TEMPERATURE_SENSORS

int TC74_read(uint8_t address, int *temp)
{
	int n;
        uint8_t reg;

	I2c.setSpeed(0);	// slow 100KHz
	I2c.timeOut(3);	// wait 3ms
	n = I2c.read(address, 0, 1, &reg);
        *temp = reg;
	return (-n);
}


int TC74_write(int address, int data)
{
	int n;

	I2c.setSpeed(0);
	I2c.timeOut(3);
	n = I2c.write(address, 0, data); 
	return (-n);
}
#endif
