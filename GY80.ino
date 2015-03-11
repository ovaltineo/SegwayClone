#ifdef GY80_STEERING
static sensorType GY80_BOARD;

void initBoardSensor(boolean calibrate)
{
	initL3G4200D(&GY80_BOARD, calibrate);
	initADXL345(&GY80_BOARD, calibrate);
}

void initSteerSensor(boolean calibrate)
{
}

float getBoardAccel(void)
{
	getAnglesADXL345((sensorType *)&GY80_BOARD);	
	return -GY80_BOARD.x_accel;
}

float getBoardGyro(void)
{
	getAnglesL3G4200D((sensorType *)&GY80_BOARD);	
	return GY80_BOARD.y_gyro;
}

#ifndef POT_STEERING
float getSteerAccel(void)
{
	return -GY80_BOARD.y_accel;
}

float getSteerGyro(void)
{
	return GY80_BOARD.x_gyro;
}

#endif
	
#endif
