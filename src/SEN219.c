
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port

//--- comment or uncomment the line below to show or not debug messages 
//#define DEBUG

#define SLAVE_ADDR 0x45
#define STATUS_OK  0
#define BUFF_SIZE  60

//--- REGISTERS CODE FOR THE INA219
#define CONFIG_REG  0x00
#define SHUNT_REG   0x01
#define BUS_REG	    0x02
#define POWER_REG   0x03
#define CURRENT_REG 0x04
#define CALIB_REG   0x05

//--- DEFAULT CONFIGURATION VALUE FOR QARPEDIEM REQUIREMENTS:
//--- 16V range, 12bits resolution, without averaging, shunt & bus
//--- For other configurations look at the page 19 of the INA219 datasheet
#define FAST_CONFIG 0x1E47 

// -- Calibration constants
#define CALIBRATION_CST 		0.04096
#define SHUNT_RESISTOR  		0.1
#define CALIBRATION_MSK			0xFFFE

int i2c_fd;
int length;
unsigned char wr_buffer[BUFF_SIZE] = {0};
unsigned char rd_buffer[BUFF_SIZE] = {0};
////////////////////////////////////////////////////////////////////////////////////
//--- The methode of calculation used to calibrate the Sensor is present in the
//--- INA219 datasheet on page 12
unsigned int getCalibrationValue(float maxExpectedCurrent){
	float currentLSB= (maxExpectedCurrent)/(0x8000);
	unsigned int value= (unsigned int)((CALIBRATION_CST)/(currentLSB*SHUNT_RESISTOR));
	value=value & CALIBRATION_MSK;
	#ifdef DEBUG
		printf("cal value:%d\n",value);
	#endif
	return value;
}

////////////////////////////////////////////////////////////////
int I2C_init(char* filename, int* i2c_fd, int slaveAddr){
	if ((*i2c_fd = open(filename, O_RDWR)) < 0)
	{
		printf("Failed to open the i2c bus");
		return -1;
	}

	if (ioctl(*i2c_fd, I2C_SLAVE, slaveAddr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
		return -1;
	}
	return 0;
}
/////////////////////////////////////////////////////////////////////////////
int I2C_write(int i2c_fd, unsigned char *wr_buffer,int length){
	//write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
	if (write(i2c_fd, wr_buffer, length) != length)
	{
		printf("Failed to write to the i2c bus.\n");
		return -1;
	}
	return 0;
}
//////////////////////////////////////////////////////////////////////////////
int I2C_read(int i2c_fd, unsigned char *rd_buffer, int length){
	//read() returns the number of bytes actually read, if it doesn't match then an error occurred (ex no response from the device)
	if (read(i2c_fd, rd_buffer, length) != length) return -1;
	else return 0 ;
}

//////////////////////////////////////////////////////////////////////////////
//--- Allows you to get the value of a specific register on the Sensor
// 	  The parameter is the registers code (DEFINED ABOVE)
int get(int parameter){
    wr_buffer[0] = parameter;
    length = 1;

    if( I2C_write(i2c_fd, wr_buffer, length)  != STATUS_OK) perror("I2C_write");
	
	switch(parameter) {
	case CONFIG_REG  :
		#ifdef DEBUG
			printf("Reading Configuration:\n");
		#endif
		break; 
	case SHUNT_REG  :
		#ifdef DEBUG
			printf("Reading Shunt Voltage:\n");
		#endif
		break; 
	case BUS_REG  :
		#ifdef DEBUG
			printf("Reading Bus Voltage:\n");
		#endif
		break; 
	case POWER_REG  :
		#ifdef DEBUG
			printf("Reading Power:\n");
		#endif
		break; 
	case CURRENT_REG  :
		#ifdef DEBUG
			printf("Reading Current:\n");
		#endif
		break; 
	case CALIB_REG  :
		#ifdef DEBUG
			printf("Reading Calibration:\n");
		#endif
		break; 
	default : 
		#ifdef DEBUG
			printf("Unsupported  parameter !\n");
		#endif
		return -1;
	}
	
	//----- READ 2 BYTES On I2C
	length = 2;
	if( I2C_read( i2c_fd, rd_buffer, length) != STATUS_OK ) perror("I2C_read");
	int decimal= ((rd_buffer[0] & 0xFFFF)<<8) | (rd_buffer[1] & 0xFFFF);
	
	#ifdef DEBUG
		printf("\tResult: 0x%hhx | 0x%hhx  \tdecimal: %d\n\n", rd_buffer[0],rd_buffer[1], decimal);
	#endif
	return decimal;
}

//////////////////////////////////////////////////////////////////////////////
void set(int param, int value){
        if(!(param==CALIB_REG || param==CONFIG_REG)) {
			printf("Bad Parameter!\n");
			return;
		}
		wr_buffer[0] = param;
        wr_buffer[1] = (unsigned char) ((0xFF00 & value)>>8);
	    wr_buffer[2] = (unsigned char) (0x00FF & value);
	    #ifdef DEBUG
			printf("Written ======> 0x%hhx | 0x%hhx \n\n", wr_buffer[1], wr_buffer[2]);
	    #endif
		length = 3;
        if( I2C_write(i2c_fd, wr_buffer, length)  != STATUS_OK) perror("I2C_write");
}
/////////////////////////////////////////////////////////////////////////////
//--- Gives the value in mV of the bus voltage (between IN- & GND)
int getBusVoltage(){
	int val= get(BUS_REG);
	val = (val>>3);
	val = val * 4;
	return val;
}
//////////////////////////////////////////////////////////////////////////////
void initSensor(){
	set(CONFIG_REG, FAST_CONFIG);
	get(CONFIG_REG);
	set(CALIB_REG, getCalibrationValue(1000));
	get(CALIB_REG);
}
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
int main(){
	//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";
	if( I2C_init(filename, &i2c_fd, SLAVE_ADDR) != STATUS_OK) perror("init");

	//----- INITIALIZE AND CALIBRATE THE SENSOR
	initSensor();

	printf("Bus Voltage:%d mV\n",getBusVoltage());

	return 0;
}

