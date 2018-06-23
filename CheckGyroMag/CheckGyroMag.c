/*******************************************************************************
* CheckGyroMag.c
*
* Justin Creaby 2018
* This program checks teh gyro and mag values after calibration.
* 
* It removes the bias from both gyro and mag and prints the new values.
* And, it calculates a heading from both Gyro and Mag.
* 
* Example of calling this function:
* 
*		sudo ./CheckGyroMag
* 
*******************************************************************************/

#include <rc_usefulincludes.h>
#include <roboticscape.h>

int main(int argc, char *argv[]){

	
	double timeStep = 0.2; // Time step of program, (seconds).
	double magX;
	double magY;
	double magXBias = 20.0;//16.5;
	double magYBias = -2.362458;//-2.5;
	double gyroX;
	double gyroY;
	double gyroZ;
	double gyroXBias = -0.65;
	double gyroYBias = -1.4;
	double gyroZBias = -0.7;
	double DECLINATION = -8.0;
	double gyroHeading = 0;
	uint64_t startTimerNanoSeconds;
	rc_imu_data_t data; //struct to hold new data

	// okay, off we go!
	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}

	//Configure IMU
	// use defaults for now, except also enable magnetometer.
	rc_imu_config_t conf = rc_default_imu_config();
	conf.enable_magnetometer=1;

	if(rc_initialize_imu(&data, conf)){
		fprintf(stderr,"rc_initialize_imu_failed\n");
		return -1;
	}
	
	// Get current UTC date and time
/*	time_t rawtime;
    struct tm * timeinfo;
    time ( &rawtime );
    timeinfo = localtime ( &rawtime );*/
    //printf("Date dd/mm/yyyy: %d/%d/%d, Time: hr:min:sec: %d:%d:%d \n",timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

	printf("gyroX, gyroY, gyroZ, magX, magY, magHeading, gyroHeading\n");
		
	// Main loop runs at frequency_hz
	while(rc_get_state()!=EXITING)
	{
		startTimerNanoSeconds = rc_nanos_since_boot();
		
		// Read magnetometer data
		rc_read_mag_data(&data);
		magX = 1*(data.mag[0]-magXBias);
		magY = -1*(data.mag[1]-magYBias);
		
		double magHeading;
		if (magX == 0)
		{
    		magHeading = (magY > 0) ? PI : 0;
		}
		else
		{
    		magHeading = atan2(magX, magY);
		}
    
		magHeading = magHeading - DECLINATION * PI / 180;
  
  if (magHeading > PI)
  {
  	magHeading -= (2 * PI);
  }
  else if (magHeading < -PI)
  {
  	magHeading += (2 * PI);
  }
  else if (magHeading < 0)
  {
  	magHeading += 2 * PI;
  }
  
  // Convert everything from radians to degrees:
  magHeading *= 180.0 / PI;
		
		// Get Gyro Data
		rc_read_gyro_data(&data);
		gyroX = data.gyro[0] - gyroXBias;
		gyroY = data.gyro[1] - gyroYBias;
		gyroZ = data.gyro[2] - gyroZBias;
		gyroHeading = gyroHeading + gyroZ*timeStep;
		
		printf("\r");
		printf("%f, %f, %f, %f, %f, %f, %f\n", gyroX, gyroY, gyroZ, magX, magY, magHeading, gyroHeading);
		fflush(stdout);
		
		// Stay in this loop until the amount of time for the loop has reached timeStep
		while ( (rc_nanos_since_boot() - startTimerNanoSeconds) < ( (uint64_t) (timeStep * 1000000000)) )
		{
			// printf("time Step time =  %lldns\n", rc_nanos_since_boot()-startTimer);
			// Wait until 1/frequ milliseconds has elapsed
		}
		

	}
	

	
	rc_cleanup();
	return 0;
}
