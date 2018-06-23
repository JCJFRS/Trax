/*******************************************************************************
* CalbrateGyroMag.c
*
* Justin Creaby 2018
* This calibration program calibrates the gyros and the magnetometer.
* 
* Gyro Bias:
* It removes the bias of all 3 gyros by keeping the vehicle completely still. 
* The average of the gyro value is the bias.
* 
* Magnetometer:
* After calculating the gyro bias, the vehicle starts to drive in a tight 
* circle. It drives a number of circles, at a slow speed, taking the average of
* the x and y axis mag values. Note, it is not possible to calibrate the 
* magnetometer in all axes unless the vehicle can be rolled and pitched. The
* average values of the x and y axis is the bias of the magnetometer.
* 
* The raw data is logged in a time stamped .csv file. And, the calculated biases
* are appended to the calibration file in order to keep a historical log of the 
* calibration values.
* 
* Example of calling this function. Averages the gyros or 10 seconds and runs
* 5 circles for the mag calibration:
* 
*		sudo ./avc_calibrate_gyro_mag 10 5
* 
*******************************************************************************/

#include <rc_usefulincludes.h>
#include <roboticscape.h>


int main(int argc, char *argv[]){
	if(argc != 3 )
	{
		printf("    Incorrect usage.\n");
		printf("    Need to specify amount of time (sec) to calibrate gyros.\n");
		printf("    Need to specify number of circles to calibrate mags.\n");
		printf("    Example: sudo ./avc_calibrate_gyro_mag 10 3\n");
		return 0;
	}
	
	double esc_throttle = 0.57;
	double timeStep = 0.02; // Time step of program, (seconds).
	double timeMax = 200.0; // Max time for program to run, (seconds).
	double timeElapsed = 0;
	int gyroCount = 1;
	double gyroCalTime = atof(argv[1]);
	int maxGyroCount = (int) (gyroCalTime / timeStep); 
	double steeringAngle = -0.75;
	double gyroZ;
	int calcGyroBias = 1;
	double gyroXBias = 0;
	double gyroYBias = 0;
	double gyroZBias = 0;
	double heading = 0;
	double circles = 0;
	double magCalCircles = atof(argv[2]);
	int calcMagBias = 1;
	int magCount = 1;
	double magXBias = 0;
	double magYBias = 0;
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
	time_t rawtime;
    struct tm * timeinfo;
    time ( &rawtime );
    timeinfo = localtime ( &rawtime );
    printf("Date dd/mm/yyyy: %d/%d/%d, Time: hr:min:sec: %d:%d:%d]",timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

	// Prepare file for writing
	FILE *fptr;
	if( access( "GyroMagCalRecord", F_OK ) != -1 ) {
    	// file exists
    	fptr = fopen("GyroMagCalRecord", "a");
	} else {
    	// file doesn't exist
    	fptr = fopen("GyroMagCalRecord", "a");
		fprintf(fptr, "Day, Month, Year, Hour, Minute, Second, GyroXBias, GyroYBias, GyroZBias, MagXBias, MagYBias, GyroCalDuration, MagCalCircles\n");
	}

	// Main loop runs at frequency_hz
	while(rc_get_state()!=EXITING && timeElapsed<timeMax && (calcMagBias || calcGyroBias))
	{
		startTimerNanoSeconds = rc_nanos_since_boot();
		
		// Read gyro data
		if(rc_read_gyro_data(&data)<0)
		{
			printf("read gyro data failed\n");
		};
		
		// Read magnetometer data
		if(rc_read_mag_data(&data)<0)
		{
			printf("read mag data failed\n");
		}
		
		// Calculate Gyro Bias
		if (calcGyroBias)
		{
			gyroXBias = gyroXBias+(data.gyro[0]-gyroXBias)/gyroCount;
			gyroYBias = gyroYBias+(data.gyro[1]-gyroYBias)/gyroCount;
			gyroZBias = gyroZBias+(data.gyro[2]-gyroZBias)/gyroCount;
			printf("Elapsed time: %f Calibrating Gyro Bias:    X Bias = %f, Y Bias = %f, Z Bias = %f\n", timeElapsed, gyroXBias, gyroYBias, gyroZBias);
			
			// Initialize ESC motor and steering.
			rc_send_servo_pulse_normalized(1, 0.0);
			rc_send_esc_pulse_normalized(2, 0.50);
				
			gyroCount = gyroCount + 1;
			if (gyroCount >= maxGyroCount)
			{
				calcGyroBias = 0;
			}
		}
		else if (calcMagBias) // Calculate Mag Bias
		{
			gyroZ = data.gyro[2] - gyroZBias;
			heading = heading + gyroZ * timeStep;
			circles = heading / 360.0;
	
			magXBias = magXBias+(data.mag[0]-magXBias)/magCount;
			magYBias = magYBias+(data.mag[1]-magYBias)/magCount;
			magCount = magCount + 1;
			printf("Elapsed time: %f Calibrating Mag Bias:    X Bias = %f, Y Bias = %f, Heading = %f, Circles = %f\n", timeElapsed, magXBias, magYBias, heading, circles);
			
			// Send steering angle for driving a tight circle
			if (circles <= magCalCircles)
			{
				rc_send_servo_pulse_normalized(1, steeringAngle);
				rc_send_esc_pulse_normalized(2, esc_throttle);
			}
			else
			{
				calcMagBias = 0;
			}
		}

		timeElapsed = timeElapsed + timeStep;
		
		// Stay in this loop until the amount of time for the loop has reached timeStep
		while ( (rc_nanos_since_boot() - startTimerNanoSeconds) < ( (uint64_t) (timeStep * 1000000000)) )
		{
			// printf("time Step time =  %lldns\n", rc_nanos_since_boot()-startTimer);
			// Wait until 1/frequ milliseconds has elapsed
		}
		

	}
	
	// Only write to the file if both calibrations were completed.
	if (calcGyroBias == 0 && calcMagBias == 0)
	{
		printf("Day, Month, Year, Hour, Minute, Second, GyroXBias, GyroYBias, GyroZBias, MagXBias, MagYBias, GyroCalDuration, MagCalCircles\n");
		printf("%d, %d, %d, %d, %d, %d, %f, %f, %f, %f, %f, %f, %f\n",timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec, gyroXBias, gyroYBias, gyroZBias, magXBias, magYBias, gyroCalTime, magCalCircles);

		fprintf(fptr, "%d, %d, %d, %d, %d, %d, %f, %f, %f, %f, %f, %f, %f\n",timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec, gyroXBias, gyroYBias, gyroZBias, magXBias, magYBias, gyroCalTime, magCalCircles);
	}
	fclose(fptr);
	
	rc_cleanup();
	return 0;
}
