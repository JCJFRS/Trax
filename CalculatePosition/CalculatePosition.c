/*******************************************************************************
* CalculatePsition.c
*
* Justin Creaby 2018
* This program calculates the vehicle position based on heading and encoder info.
* 
* Start the program, and drive the vehicle around a path.
* See if the calculated position matches the real position.
* 
* Example of calling this function:
* 
*		sudo ./CalculatePosition
* 
*******************************************************************************/
#include <rc_usefulincludes.h>
#include <roboticscape.h>

int main(int argc, char *argv[]){
	double timeStep = 0.02; // Time step of program, (seconds).
	double timeElapsed = 0;
	int encoderPos = rc_get_encoder_pos(1);
	int lastEncoderPos = rc_get_encoder_pos(1);
	int encoderCount = 0;
	double distanceIncrement = 20.0/49.0; // 48 pulses per 20 ft, 49 / 20 ft, 49 / 20 ft
	double xPosition = 0;
	double yPosition = 0;
	double gyroZ;
	double gyroZBias = -0.592298;
	double gyroHeading = 0;
	double xPath[300];
	double yPath[300];
	rc_imu_data_t data; //struct to hold new data

	uint64_t startTimerNanoSeconds;

	// okay, off we go!
	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}

	//Configure IMU
	// use defaults for now, except also enable magnetometer.
	rc_imu_config_t conf = rc_default_imu_config();

	if(rc_initialize_imu(&data, conf)){
		fprintf(stderr,"rc_initialize_imu_failed\n");
		return -1;
	}
	
	printf("Time, EncoderPulse, EncoderCount, Gyro Heading, X Position, Y Position\n");

	int pathIndex = 0;
	// Main loop runs at frequency_hz
	while(rc_get_state()!=EXITING)
	{
		startTimerNanoSeconds = rc_nanos_since_boot();
		
		// Get Gyro Data
		rc_read_gyro_data(&data);
		gyroZ = data.gyro[2] - gyroZBias;
		gyroHeading = gyroHeading + gyroZ*timeStep;
		
		// Get Encoder Value
		encoderPos = rc_get_encoder_pos(1);
		if (encoderPos > lastEncoderPos)
		{
			encoderCount = encoderCount + 1;
			xPosition = xPosition-distanceIncrement*(sin(gyroHeading*2.0*PI/360.0));
			yPosition = yPosition+distanceIncrement*(cos(gyroHeading*2.0*PI/360.0));
			xPath[pathIndex] = xPosition;
			yPath[pathIndex] = yPosition;
			pathIndex = pathIndex + 1;
		}
		lastEncoderPos = encoderPos;
		
		printf("\r");
		printf("%7.2f | %d | %6d | %6.2f | %6.2f | % 6.2f",timeElapsed, encoderPos, encoderCount, gyroHeading, xPosition, yPosition);
		fflush(stdout);

		timeElapsed = timeElapsed + timeStep;
		
		// Stay in this loop until the amount of time for the loop has reached timeStep
		while ( (rc_nanos_since_boot() - startTimerNanoSeconds) < ( (uint64_t) (timeStep * 1000000000)) )
		{
			// printf("time Step time =  %lldns\n", rc_nanos_since_boot()-startTimer);
			// Wait until 1/frequ milliseconds has elapsed
		}
	}
	
	int j;
    for (j = 0; j <= 200; j++ ) {
      printf("%0.2f,", xPath[j]);
    }
    printf("\n\n");
    for (j = 0; j <= 200; j++ ) {
      printf("%0.2f,", yPath[j]);
    }
	rc_cleanup();
	return 0;
}
