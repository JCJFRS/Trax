/*******************************************************************************
* FollowPath.c
*
* Justin Creaby 2018
* This program follows a predefined path.
* 
* The car will stop when a position of 0, 0 is desired or by hitting control C
* 
* Example of calling this function:
* 
*		sudo ./FollowPath
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
	double initializeCount = 0;
	double pathX[300] = {0.00,-0.01,-0.03,-0.06,-0.13,-0.25,-0.42,-0.65,-0.92,-1.23,-1.56,-1.91,-2.26,-2.61,-2.96,-3.32,-3.69,-4.05,-4.42,-4.78,-5.13,-5.48,-5.81,-6.13,-6.43,-6.71,-6.96,-7.19,-7.40,-7.57,-7.73,-7.85,-7.94,-8.00,-8.03,-8.04,-8.04,-8.03,-8.00,-7.94,-7.86,-7.75,-7.62,-7.45,-7.26,-7.02,-6.76,-6.47,-6.15,-5.81,-5.45,-5.08,-4.69,-4.29,-3.89,-3.48,-3.08,-2.67,-2.26,-1.86,-1.46,-1.06,-0.66,-0.28,0.11,0.48,0.85,1.21,1.58,1.94,2.30,2.65,3.01,3.37,3.74,4.10,4.47,4.84,5.22,5.59,5.97,6.35,6.72,7.09,7.45,7.80,8.12,8.41,8.67,8.90,9.09,9.24,9.36,9.46,9.55,9.62,9.69,9.76,9.83,9.91,9.99,10.09,10.19,10.31,10.44,10.57,10.70,10.85,10.99,11.13,11.26,11.38,11.50,11.60,11.69,11.74,11.77,11.78,11.78,11.77,11.75,11.70,11.62,11.53,11.40,11.24,11.05,10.81,10.55,10.26,9.94,9.61,9.25,8.89,8.52,8.14,7.75,7.37,6.99,6.60,6.21,5.81,5.41,5.01,4.60,4.19,3.78,3.38,2.99,2.60,2.23,1.88,1.56,1.27,1.00,0.77,0.58,0.43,0.33,0.27,0.23,0.19,0.15,0.11,0.07,0.03,-0.01,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00};
	double pathY[300] = {0.41,0.82,1.22,1.63,2.03,2.42,2.79,3.13,3.43,3.70,3.94,4.16,4.37,4.58,4.78,4.97,5.16,5.34,5.52,5.71,5.92,6.13,6.37,6.63,6.90,7.20,7.52,7.86,8.21,8.58,8.96,9.35,9.74,10.15,10.56,10.96,11.37,11.78,12.19,12.59,12.99,13.38,13.77,14.14,14.50,14.84,15.15,15.43,15.69,15.92,16.11,16.27,16.39,16.49,16.56,16.61,16.62,16.61,16.58,16.51,16.43,16.34,16.24,16.12,15.97,15.81,15.63,15.45,15.27,15.08,14.88,14.68,14.49,14.30,14.12,13.94,13.76,13.59,13.42,13.26,13.11,12.96,12.80,12.62,12.43,12.21,11.96,11.68,11.36,11.02,10.66,10.28,9.89,9.50,9.10,8.70,8.29,7.89,7.49,7.09,6.69,6.29,5.90,5.51,5.12,4.73,4.35,3.97,3.58,3.20,2.81,2.42,2.03,1.64,1.24,0.83,0.43,0.02,-0.39,-0.80,-1.20,-1.61,-2.01,-2.41,-2.80,-3.17,-3.53,-3.86,-4.17,-4.46,-4.72,-4.95,-5.16,-5.35,-5.51,-5.66,-5.80,-5.95,-6.09,-6.22,-6.33,-6.42,-6.50,-6.54,-6.56,-6.55,-6.51,-6.44,-6.33,-6.20,-6.03,-5.81,-5.57,-5.28,-4.97,-4.63,-4.27,-3.89,-3.50,-3.10,-2.69,-2.28,-1.88,-1.47,-1.06,-0.66,-0.25,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00};
	int pathIndex = 0;
	double steeringSaturationAngle = 30; // degrees
	double steeringRateLimit = 20;
	double lastSteeringAngle = 0;
	double wheelBase = 1.083; // 13 inches to feet
	
	rc_imu_data_t data; //struct to hold new data

	uint64_t startTimerNanoSeconds;

	// okay, off we go!
	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}


	// Initialize ESC motor and steering.
	// Send commands for 1 second.
	while (initializeCount<50)
	{
		rc_send_servo_pulse_normalized(1, 0.0);
		rc_send_esc_pulse_normalized(2, 0.50);
		initializeCount = initializeCount+1;
	}
			
	//Configure IMU
	// use defaults for now, except also enable magnetometer.
	rc_imu_config_t conf = rc_default_imu_config();

	if(rc_initialize_imu(&data, conf)){
		fprintf(stderr,"rc_initialize_imu_failed\n");
		return -1;
	}
	
	printf("Time, EncoderPulse, EncoderCount, Gyro Heading, X Position, Y Position, pathX, pathY, steeringAngle\n");

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
		}
		lastEncoderPos = encoderPos;
		
		// Determine Goal Point
		// While within 2 foot of the desired point, increase the index of desired point
        while (sqrt((pathX[pathIndex] - xPosition)*(pathX[pathIndex] - xPosition) + (pathY[pathIndex] - yPosition)*(pathY[pathIndex] - yPosition)) < 2)
        {
            pathIndex = pathIndex+1;
			if (pathIndex == 299)
			{
				printf("\n1st break\n");
				break;
			}
        }
	
	if (pathX[pathIndex] == 0 && pathY[pathIndex] == 0)
    {
      	printf("2nd break\n");
      	break;
    }	
		
		
		// Calculate the desired steering angle based on the Pure Pursuit method
    double dE = pathX[pathIndex] - xPosition;
    double dN = pathY[pathIndex] - yPosition;
    double dist2DesiredPoint = sqrt(dE*dE + dN*dN);
    double goalPointAngle = -1*atan2(dE,dN); // same as from matlab script: double matlabangle = -(PI/2-atan2(dN,dE));
	double alpha = goalPointAngle - gyroHeading/360*(2*PI);
    double k = 2 * sin(alpha) / dist2DesiredPoint;
    double steeringAngle = atan(k*wheelBase) * 360/(2*PI) * (2);			////////////////// STEERING ANGLE GAIN ////////////////////////////////
    
    
    // Saturate the steering angle
    if (steeringAngle > steeringSaturationAngle)
    {
        steeringAngle = steeringSaturationAngle;
    }
    if (steeringAngle < -steeringSaturationAngle)
    {
        steeringAngle = -steeringSaturationAngle;
    }
    
    // Rate limit the steering angle
    if ((steeringAngle - lastSteeringAngle) > (steeringRateLimit * timeStep))
    {
        steeringAngle = lastSteeringAngle + (steeringRateLimit * timeStep);
    }
    if ((-steeringAngle + lastSteeringAngle) > (steeringRateLimit * timeStep))
    {
        steeringAngle = lastSteeringAngle - (steeringRateLimit * timeStep);
    }
        
    lastSteeringAngle = steeringAngle;
	
	rc_send_servo_pulse_normalized(1, (0.70/-30)*steeringAngle);
	rc_send_esc_pulse_normalized(2, 0.57);
		
		printf("\r");
		printf("%7.2f | %d | %6d | %6.2f | %6.2f | % 6.2f | % 6.2f | % 6.2f | % 6.2f",timeElapsed, encoderPos, encoderCount, gyroHeading, xPosition, yPosition, pathX[pathIndex], pathY[pathIndex], steeringAngle);
		fflush(stdout);

		timeElapsed = timeElapsed + timeStep;
		
		// Stay in this loop until the amount of time for the loop has reached timeStep
		while ( (rc_nanos_since_boot() - startTimerNanoSeconds) < ( (uint64_t) (timeStep * 1000000000)) )
		{
			// printf("time Step time =  %lldns\n", rc_nanos_since_boot()-startTimer);
			// Wait until 1/frequ milliseconds has elapsed
		}
	}
	
	rc_send_servo_pulse_normalized(1, 0);
	rc_cleanup();
	return 0;
}
