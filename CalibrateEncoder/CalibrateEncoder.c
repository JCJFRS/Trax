/*******************************************************************************
* CalibateEncoder.c
*
* Justin Creaby 2018
* This calibration program calibrates the motor encoder.
* 
* The encoder is monitored and all the pulses are counted.
* The user must drive the vehicle along a straight path for a 
* predefined distance.
* 
* Example of calling this function:
* 
*		sudo ./CalibateEncoder
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
	float encoderCnttoDistance = 20.0/49.0; // 48 pulses per 20 ft, 49 / 20 ft, 49 / 20 ft
	float distance = 0;

	uint64_t startTimerNanoSeconds;

	// okay, off we go!
	// initialize hardware first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to run rc_initialize(), are you root?\n");
		return -1;
	}


	printf("Time, EncoderPulse, EncoderCount, Distance\n");


	// Main loop runs at frequency_hz
	while(rc_get_state()!=EXITING)
	{
		startTimerNanoSeconds = rc_nanos_since_boot();
		
		// Get Encoder Value
		encoderPos = rc_get_encoder_pos(1);
		if (encoderPos > lastEncoderPos)
		{
			encoderCount = encoderCount + 1;
			distance = encoderCount * encoderCnttoDistance;
		}
		lastEncoderPos = encoderPos;
		
		printf("\r");
		printf("%7.2f | %d | %6d | %6.3f",timeElapsed, encoderPos, encoderCount, distance);
		fflush(stdout);

		timeElapsed = timeElapsed + timeStep;
		
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
