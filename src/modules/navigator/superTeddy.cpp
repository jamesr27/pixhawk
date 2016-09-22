// James Reeves. Super Teddy waypoint/mission helper class.

#include <sys/types.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <float.h>

#include <systemlib/err.h>
#include <geo/geo.h>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/wind_estimate.h>

#include "navigator.h"
#include "mission_block.h"

superTeddy::superTeddy():
	teddy_state(0)
{
}

superTeddy::~superTeddy()
{
}

bool superTeddy::isStateComplete()
{
	// Check if the current stage of the mission is complete. If it is return true.
	// I think we'll put in a kind of switch case here, as the different stages have different goals and criteria.

	// We'll start off by getting the distance we are way from the target drop zone.


	// State 0 is not a real state and never used.

	// State 1: Simply need to reach the dropzone, same as a default pixhawk waypoint...
	// We are only going to check position though. We have a distance, check it against a tolerance. Let's use 40 meters.
	if (teddy_state == 1){
		if (dist <= 60.f && dist >= 0.0f){
			// Get time that wind measurement starts from.
			measureStartTime = hrt_absolute_time();
			return true;
		}
	}

	// State 2: Here we will loiter for 30 seconds, at the end we'll pull the wind estimate out of the
	// Kalman filter. Then move on.
	// So we need a start time, and an hold of the current time.
	if (teddy_state == 2){
		uint64_t timeNow = hrt_absolute_time();
			if((float)(timeNow - measureStartTime) >= (windMeasureTime)*1000000){
			// Pull data from the uORB.
			// Assign to class variables.
			wind_vx = windRunning_vx;
			wind_vy = windRunning_vy;
			printf("Teddy measured wind: %0.3f %0.3f\n",(double)wind_vx,(double)wind_vy);
			return true;
		}
	}

	// State 3: Pre-delivery

	// State 4: Delivery


	return false;
}

void superTeddy::advanceState(int& input){
	if(input <= 5 ){
		input++;
	}
	printf("Teddy advance to: %d\n",input);
}

void superTeddy::assignWind(float vx, float vy){
	windRunning_vx = vx;
	windRunning_vy = vy;
	//printf("Teddy wind %0.3f %0.3f\n",(double)windRunning_vx,(double)windRunning_vy);
}




