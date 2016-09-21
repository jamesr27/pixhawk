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

	// State 1: Simply need to reach the dropzone, same as a default pixhawk waypoint... Try to copy that code.
	// We are only going to check position though.
	if (teddy_state == 1){

	}


	return false;
}

void superTeddy::advanceState(int& input){
	if(input <= 5 ){
		input++;
	}
	printf("Teddy advance to: %d\n",input);
}




