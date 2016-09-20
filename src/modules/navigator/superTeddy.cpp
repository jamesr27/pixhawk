// James Reeves. Super Teddy waypoint/mission helper class.
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <geo/geo.h>

#include <uORB/uORB.h>
#include <navigator/navigation.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vtol_vehicle_status.h>

#include "navigator.h"
//#include "superTeddy.h"

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


	return false;
}

void superTeddy::advanceState(int& input){

	if(input <= 5 ){
		input++;
	}

	printf("Teddy advance to: %d\n",input);

}




