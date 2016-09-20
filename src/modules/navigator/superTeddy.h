// Super Teddy mission/waypoint class.
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#include <navigator/navigation.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>

#include "navigator_mode.h"
#include "mission_block.h"


class superTeddyNav
{
public:
	enum teddy_state{
		UNINITIALISED = 0,
		WIND_MEASURING = 1,
		PREDELIVERY = 2,
		PACKAGE_DELIVERY = 3,
		PACKAGE_DELIVERED = 4,
		MISSION_COMPLETED = 5
	};

	float tetherLength = 60.0f;

	float wind_vx = 0.0f;				// Wind direction in direction of flow. None of this weather definition crap.
	float wind_vy = 0.0f;
	float windMeasureTime = 30.0f;	// Estimate wind over 30 seconds.
	float windx = 0.0f;			// Position in meters away from the drop target that we want to smaple the wind at.
	float windy = 0.0f;


	float descentRate = 2.0f;		// [m/s] descent rate during package delivery.
	bool missionCompleted = false;	// We are going to trigger the mission completed on a transmitter switch for now.

private:


};


