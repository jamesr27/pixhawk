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
		PACKAGE_DELIVERY = 2,
		PACKAGE_DELIVERED = 3
	};

	float tetherLength = 60.0f;
	float windx = 0.0f;				// Wind direction in direction of flow. None of this weather definition crap.
	float windy = 0.0f;
	float windMeasureTime = 30.0f;	// Estimate wind over 30 seconds.
	float descentRate = 2.0f;		// [m/s] descent rate during package delivery.



};


