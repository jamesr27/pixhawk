// Super Teddy mission/waypoint class.
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#include <navigator/navigation.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>

//#include "navigator_mode.h"
//#include "mission_block.h"


class superTeddy
{
public:

	superTeddy();

	virtual ~superTeddy();

//	enum teddy_state_t{
//		UNINITIALISED = 0,
//		ENROUTE_TO_WIND = 1,
//		WIND_MEASURING = 2,
//		PREDELIVERY = 3,
//		PACKAGE_DELIVERY = 4,
//		PACKAGE_DELIVERED = 5,
//		MISSION_COMPLETED = 6
//	} teddy_state;

	int teddy_state;


	float tetherLength = 60.0f;

	float wind_vx = 0.0f;				// Wind direction in direction of flow. None of this weather definition crap.
	float wind_vy = 0.0f;
	float windMeasureTime = 30.0f;	// Estimate wind over 30 seconds.
	float windx = 0.0f;			// Position in meters away from the drop target that we want to smaple the wind at.
	float windy = 0.0f;
	float windMeasureLoiterRadius = 40.0f; // [m].
	float windMeasurementAltitude = 100.0f;	// [m].

	float dist = -1.0f;	// [m]. Distance we are away from current waypoint.

	uint64_t measurestartTime = 0;	// Place holder for wind measurement start time.


	float descentRate = 2.0f;		// [m/s] descent rate during package delivery.
	bool missionCompleted = false;	// We are going to trigger the mission completed on a transmitter switch for now.
	bool advanceTeddyState = false; // Keep track of whether we should advance the mission state.

	bool isStateComplete();
	void advanceState(int& input);

private:

};


