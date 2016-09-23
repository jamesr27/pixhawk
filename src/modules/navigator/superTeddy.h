// Super Teddy mission/waypoint class.
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#include <navigator/navigation.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/wind_estimate.h>




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
//		PACKAGE_DELIVERED = 5,  (Ascend and then enter normal flight again. I'm just going to reverse the manoevure.
//		MISSION_COMPLETED = 6
//	} teddy_state;

	int teddy_state;

	// Manouevre parameters
	float tetherLength = 80.0f;	// [m] tether length
	float circleRadius = 35.0f;	// [m] teddy circle radius.
	float teddySpeed = 20.0f;	// [m/s] speed during delivery
	float descentRate = 2.0f;	// [m/s] descent rate during package delivery.
	float teddyG = 1.6f;		// [g] G to pull during the turn, not used at present.
	float startDeliveryOffset = 20.0f;	//[m] Altitude offset to start circling manouevre at. Roughly achieve 2 circles
										// before package gets close to ground.
	float endDeliveryOffset = 20.0f;	// [m] Amount we descend below the tether length during the delivery.

	// Manouevre geometry
	float man_xc = 0.0f;
	float man_yc = 0.0f;
	float man_xm = 0.0f;
	float man_ym = 0.0f;
	float man_zc = 0.0f;
	float man_zm = 0.0f;
	float zsp = 0.0f;
	uint64_t teddyStartTime = 0;
	uint64_t timeNow = 0;

	// Other stuff.
	float wind_vx = 0.0f;				// Wind direction in direction of flow. None of this weather definition crap.
	float wind_vy = 0.0f;
	float windMeasureTime = 45.0f;	// Estimate wind over 45 seconds.
	float windx = 0.0f;			// Position in meters away from the drop target that we want to sample the wind at.
	float windy = 0.0f;
	float windRunning_vx = 0;
	float windRunning_vy = 0;
	float windMeasureLoiterRadius = 50.0f; // [m].
	float windMeasurementAltitude = 100.0f;	// [m].
	float channel9 = -1.0f;	// The radio channel we are using to trigger certain events in superTeddy.

	float dist = -1.0f;	// [m]. Distance we are away from current waypoint.

	uint64_t measureStartTime = 0;	// Place holder for wind measurement start time.

	bool missionCompleted = false;	// We are going to trigger the mission completed on a transmitter switch for now.
	bool advanceTeddyState = false; // Keep track of whether we should advance the mission state.
	bool deliveryManoeuvreCompleted = false;	// Keep track of when we have finished the descent in the delivery phase.

	// Functions.
	bool isStateComplete();
	void advanceState(int& input);
	void assignWind(float vx, float vy);
	void getRadio(float input);
	void latLonToMeterScaling(float (&xy)[2],float lat,float lon);
	void manoeuvreGeometry();
	void updateManouevreSetPoint(float (&offset)[2],float lat, float lon);
	void updateManouevreSetPointAscent(float (&offset)[2],float lat, float lon);

private:

};


