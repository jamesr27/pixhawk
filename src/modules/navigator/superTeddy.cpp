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
		timeNow = hrt_absolute_time();
			if((float)(timeNow - measureStartTime) >= (windMeasureTime)*1000000){
			// Pull data from the uORB.
			// Assign to class variables.
			wind_vx = windRunning_vx;
			wind_vy = windRunning_vy;
			printf("Teddy measured wind: %0.3f %0.3f\n",(double)wind_vx,(double)wind_vy);
			// Calculate the manouevre geometry for stage 4.
			manoeuvreGeometry();
			windManouevreEndTime = timeNow;
			return true;
		}
	}

	// State 3: Pre-delivery. This is a waypoint upwind. We move to this after we've measured the wind. After
	// this we then start the teddy manoeuvre.
	// The dist here may not work as we expect, but let's go with it for now.
	// No it really doesn't work as we want. We'll hack in a time delay for now.
	if (teddy_state == 3){
		timeNow = hrt_absolute_time();
		if (dist <= 30.f && dist >= 0.0f){
			teddyStartTime = hrt_absolute_time();
			return true;
		}
	}

	// State 4: Delivery. We need to have reached the bottom of the delivery sequence, and wait for a
	// command to ascend from the transmitter.
	// We'll set a flag once we are at the bottom, and check for this and a transmitter switch (channel 9)
	// before continuing.
	if (teddy_state == 4){
		if(deliveryManoeuvreCompleted == true && channel9 > 0.0f){
			//  Reset teddy start time so it can be used again during the ascent.
			teddyStartTime = hrt_absolute_time();
			return true;
		}
	}

	// State 5: The ascent
	if (teddy_state == 5){
		// Do nothing for now.
		if(missionCompleted == true){
			return true;
		}
	}



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

void superTeddy::getRadio(float input){
	channel9 = input;
}

// Give this function x and y in meters, and lat and lon of the teddy waypoint.
// It calculates an offet in decimal degrees, and returns it.
void superTeddy::latLonToMeterScaling(float (&xy)[2],float lat,float lon){

	float degToMeters0 = 111.32f * 1000.0f;	// 1 degree is this many meters at the equator (longitude)
											// In latitude, it is always this amount of meters...
	// Do latitude
	xy[0] = xy[0] / degToMeters0;
	// longitude
	xy[1] = xy[1] / (degToMeters0 * cosf(lat * 3.141592654f/180.0f));
}

void superTeddy::manoeuvreGeometry(){
	// Calculate the manouevre start point.
	man_xc = -wind_vx * startDeliveryOffset/descentRate; // North.
	man_yc = -wind_vy * startDeliveryOffset/descentRate; // East.

	// Gradients/deltas/whatever you want to call it.
	man_xm = -man_xc / (startDeliveryOffset/descentRate);
	man_ym = -man_yc / (startDeliveryOffset/descentRate);

	// Altitude data. This spans the entire manoeuvre.
	man_zc = tetherLength + startDeliveryOffset;
	man_zm = -descentRate;
}

// This function will use the start time and current time to calculate the current position setpoint.
// The inputs are the decimal coordinates of the current waypoint.
void superTeddy::updateManouevreSetPoint(float (&offset)[2], float lat, float lon){
	timeNow = hrt_absolute_time();
	float manTime = (float)(timeNow - teddyStartTime)/1000000.0f; //Time we are into the manoevure in seconds.
	float returnValue[]={0,0};	// [m], x and y for now.

	// Some hacking to the below equations has occurred. Mostly for testing pruposes.

	// We have 3 phases (currently).
	//	1. We don't move the setpoint for the first 20 seconds.
	//	2. We then descend along the path.
	//	3. We descend vertically to below the tether length by the offset specified.
	//
	// Remember that we are calculating an offset from the original waypoint in meters. We then need to transform this
	// to a value in decimal degrees. This gets added to the original waypoint coordinates later.

	// Hack in an effort to delay the whole manouevre by 20 seconds.
	manTime = manTime - 20.0f;
	// 1.
	if (manTime <= 20.0f){
		returnValue[0] = man_xm * (manTime) + 3*man_xc;
		returnValue[1] = man_ym * (manTime) + 3*man_yc;
		//returnValue[0] = man_xc;
		//returnValue[1] = man_yc;
		// Alt
		zsp = man_zc;
	}

	// 2.
	if (manTime > 20.0f && manTime <= (startDeliveryOffset/descentRate + 20.0f)){
		returnValue[0] = man_xm * (manTime - 20.0f) + man_xc;
		returnValue[1] = man_ym * (manTime - 20.0f) + man_yc;
		// Alt
		zsp = man_zc + man_zm * (manTime - 20.0f);
	}

	// 3. Stay in position until transmitter signal tells us to move on.
	if (manTime > (startDeliveryOffset/descentRate + 20.0f)){
		returnValue[0] = 0;
		returnValue[1] = 0;

		// Alt
		zsp = man_zc + man_zm * (manTime - 20.0f);
		if (zsp < (tetherLength - endDeliveryOffset)){
			zsp = tetherLength - endDeliveryOffset;
			deliveryManoeuvreCompleted = true;
		}
	}

	printf("Man: %0.3f %0.3f %0.3f ",(double)returnValue[0],(double)returnValue[1],(double)zsp);

	// Convert the values into degrees.
	float xyM[] = {0,0};
	xyM[0] = returnValue[0];
	xyM[1] = returnValue[1];

	latLonToMeterScaling(xyM,lat,lon);

	returnValue[0] = xyM[0];
	returnValue[1] = xyM[1];

	// Assign to offset?
	offset[0] = returnValue[0];
	offset[1] = returnValue[1];

	printf("%0.7f %0.7f\n",(double)returnValue[0],(double)returnValue[1]);
}

// This function updates the manouevre setpoint for the ascent. It is very the same as the previous function,
// except we now perform the manouevre in reverse.
void superTeddy::updateManouevreSetPointAscent(float (&offset)[2], float lat, float lon){
	timeNow = hrt_absolute_time();
	float manTime = (float)(timeNow - teddyStartTime)/1000000.0f; //Time we are into the manoevure in seconds.
	float returnValue[]={0,0};	// [m], x and y for now.

	//	We'll only have one phase here.
	//		Ascend along the wind speed path until we are back at teterlength + altitude offset.
	//		Once we have gone through this procedure we mark the entire waypoint as complete.

	//
	// Remember that we are calculating an offset from the original waypoint in meters. We then need to transform this
	// to a value in decimal degrees. This get added to the original waypoint coordinates later.

	// Some more hacking. Make it ascend quickly.

	// 1.
	if (manTime <= ((startDeliveryOffset + endDeliveryOffset)/descentRate)){
		returnValue[0] = man_xm * manTime;
		returnValue[1] = man_ym * manTime;
		// Alt
		//zsp = -man_zm * (manTime) + (tetherLength - endDeliveryOffset);
		zsp = tetherLength + endDeliveryOffset;
	}
	else
	{
		// We have finished the above, mark the mission as complete
		missionCompleted = true;
	}

	printf("Man: %0.3f %0.3f %0.3f ",(double)returnValue[0],(double)returnValue[1],(double)zsp);

	// Convert the values into degrees.
	float xyM[] = {0,0};
	xyM[0] = returnValue[0];
	xyM[1] = returnValue[1];

	latLonToMeterScaling(xyM,lat,lon);

	returnValue[0] = xyM[0];
	returnValue[1] = xyM[1];

	// Assign to offset?
	offset[0] = returnValue[0];
	offset[1] = returnValue[1];

	printf("%0.7f %0.7f\n",(double)returnValue[0],(double)returnValue[1]);
}



