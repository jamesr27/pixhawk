/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2015
 *
 ****************************************************************************/

/**
 * @file heli_flight_mode.cpp
 *
 * @brief Helicopter autonomous controller flight mode state tracker
 *
 * Just a very simple daemon to keep track of which switch is where 
 * so that I don't have to tie the control system to the rc channels necessarily. 
 * Will be necessary later. 
 */
#include "heli_flight_mode_control.h"

namespace flight_mode {

#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

HelicopterFlightModeControl *g_control;

}


// Comment back in when you want to use it.
//static orb_advert_t _mavlink_fd = 0;

//Constructor
HelicopterFlightModeControl::HelicopterFlightModeControl() :
	_task_should_exit(false),
	_control_task(-1),

	/* publications */
	_heli_flight_mode_pub(nullptr)
{
	po = new Poller();
	memset(&_flight_mode, 0, sizeof(_flight_mode));
}

//Destructor
HelicopterFlightModeControl::~HelicopterFlightModeControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	flight_mode::g_control = nullptr;
}

//Shim for calling task_main from task_create.
 void
 HelicopterFlightModeControl::task_main_trampoline(int argc, char *argv[])
 {
 	flight_mode::g_control->task_main();
 }

//ALl state switching will eventually be built here. 
void
HelicopterFlightModeControl::flight_mode(float dt)
{
	//Still need to figure out behaviour on a switch here.
	// James gets rid of altitude hold switching. We don't need it.
	// Tracking moved to channel 5
	// Mission on 8
	// Throttle on 9

	// Additionally, mission mode governs whether we are tracking or not.
	// This means only one switch is needed, and allows us to toggle whether we are tracking
	// We do this by subscribing to heli_mission_mode orb.



	_flight_mode.isAltitudeHold = false;
//	if(po->_rc_chan.channels[4] < 0.0f) {
//		_flight_mode.isAltitudeHold = false;
//	} else _flight_mode.isAltitudeHold = true;

	if(po->_rc_chan.channels[7] > 0.0f) {
	 	_flight_mode.isMission = true;

	 	if(po->_heli_mission_mode.missionMode == heli_mission_mode_s::FTAP_LAUNCH){
	 		_flight_mode.isTracking = false;
	 	} else {
	 		_flight_mode.isTracking = true;
	 	}
	} else {
		_flight_mode.isMission = false;
		_flight_mode.isTracking = false;
	}

	if(po->_rc_chan.channels[5] < 0.0f) {
		_flight_mode.isVelocityCommand = true;
		_flight_mode.isAttitudeCommand = false;
	} else {
		_flight_mode.isVelocityCommand = false;
		_flight_mode.isAttitudeCommand = true;
		_flight_mode.isTracking = false;
		_flight_mode.isMission = false;
	} 
	
	//Throttle controls. Actual speed set in command model. 
	if (po->_rc_chan.channels[8] < -0.3f)
	{
		_flight_mode.rotorMode = heli_flight_mode_s::MAIN_STATE_GROUND;
	}
	if (po->_rc_chan.channels[8] >= -0.3f && po->_rc_chan.channels[8] <= 0.3f)
	{
		_flight_mode.rotorMode = heli_flight_mode_s::MAIN_STATE_IDLE;
	}
	if (po->_rc_chan.channels[8] > 0.3f)
	{
		_flight_mode.rotorMode = heli_flight_mode_s::MAIN_STATE_FLY;
	}
}

//Main task handled here.  

void
HelicopterFlightModeControl::task_main() 
{
	//_mavlink_fd = px4_open(MAVLINK_LOG_DEVICE, 0);

	/* get an initial update for all sensor and status data */
	po->do_subscriptions();
	po->poll_all();

	/* wakeup source */
	px4_pollfd_struct_t fds[1];
	fds[0].fd = po->_v_att_sub;
	fds[0].events = POLLIN;

	/*
	* Currently developing state switching in the command model for 
	* clean execution. 
	*/ 

	//Why?
	// usleep(1000000);

	while (!_task_should_exit) {

		/* wait for up to 100ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			continue;
		}
		/* this is undesirable but not much we can do */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		// RL
		float dt = 0.004;
		// Sim
		// float dt = 0.02f;

		//Run the command model based on the initialized state. 
		po->poll_all();
		_flight_mode = po->_flight_mode; //So we don't accidentally overwrite fields from elsewhere. 

		flight_mode(dt);

		//Publish the results in the right places. 

		if (_heli_flight_mode_pub != nullptr) orb_publish(ORB_ID(heli_flight_mode), _heli_flight_mode_pub, &_flight_mode);
		else _heli_flight_mode_pub = orb_advertise(ORB_ID(heli_flight_mode), &_flight_mode);
	}

	return;
}


int
HelicopterFlightModeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("heli_flight_mode",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       1500,
				       (px4_main_t)&HelicopterFlightModeControl::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int 
heli_flight_mode_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: heli_flight_mode_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (flight_mode::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		flight_mode::g_control = new HelicopterFlightModeControl;

		if (flight_mode::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != flight_mode::g_control->start()) {
			delete flight_mode::g_control;
			flight_mode::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (flight_mode::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete flight_mode::g_control;
		flight_mode::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (flight_mode::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}



