/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2015
 *
 ****************************************************************************/

/**
 * @file heli_path_planner.cpp
 *
 * @brief Helicopter path planner
 * 	
 * 	A path planner based on elevation angle and azimuth. 
 *  Maintains the helicopter nose towards the middle of the circle such that
 *  the tether never catches. 
 * 
 *
 */
#include "heli_path_planner.h"

 namespace path_planner {

#ifdef ERROR
# undef ERROR
#endif
 	static const int ERROR = -1;

 	HelicopterPathPlanner *g_control;

 }

//Constructor
 HelicopterPathPlanner::HelicopterPathPlanner() :
 _task_should_exit(false),
 _control_task(-1),
 _mavlink_fd(-1),
 _elevation_measured(0), 
 _elevation_target(0),
 _gamma_measured(0), 
 _gamma_target(0),
 _initial_x(0),
 _initial_y(0),
 _gammaM_prev(0),
 _wrap_offset(0),
 _param_counter(0),
 _gamma_direction(false),


	/* publications */
 _local_pos_sp_pub(nullptr), 
 _heli_mission_mode_pub(nullptr)

 {
 	po = new Poller();
 	memset(&_local_pos_sp, 0, sizeof(_local_pos_sp));
 	memset(&_mission_mode, 0, sizeof(_mission_mode));
 	_target.zero();
 	_target_position.zero();

 	_params_handles.gamma_target 		= param_find("PATH_GAMMA_T");
 	_params_handles.elevation_target 	= param_find("PATH_ELEV_T");
 	_params_handles.take_off_elevation 	= param_find("PATH_TO_ELEV");
 	_params_handles.take_off_lead 		= param_find("PATH_TO_LEAD");
 	_params_handles.rate_angle 			= param_find("PATH_RATE_ANGLE");
 	_params_handles.rate_pos			= param_find("PATH_RATE_POS");

	memset(&_flight_mode, 0, sizeof(_flight_mode));

 }

//Destructordo_subscriptions
 HelicopterPathPlanner::~HelicopterPathPlanner()
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

 	path_planner::g_control = nullptr;
 }

//Parameter forced updates if system isn't listening
 int
 HelicopterPathPlanner::parameters_update()
 {

 	if(_param_counter%50 == 0) {
	 	param_get(_params_handles.gamma_target, &_params.gamma_target);
	 	param_get(_params_handles.elevation_target, &_params.elevation_target);
	 	param_get(_params_handles.take_off_elevation, &_params.take_off_elevation);
	 	param_get(_params_handles.take_off_lead, &_params.take_off_lead);
	 	param_get(_params_handles.rate_angle, &_params.rate_angle);
	 	param_get(_params_handles.rate_pos, &_params.rate_pos);
	 	_param_counter = 0;
	 } 
	 
	_param_counter++;
 	return OK;
 }

 void
 HelicopterPathPlanner::rate_transition(float &input, float goal, float dt, bool isPos) { 

 	float distance = 0;
 	if(!isPos) { 
 		distance = _params.rate_angle * dt;
 	} else { 
 		distance = _params.rate_pos * dt;
 	}
	
	if(fabsf(goal - input) < distance) { 
		
		input = goal;

	} else { 

		if(goal - input >  0) { // Add to input

			input += distance;
		} else { 

			input -= distance;
		}
	}
 }

 void
 HelicopterPathPlanner::get_desired_position(float dt) {


 	float zR = -po->_local_pos.z - _target(2);
 	float xR = zR/(tanf(_elevation_target * (float) M_PI/180.0f));


 	_target_position(0)  = xR * cosf(_gamma_target) + _target(0);
 	_target_position(1) = xR * sinf(_gamma_target) + _target(1);

 	if(_mission_mode.missionMode == heli_mission_mode_s::FTAP_LAUNCH || _mission_mode.missionMode == heli_mission_mode_s::FTAP_ON_GROUND)
 	{
 		_local_pos_sp.x = _initial_x;
 		_local_pos_sp.y = _initial_y;

 	} else {

 		rate_transition(_local_pos_sp.x, _target_position(0), dt, true);
 		rate_transition(_local_pos_sp.y, _target_position(1), dt, true);
 	}

 	// Calculate wrap offset.
 	// We need to reset to 0 if not in flight mode.

	if((_gamma_measured - _gammaM_prev) > 1.5f * (float) M_PI) { //If the change is greater than 3pi/2
		_wrap_offset += -2.0f * (float) M_PI;

	} else if((_gamma_measured - _gammaM_prev) < -1.5f * (float) M_PI) {//If less than -3pi/2
		_wrap_offset += +2.0f * (float) M_PI;
	}

	// Reset to zero here....?
	if(!po->_flight_mode.isTracking) {
		_wrap_offset = 0.0f;
	//	printf("In reset wrap offset planner\n");
	}


	float gamma_measured_unwrapped = _gamma_measured + _wrap_offset;

	if ((_gamma_target - gamma_measured_unwrapped) < 0.1f)
	{
		if(_gamma_direction) {
			rate_transition(_local_pos_sp.yaw, _gamma_target - (float) M_PI, dt, false);
		} else {
			rate_transition(_local_pos_sp.yaw, _gamma_target + (float) M_PI, dt, false);
		}
	}
	else {
		if(_gamma_direction) {
			rate_transition(_local_pos_sp.yaw, gamma_measured_unwrapped - (float) M_PI, dt, false);
		} else {
			rate_transition(_local_pos_sp.yaw, gamma_measured_unwrapped + (float) M_PI, dt, false);

		}
	}

	_gammaM_prev = _gamma_measured;


	//printf("gt: %.3f gm: %.3f gtu: %.3f\n",(double)_gamma_target,(double)_gamma_measured,(double)gamma_measured_unwrapped);
 }

 void
 HelicopterPathPlanner::measure_data() {

	//Get the relative X, Y, Z( altitude)
 	math::Vector<3> relPos(po->_local_pos.x - _target(0), po->_local_pos.y - _target(1), -po->_local_pos.z - _target(2));
 	_gamma_measured = atan2f(relPos(1),relPos(0));

	//Rotate it into the plane 
 	math::Vector<2> planeRelPos(relPos(0),relPos(1));

 	math::Vector<2> pos1 = in_plane_rotation( _gamma_measured, planeRelPos);

	float x1 = pos1(0);  // x position in rotated coordinates.

	if (relPos(2) < 0) {
		relPos(2) = 0;
	}

	_elevation_measured = atan2f(relPos(2),x1);

	if(_elevation_measured < 0) {
		_elevation_measured = 0;

	} else if (_elevation_measured > (float) M_PI) {
		_elevation_measured = M_PI;
	}

}

void
HelicopterPathPlanner::mode_logic() { 

	float lead_z = -_params.take_off_lead * tanf(_params.take_off_elevation * (float) M_PI/180.0f); // Convert to rad
	float inPlaneOffset = (po->_local_pos.z - lead_z)/tanf(_params.take_off_elevation * (float) M_PI/180.0f);

	// Determine whether we are in 'takeoff' mode or normal flight mode.

	_mission_mode.missionMode = heli_mission_mode_s::FTAP_ON_GROUND;


	//if (-po->_local_pos.z  < 10.0f && po->_flight_mode.isMission && _missionModePrev != heli_mission_mode_s::FTAP_TO_AND_LAND) {
	if (-po->_local_pos.z  < 10.0f && po->_flight_mode.isMission) {

		//Launch phase & Recover phase
		_mission_mode.missionMode = heli_mission_mode_s::FTAP_LAUNCH;
		//printf("In launch phase...\n");

	} else  if (-po->_local_pos.z > -lead_z) {
		//We are in normal flight
		_mission_mode.missionMode = heli_mission_mode_s::FTAP_NORMAL_FLIGHT;

	} else if(-po->_local_pos.z <= -lead_z) {
		
		_mission_mode.missionMode = heli_mission_mode_s::FTAP_TO_AND_LAND;					//Target is below ground, and we are in takeoff mode.
	}



	/**
	* Now we know which missionMode we should be in, we can determine what
	* gammaTarget should be.
	* Logic:
	*  1. When we switch from 0 to 1 flight mode, lock gammaTarget based on
	*  current psi from state estimate. We then hold this until out of
	*  'takeoff' mode.
	*  2. If we have remained in flightMode 1 (i.e. we are flying)
	*      a. If in take-off mode gammaTarget is the previous value. This will
	*      either be the value set in case 1 above, or new value that is
	*      determined when transitioning from normal flight mode to takeoff
	*      mode.
	*      b. If in normal flight, we just follow the gammaInput from
	*      upstream.
	*/ 

	if (!po->_flight_mode.isMission) //We're on the ground...
	{

		_initial_x = po->_local_pos.x;
		_initial_y = po->_local_pos.y;

		if (po->_v_att.yaw <= 0){
			_gamma_target = po->_v_att.yaw + (float) M_PI;
			_gamma_direction = true;
		}
		else{
			_gamma_target = po->_v_att.yaw - (float) M_PI;
			_gamma_direction = false;
		}

		_mission_mode.missionMode = heli_mission_mode_s::FTAP_ON_GROUND;

	}

	if (po->_flight_mode.isMission) {

	    // If we have just entered flying lock the gammaTarget based on current yaw estimate.
		if(po->_flight_mode.isMission  && !_isMissionPrev)
		{
			_initial_x = po->_local_pos.x;
			_initial_y = po->_local_pos.y;
			if (po->_v_att.yaw <= 0){
				_gamma_target = po->_v_att.yaw + (float) M_PI;
				_gamma_direction = true;
			}
			else{
				_gamma_target = po->_v_att.yaw - (float) M_PI;
				_gamma_direction = false;
			}
			//printf("Mission initialise...\n");

		} 

		else if (po->_flight_mode.isMission && _isMissionPrev) //If we were already flying
		{ 

			if(_mission_mode.missionMode == heli_mission_mode_s::FTAP_TO_AND_LAND) {
				_gamma_target = _gamma_target;
				_elevation_target = _params.take_off_elevation;
				//printf("In takeoff mode\n");
//				usleep(100000);

			} 

			if(_mission_mode.missionMode == heli_mission_mode_s::FTAP_TO_AND_LAND && _missionModePrev == heli_mission_mode_s::FTAP_NORMAL_FLIGHT) 
			{
				_gamma_target = _gamma_target; // What do we want here? 
				_elevation_target = _params.take_off_elevation;
				//printf("Land mode\n");
//				usleep(10000);
			}

			if(_mission_mode.missionMode == heli_mission_mode_s::FTAP_NORMAL_FLIGHT) {
				_gamma_target = _params.gamma_target;
				_elevation_target = _params.elevation_target;
				//printf("Normal Flight\n");
//				usleep(10000);
			}
		}

//		printf("E: %.3f G: %.3f\n",(double)_elevation_target,(double)_gamma_target);
//		usleep(1000);

	}

	// Calculate the target

	math::Vector<3> offset;

	if(_gamma_direction) {
		offset(0) = inPlaneOffset*cosf(_gamma_target -(float) M_PI);
		offset(1) = inPlaneOffset*sinf(_gamma_target- (float) M_PI);
	} else {
		offset(0) = inPlaneOffset*cosf(_gamma_target + (float) M_PI);
		offset(0) = inPlaneOffset*sinf(_gamma_target + (float) M_PI);
	}

	offset(2) = lead_z;

	_target(0) = offset(0);
	_target(1) = offset(1);
	_target(2) = -po->_local_pos.z + offset(2);


	if(_mission_mode.missionMode == heli_mission_mode_s::FTAP_NORMAL_FLIGHT && po->_flight_mode.isMission) {
		_target(0) = 0;
		_target(1) = 0;
		_target(2) = 0;
	}

//	printf("Target: %.3f %.3f %.3f\n",(double)_target(0),(double)_target(1),(double)_target(2));
}


//Shim for calling task_main from task_create.
void
HelicopterPathPlanner::task_main_trampoline(int argc, char *argv[])
{
	path_planner::g_control->task_main();
}

//Main task handled here.  

void
HelicopterPathPlanner::task_main() 
{
	_mavlink_fd = px4_open(MAVLINK_LOG_DEVICE, 0);

	/* get an initial update for all sensor and status data */
	po->do_subscriptions();
	parameters_update();

	po->poll_all();

	/* wakeup source */
	px4_pollfd_struct_t fds[1];
	fds[0].fd = po->_v_att_sub;
	fds[0].events = POLLIN;

	measure_data();
	_gammaM_prev = _gamma_measured;


	usleep(1500000);
	/*
	* Currently developing state switching in the command model for 
	* clean execution. 
	*/ 

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
		// float dt = 0.02;

		_isMissionPrev = po->_flight_mode.isMission;

		po->poll_all();

		_missionModePrev = _mission_mode.missionMode;

		parameters_update();

		mode_logic();

		measure_data();

		get_desired_position(dt);

		//Publish the results in the right places. 
		if (_local_pos_sp_pub != nullptr) orb_publish(ORB_ID(vehicle_custom_position_setpoint), _local_pos_sp_pub, &_local_pos_sp);
		else _local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_custom_position_setpoint), &_local_pos_sp);

		if (_heli_mission_mode_pub != nullptr) orb_publish(ORB_ID(heli_mission_mode), _heli_mission_mode_pub, &_mission_mode);
		else _heli_mission_mode_pub = orb_advertise(ORB_ID(heli_mission_mode), &_mission_mode);

	}

	return;
}


int
HelicopterPathPlanner::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("heli_path_planner",
		SCHED_DEFAULT,
		SCHED_PRIORITY_MAX - 5,
		1500,
		(px4_main_t)&HelicopterPathPlanner::task_main_trampoline,
		nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int 
heli_path_planner_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: heli_path_planner {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (path_planner::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		path_planner::g_control = new HelicopterPathPlanner;

		if (path_planner::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != path_planner::g_control->start()) {
			delete path_planner::g_control;
			path_planner::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (path_planner::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete path_planner::g_control;
		path_planner::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (path_planner::g_control) {
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



