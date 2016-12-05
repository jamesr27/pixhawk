/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2015
 *
 ****************************************************************************/

/**
 * @file heli_state_machine_main.h
 *
 * @brief Helicopter autonomous controller
 *
 * State machine for FTAP. 
 */


#include "heli_state_machine.h"

 namespace state_machine {

#ifdef ERROR
# undef ERROR
#endif
 	static const int ERROR = -1;

 	HelicopterStateMachine *g_control;

 }

 // Comment back in when you want to use it.
 //static orb_advert_t _mavlink_fd = 0;

//Constructor
 HelicopterStateMachine::HelicopterStateMachine() :
 _task_should_exit(false),
 _control_task(-1),
 _distance_sub(-1),
 _edison_sub(-1),
 _heli_state_sub(-1),
 _ground_commands_sub(-1),
 _arming_sub(-1),

	/* publications */
 _heli_state_pub(nullptr)

 {
 	memset(&_distance, 0, sizeof(_distance));
 	memset(&_edison, 0, sizeof(_edison));
 	memset(&_heli_state, 0, sizeof(_heli_state));
 	memset(&_ground_commands, 0, sizeof(_ground_commands));
 	memset(&_armed, 0, sizeof(_armed));

 	_params_handles.rc_control 		= 	param_find("STATE_RC_CONTROL");
 	_params_handles.cutoff_height	= 	param_find("STATE_LIDAR_CUT");

 	parameters_update();
 }

//Destructor
 HelicopterStateMachine::~HelicopterStateMachine()
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

 	state_machine::g_control = nullptr;
 }


//Parameter forced updates if system isn't listening
 int
 HelicopterStateMachine::parameters_update()
 {
 	
 	param_get(_params_handles.rc_control, &_params.rc_control);
 	param_get(_params_handles.cutoff_height, &_params.cutoff_height);

 	return OK;
 }

 void 
 HelicopterStateMachine::poll_all() {

 	bool updated;

	/* Check if parameters have changed */
 	orb_check(_heli_state_sub, &updated);
 	if (updated) {
 		orb_copy(ORB_ID(heli_state), _heli_state_sub, &_heli_state);
 	}

 	orb_check(_ground_commands_sub, &updated);
 	if (updated) {
 		orb_copy(ORB_ID(ground_commands), _ground_commands_sub, &_ground_commands);
 	}

 	orb_check(_distance_sub, &updated);	
 	if(updated) {
 		orb_copy(ORB_ID(distance_sensor), _distance_sub, &_distance);
 	}

 	orb_check(_arming_sub, &updated);	
 	if(updated) {
 		orb_copy(ORB_ID(actuator_armed), _arming_sub, &_armed);
 	}
 }

 void 
 HelicopterStateMachine::check_flight_state() 
 {
 	//We only get here if we're flying.
 	// So if we're in takeoff mode: 
 	if(_heli_state.flightState == heli_state_s::TAKEOFF) {

 		//Exit condition for takeoff here. (false placeholder)
 		if(false) {

 			if(_heli_state.windState == heli_state_s::LOW_WIND)
 			{
		 		//execute some path. 
 				_heli_state.flightState = heli_state_s::LOITER;

 			} else if(_heli_state.windState == heli_state_s::HIGH_WIND)
 			{
 				//Do some other path. For now we'll call it a hover
 				_heli_state.flightState = heli_state_s::HOVER;

 			}
 		} 

 	//or if we're in landing mode. 
 	} else if(_ground_commands.flightState == ground_commands_s::LAND) {

 		_heli_state.flightState = heli_state_s::LAND;
 		
 	//Otherwise we're just flying around. 
 	} else {

 		if(_heli_state.windState == heli_state_s::LOW_WIND)
 		{
			//execute some path. 
 			_heli_state.flightState = heli_state_s::LOITER;

 		} else if(_heli_state.windState == heli_state_s::HIGH_WIND)
 		{
			//Do some other path. For now we'll call it a hover
 			_heli_state.flightState = heli_state_s::HOVER;
 		}
 	}
 }


 void
 HelicopterStateMachine::check_wind_state() {

 	//If they don't match (operator decides that there's high wind). 
 	//Honestly not sure what to do with this information but we'll keep it for now.
 	if(_heli_state.windState != _ground_commands.windState)
 		_heli_state.windState = _ground_commands.windState;
 }

 void 
 HelicopterStateMachine::check_arm_status() {

 	//Note. Since this is only accessible when were on the ground, 
 	//It's actually impossible to do anything when we're in the air and armed. 

 	//If we're in standby
 	if(_heli_state.armingState == heli_state_s::STANDBY)
 	{
 		//Allow prearming.
 		if(_ground_commands.armingState == ground_commands_s::PREARMED) {
 			_heli_state.armingState = ground_commands_s::PREARMED;

 		//But not arming.
 		} else if(_ground_commands.armingState == ground_commands_s::ARMED)
 		{
 			warnx("Can't arm from standby. Prearm first.");
 			_heli_state.armingState = heli_state_s::STANDBY;

 		//Otherwise standby. 
 		} else {
 			_heli_state.armingState = heli_state_s::STANDBY;
 		}

 	//If we're in prearm	
 	} else if(_heli_state.armingState == heli_state_s::PREARMED) {	

 		//Allow arming. 
 		if(_ground_commands.armingState == ground_commands_s::ARMED) {
 			_heli_state.armingState = ground_commands_s::ARMED;

 		//Allow standby
 		} else if(_ground_commands.armingState == ground_commands_s::STANDBY)
 		{
 			_heli_state.armingState = heli_state_s::STANDBY;

 		//By default allow prearm.
 		} else {
 			_heli_state.armingState = heli_state_s::PREARMED;
 		}
 	}
 }


 int
 HelicopterStateMachine::check_ground_status()
 {
 	//If our sensor says we're on the ground. (This needs to be improved)
 	if(_distance.current_distance < _params.cutoff_height) 
 	{
 		//If we were flying and landing, and now we're low enough. 
 		if(_heli_state.isFlying == heli_state_s::IN_FLIGHT && _heli_state.flightState == heli_state_s::LAND) 
 		{
 			//Set system to landed completely. 
 			_heli_state.isFlying = heli_state_s::ON_GROUND;
 			_heli_state.flightState = heli_state_s::GROUNDED;
 			return heli_state_s::ON_GROUND;

 		//If we are on the ground, check to see if we've been commanded to take off. 
 		} else if(_heli_state.isFlying == heli_state_s::ON_GROUND && _heli_state.flightState == heli_state_s::GROUNDED) 
 		{	
 			//If we're armed and trying to takeoff, then do it. 
 			if(_ground_commands.flightState == ground_commands_s::TAKEOFF && _heli_state.armingState == heli_state_s::ARMED)
 			{
 				_heli_state.isFlying = heli_state_s::IN_FLIGHT;
 				_heli_state.flightState = heli_state_s::TAKEOFF; //Should this go here? Maybe not. 
 				return heli_state_s::IN_FLIGHT;

 			//But if we're not armed, then don't. 
 			} else if(_ground_commands.flightState == ground_commands_s::TAKEOFF && _heli_state.armingState != heli_state_s::ARMED) {
 				warnx("Can't take off: NOT ARMED.");
 				return heli_state_s::ON_GROUND;

	 		//Tried to do something that doesn't make sense. 
 			} else {

 				warnx("Tried to do something that's not allowed from the current state: GROUNDED.");
 				return heli_state_s::ON_GROUND;

 			}
 		}
 	} 
 	//Or we're in the air (above sensor 0)
 	else {

 		//If we're still grounded (due to sensor drift?)
 		if(_heli_state.isFlying == heli_state_s::ON_GROUND && _heli_state.flightState == heli_state_s::GROUNDED) 
 		{
 			warnx("Sensor drifted! Thinks we're in the air but you should be grounded!");
 			return heli_state_s::ON_GROUND;

 		//Otherwise we're just flying! We'll deal with other things later. 
 		} else if(_heli_state.isFlying == heli_state_s::IN_FLIGHT) {

 			return heli_state_s::IN_FLIGHT;
 		}
 	}

 	return -1;
 }

 void 
 HelicopterStateMachine::check_emergency_conditions() 
 {


 }


 void
 HelicopterStateMachine::run_state_machine()
 {	

 	int ground_status  = check_ground_status();

 	if(ground_status == heli_state_s::IN_FLIGHT) {

 		check_wind_state();
 		check_flight_state();

 	} else if(ground_status == heli_state_s::ON_GROUND){

 		check_arm_status();

 	} else {

 		warnx("Ground sensing failure! Hit impossible condition.");
 	}

 	check_emergency_conditions();

 }


//Main task handled here.  

 void
 HelicopterStateMachine::task_main() 
 {
 	//_mavlink_fd = px4_open(MAVLINK_LOG_DEVICE, 0);

	/* get an initial update for all sensor and status data */
 	parameters_update();

 	_distance_sub = orb_subscribe(ORB_ID(distance_sensor));
 	_edison_sub = orb_subscribe(ORB_ID(edison));
 	_heli_state_sub = orb_subscribe(ORB_ID(heli_state));
 	_ground_commands_sub = orb_subscribe(ORB_ID(ground_commands));
 	_arming_sub = orb_subscribe(ORB_ID(actuator_armed));

 	poll_all();

	/* wakeup source */
 	px4_pollfd_struct_t fds[1];
 	fds[0].fd = _edison_sub;
 	fds[0].events = POLLIN; 

 	int counter = 0;

 	while (!_task_should_exit) {
		/* wait for up to 100ms for data */
 		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

		/* timed out - periodic check for _task_should_exit */
 		if (pret == 0) {

 			//Do nothing

 		} else if (pret < 0) { /* this is undesirable but not much we can do */

 			warn("poll error %d, %d", pret, errno);

 		} else {

 			poll_all();
 			parameters_update();

 			static uint64_t last_run = 0;
 			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
 			last_run = hrt_absolute_time();

		/* guard against too small (< 2ms) and too large (> 20ms) dt's */
 			if (dt < 0.001f) {
 				dt = 0.001f;

 			} else if (dt > 0.02f) {
 				dt = 0.02f;
 			}

 			if(counter > 0) {

 				run_state_machine();	

 			} else {

 				_heli_state.time_usec = last_run;
 				_heli_state.rcControl = heli_state_s::PILOTED;

 				_heli_state.isFlying = heli_state_s::ON_GROUND;

 				_heli_state.flightState = heli_state_s::GROUNDED;
 				_heli_state.windState = heli_state_s::LOW_WIND;
 				_heli_state.armingState = heli_state_s::STANDBY;

 				_ground_commands.windState = ground_commands_s::LOW_WIND;
 				_ground_commands.flightState = ground_commands_s::GROUNDED;
 				_ground_commands.armingState = ground_commands_s::STANDBY;
				orb_advertise(ORB_ID(ground_commands), &_ground_commands); //Only do this once. 

 				counter++; //We start the state machine up as such. 
 			}

 			assign_and_publish();
 		}
 	}

 	return;
 }

 void
 HelicopterStateMachine::assign_and_publish()
 {
 	if (_heli_state_pub != nullptr) orb_publish(ORB_ID(heli_state), _heli_state_pub, &_heli_state);
 	else _heli_state_pub = orb_advertise(ORB_ID(heli_state), &_heli_state);
 	return;
 }

 int
 HelicopterStateMachine::start()
 {
 	ASSERT(_control_task == -1);

	/* start the task */
 	_control_task = px4_task_spawn_cmd("heli_state_machine",
 		SCHED_DEFAULT,
 		SCHED_PRIORITY_MAX,
 		1500,
 		(px4_main_t)&HelicopterStateMachine::task_main_trampoline,
 		nullptr);

 	if (_control_task < 0) {
 		warn("task start failed");
 		return -errno;
 	}

 	return OK;
 }

//Shim for calling task_main from task_create.
 void
 HelicopterStateMachine::task_main_trampoline(int argc, char *argv[])
 {
 	state_machine::g_control->task_main();
 }

 int 
 heli_state_machine_main(int argc, char *argv[])
 {
 	if (argc < 2) {
 		warnx("usage: heli_state_machine {start|stop|status}");
 		return 1;
 	}

 	if (!strcmp(argv[1], "start")) {

 		if (state_machine::g_control != nullptr) {
 			warnx("already running");
 			return 1;
 		}

 		state_machine::g_control = new HelicopterStateMachine;

 		if (state_machine::g_control == nullptr) {
 			warnx("alloc failed");
 			return 1;
 		}

 		if (OK != state_machine::g_control->start()) {
 			delete state_machine::g_control;
 			state_machine::g_control = nullptr;
 			warnx("start failed");
 			return 1;
 		}

 		return 0;
 	}

 	if (!strcmp(argv[1], "stop")) {
 		if (state_machine::g_control == nullptr) {
 			warnx("not running");
 			return 1;
 		}

 		delete state_machine::g_control;
 		state_machine::g_control = nullptr;
 		return 0;
 	}

 	if (!strcmp(argv[1], "status")) {
 		if (state_machine::g_control) {
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



