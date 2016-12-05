/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2015
 *
 ****************************************************************************/

/**
 * @file heli_command_model_main.cpp
 *
 * @brief Helicopter autonomous controller
 *
 * Implements a controller with rate control for collective and yaw, 
 * and with attitude command for the roll/pitch axes. For now we're treating
 * all of the axes as independent. This function will probably function as a passthrough, 
 * we don't need to integrate the automatic flight behavior in here just yet. 
 *
 * The structure of this application will be the same as the default pixhawk controllers
 */
#include "heli_command_model.h"

 namespace command_model {

#ifdef ERROR
# undef ERROR
#endif
 	static const int ERROR = -1;

 	HelicopterCommandModel *g_control;

 }

// Comment back in when you want to use it.
 //static orb_advert_t _mavlink_fd = 0;

//Constructor
 HelicopterCommandModel::HelicopterCommandModel() :
 _task_should_exit(false),
 _control_task(-1),


	/** Initialize state */ 
 _command_mode(ATT_COM),
 _alt_mode(ALT_OPEN),
 _coll_zero(0.0f),
 _param_counter(0),
 _rotor_speed(-1),
 _rotor_off(true),
 _isInitial(true),

	/* publications */
 _att_sp_pub(nullptr)

 {
 	po = new Poller();
 	memset(&_att_sp, 0, sizeof(_att_sp));

 	_roll_channel = new C_Signal[NUM_SIGNALS];
 	_pitch_channel = new C_Signal[NUM_SIGNALS];
 	_yaw_channel = new C_Signal[NUM_SIGNALS];
 	_collective_channel = new C_Signal[NUM_SIGNALS];



 	for(int i = 0; i < NUM_SIGNALS; i++) {
 		_roll_channel[i].zero();
 		_pitch_channel[i].zero();
 		_yaw_channel[i].zero();
 		_collective_channel[i].zero();
 	}

 	_velocity_prefilter = new C_Signal[2];
 	_stick_inputs = new C_Signal[NUM_CHANNELS];
 	for(int i = 0; i < NUM_CHANNELS; i++)
 	{
 		_stick_inputs[i].zero();
 	}

 	_all_outputs[0] = _roll_channel;
 	_all_outputs[1] = _pitch_channel;
 	_all_outputs[2] = _yaw_channel;
 	_all_outputs[3] = _collective_channel;

 	_params_handles.roll_gain 	= param_find("HELI_ROLL_GAIN");
 	_params_handles.roll_nf 	= param_find("HELI_ROLL_NF");
 	_params_handles.roll_dr 	= param_find("HELI_ROLL_DR");

 	_params_handles.roll_gainv	= param_find("HELI_ROLL_GAINV");
 	_params_handles.roll_nfv	= param_find("HELI_ROLL_NFV");
 	_params_handles.roll_drv 	= param_find("HELI_ROLL_DRV");
 	_params_handles.roll_wash	= param_find("HELI_ROLL_WASH");

 	_params_handles.pitch_gain 	= param_find("HELI_PITCH_GAIN");
 	_params_handles.pitch_nf 	= param_find("HELI_PITCH_NF");
 	_params_handles.pitch_dr 	= param_find("HELI_PITCH_DR");

 	_params_handles.pitch_gainv	= param_find("HELI_PITCH_GAINV");
 	_params_handles.pitch_nfv	= param_find("HELI_PITCH_NFV");
 	_params_handles.pitch_drv 	= param_find("HELI_PITCH_DRV");
 	_params_handles.pitch_wash	= param_find("HELI_PITCH_WASH");

 	_params_handles.yawr_gain	= param_find("HELI_YAWR_GAIN");
 	_params_handles.yawr_lag 	= param_find("HELI_YAWR_LAG");

 	_params_handles.collective_gain	= param_find("HELI_COLL_GAIN");
 	_params_handles.collective_lag 	= param_find("HELI_COLL_LAG");

 	_params_handles.rollv_clim  = param_find("HELI_ROLLV_CLIM");
 	_params_handles.pitchv_clim = param_find("HELI_PITCHV_CLIM");

 	_params_handles.rollx_clim  = param_find("HELI_ROLLX_CLIM");
 	_params_handles.pitchx_clim = param_find("HELI_PITCHX_CLIM");

 	_params_handles.rotor_rate =  param_find("HELI_ROTOR_RATE");


 	parameters_update(true);
 }

//Destructor
 HelicopterCommandModel::~HelicopterCommandModel()
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

 	command_model::g_control = nullptr;
 	delete po;

 }


//Parameter forced updates if system isn't listening
 int
 HelicopterCommandModel::parameters_update(bool force)
 {
 

 	if (force && _param_counter%50 == 0) {
		/* update C++ param system */

		/* Model command parameters here. Lag rate feed back for rates and a */
 		float v;

 		param_get(_params_handles.roll_gain, &v);
 		_params.cyclic_gain(0) = v;
 		param_get(_params_handles.pitch_gain, &v);
 		_params.cyclic_gain(1) = v;

 		param_get(_params_handles.roll_gainv, &v);
 		_params.cyclic_gainv(0) = v;
 		param_get(_params_handles.pitch_gainv, &v);
 		_params.cyclic_gainv(1) = v;

 		param_get(_params_handles.roll_nf, &v);
 		_params.cyclic_nf(0) = v;
 		param_get(_params_handles.pitch_nf, &v);
 		_params.cyclic_nf(1) = v;

 		param_get(_params_handles.roll_nfv, &v);
 		_params.cyclic_nfv(0) = v;
 		param_get(_params_handles.pitch_nfv, &v);
 		_params.cyclic_nfv(1) = v;

 		param_get(_params_handles.roll_dr, &v);
 		_params.cyclic_dr(0) = v;
 		param_get(_params_handles.pitch_dr, &v);
 		_params.cyclic_dr(1) = v;

 		param_get(_params_handles.roll_drv, &v);
 		_params.cyclic_drv(0) = v;
 		param_get(_params_handles.pitch_drv, &v);
 		_params.cyclic_drv(1) = v;

 		param_get(_params_handles.yawr_gain, &v);
 		_params.rate_gains(2) = v;
 		param_get(_params_handles.collective_gain, &v);
 		_params.rate_gains(3) = v;

 		param_get(_params_handles.yawr_lag, &v);
 		_params.rate_lags(2) = v;
 		param_get(_params_handles.collective_lag, &v);
 		_params.rate_lags(3) = v;	

 		param_get(_params_handles.roll_wash, &v);
 		_params.wash_lags(0) = v;
 		param_get(_params_handles.pitch_wash, &v);
 		_params.wash_lags(1) = v;


		/* Integrator limits parameters *******************************************************/

 		param_get(_params_handles.rollv_clim, &v);
 		_params.vel_climits(0) = v;

 		param_get(_params_handles.pitchv_clim, &v);
 		_params.vel_climits(1) = v;

 		param_get(_params_handles.rollx_clim, &v);
 		_params.pos_climits(0) = v;

 		param_get(_params_handles.pitchx_clim, &v);
 		_params.pos_climits(1) = v;

 		param_get(_params_handles.rotor_rate, &v);
 		_params.rotor_rate = v;

 		_param_counter = 0;

 	}

 	_param_counter++;
 	return OK;
 }

//Shim for calling task_main from task_create.
 void
 HelicopterCommandModel::task_main_trampoline(int argc, char *argv[])
 {
 	command_model::g_control->task_main();
 }


 void
 HelicopterCommandModel::rate_transition(float &input, float goal, float dt) { 

 	float distance = _params.rotor_rate * dt;
	
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
 HelicopterCommandModel::filter_att_vel(int axis, float dt, float K, float cross_term, float omega2)
 {	

 	if(_command_mode == ATT_COM) {

		_all_outputs[axis][1].bump_current(filter_signal(1.0f, cross_term, omega2, 0.0f, K*omega2, 0.0f, //Ks
			_stick_inputs[axis], _all_outputs[axis][1], dt));

	} else if(_command_mode == VEL_COM) {

	 	_velocity_prefilter[axis].bump_current(filter_signal(1.0f, cross_term, omega2, K*omega2, 0.0f, 0.0f, //Ks^2
	 		_stick_inputs[axis], _velocity_prefilter[axis], dt));
	 	_all_outputs[axis][1].bump_current(filter_signal(0.0f, 1.0f, 1.0f/_params.wash_lags(axis), 0.0f, 0.0f, 
	 		(1.0f/_params.wash_lags(axis))/CONSTANTS_ONE_G, _velocity_prefilter[axis], _all_outputs[axis][1], dt));

	 }

	//Accelerations
	_all_outputs[axis][0].bump_current(derivative_signal(1.0f, _all_outputs[axis][1], dt)); //We need to do something to smooth these derivatives. 

	//Attitudes
	_all_outputs[axis][2].bump_current(integrate_signal(1.0f, _all_outputs[axis][1], _all_outputs[axis][2], dt, 0.0f, false));

	if(axis == 1) {
		_all_outputs[axis][3].bump_current(integrate_signal(-CONSTANTS_ONE_G, _all_outputs[axis][2], _all_outputs[axis][3], dt, _params.vel_climits(axis), true));
	} else {
		_all_outputs[axis][3].bump_current(integrate_signal(CONSTANTS_ONE_G, _all_outputs[axis][2], _all_outputs[axis][3], dt, _params.vel_climits(axis), true));
	}
	
	_all_outputs[axis][4].bump_current(integrate_signal(1.0f, _all_outputs[axis][3], _all_outputs[axis][4], dt, _params.pos_climits(axis), true));
}

void 
HelicopterCommandModel::wrap_yaw()
{

	float wrap_offset = 0.0f;

	if((po->_v_att.yaw - _yaw_prev) > 1.5f * (float) M_PI) { //If the change is greater than 3pi/2
		wrap_offset = 2.0f * (float) M_PI;
		//printf(" Command Try wrap 1\n");

	} else if((po->_v_att.yaw - _yaw_prev) < -1.5f * (float) M_PI) {//If less than -3pi/2
		wrap_offset = -2.0f * (float) M_PI;
		//printf("Command Try wrap 2\n");
	}

	_all_outputs[2][2].modify_history(0, _all_outputs[2][2].current() + wrap_offset);
	_all_outputs[2][2].modify_history(1, _all_outputs[2][2].previous() + wrap_offset);
	_all_outputs[2][2].modify_history(2, _all_outputs[2][2].previous2() + wrap_offset);
//	printf("AO: %.3f\n",(double)_all_outputs[2][2].current());
	_yaw_prev = po->_v_att.yaw;
}

void
HelicopterCommandModel::rate_filter(int axis, float dt, float K, float a) 
{

	_all_outputs[axis][1].bump_current(filter_signal(0.0f, 1.0f, a, 0.0f, 0.0f, K*a, 
														_stick_inputs[axis], _all_outputs[axis][1], dt)); //Ka/(s+a) //Rate level
	
	_all_outputs[axis][0].bump_current(derivative_signal(1.0f, _all_outputs[axis][1], dt)); //Interesting. Better ways to compute numerical derivatives?

	_all_outputs[axis][2].bump_current(integrate_signal(1.0f, _all_outputs[axis][1], _all_outputs[axis][2], dt, 0.0f, false));

	if(axis == 2) {
		wrap_yaw();
	}

}

float 
HelicopterCommandModel::scaled_collective(float collective_input) 
{
	float collective_output = 0.0f;

	if(!po->_flight_mode.isAltitudeHold) { // If channel 5 less than 0 then open loop. 
		_alt_mode = ALT_OPEN; 
		_coll_zero = 0.0f;
		collective_output = collective_input;

	} else {

		if(_alt_mode == ALT_OPEN) 
		{	
			_coll_zero = po->_manual.z;
			_stick_inputs[3].zero();
			_all_outputs[3][0].zero();
			_all_outputs[3][1].zero();
			_all_outputs[3][2].set_all(-po->_local_pos.z);

		}
		_alt_mode = ALT_HOLD;
		
		float deadband = 0.15f;	

		if(fabsf(po->_manual.z - _coll_zero) < deadband) {
			_stick_inputs[3].zero();
			_all_outputs[3][0].zero();
			_all_outputs[3][1].zero();
			collective_output = 0.0f;

		} else if(po->_manual.z - _coll_zero > deadband) {
			collective_output = (po->_manual.z - _coll_zero - deadband)/(1.0f - _coll_zero - deadband);

		} else if(po->_manual.z - _coll_zero < -deadband) {
			collective_output = (po->_manual.z - _coll_zero + deadband)/(_coll_zero - deadband);
		}
	}
	return collective_output; 
}


float 
HelicopterCommandModel::rescale_command(float command, float scale) {

	float rescaled_command = command/scale;

	if(rescaled_command > 1.0f) {
		return 1.0f;
 	} else if(rescaled_command < -1.0f) {
 		return -1.0f;
 	} else return rescaled_command;
}

//Will change depending on the origin  of the signals
void
HelicopterCommandModel::initialize_signals() 
{

	if(!po->_flight_mode.isTracking) { //For now just stick inputs. Will change that soon.
		
		_stick_inputs[0].bump_current(po->_manual.y);
		_stick_inputs[1].bump_current(po->_manual.x);
		_stick_inputs[2].bump_current(po->_manual.r);
		_stick_inputs[3].bump_current(scaled_collective(po->_manual.z)); 
	
	} else {

		_stick_inputs[0].bump_current(rescale_command(po->_track.vy, fabsf(_params.cyclic_gainv(0))));
		_stick_inputs[1].bump_current(rescale_command(po->_track.vx, fabsf(_params.cyclic_gainv(1))));
		_stick_inputs[2].bump_current(rescale_command(po->_track.vr, fabsf(_params.rate_gains(2))));
		// The line below puts yaw rate control on. James turns it off.
		//_stick_inputs[2].bump_current(po->_manual.r);
		_stick_inputs[3].bump_current(rescale_command(po->_track.vz, fabsf(_params.rate_gains(3))));
	}
}

//All state switching will eventually be built here.
void
HelicopterCommandModel::command_model(float dt)
{
	//Still need to figure out behavior on a switch here. 

	initialize_signals(); //Depending on where the signal comes from this will change. Right now it's the sticks.

	if(po->_flight_mode.isAttitudeCommand) { //Attitude control, i.e. flies like the previous state of the machine. 
		
		if(_command_mode == VEL_COM) {
			
			_command_mode = ATT_COM;
			_stick_inputs[0].zero();
			_stick_inputs[1].zero();

			for(int i = 0; i < NUM_SIGNALS; i++){ 
				_all_outputs[0][i].zero();
				_all_outputs[1][i].zero();
			}
		}

		filter_att_vel(0, dt, _params.cyclic_gain(0),
						2.0f * _params.cyclic_nf(0) * _params.cyclic_dr(0), powf(_params.cyclic_nf(0), 2.0f));

		filter_att_vel(1, dt, _params.cyclic_gain(1), 
						2.0f * _params.cyclic_nf(1) * _params.cyclic_dr(1), powf(_params.cyclic_nf(1), 2.0f)); 

	} else { //Velocity command mode. 
		
		if(_command_mode == ATT_COM) {
			
			_command_mode = VEL_COM;
			_stick_inputs[0].zero();
			_stick_inputs[1].zero();

			_velocity_prefilter[0].zero();
			_velocity_prefilter[1].zero();

			
			for(int i = 0; i < NUM_SIGNALS; i++){ 
				_all_outputs[0][i].zero();
				_all_outputs[1][i].zero();
			}
		}

		filter_att_vel(0, dt, _params.cyclic_gainv(0), 
						2.0f * _params.cyclic_nfv(0) * _params.cyclic_drv(0), powf(_params.cyclic_nfv(0), 2.0f)); 

		filter_att_vel(1, dt, _params.cyclic_gainv(1), 
						2.0f * _params.cyclic_nfv(1) * _params.cyclic_drv(1), powf(_params.cyclic_nfv(1), 2.0f)); 
	}

	// What it this? James takes it out.
	if(_isInitial && po->_armed.armed) {
		_all_outputs[2][2].modify_history(0, po->_v_att.yaw);
		_all_outputs[2][2].modify_history(1, po->_v_att.yaw);
		_all_outputs[2][2].modify_history(2, po->_v_att.yaw);
		_isInitial = false;
	}

	if(!po->_armed.armed) {
		_isInitial = true;
	}

	rate_filter(2, dt, _params.rate_gains(2), 1.0f/_params.rate_lags(2)); 	
	rate_filter(3, dt, _params.rate_gains(3), 1.0f/_params.rate_lags(3));
}

//Main task handled here.  

void
HelicopterCommandModel::task_main() 
{
	//_mavlink_fd = px4_open(MAVLINK_LOG_DEVICE, 0);



	/* get an initial update for all sensor and status data */
	parameters_update(true);
	po->do_subscriptions();
	po->poll_all();

	_yaw_prev = po->_v_att.yaw;

	/* wakeup source */
	px4_pollfd_struct_t fds[1];
	fds[0].fd = po->_v_att_sub;
	fds[0].events = POLLIN;


	/*
	* Currently developing state switching before the command model for 
	* clean execution. 
	*/ 

	while (!_task_should_exit) {
		/* wait for up to 100ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

		//printf("Poll returned : %d\n", pret);
		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			continue;
		}
		/* this is undesirable but not much we can do */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		po->poll_all();
		parameters_update(true);

		static uint64_t last_run = 0;
		float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
		last_run = hrt_absolute_time();

		/* guard against too small (< 2ms) and too large (> 20ms) dt's */
		if (dt < 0.001f) {
			dt = 0.001f;

		} else if (dt > 0.02f) {
			dt = 0.02f;
		}

		// RL
		dt = 0.004;
		// Sim
		// dt = 0.02;

		//Run the command model based on the initialized state. 
		command_model(dt);

		//Publish the results in the right places. 
		assign_and_publish(dt);
	}

	return;
}

void
HelicopterCommandModel::assign_and_publish(float dt)
{	
	_att_sp.roll_ax = _all_outputs[0][0].current();
	_att_sp.roll_v = _all_outputs[0][1].current();
	_att_sp.roll_body = _all_outputs[0][2].current();
	_att_sp.v = _all_outputs[0][3].current();
	_att_sp.pos_y = _all_outputs[0][4].current();

	_att_sp.pitch_ax = _all_outputs[1][0].current();
	_att_sp.pitch_v = _all_outputs[1][1].current();
	_att_sp.pitch_body =_all_outputs[1][2].current();
	_att_sp.u = _all_outputs[1][3].current();
	_att_sp.pos_x = _all_outputs[1][4].current();

	_att_sp.yaw_ax = _all_outputs[2][0].current();
	_att_sp.yaw_v = _all_outputs[2][1].current();
	_att_sp.yaw_body = _all_outputs[2][2].current();

//	 printf("yawSP: %.3f\n", (double)_att_sp.yaw_body);

	_att_sp.coll_ax =  _all_outputs[3][0].current();
	_att_sp.coll_v =  _all_outputs[3][1].current();
	_att_sp.coll = _all_outputs[3][2].current();

	// Throttle:
	// 	We have 3 settings, off, idle , flight. We hard code them here.
	if (po->_flight_mode.rotorMode == heli_flight_mode_s::MAIN_STATE_GROUND)
	{	
		_rotor_speed = -1.0f;
		_rotor_off = true;

	} else if (po->_flight_mode.rotorMode == heli_flight_mode_s::MAIN_STATE_IDLE || po->_flight_mode.rotorMode == heli_flight_mode_s::MAIN_STATE_FLY)
	{
		if(_rotor_off) {
			_rotor_speed = -0.1f;
			_rotor_off = false;
		}
		rate_transition(_rotor_speed, po->_rc_chan.channels[8], dt);
	}
	
	_att_sp.head_speed = _rotor_speed;

	// Overwrite the previous throttle cases and just passes through the stick.
	// For 700 and testing uses.
	_att_sp.timestamp = hrt_absolute_time();

	if (_att_sp_pub != nullptr) orb_publish(ORB_ID(custom_attitude_setpoint), _att_sp_pub, &_att_sp);
	else _att_sp_pub = orb_advertise(ORB_ID(custom_attitude_setpoint), &_att_sp);

	return;
}


int
HelicopterCommandModel::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("heli_command_model",
		SCHED_DEFAULT,
		SCHED_PRIORITY_MAX - 5,
		1500,
		(px4_main_t)&HelicopterCommandModel::task_main_trampoline,
		nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int 
heli_command_model_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: heli_command_model {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (command_model::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		command_model::g_control = new HelicopterCommandModel;

		if (command_model::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != command_model::g_control->start()) {
			delete command_model::g_control;
			command_model::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (command_model::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete command_model::g_control;
		command_model::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (command_model::g_control) {
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



