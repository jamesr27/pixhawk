/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2015
 *
 ****************************************************************************/

/**
 * @file heli_att_control_main.cpp
 *
 * @brief Helicopter autonomous attitude controller
 *
 * Implements inner loop stabilization for FTAP platform. 
 *
 * The structure of this application will be based off "Control Law Design and Development 
 * for an Autonomous Rotorcraft" by M. Takahashi. 
 */

#include "heli_att_control.h"

 namespace heli_att_control
 {

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
 	static const int ERROR = -1;

 	HelicopterAttitudeControl	*g_control;
 }

 HelicopterAttitudeControl::HelicopterAttitudeControl() :

 _task_should_exit(false),
 _control_task(-1),
 _mavlink_fd(-1),

/* publications */
 _actuators_0_pub(nullptr),
 _actuators_id(0),
 _test_val_pub(nullptr),

/* Helpful counters and state of collective*/
 counter(0), 
 _coll_switch(0),
 _alt_mode(ALT_OPEN),
 _command_mode(ATT_COM),
 _pos_mode(INT_POS)


 {
 	po = new Poller;

 	stime = hrt_absolute_time();
 	_actuators_id = ORB_ID(actuator_controls_0);

 	_att_signal = new C_Signal[NUM_CHANNELS];
 	_vel_signal = new C_Signal[NUM_CHANNELS];
 	_pos_signal = new C_Signal[NUM_CHANNELS];
 	_intp_signal = new C_Signal[NUM_CHANNELS];

 	_params.yaw_cf = 0.0f;

 	_params_handles.roll_p			= 	param_find("HELI_ROLL_P");
 	_params_handles.roll_d 			= 	param_find("HELI_ROLL_D");
 	_params_handles.roll_i			= 	param_find("HELI_ROLL_I");
 	_params_handles.roll_pos		= 	param_find("HELI_ROLL_POS");
 	_params_handles.roll_lp			= 	param_find("HELI_ROLL_LP");
 	_params_handles.roll_cp 		= 	param_find("HELI_ROLL_CP");

 	_params_handles.pitch_p			= 	param_find("HELI_PITCH_P");
 	_params_handles.pitch_d 		= 	param_find("HELI_PITCH_D");
 	_params_handles.pitch_i			= 	param_find("HELI_PITCH_I");
 	_params_handles.pitch_pos		= 	param_find("HELI_PITCH_POS");
 	_params_handles.pitch_mq		= 	param_find("HELI_PITCH_MQ");
 	_params_handles.pitch_cp		= 	param_find("HELI_PITCH_CP");

 	_params_handles.yaw_p			=	param_find("HELI_YAW_P");
 	_params_handles.yaw_d 			= 	param_find("HELI_YAW_D");
 	_params_handles.yaw_nr 			= 	param_find("HELI_YAW_NR");
 	_params_handles.yaw_cp 			= 	param_find("HELI_YAW_CP");
 	_params_handles.yaw_cf			= 	param_find("HELI_YAW_CF");

 	_params_handles.coll_p 			= 	param_find("HELI_COLL_P");
 	_params_handles.coll_d 			= 	param_find("HELI_COLL_D");
 	_params_handles.coll_zw 		= 	param_find("HELI_COLL_ZW");
 	_params_handles.coll_cp			= 	param_find("HELI_COLL_CP");

 	_params_handles.coll_scale  	= 	param_find("HELI_COLL_SCALE");

 	_params_handles.roll_off 		= 	param_find("HELI_ROLL_OFF");
 	_params_handles.pitch_off 		= 	param_find("HELI_PITCH_OFF");
 	_params_handles.yaw_off 		= 	param_find("HELI_YAW_OFF");
 	_params_handles.coll_off 		= 	param_find("HELI_COLL_OFF");


	/* fetch initial parameter values */
 	parameters_update();
 }


 HelicopterAttitudeControl::~HelicopterAttitudeControl()
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

 	heli_att_control::g_control = nullptr;
 }


 int
 HelicopterAttitudeControl::parameters_update()
 {
 	float v;

	/* Roll gains *******************************************************/
 	param_get(_params_handles.roll_p, &v);
 	if(v < 0.0f) {
 		mavlink_log_critical(_mavlink_fd, "Feed back gain less than 0!")
 		v = 0.0f;
 	}
 	_params.att_p(0) = v;

 	param_get(_params_handles.roll_d, &v);
 	if(v < 0.0f) {
 		mavlink_log_critical(_mavlink_fd, "Feed back gain less than 0!")
 		v = 0.0f;
 	}
 	_params.att_d(0) = v;

 	param_get(_params_handles.roll_i, &v);
 	if(v < 0.0f) {
 		mavlink_log_critical(_mavlink_fd, "Feed back gain less than 0!")
 		v = 0.0f;
 	}
 	_params.att_i(0) = v;

 	param_get(_params_handles.roll_pos, &v);
 	if(v < 0.0f) {
 		mavlink_log_critical(_mavlink_fd, "Feed back gain less than 0!")
 		v = 0.0f;
 	}
 	_params.att_pos(0) = v;

 	param_get(_params_handles.roll_lp, &v);

 	if(v > 0.0f) {
 		mavlink_log_critical(_mavlink_fd, "Damping ratio positive!")
 		v = 0.0f;
 	}
 	_params.stab_damp(0) = v;

 	param_get(_params_handles.roll_cp, &v);

 	if(v < 1.0f) {
 		mavlink_log_critical(_mavlink_fd, "Control power too small")
 		v = 1000.0f;
 	}
 	_params.control_power(0) = v;

	/*******************************************************************/

	/* Pitch gains *******************************************************/
 	param_get(_params_handles.pitch_p, &v);
 	if(v < 0.0f) {
 		mavlink_log_critical(_mavlink_fd, "Feed back gain less than 0!")
 		v = 0.0f;
 	}
 	_params.att_p(1) = v;

 	param_get(_params_handles.pitch_d, &v);
 	if(v < 0.0f) {
 		mavlink_log_critical(_mavlink_fd, "Feed back gain less than 0!")
 		v = 0.0f;
 	}
 	_params.att_d(1) = v;

 	param_get(_params_handles.pitch_i, &v);
 	if(v < 0.0f) {
 		mavlink_log_critical(_mavlink_fd, "Feed back gain less than 0!")
 		v = 0.0f;
 	}
 	_params.att_i(1) = -v;

 	param_get(_params_handles.pitch_pos, &v);
 	if(v < 0.0f) {
 		mavlink_log_critical(_mavlink_fd, "Feed back gain less than 0!")
 		v = 0.0f;
 	}
 	_params.att_pos(1) = v;

 	param_get(_params_handles.pitch_mq, &v);
 	if(v > 0.0f) {
 		mavlink_log_critical(_mavlink_fd, "Damping ratio positive!")
 		v = 0.0f;
 	}
 	_params.stab_damp(1) = v;

 	param_get(_params_handles.pitch_cp, &v);
 	if(v < 1.0f) {
 		mavlink_log_critical(_mavlink_fd, "Control power too small")
 		v = 1000.0f;
 	}
 	_params.control_power(1) = v;

	/*******************************************************************/

	/* Yaw gains *******************************************************/
 	param_get(_params_handles.yaw_p, &v);
 	if(v < 0.0f) {
 		mavlink_log_critical(_mavlink_fd, "Feed back gain less than 0!")
 		v = 0.0f;
 	}
 	_params.att_p(2) = v;

 	param_get(_params_handles.yaw_d, &v);
 	if(v < 0.0f) {
 		mavlink_log_critical(_mavlink_fd, "Feed back gain less than 0!")
 		v = 0.0f;
 	}
 	_params.att_d(2) = v;

 	param_get(_params_handles.yaw_nr, &v);
 	if(v > 0.0f) {
 		mavlink_log_critical(_mavlink_fd, "Damping ratio positive!")
 		v = 0.0f;
 	}
 	_params.stab_damp(2) = v;

 	param_get(_params_handles.yaw_cp, &v);
 	if(v < 1.0f) {
 		mavlink_log_critical(_mavlink_fd, "Control power too small")
 		v = 1000.0f;
 	}
 	_params.control_power(2) = v;

 	param_get(_params_handles.yaw_cf, &_params.yaw_cf);

	/**************************************************************/

	/* Collective gains *******************************************************/

 	param_get(_params_handles.coll_p, &v);
 	if(v < 0.0f) {
 		mavlink_log_critical(_mavlink_fd, "Feed back gain less than 0!")
 		v = 0.0f;
 	}
 	_params.att_p(3) = v;

 	param_get(_params_handles.coll_d, &v);
 	if(v < 0.0f) {
 		mavlink_log_critical(_mavlink_fd, "Feed back gain less than 0!")
 		v = 0.0f;
 	}
 	_params.att_d(3) = v;

 	param_get(_params_handles.coll_zw, &v);
 	if(v > 0.0f) {
 		mavlink_log_critical(_mavlink_fd, "Damping ratio positive!")
 		v = 0.0f;
 	}
 	_params.stab_damp(3) = v;

 	param_get(_params_handles.coll_cp, &v);
 	if(v < 1.0f) {
 		mavlink_log_critical(_mavlink_fd, "Control power too small")
 		v = 1000.0f;
 	}
 	_params.control_power(3) = v;

	/**************************************************************/

 	param_get(_params_handles.roll_off, &v);
 	_params.offsets(0) = v;

 	param_get(_params_handles.pitch_off, &v);
 	_params.offsets(1) = v;

 	param_get(_params_handles.yaw_off, &v);
 	_params.offsets(2) = v;

 	param_get(_params_handles.coll_off, &v);
 	_params.offsets(3) = v;

 	param_get(_params_handles.coll_scale, &v);
 	_params.coll_scale = v;

 	return OK;
 }

 void 
 HelicopterAttitudeControl::inverse_plant(float dt)
 {
 	math::Vector<4> command_ax;
	command_ax(0) = po->_v_att_sp.roll_ax; //pdot
	command_ax(1) = po->_v_att_sp.pitch_ax; //qdot
	command_ax(2) = po->_v_att_sp.yaw_ax; //rdot
	command_ax(3) = po->_v_att_sp.coll_ax; // h double dot

	_inverse_p = command_ax - _command_rate.emult(_params.stab_damp);
	_inverse_p = _inverse_p.edivide(_params.control_power);

}

void 
HelicopterAttitudeControl::feed_forward(float dt) 
{

	if(_command_mode == ATT_COM) { //Attitude control
		
		_fforward = _params.att_d.emult(_command_rate + _params.att_p.emult(_command_att + _params.att_i.emult(_command_vel))); //PID attitude
	//	printf("Here. Ri: %.3f, Pi: %.3f\n", (double) _params.att_i(0), (double) _params.att_i(1));
	//	usleep(10000);
	} else { //Velocity control
		
		_fforward = _params.att_d.emult(_command_rate + _params.att_p.emult(_command_att + _params.att_i.emult(_command_vel 
				+ _params.att_pos.emult(_command_pos)))); // Position feedback turned on. 
	}

}

void
HelicopterAttitudeControl::feed_back(float dt)
{

	if(_command_mode == ATT_COM) { //Integrated attitude
		
		_fb_err = _params.att_d.emult(_rates + _params.att_p.emult(_attitude + _params.att_i.emult(_velocities))); //PID
	//	printf("Here. Ri: %.3f, Pi: %.3f\n", (double) _params.att_i(0), (double) _params.att_i(1));
	//	printf("Velocities: %.3f, %.3f\n", (double) _velocities(0), (double) _velocities(1));
	} else {	//Integrated position. 

		_fb_err = _params.att_d.emult(_rates + _params.att_p.emult(_attitude + 
					_params.att_i.emult(_velocities+ _params.att_pos.emult(_positions)))); //Position feedback is either integrated or not. 
	}
	
	if(fabsf(_command_att(2) - _attitude(2)) > (float) M_PI) //Wrap around
	{
		if (_attitude(2) < 0.0f)
		{ 
			_fb_err(2) = _params.att_d(2) * (_rates(2) + _params.att_p(2) * (_attitude(2) + 2.0f* (float) M_PI));
		} else {
			_fb_err(2) = _params.att_d(2) * (_rates(2) + _params.att_p(2) * (_attitude(2) - 2.0f* (float) M_PI));
		}
	}

	_att_control = -_fb_err + _fforward + _inverse_p + _params.offsets; //Adding back in the feedforward block and the inverse plant. 

	_att_control(3) = _att_control(3) + _coll_switch;

	if(_alt_mode == ALT_OPEN) { // Manual passthrough on the collective
		_att_control(3) = po->_manual.z * _params.coll_scale + _params.offsets(3);
		_coll_switch = _att_control(3);
	}
}

void
HelicopterAttitudeControl::integrate_states(float dt)
{

	if(_command_mode == ATT_COM) {

		_vel_signal[0].bump_current(integrate_signal(1.0f, _att_signal[0], _vel_signal[0], dt));
		_vel_signal[1].bump_current(integrate_signal(-1.0f, _att_signal[1], _vel_signal[1], dt));

	} else { //VELOCITY COMMAND

		if(_command_mode == INT_POS) {

			_pos_signal[0].bump_current(integrate_signal(1.0f, _vel_signal[0], _pos_signal[0], dt));
			_pos_signal[1].bump_current(integrate_signal(1.0f, _vel_signal[1], _pos_signal[1], dt));

		} else {

			_intp_signal[0].bump_current(integrate_signal(1.0f, _pos_signal[0], _intp_signal[0], dt));
			_intp_signal[1].bump_current(integrate_signal(1.0f, _pos_signal[1], _intp_signal[1], dt));
		}
	}

	for(int i = 0 ; i < 2; i++) {
		_velocities(i) = _vel_signal[i].current();
		_positions(i) = _pos_signal[i].current() + _intp_signal[i].current();
	}
}

void 
HelicopterAttitudeControl::switching_logic()
{
	_alt_mode = po->_rc_chan.channels[4] < 0.0f ? ALT_OPEN : ALT_HOLD; // If less than 0, then open loop. 

	if(po->_rc_chan.channels[5] > 0.0f) { //Integrate the attitude for PID on attitude.

		if(_command_mode == VEL_COM) //If we're switching in, initialize the integrator
		{
			_command_mode = ATT_COM; 

			for(int i = 0; i < 2; i++) {
				_att_signal[i].set_all(_attitude(i));
				_vel_signal[i].set_all(_velocities(i)); //In the body frame
			}
		}

		_att_signal[0].bump_current(_attitude(0));
		_att_signal[1].bump_current(_attitude(1)); 

	} else { //Use velocity feedback. Also ask if we should have position feedback? 

		if(_command_mode == ATT_COM) {
			_command_mode = VEL_COM;
		}

		if(po->_rc_chan.channels[7] > 0.0f)  //Integrate velocity
		{
			if(_command_mode == FB_POS) 
			{
				_command_mode = INT_POS;
				for(int i = 0; i < 2; i++) {
					_vel_signal[i].set_all(_velocities(i));
					_pos_signal[i].set_all(_positions(i)); //In the body frame
					_intp_signal[i].zero();
				}
			}

			_vel_signal[0].bump_current(_velocities(0));
			_vel_signal[1].bump_current(_velocities(1));

		} else {

			if(_command_mode == INT_POS) 
			{
				_command_mode = FB_POS;
				for(int i = 0; i < 2; i++) {
					_pos_signal[i].set_all(_positions(i));
					_intp_signal[i].zero(); //Uhhhh what should this be
				}
			}

			_pos_signal[0].bump_current(_positions(0));
			_pos_signal[1].bump_current(_positions(1));
		}
	}
}

void 
HelicopterAttitudeControl::get_state()
{
	po->poll_all();
	
	//Incoming from command model. Kinematics are performed upstream. 

	_command_rate(0) = po->_v_att_sp.roll_v; //P
	_command_rate(1) = po->_v_att_sp.pitch_v; //Q
	_command_rate(2) = po->_v_att_sp.yaw_v; //R
	_command_rate(3) = po->_v_att_sp.coll_v; //dh/dt
	
	_command_att(0) = po->_v_att_sp.roll_body;
	_command_att(1) = po->_v_att_sp.pitch_body;
	_command_att(2) = po->_v_att_sp.yaw_body;
	_command_att(3) = po->_v_att_sp.coll;

	_command_vel(0) = po->_v_att_sp.v/CONSTANTS_ONE_G;
	_command_vel(1) = po->_v_att_sp.u/CONSTANTS_ONE_G;

	_command_pos(0) = po->_v_att_sp.pos_y/CONSTANTS_ONE_G;
	_command_pos(1) = po->_v_att_sp.pos_x/CONSTANTS_ONE_G;

	// Current state of machine. We would do kinematics here. 
	
	_rates(0) = po->_v_att.rollspeed; // p
	_rates(1) = po->_v_att.pitchspeed; //q
	_rates(2) = po->_v_att.yawspeed;//r
	_rates(3) = -po->_local_pos.vz;	///THERES A MINUS SIGN HERE (Feeding back hdot)

	_attitude(0) = po->_v_att.roll;
	_attitude(1) = po->_v_att.pitch;
	_attitude(2) = po->_v_att.yaw;
	_attitude(3) = -po->_local_pos.z; ///THERE'S A MINUS SIGN HERE (Feeding back h)


	_velocities(0) = po->_local_pos.vy/CONSTANTS_ONE_G; 
	_velocities(1) = po->_local_pos.vx/CONSTANTS_ONE_G;

	_positions(0) = po->_local_pos.y/CONSTANTS_ONE_G;
	_positions(1) = po->_local_pos.x/CONSTANTS_ONE_G;

	_velocities = euler_321_rotation4d(_attitude(0), _attitude(1), _attitude(2), _velocities); //From inertial to body
	_positions = euler_321_rotation4d(_attitude(0), _attitude(1), _attitude(2), _positions);

	switching_logic();

	/****** For testing purposes */ 
	po->_test_val.value1 = _vel_signal[0].current();
	po->_test_val.value2 = _vel_signal[1].current();
	po->_test_val.value3 = _att_signal[0].current();
	po->_test_val.value4 = _att_signal[1].current();
	po->_test_val.value5 = _command_vel(0);
	po->_test_val.value6 = _command_vel(1);


}

void
HelicopterAttitudeControl::task_main()
{

	_mavlink_fd = px4_open(MAVLINK_LOG_DEVICE, 0);

	/* initialize parameters cache */
	parameters_update();
	po->do_subscriptions();

	/* wakeup source: vehicle attitude */
	px4_pollfd_struct_t fds[1];

	fds[0].fd = po->_v_att_sp_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) 
	{
		parameters_update();
		/* wait for up to 100ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(10000);
			continue;
		}

		/* run controller on attitude changes */
		if (fds[0].revents & POLLIN) {
			
			static uint64_t last_run = 0;
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too small (< 2ms) and too large (> 20ms) dt's */
			if (dt < 0.002f) {
				dt = 0.002f;

			} else if (dt > 0.02f) {
				dt = 0.02f;
			}

			/* Check the system status and get the necessary data.*/
			get_state();

			/* Integrate states as needed */ 
			integrate_states(dt);
			
			/* Calculate the inverse plant dynamics */ 
			inverse_plant(dt);

			/* Run the feed-forward block */
			feed_forward(dt);

			/* Stabilize, using the input from the previous block */ 
			feed_back(dt);

			/* Write to the premixer. */ 
			assign_and_publish();
		}
	}
	
	_control_task = -1;
	return;
}



void 
HelicopterAttitudeControl::assign_and_publish()
{

	po->_actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
	po->_actuators.control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
	po->_actuators.control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
	po->_actuators.control[3] = (PX4_ISFINITE(_att_control(3))) ? _att_control(3) : 0.0f;

	po->_actuators.control[6] = (PX4_ISFINITE(po->_rc_chan.channels[6])) ? po->_rc_chan.channels[6] : 0.0f;	//Throttle passthrough for now.

	po->_actuators.timestamp = hrt_absolute_time();
	po->_actuators.timestamp_sample = po->_v_att.timestamp;

	// publish actuators 
	//Changing some logic here, let's see if it works or not.
	if(po->_armed.armed || po->_armed.prearmed || po->_vehicle_status.hil_state == vehicle_status_s::HIL_STATE_ON) {
		if (_actuators_0_pub != nullptr) {
			orb_publish(_actuators_id, _actuators_0_pub, &po->_actuators);
		} else if (_actuators_id) {
			_actuators_0_pub = orb_advertise(_actuators_id, &po->_actuators);
		}
	}

	/***** For testing purposes ****/ 
	// if (_test_val_pub != nullptr) {
	// 	orb_publish(ORB_ID(test_values), _test_val_pub, &po->_test_val);
	// } else {
	// 	_test_val_pub =	orb_advertise(ORB_ID(test_values), &po->_test_val);
	// }
	
}


void
HelicopterAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	heli_att_control::g_control->task_main();
}

int
HelicopterAttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("heli_att_control",
		SCHED_DEFAULT,
		SCHED_PRIORITY_MAX - 5,
		1500,
		(px4_main_t)&HelicopterAttitudeControl::task_main_trampoline,
		nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int heli_att_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: heli_att_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (heli_att_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		heli_att_control::g_control = new HelicopterAttitudeControl;

		if (heli_att_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != heli_att_control::g_control->start()) {
			delete heli_att_control::g_control;
			heli_att_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (heli_att_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete heli_att_control::g_control;
		heli_att_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (heli_att_control::g_control) {
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


