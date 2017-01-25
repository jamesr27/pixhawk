/****************************************************************************
 *
 *  Arjun Bhargava
 *  Otherlab 2015
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

#include "./heli_att_control.h"

namespace heli_att_control {

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
    #undef ERROR
#endif
    static const int ERROR = -1;

    HelicopterAttitudeControl   *g_control;
}

static orb_advert_t _mavlink_fd = 0;

HelicopterAttitudeControl::HelicopterAttitudeControl() :

_task_should_exit(false),
_control_task(-1),


/* publications */
_actuators_0_pub(nullptr),
_actuators_id(0),
_test_val_pub(nullptr),
_feedback_pub(nullptr),

/* Helpful counters and state of collective*/
_param_counter(0),
_coll_switch(0),
_alt_mode(ALT_OPEN),
_command_mode(ATT_COM)

{
    po = new Poller;

    stime = hrt_absolute_time();
    _actuators_id = ORB_ID(actuator_controls_0);

    _att_signal = new C_Signal[NUM_CHANNELS];
    _vel_signal = new C_Signal[NUM_CHANNELS];
    _pos_signal = new C_Signal[NUM_CHANNELS];

    // Only on cyclic for now.
    _pre_notch = new C_Signal[2];
    _post_notch = new C_Signal[2];

    _params_handles.roll_p          =   param_find("HELI_ROLL_KP");
    _params_handles.roll_d          =   param_find("HELI_ROLL_KD");
    _params_handles.roll_i          =   param_find("HELI_ROLL_KI");
    _params_handles.roll_pos        =   param_find("HELI_ROLL_KX");

    _params_handles.pitch_p         =   param_find("HELI_PITCH_KP");
    _params_handles.pitch_d         =   param_find("HELI_PITCH_KD");
    _params_handles.pitch_i         =   param_find("HELI_PITCH_KI");
    _params_handles.pitch_pos       =   param_find("HELI_PITCH_KX");

    _params_handles.yaw_p           =   param_find("HELI_YAW_KP");
    _params_handles.yaw_d           =   param_find("HELI_YAW_KD");

    _params_handles.coll_p          =   param_find("HELI_COLL_KP");
    _params_handles.coll_d          =   param_find("HELI_COLL_KD");

    _params_handles.roll_off        =   param_find("HELI_ROLL_OFF");
    _params_handles.pitch_off       =   param_find("HELI_PITCH_OFF");
    _params_handles.yaw_off         =   param_find("HELI_YAW_OFF");
    _params_handles.coll_off        =   param_find("HELI_COLL_OFF");

    _params_handles.coll_scale      =   param_find("HELI_COLL_SCALE");

    _params_handles.rollv_lim       =   param_find("HELI_ROLLV_LIM");
    _params_handles.pitchv_lim      =   param_find("HELI_PITCHV_LIM");

    _params_handles.rollx_lim       =   param_find("HELI_ROLLX_LIM");
    _params_handles.pitchx_lim      =   param_find("HELI_PITCHX_LIM");

    _params_handles.tail_on      =   param_find("HELI_TAIL_ON");

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
    delete po;

}


int
HelicopterAttitudeControl::parameters_update()
{
    if(_param_counter%500 == 0) {

        float v; //For general variables.

        param_get(_params_handles.roll_p, &v);
        if(v < 0.0f) {
            mavlink_log_critical(&_mavlink_fd, "Feed back gain less than 0!")
            v = 0.0f;
        }
        _params.att_gains(0) = v;

        param_get(_params_handles.roll_d, &v);
        if(v < 0.0f) {
            mavlink_log_critical(&_mavlink_fd, "Feed back gain less than 0!")
            v = 0.0f;
        }
        _params.rate_gains(0) = v;

        param_get(_params_handles.roll_i, &v);
        if(v < 0.0f) {
            mavlink_log_critical(&_mavlink_fd, "Feed back gain less than 0!")
            v = 0.0f;
        }
        _params.vel_gains(0) = v;

        param_get(_params_handles.roll_pos, &v);
        if(v < 0.0f) {
            mavlink_log_critical(&_mavlink_fd, "Feed back gain less than 0!")
            v = 0.0f;
        }
        _params.pos_gains(0) = v;

        /* Pitch gains *******************************************************/
        param_get(_params_handles.pitch_p, &v);
        if(v < 0.0f) {
            mavlink_log_critical(&_mavlink_fd, "Feed back gain less than 0!")
            v = 0.0f;
        }
        _params.att_gains(1) = v;

        param_get(_params_handles.pitch_d, &v);
        if(v < 0.0f) {
            mavlink_log_critical(&_mavlink_fd, "Feed back gain less than 0!")
            v = 0.0f;
        }
        _params.rate_gains(1) = v;

        param_get(_params_handles.pitch_i, &v);
        if(v < 0.0f) {
            mavlink_log_critical(&_mavlink_fd, "Feed back gain less than 0!")
            v = 0.0f;
        }
        _params.vel_gains(1) = -v;

        param_get(_params_handles.pitch_pos, &v);
        if(v < 0.0f) {
            mavlink_log_critical(&_mavlink_fd, "Feed back gain less than 0!")
            v = 0.0f;
        }
        _params.pos_gains(1) = v;

        /* Yaw gains *******************************************************/

        param_get(_params_handles.yaw_p, &v);
        if(v < 0.0f) {
            mavlink_log_critical(&_mavlink_fd, "Feed back gain less than 0!")
            v = 0.0f;
        }
        _params.att_gains(2) = v;

        param_get(_params_handles.yaw_d, &v);
        if(v < 0.0f) {
            mavlink_log_critical(&_mavlink_fd, "Feed back gain less than 0!")
            v = 0.0f;
        }
        _params.rate_gains(2) = v;

        /* Collective gains *******************************************************/

        param_get(_params_handles.coll_p, &v);
        if(v < 0.0f) {
            mavlink_log_critical(&_mavlink_fd, "Feed back gain less than 0!")
            v = 0.0f;
        }
        _params.att_gains(3) = v;

        param_get(_params_handles.coll_d, &v);
        if(v < 0.0f) {
            mavlink_log_critical(&_mavlink_fd, "Feed back gain less than 0!")
            v = 0.0f;
        }
        _params.rate_gains(3) = v;

        /* Offsets for leveling swashplate *************************************************************/

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

        /* Integrator limits parameters *******************************************************/


        param_get(_params_handles.rollv_lim, &v);
        _params.vel_limits(0) = v;

        param_get(_params_handles.pitchv_lim, &v);
        _params.vel_limits(1) = v;

        param_get(_params_handles.rollx_lim, &v);
        _params.pos_limits(0) = v;

        param_get(_params_handles.pitchx_lim, &v);
        _params.pos_limits(1) = v;

        /* Misc **********/


        param_get(_params_handles.tail_on, &v);
        _params.tail_on = v;
        
        _param_counter = 0;
    }

    _param_counter++;
    return OK;
}

void 
HelicopterAttitudeControl::feed_forward(float dt) 
{

    if(_command_mode == ATT_COM) { //Attitude control

        _fforward = _params.rate_gains.emult(_command_rate + _params.att_gains.emult(_command_att + _params.vel_gains.emult(_command_vel))); //PID attitude

    } else { //Velocity control

        _fforward = _params.rate_gains.emult(_command_rate + _params.att_gains.emult(_command_att + _params.vel_gains.emult(_command_vel + _params.pos_gains.emult(_command_pos)))); // Position feedback turned on (via integral). 
    }
}

void
HelicopterAttitudeControl::feed_back(float dt)
{

    if(_command_mode == ATT_COM) { //Integrated attitude

        _fb_err = _params.rate_gains.emult(_rates + _params.att_gains.emult(_attitude + _params.vel_gains.emult(_velocities))); //PID

    } else {    //Integrated velocity 

        _fb_err = _params.rate_gains.emult(_rates + _params.att_gains.emult(_attitude + _params.vel_gains.emult(_velocities+ _params.pos_gains.emult(_positions)))); //Position feedback is integrated 
    }

    po->_feedback.u = _velocities(1);
    po->_feedback.v = _velocities(0);

    po->_feedback.intx = _positions(1);
    po->_feedback.inty = _positions(0);
}

void
HelicopterAttitudeControl::finalize_outputs(float dt) {

    _att_control = -_fb_err + _fforward; //Adding back in the feedforward block and the inverse plant. 
    _att_control(3) = _att_control(3) + _coll_switch;


    // Manual passthrough on the collective. This is where we can just fix the collective position. 
    if(_alt_mode == ALT_OPEN) { 

        _att_control(3) = po->_manual.z * _params.coll_scale;
        _coll_switch = _att_control(3);
    }

    _att_control = _att_control + _params.offsets;

	 po->_test_val.value1 = _att_control(0);
	 po->_test_val.value2 = _att_control(1);
	 po->_test_val.value3 = _att_control(2);
	 po->_test_val.value4 = _att_control(3);
	// po->_test_val.value5 = _positions(2);	
}

void
HelicopterAttitudeControl::integrate_states(float dt)
{

    if(_command_mode == ATT_COM) {

        _vel_signal[0].bump_current(integrate_signal(1.0f, _att_signal[0], _vel_signal[0], dt, _params.vel_limits(0)/CONSTANTS_ONE_G, true));
        _vel_signal[1].bump_current(integrate_signal(-1.0f, _att_signal[1], _vel_signal[1], dt, _params.vel_limits(1)/CONSTANTS_ONE_G, true));

    } else { //VELOCITY COMMAND

        _pos_signal[0].bump_current(integrate_signal(1.0f, _vel_signal[0], _pos_signal[0], dt, _params.pos_limits(0)/CONSTANTS_ONE_G, true));
        _pos_signal[1].bump_current(integrate_signal(1.0f, _vel_signal[1], _pos_signal[1], dt, _params.pos_limits(1)/CONSTANTS_ONE_G, true));
    }

    for(int i = 0 ; i < 2; i++) {
        _velocities(i) = _vel_signal[i].current();
        _positions(i) = _pos_signal[i].current();
    }
}

void 
HelicopterAttitudeControl::switching_logic()
{
    _alt_mode = po->_flight_mode.isAltitudeHold ? ALT_HOLD: ALT_OPEN; // If less than 0, then open loop. 

    if(po->_flight_mode.isAttitudeCommand) { //Integrate the attitude for PID on attitude.

        if(_command_mode == VEL_COM) //If we're switching in, initialize the integrator
        {
            _command_mode = ATT_COM; 

            for(int i = 0; i < 2; i++) {
                _att_signal[i].set_all(_attitude(i));
                _vel_signal[i].set_all(0); //Initialize the integrated velocity to 0.
            }
        }

        _att_signal[0].bump_current(_attitude(0));
        _att_signal[1].bump_current(_attitude(1)); 

    } else { //Use velocity feedback. Also ask if we should have position feedback? 

        if(_command_mode == ATT_COM) 
        {
            _command_mode = VEL_COM;
            _vel_signal[0].set_all(_velocities(1));
            _vel_signal[1].set_all(_velocities(0)); //Set the initi
            _pos_signal[0].set_all(0); //Initialize the integrated position to 0. 
            _pos_signal[1].set_all(0); //Initialize the integrated position to 0. 

        }
		_vel_signal[0].bump_current(_velocities(1)); //THIS is the right order for the actual feedback. 
		_vel_signal[1].bump_current(_velocities(0));
	}

	// // If collective actuator is below 10%, integrators are set to zero.
	if(po->_rc_chan.channels[4] < 0.0f) {
		
        if(_command_mode == ATT_COM) {
            // Attitudes
    		// _att_signal[0].set_all(_attitude(0));
    		_vel_signal[0].set_all(0); //Initialize the integrated velocity to 0.
    		// _att_signal[1].set_all(_attitude(1));
    		_vel_signal[1].set_all(0); //Initialize the integrated velocity to 0.

        } else if (_command_mode == VEL_COM) {
    		// Velocities
    		// _vel_signal[0].set_all(_velocities(1));
    		// _vel_signal[1].set_all(_velocities(0)); //Set the init
    		_pos_signal[0].set_all(0); //Initialize the integrated position to 0.
    		_pos_signal[1].set_all(0); //Initialize the integrated position to 0.
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
    _rates(3) = -po->_local_pos.vz; ///THERES A MINUS SIGN HERE (Feeding back hdot)

    _attitude(0) = po->_v_att.roll;
    _attitude(1) = po->_v_att.pitch;
    _attitude(2) = po->_v_att.yaw;
    _attitude(3) = -po->_local_pos.z; ///THERE'S A MINUS SIGN HERE (Feeding back h)

    _velocities(0) = po->_local_pos.vx/CONSTANTS_ONE_G; //This is the correct order for kinematics ONLY
    _velocities(1) = po->_local_pos.vy/CONSTANTS_ONE_G; 
    _velocities(2) = po->_local_pos.vz/CONSTANTS_ONE_G;
    
    _velocities = euler_321_rotation4d(_attitude(0), _attitude(1), _attitude(2), _velocities); //From inertial to body

    switching_logic();

}

void
HelicopterAttitudeControl::task_main()
{


	//_mavlink_fd = px4_open(MAVLINK_LOG_DEVICE, 0);

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

			// RL
			//dt = 0.004;
			//Sim
			dt = 0.02;

			/* Check the system status and get the necessary data.*/
			get_state();

			/* Integrate states as needed */ 
			integrate_states(dt);
			
			/* Run the feed-forward block */
			feed_forward(dt);

			/* Stabilize, using the input from the previous block */ 
			feed_back(dt);

			/* Combine everything back together and do any final checks */ 
			finalize_outputs(dt);

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

	if(po->_armed.armed) {
		if(po->_ftap_safety.safety_flag == ftap_safety_s::SERVOS_ON || po->_ftap_safety.safety_flag == ftap_safety_s::MOTORS_ON) {
    		po->_actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
    		po->_actuators.control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
    		po->_actuators.control[3] = (PX4_ISFINITE(_att_control(3))) ? _att_control(3) : 0.0f;
		}


		if(po->_ftap_safety.safety_flag == ftap_safety_s::MOTORS_ON) {
			// po->_actuators.control[5] = PX4_ISFINITE(po->_v_att_sp.head_speed) ? po->_v_att_sp.head_speed : 0.0f;	//Throttle passthrough for now.
    		if(po->_manual.z > 0.02f) {// && po->_ftap_safety.safety_flag == ftap_safety_s::MOTORS_ON) {

    			po->_actuators.control[5] = PX4_ISFINITE(po->_v_att_sp.head_speed) ? po->_v_att_sp.head_speed : -1.0f;

    		} else { 

    			po->_actuators.control[5] = -1;
    		}
        
            if(po->_flight_mode.rotorMode == heli_flight_mode_s::MAIN_STATE_GROUND) {

                po->_actuators.control[2] = 1.0f;

            } else if((po->_flight_mode.rotorMode == heli_flight_mode_s::MAIN_STATE_IDLE || po->_flight_mode.rotorMode == heli_flight_mode_s::MAIN_STATE_FLY) && po->_actuators.control[5] > _params.tail_on) {
            
                po->_actuators.control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;

                if(po->_actuators.control[2] > 0.7f) { 
            
                    po->_actuators.control[2] = 0.7f;
                }
            }
        }
		
	} else {

		po->_actuators.control[2] = 1.0f;
	}

	po->_actuators.timestamp = hrt_absolute_time();
	po->_actuators.timestamp_sample = po->_v_att.timestamp;

	// Publish actuators.  
	if(po->_armed.armed || po->_armed.prearmed || po->_vehicle_status.hil_state == vehicle_status_s::HIL_STATE_ON) {
		if (_actuators_0_pub != nullptr) {
			orb_publish(_actuators_id, _actuators_0_pub, &po->_actuators);
		} else if (_actuators_id) {
			_actuators_0_pub = orb_advertise(_actuators_id, &po->_actuators);
		}
	}

	/***** For testing purposes ****/ 
	  if (_test_val_pub != nullptr) {
	  	orb_publish(ORB_ID(test_values), _test_val_pub, &po->_test_val);
	  } else {
	  	_test_val_pub =	orb_advertise(ORB_ID(test_values), &po->_test_val);
	  }
	if (_feedback_pub != nullptr) {
		orb_publish(ORB_ID(feedback), _feedback_pub, &po->_feedback);
	} else {
		_feedback_pub =	orb_advertise(ORB_ID(feedback), &po->_feedback);
	}
}


void
HelicopterAttitudeControl::task_main_trampoline(int argc, char *argv[]) {
    heli_att_control::g_control->task_main();
}

int
HelicopterAttitudeControl::start() {
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

int heli_att_control_main(int argc, char *argv[]) {
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


