/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2015
 *
 ****************************************************************************/

/**
 * @file heli_tracking_control_main.h
 *
 * @brief Helicopter autonomous controller
 *
 * Tracking control as described in Autonomous CLAWS paper. First steps in the outer loops
 * of the model following controller. 
 *
 * James modifies the yaw command.
 */


#include "heli_tracking_control.h"

 namespace tracking_control {

#ifdef ERROR
# undef ERROR
#endif
 	static const int ERROR = -1;

 	HelicopterTrackingControl *g_control;

 }

 // Comment back in when you want to use it.
 //static orb_advert_t _mavlink_fd = 0;

//Constructor
 HelicopterTrackingControl::HelicopterTrackingControl() :
 _task_should_exit(false),
 _isTracking(false),
 _control_task(-1),
 _param_counter(0),

	/* publications */
 _track_sp_pub(nullptr), 
 _test_val_pub(nullptr)

 {
 	po = new Poller;
 	memset(&_track_sp, 0, sizeof(_track_sp));

 	_path_inputs = new C_Signal[NUM_TRACKS];
 	_filtered_path = new C_Signal[NUM_TRACKS];

 	_pos_err_signal = new C_Signal[NUM_TRACKS];
 	_filtered_pos_err = new C_Signal[NUM_TRACKS];

 	_all_outputs = new C_Signal[NUM_TRACKS];

 	_v_total.zero();
 	_pos_err.zero();
 	_velocities.zero();

 	_params.path_gain.zero();
 	_params.path_nf.zero();
 	_params.path_dr.zero();

 	_params.pos_p.zero();
 	_params.pos_i.zero();

 	_params.v_gain.zero();

 	_params_handles.roll_gain 		= param_find("TRACK_ROLL_GAIN");
 	_params_handles.roll_nf 		= param_find("TRACK_ROLL_NF");
 	_params_handles.roll_dr 		= param_find("TRACK_ROLL_DR");
 	_params_handles.roll_p			= param_find("TRACK_ROLL_P");
 	_params_handles.roll_i 			= param_find("TRACK_ROLL_I");

 	_params_handles.pitch_gain 		= param_find("TRACK_PITCH_GAIN");
 	_params_handles.pitch_nf 		= param_find("TRACK_PITCH_NF");
 	_params_handles.pitch_dr 		= param_find("TRACK_PITCH_DR");
 	_params_handles.pitch_p			= param_find("TRACK_PITCH_P");
 	_params_handles.pitch_i 		= param_find("TRACK_PITCH_I");

 	_params_handles.yaw_gain 		= param_find("TRACK_YAW_GAIN");
 	_params_handles.yaw_nf 			= param_find("TRACK_YAW_NF");
 	_params_handles.yaw_dr 			= param_find("TRACK_YAW_DR");
 	_params_handles.yaw_p			= param_find("TRACK_YAW_P");
 	_params_handles.yaw_i 			= param_find("TRACK_YAW_I");

 	_params_handles.coll_gain 		= param_find("TRACK_COLL_GAIN");
 	_params_handles.coll_nf 		= param_find("TRACK_COLL_NF");
 	_params_handles.coll_dr 		= param_find("TRACK_COLL_DR");
 	_params_handles.coll_p			= param_find("TRACK_COLL_P");
 	_params_handles.coll_i 			= param_find("TRACK_COLL_I");

 	_params_handles.vroll_gain 		= param_find("TRACK_ROLL_V");
 	_params_handles.vpitch_gain 	= param_find("TRACK_PITCH_V");
 	_params_handles.vyaw_gain 		= param_find("TRACK_YAW_V");
 	_params_handles.vcoll_gain		= param_find("TRACK_COLL_V");

 	parameters_update(true);
 }

//Destructor
 HelicopterTrackingControl::~HelicopterTrackingControl()
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

 	tracking_control::g_control = nullptr;
 	delete po;
 }


//Parameter forced updates if system isn't listening
 int
 HelicopterTrackingControl::parameters_update(bool force)
 {
 
 	if (force && _param_counter%50 == 0) {
		/* update C++ param system */

		/* Model command parameters here. Lag rate feed back for rates and a */
 		float v;

 		param_get(_params_handles.roll_gain, &v);
 		_params.path_gain(0) = v;
 		param_get(_params_handles.roll_nf, &v);
 		_params.path_nf(0) = v;
 		param_get(_params_handles.roll_dr, &v);
 		_params.path_dr(0) = v;
 		param_get(_params_handles.roll_p, &v);
 		_params.pos_p(0)  = v;
 		param_get(_params_handles.roll_i, &v);
 		_params.pos_i(0)  = v;

 		param_get(_params_handles.pitch_gain, &v);
 		_params.path_gain(1) = v;
 		param_get(_params_handles.pitch_nf, &v);
 		_params.path_nf(1) = v;
 		param_get(_params_handles.pitch_dr, &v);
 		_params.path_dr(1) = v;
 		param_get(_params_handles.pitch_p, &v);
 		_params.pos_p(1)  = v;
 		param_get(_params_handles.pitch_i, &v);
 		_params.pos_i(1)  = v;

 		param_get(_params_handles.yaw_gain, &v);
 		_params.path_gain(2) = v;
 		param_get(_params_handles.yaw_nf, &v);
 		_params.path_nf(2) = v;
 		param_get(_params_handles.yaw_dr, &v);
 		_params.path_dr(2) = v;
 		param_get(_params_handles.yaw_p, &v);
 		_params.pos_p(2)  = v;
 		param_get(_params_handles.yaw_i, &v);
 		_params.pos_i(2)  = v;


 		param_get(_params_handles.coll_gain, &v);
 		_params.path_gain(3) = v;
 		param_get(_params_handles.coll_nf, &v);
 		_params.path_nf(3) = v;
 		param_get(_params_handles.coll_dr, &v);
 		_params.path_dr(3) = v;
 		param_get(_params_handles.coll_p, &v);
 		_params.pos_p(3)  = v;
 		param_get(_params_handles.coll_i, &v);
 		_params.pos_i(3)  = v;

 		param_get(_params_handles.vroll_gain, &v);
 		_params.v_gain(0) = v;
 		param_get(_params_handles.vpitch_gain, &v);
 		_params.v_gain(1) = v;
 		param_get(_params_handles.vyaw_gain, &v);
 		_params.v_gain(2) = v;
 		param_get(_params_handles.vcoll_gain, &v);
 		_params.v_gain(3) = v;
 		_param_counter = 0;

 	}
 	_param_counter++;

 	return OK;
 }


float
HelicopterTrackingControl::yawCommandPreCalc()
{
	// We receive a +- pi wrapped path input. Here we compare this with a wrapped yaw estimate,
	// and give out a + for yaw right, and -ve for yaw left command in radians. It will always try to go the shortest route
	// to the target yaw attitude. The easiest way to make sure this can never be wrong is to use vector algebra to calculate the command.
	// We essentially move the yaw error calculation infront of everything else. This is to remove winding up and down
	// yaw behaviour. We never want it to do this.

	// Create and normalise the yaw command vector
	math::Vector<3> yawCVector;
	yawCVector(0) = cosf(_path_inputs[2].current());
	yawCVector(1) = sinf(_path_inputs[2].current());
	yawCVector(2) = 0;
	yawCVector.normalize();

	// Create and normalise the yaw estimate vector
	math::Vector<3> yawEVector;
	yawEVector(0) = cosf(po->_v_att.yaw);
	yawEVector(1) = sinf(po->_v_att.yaw);
	yawEVector(2) = 0;
	yawEVector.normalize();

	// Take dot product of command with estimate.
	float dotProd = yawCVector(0)*yawEVector(0) + yawCVector(1)*yawEVector(1) + yawCVector(2)*yawEVector(2);
	// Calculate angle between the two vectors from the dot product.
	float dotAngle = acosf(dotProd);  // I assume that the two vectors are normalised prior to this.

	// Calculate the cross product. This is built into math.h.
	math::Vector<3> crossProdV = yawCVector%yawEVector;

	// Now if else check on the sense of the command output.
	float commandErrorOut = 0;
	if (crossProdV(2) >= 0)
	{
		commandErrorOut = - dotAngle;
	}
	else if (crossProdV(2) < 0)
	{
		commandErrorOut = dotAngle;
	}

	return commandErrorOut;
}

void 
HelicopterTrackingControl::position_prefilter(float dt)
{
	for(int i = 0; i < NUM_TRACKS; i++)  //All the channels thankfully look the same!
	{
		float K = _params.path_gain(i);
		float cross_term = 2.0f * _params.path_nf(i) * _params.path_dr(i);
		float omega2 = powf(_params.path_nf(i), 2.0f);

		//Check the form here but I'm pretty sure it's correct. Just Kwn^2 in numerator. 
		_filtered_path[i].bump_current(filter_signal(1.0f, cross_term, omega2, 0.0f, 0.0f, K*omega2, _path_inputs[i], _filtered_path[i], dt));
	}
}

//Computes the derivative of the input signal and then applies a gain.
void
HelicopterTrackingControl::velocity_feedback(float dt)
{
	math::Vector<4> measured_v;
	measured_v(0) = po->_local_pos.vy;
	measured_v(1) = po->_local_pos.vx;
	measured_v(2) = po->_v_att.yawspeed;
	measured_v(3) = -po->_local_pos.vz;

	math::Vector<4> command_v;

	for(int i = 0; i < NUM_TRACKS; i++) 
	{
		command_v(i) = derivative_signal(1, _filtered_path[i], dt); //1 is the scale on the derivative
	} 



	_v_total = _params.v_gain.emult(command_v - measured_v) + command_v;
}

//Takes in the estimated position and command. Applies PI control and returns combined signal. 
void
HelicopterTrackingControl::position_feedback(float dt)
{
	math::Vector<4> positions;
	positions(0) = po->_local_pos.y;
	positions(1) = po->_local_pos.x;
	positions(2) = po->_v_att.yaw;
	positions(3) = -po->_local_pos.z;

	for(int i = 0; i < NUM_TRACKS; i++) 
	{
		// For yaw the input track is already an "error". The rest leave as it was.
		if (i == 2)
		{
			_pos_err_signal[i].bump_current(_filtered_path[i].current());
			_filtered_pos_err[i].bump_current(filter_signal(0.0, 1.0f, 0.0f, 0.0f, _params.pos_p(i), _params.pos_i(i), _pos_err_signal[i], _filtered_pos_err[i], dt));
			_pos_err(i) = _filtered_pos_err[i].current();
		}
		else
		{
			_pos_err_signal[i].bump_current(_filtered_path[i].current() - positions(i));
			_filtered_pos_err[i].bump_current(filter_signal(0.0, 1.0f, 0.0f, 0.0f, _params.pos_p(i), _params.pos_i(i), _pos_err_signal[i], _filtered_pos_err[i], dt));
			_pos_err(i) = _filtered_pos_err[i].current();
		}
	}
}



//Will change depending on the origin  of the signals
void
HelicopterTrackingControl::get_path_signals() 
{
	//Get the inputs from the local position setpoint from the path planner. 
	if(po->_flight_mode.isTracking) {
		 _path_inputs[0].bump_current(po->_local_pos_sp.y);
		 _path_inputs[1].bump_current(po->_local_pos_sp.x);
		 _path_inputs[2].bump_current(yawCommandPreCalc());	// We get the yaw path input from this function now.
		 	 	 	 	 	 	 	 	 	 	 	 	 	// This is now an "error". So we change the position feedback bit on yaw too.
		 //_path_inputs[2].bump_current(po->_local_pos_sp.yaw);
		// _path_inputs[0].bump_current(0);
		// _path_inputs[1].bump_current(0);
		// _path_inputs[2].bump_current(0);
//		 printf("Pos SP: %.3f %.3f\n", (double) po->_local_pos_sp.y,(double) po->_local_pos_sp.x);
		_path_inputs[3].bump_current(0);
	} else {
		_path_inputs[0].bump_current(po->_local_pos.y);
		_path_inputs[1].bump_current(po->_local_pos.x);
		_path_inputs[2].bump_current(po->_v_att.yaw);
		_path_inputs[3].bump_current(0);
	}
}

void
HelicopterTrackingControl::tracking_control(float dt)
{

	position_prefilter(dt);

	velocity_feedback(dt);

	position_feedback(dt);
	
	// The final gains are in the command model.
	// 0 is vy and 1 is vx
	
	for(int i = 0; i < NUM_TRACKS; i++) {
		_all_outputs[i].bump_current(_v_total(i) + _pos_err(i));
	}
}

//Main task handled here.  

void
HelicopterTrackingControl::task_main() 
{
	//_mavlink_fd = px4_open(MAVLINK_LOG_DEVICE, 0);

	/* get an initial update for all sensor and status data */
	parameters_update(true);


	po->do_subscriptions();
	po->poll_all();

	/* wakeup source */
	px4_pollfd_struct_t fds[1];
	fds[0].fd = po->_v_att_sub;
	fds[0].events = POLLIN;

	_yaw_prev = po->_v_att.yaw;

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
		//dt = 0.004f;
		// Sim
		dt = 0.02;


		//Depending on where the signal comes from this will change.
		get_path_signals();

		//Run the command model based on the initialized state.
		tracking_control(dt);

		//Publish the results in the right places.
		assign_and_publish();

	}

	return;
}

void
HelicopterTrackingControl::assign_and_publish()
{

	_velocities(1) = _all_outputs[0].current(); //Integrates up the VY inertial
	_velocities(0) = _all_outputs[1].current(); //Integrates up the VX inertial
	_velocities(2) = 0;
	_velocities(3) = 0;
	


	_track_sp.p1 = _velocities(1);
	_track_sp.p2 = _velocities(0);
	_track_sp.p3 = _pos_err(1);
	_track_sp.p4 = _pos_err(0);

	_velocities = euler_321_rotation4d(0, 0, po->_v_att.yaw, _velocities); //From inertial to body

	_track_sp.vy = _velocities(1); //v
	_track_sp.vx = -_velocities(0); //u   //James, puts the minus here.
	_track_sp.vr = _all_outputs[2].current();
	_track_sp.vz = _all_outputs[3].current();



	if (_track_sp_pub != nullptr) orb_publish(ORB_ID(track_setpoint), _track_sp_pub, &_track_sp);
	else _track_sp_pub = orb_advertise(ORB_ID(track_setpoint), &_track_sp);

	return;
}

int
HelicopterTrackingControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("heli_tracking_control",
		SCHED_DEFAULT,
		SCHED_PRIORITY_MAX - 5,
		1500,
		(px4_main_t)&HelicopterTrackingControl::task_main_trampoline,
		nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

//Shim for calling task_main from task_create.
void
HelicopterTrackingControl::task_main_trampoline(int argc, char *argv[])
{
	tracking_control::g_control->task_main();
}

int 
heli_tracking_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: heli_tracking_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (tracking_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		tracking_control::g_control = new HelicopterTrackingControl;

		if (tracking_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != tracking_control::g_control->start()) {
			delete tracking_control::g_control;
			tracking_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (tracking_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete tracking_control::g_control;
		tracking_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (tracking_control::g_control) {
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



