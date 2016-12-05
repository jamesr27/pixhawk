/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2015
 *
 ****************************************************************************/

/**
 * @file heli_att_control_main.h
 *
 * @brief Helicopter autonomous attitude controller
 *
 * Description of functions in attitude controller. 
 *
 * The structure of this application will be based off "Control Law Design and Development 
 * for an Autonomous Rotorcraft" by M. Takahashi. 
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/mc_att_ctrl_status.h>
#include <uORB/topics/custom_attitude_setpoint.h> 
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/test_values.h>
#include <uORB/topics/feedback.h>

// James changes to systemlib
#include <systemlib/mavlink_log.h>

#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <heli_lib/digital_filter/digital_filter.h>
#include <heli_lib/rotations/rotations.h>
#include <heli_lib/c_signal/c_signal.h>
#include <heli_lib/poller/poller.h>

/**
 * Helicopter attitude control app start / stop handling function
 *
 * @ingroup apps
 */

extern "C" __EXPORT int heli_att_control_main(int argc, char *argv[]);

#define NUM_CHANNELS 4

//Lots of switches here. 
#define ALT_OPEN 0
#define ALT_HOLD 1

#define ATT_COM 0
#define VEL_COM 1

class HelicopterAttitudeControl
{
public:
	/**
	 * Constructor
	 */
	HelicopterAttitudeControl();

	/**
	 * Destructor, also kills the main task
	 */
	~HelicopterAttitudeControl();

	/**
	 * Start the helicopter attitude control task.
	 *
	 * @return		OK on success.
	 */
	int			start();

		//Update parameter cache
	int			parameters_update();

	//Helper function 
	void 		feed_forward(float dt);

	//Write premixer signals.
	void 		assign_and_publish();
	//
	void 		switching_logic();


	void 		finalize_outputs(float dt);

	//Integrates the feedback if necessary
	void 		integrate_states(float dt);

	//Gets values from command model and reads sensors
	void 		get_state();

	//Wrapper for controlling everything
	void 		feed_back(float dt);
	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task.
	 */
	void		task_main();

private:

	Poller 	*po;

	bool	_task_should_exit;		/**< if true, task_main() should exit */
	int		_control_task;			/**< task handle */


	orb_advert_t	_actuators_0_pub;	/**< attitude actuator controls publication */
	orb_id_t 		_actuators_id;	/**< pointer to correct actuator controls0 uORB metadata structure */
	orb_advert_t 	_test_val_pub;
	orb_advert_t 	_feedback_pub;
	bool			_actuators_0_circuit_breaker_enabled;	/**< circuit breaker to suppress output */

	int _param_counter;
	float _coll_switch;

	bool _alt_mode;			/** < Used to initialize the collective when switchin states */
	bool _command_mode;		/** < Velocity vs attitude command. */

	math::Vector<4> 	_command_pos;		/** < Commanded position */
	math::Vector<4>		_command_vel;		/** < Commanded velocity */ 
	math::Vector<4> 	_command_att;		/** < Attitude from the command model */ 
	math::Vector<4> 	_command_rate;		/** < Commanded attitude rate */
	
	math::Vector<4>		_positions;			/** < The corresponding feedback for above (for heading and heave last 2 elements are 0) */
	math::Vector<4> 	_velocities;		
	math::Vector<4>		_attitude; 			
	math::Vector<4> 	_rates;				

	math::Vector<4>		_fforward;			/** < Feed forward */
	math::Vector<4>		_fb_err; 			/** < Feedback error */
	math::Vector<4>		_att_control;		/** < Attitude control vector */

	//We'll also track the feedback as signals in case we need to integrate anything. 

	C_Signal *	_att_signal; //Attitude signal
	C_Signal * 	_vel_signal; //Velocity signal
	C_Signal * 	_pos_signal; //Position signal
	C_Signal * 	_pre_notch;
	C_Signal * 	_post_notch;


	hrt_abstime 		stime;

	struct {
		
		param_t roll_p;
		param_t roll_d;
		param_t roll_i;
		param_t roll_pos;
		param_t roll_off;
		param_t rollv_lim;
		param_t rollx_lim;

		param_t pitch_p;
		param_t pitch_d;
		param_t pitch_i;
		param_t pitch_pos;
		param_t pitch_off;
		param_t pitchv_lim;
		param_t pitchx_lim;

		param_t yaw_p;
		param_t yaw_d;
		param_t yaw_off;

		param_t coll_p;
		param_t coll_d;
		param_t coll_off;

		param_t coll_scale;

		param_t roll_notch_wc;
		param_t roll_notch_w;
		param_t roll_notch_z;
		param_t roll_notch_wh;

		param_t pitch_notch_wc;
		param_t pitch_notch_w;
		param_t pitch_notch_z;
		param_t pitch_notch_wh;

		param_t tail_on;

		}		_params_handles;		/**< handles for interesting parameters */

	struct {

		math::Vector<4> rate_gains;				/**< P gain for angular error */
		math::Vector<4> att_gains;				/**< P gain for angular rate error */
		math::Vector<4> vel_gains;				/**< P gain for velocity error */
		math::Vector<4> pos_gains;			/**< P gain for position error */

		math::Vector<4> offsets;

		math::Vector<2> notch_wc;
		math::Vector<2> notch_w;
		math::Vector<2> notch_z;
		math::Vector<2> notch_wh;

		math::Vector<2> vel_limits;
		math::Vector<2> pos_limits;

		float coll_scale;		/**< For open loop collective */
		float tail_on;

	}		_params;

};
