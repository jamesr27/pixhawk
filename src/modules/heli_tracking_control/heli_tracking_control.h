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
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <functional>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <systemlib/mavlink_log.h>
#include <platforms/px4_defines.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/track_setpoint.h>

#include <heli_lib/rotations/rotations.h>
#include <heli_lib/digital_filter/digital_filter.h>
#include <heli_lib/c_signal/c_signal.h>
#include <heli_lib/poller/poller.h>

extern "C" __EXPORT int heli_tracking_control_main(int argc, char *argv[]);

#define NUM_TRACKS 4

class HelicopterTrackingControl
{
public:
	
	/**
	 * Constructor
	 */
	
	HelicopterTrackingControl();

	/**
	 * Destructor, also kills task.
	 */
	
	~HelicopterTrackingControl();
	
	/**
	 * Filter the incoming path signals
	 */
	
	void		position_prefilter(float dt);

	/**
	 * Position feedback control
	*/

	void 		wrap_yaw();


	void 		position_feedback(float dt);

	/**
	 * Velocity feedback control
	*/

	void 		velocity_feedback(float dt);

	/**
	 * Update our local parameter cache.
	 */
	int			parameters_update(bool force);

	/**
	* Get the signals and put them in the right spots
	*/

	void 		get_path_signals();
	/** 
	* Switch states for the command model here.
	*/

	void 		tracking_control(float dt);

	/** 
	* For readability
	*/

	void 		assign_and_publish();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();
	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	//Tracks all the signals for the command model

	Poller *po;

	bool	_task_should_exit;		/**< if true, task should exit */
	bool 	_isTracking;
	int 	_control_task;
	int		_mavlink_fd;			/**< mavlink fd */
	float 	_wrap_offset;
	int 	_param_counter;

	math::Vector<4> _v_total;
	math::Vector<4> _pos_err;
	math::Vector<4> _velocities;

	track_setpoint_s _track_sp;
	orb_advert_t _track_sp_pub;

	orb_advert_t 	_test_val_pub;

	C_Signal* _path_inputs;
	C_Signal* _filtered_path;

	C_Signal* _pos_err_signal;
	C_Signal* _filtered_pos_err;

	C_Signal* _all_outputs;

	float _yaw_prev;

	struct {
		param_t roll_gain; 			/**< Roll gain */
		param_t roll_nf;			/**< Roll natural frequency */ 
		param_t roll_dr;			/**< Roll damping ratio */

		param_t roll_p;
		param_t roll_i;

		param_t pitch_gain;
		param_t pitch_nf;
		param_t pitch_dr;
		param_t pitch_p;
		param_t pitch_i;

		param_t yaw_gain;
		param_t yaw_nf;
		param_t yaw_dr;	
		param_t yaw_p;
		param_t yaw_i;	

		param_t coll_gain;
		param_t coll_nf;
		param_t coll_dr;
		param_t coll_p;
		param_t coll_i;

		param_t vroll_gain;
		param_t vpitch_gain;
		param_t vyaw_gain;
		param_t vcoll_gain;


	}		_params_handles;		/**< handles for interesting parameters */

	struct {

		math::Vector<4> path_gain; 	/**< Path prefilter gains */
		math::Vector<4> path_nf;		/**< Path prefilter natural frequencies */
		math::Vector<4> path_dr;		/**< Path prefliter damping ratios*/

		math::Vector<4> pos_p; //PI on position difference
		math::Vector<4> pos_i;

		math::Vector<4> v_gain; //Proportional to velocity

	}		_params;	
};
