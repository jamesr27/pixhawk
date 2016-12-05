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
#include <mavlink/mavlink_log.h>
#include <platforms/px4_defines.h>
#include <uORB/uORB.h>
#include <uORB/topics/ground_commands.h>
#include <uORB/topics/heli_state.h>
#include <uORB/topics/edison.h> 
#include <uORB/topics/distance_sensor.h> 

 extern "C" __EXPORT int heli_state_machine_main(int argc, char *argv[]);

 class HelicopterStateMachine
 {
 public:

	/**
	 * Constructor
	 */

	 HelicopterStateMachine();

	/**
	 * Destructor, also kills task.
	 */

	 ~HelicopterStateMachine();


	 int		parameters_update(); 


	 void 		assign_and_publish();


	 int		check_ground_status();

	 void 		check_wind_state();

	 void 		check_flight_state();

	 void 		check_arm_status();

	 void 		check_emergency_conditions();

	 void 		poll_all();

	 void 		run_state_machine();

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

		bool 					_task_should_exit;
		int						_control_task;
		int 					_mavlink_fd;

		heli_state_s 			_heli_state;
		ground_commands_s 		_ground_commands;
		edison_s				_edison;
		distance_sensor_s		_distance;
		actuator_armed_s		_armed;

		int 					_distance_sub;
		int 					_edison_sub;
		int 					_heli_state_sub;
		int 					_ground_commands_sub;
		int						_arming_sub;

		orb_advert_t 			_heli_state_pub;	

		struct {

			param_t rc_control;
			param_t cutoff_height;

		} _params_handles;

		struct {

			int rc_control;
			float cutoff_height;

		} _params;
	};