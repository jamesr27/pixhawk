/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2016
 *
 ****************************************************************************/


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

#include <heli_lib/rotations/rotations.h>
#include <heli_lib/digital_filter/digital_filter.h>
#include <heli_lib/c_signal/c_signal.h>
#include <heli_lib/poller/poller.h>
#include <heli_lib/mathsFunctions/mathsFunctions.h>

extern "C" __EXPORT int heli_path_planner_main(int argc, char *argv[]);

class HelicopterPathPlanner
{
public:
	/**
	 * Constructor
	 */
	HelicopterPathPlanner();

	/**
	 * Destructor, also kills task.
	 */
	~HelicopterPathPlanner();

	void		rate_transition(float &input, float goal, float dt, bool isPos);

	int 		parameters_update(); 

	void 		get_desired_position(float dt); 

	void 		measure_data();

	void 		mode_logic();
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
	int		_control_task;			/**< task handle for task */

	math::Vector<3> _target;
	math::Vector<3> _target_position;

	float 			_elevation_measured;
	float 			_elevation_target;
	float 			_gamma_measured; 
	float 			_gamma_target;
	float 			_initial_x;
	float 			_initial_y;
	float			_gammaM_prev;
	float			_wrap_offset;
	int 			_param_counter;
	bool			_gamma_direction;

	orb_advert_t 	_local_pos_sp_pub; 
	vehicle_custom_position_setpoint_s _local_pos_sp;

	heli_mission_mode_s 	_mission_mode;
	orb_advert_t _heli_mission_mode_pub;

	bool _isMissionPrev;
	uint8_t _missionModePrev; 


	struct {
		param_t gamma_target;
		param_t elevation_target;
		param_t take_off_elevation;
		param_t take_off_lead;
		param_t rate_angle;
		param_t rate_pos;
		
	}		_params_handles;		/**< handles for interesting parameters */

	struct {

		float gamma_target;
		float elevation_target;
		float take_off_elevation;
		float take_off_lead;
		float rate_angle;
		float rate_pos;

	}		_params;	

	// Added for tracking and editing the flight mode.
	heli_flight_mode_s 	_flight_mode;
	orb_advert_t _heli_flight_mode_pub;

};
