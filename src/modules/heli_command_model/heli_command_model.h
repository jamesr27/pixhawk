/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2015
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
#include <mavlink/mavlink_log.h>
#include <platforms/px4_defines.h>

#include <heli_lib/rotations/rotations.h>
#include <heli_lib/digital_filter/digital_filter.h>
#include <heli_lib/c_signal/c_signal.h>
#include <heli_lib/poller/poller.h>

extern "C" __EXPORT int heli_command_model_main(int argc, char *argv[]);


#define NUM_CHANNELS 4
#define NUM_SIGNALS 5
#define ATT_COM 0 
#define VEL_COM 1
#define ALT_OPEN 0
#define ALT_HOLD 1

class HelicopterCommandModel
{
public:
	/**
	 * Constructor
	 */
	HelicopterCommandModel();

	/**
	 * Destructor, also kills task.
	 */
	~HelicopterCommandModel();


	/**
	 * Helper for rate transitions
	 */
	
	void rate_transition(float &input, float goal, float dt);

	/**
	 * Helper to wrap yaw
	 */

	void 		wrap_yaw();

	/**
	 * Filter the incoming rate signals. 
	 */
	
	void		rate_filter(int axis, float dt, float K, float a);

	/**
	 * Filter the incoming attitude signals. For proper filtering we need 3 signals out of here now. 
	 */

	void		filter_att_vel(int axis, float dt, float K, float cross_term, float omega2);

	/** 
	* Rescale the collective for altitude holding 
	*/ 

	float 		scaled_collective(float collective_input);

	/**
	 * Update our local parameter cache.
	 */
	int			parameters_update(bool force);

	/**
	* Rescale the command from the tracking controller to fit the command model structure.  
	*/

	float rescale_command(float command, float scale);
	
	/**
	* Get the signals and put them in the right spots
	*/

	void 		initialize_signals();
	/** 
	* Switch states for the command model here.
	*/

	void 		command_model(float dt);
	/** 
	* For readability
	*/

	void 		assign_and_publish(float dt);

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
	int		_mavlink_fd;			/**< mavlink fd */

	bool 	_command_mode;
	bool 	_alt_mode;
	float 	_coll_zero;
	float 	_yaw_prev;
	int 	_param_counter;
	float 	_rotor_speed;
	bool 	_rotor_off;
	bool 	_isInitial;

	struct custom_attitude_setpoint_s _att_sp;

	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
	
	C_Signal* _stick_inputs;
	C_Signal* _all_outputs[NUM_CHANNELS];
	C_Signal* _velocity_prefilter;

	C_Signal* _roll_channel;
	C_Signal* _pitch_channel;
	C_Signal* _yaw_channel;
	C_Signal* _collective_channel;

	struct {
		param_t roll_gain; 			/**< Roll gain */
		param_t roll_nf;			/**< Roll natural frequency */ 
		param_t roll_dr;			/**< Roll damping ratio */
		param_t rollr_gain;			/**< Roll rate gain */
		param_t rollr_lag;			/**< Roll rate lag constant */
	
		param_t roll_gainv;
		param_t roll_wash;
		param_t roll_nfv;
		param_t roll_drv;


		param_t pitch_gain; 		/**< Pitch gain */
		param_t pitch_nf;			/**< Pitch natural frequency */ 
		param_t pitch_dr;			/**< Pitch damping ratio */
		param_t pitchr_gain;		/**< Pitch rate gain */
		param_t pitchr_lag;			/**< Pitch rate lag constant */
	
		param_t pitch_gainv;
		param_t pitch_wash;
		param_t pitch_nfv;
		param_t pitch_drv;

		param_t yawr_gain;			/**< Yaw rate gain */
		param_t yawr_lag;			/**< Yaw rate lag constant */

		param_t collective_gain;	/**< Yaw rate gain */
		param_t collective_lag;		/**< Yaw rate lag constant */

		param_t rollv_clim;
		param_t pitchv_clim;
		param_t rollx_clim;
		param_t pitchx_clim;

		param_t rotor_rate;
		
	}		_params_handles;		/**< handles for interesting parameters */

	struct {

		math::Vector<2> cyclic_gain; /**< Pitch roll gains */
		math::Vector<2> cyclic_nf;	/**< Pitch roll natural frequencies */
		math::Vector<2> cyclic_dr;	/**< Pitch roll damping ratios*/

		math::Vector<2> cyclic_gainv;
		math::Vector<2> cyclic_nfv;
		math::Vector<2> cyclic_drv;
		math::Vector<2> wash_lags;

		math::Vector<2> vel_climits;
		math::Vector<2> pos_climits;

		math::Vector<NUM_CHANNELS> rate_gains; 
		math::Vector<NUM_CHANNELS> rate_lags;

		float rotor_rate;

	}		_params;		
};
