/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2016
 *
 ****************************************************************************/

/**
 * @file poller.h
 *
 * @brief Helper function for polling in other classes. 
 * 
 */

#ifndef POLLER_H_
#define POLLER_H_

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
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/vehicle_custom_position_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/test_values.h>
#include <uORB/topics/track_setpoint.h>
#include <uORB/topics/heli_state.h>
#include <uORB/topics/heli_flight_mode.h> 
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/heli_mission_mode.h>
#include <uORB/topics/ftap_safety.h>
#include <uORB/topics/feedback.h>
#include <uORB/topics/tether_attach.h>

class __EXPORT Poller
{
	public: 

		//Constructor. Initializes to 0. 
		Poller();

		// Deconstructor. Clears everything to 0. 
		~Poller();

		//Does what function names say. 
		//Check system parameters
		bool		parameter_update_poll();

		// Vehicle mode poll
		bool		vehicle_control_mode_poll();

		// Manual input changes
		bool		vehicle_manual_poll();

		// Attitude setpoint updates
		bool		vehicle_attitude_setpoint_poll();

		// Rate setpoint updates
		bool		vehicle_rates_setpoint_poll();

		// Arming status updates
		bool		arming_status_poll();

		// Position updates. 
		bool 		local_position_poll();

		// Position updates. 
		bool 		local_position_setpoint_poll();
		///Check for vehicle status updates.
		bool		vehicle_status_poll();

		//Look at transmitter state
		bool		rc_channels_poll();

		//Get current attitude
		bool 		vehicle_attitude_poll();

		bool 		sensor_combined_poll();

		bool 		track_setpoint_poll(); 

		bool 		heli_state_poll();

		bool 		flight_mode_poll();

		bool 		ftap_safety_poll();

		bool 		heli_mission_mode_poll();

		bool		tether_attach_poll();

		void 		poll_all();

		/** Helper to subscribe to all.Doesn't work if you call in the constructor. */
		void	 do_subscriptions();	

		struct vehicle_attitude_s					_v_att;				/**< vehicle attitude */
		struct custom_attitude_setpoint_s			_v_att_sp;			/** < custom vehicle attitude setpoint */
		struct vehicle_rates_setpoint_s				_v_rates_sp;		/**< vehicle rates setpoint */
		struct manual_control_setpoint_s			_manual;			/**< manual control setpoint */
		struct vehicle_control_mode_s				_v_control_mode;	/**< vehicle control mode */
		struct actuator_controls_s		    		_actuators;			/**< actuator controls */
		struct actuator_armed_s						_armed;				/**< actuator arming status */
		struct vehicle_status_s						_vehicle_status;	/**< vehicle status */
		struct rc_channels_s						_rc_chan;			/**< rc channels */
		struct vehicle_local_position_s				_local_pos;			/**< vehicle local position */
		struct vehicle_custom_position_setpoint_s 	_local_pos_sp;
		struct parameter_update_s 					_param_update;
		struct test_values_s						_test_val;
		struct sensor_combined_s					_sens;
		struct track_setpoint_s						_track;
		struct heli_state_s							_heli_state;
		struct heli_flight_mode_s					_flight_mode;
		struct ftap_safety_s						_ftap_safety;
		struct feedback_s							_feedback;
		struct heli_mission_mode_s					_heli_mission_mode;
		struct tether_attach_s						_tether_attach;

		int		_v_att_sub;				/**< vehicle attitude subscription */
		int 	_rc_channel_sub;		/**< RC channels subscription */
		int		_local_pos_sub;			/**< vehicle local position */
		int 	_local_pos_sp_sub;
		int		_v_att_sp_sub;			/**< vehicle attitude setpoint subscription */
		int		_v_rates_sp_sub;		/**< vehicle rates setpoint subscription */
		int		_v_control_mode_sub;	/**< vehicle control mode subscription */
		int		_params_sub;			/**< parameter updates subscription */
		int		_manual_control_sp_sub;	/**< manual control setpoint subscription */
		int		_armed_sub;				/**< arming status subscription */
		int		_vehicle_status_sub;	/**< vehicle status subscription */
		int 	_sens_sub;
		int 	_track_sub;
		int 	_heli_state_sub;
		int 	_flight_mode_sub;
		int 	_ftap_safety_sub;
		int		_heli_mission_mode_sub;
		int		_tether_attach_sub;
};


#endif /** POLLER_H_ */
