/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2016
 *
 ****************************************************************************/

/**
 * @file poller.cpp
 *
 * @brief Helper function for polling in other classes. 
 * 
 */


#include "poller.h"


 Poller::Poller()
 {
 	memset(&_v_att, 0, sizeof(_v_att));
 	memset(&_v_att_sp, 0, sizeof(_v_att_sp));
 	memset(&_v_rates_sp, 0, sizeof(_v_rates_sp));
 	memset(&_manual, 0, sizeof(_manual));
 	memset(&_v_control_mode, 0, sizeof(_v_control_mode));
 	memset(&_actuators, 0, sizeof(_actuators));
 	memset(&_armed, 0, sizeof(_armed));
 	memset(&_vehicle_status, 0, sizeof(_vehicle_status));
 	memset(&_rc_chan, 0, sizeof(_rc_chan));
 	memset(&_local_pos, 0, sizeof(_local_pos));
 	memset(&_sens, 0, sizeof(_sens));
 	memset(&_track, 0, sizeof(_track));
 	memset(&_flight_mode, 0, sizeof(_flight_mode));
 	memset(&_local_pos_sp, 0, sizeof(_local_pos_sp));
 	memset(&_ftap_safety, 0, sizeof(_ftap_safety));
 	memset(&_heli_mission_mode, 0, sizeof(_heli_mission_mode));
 	memset(&_tether_attach, 0, sizeof(_tether_attach));

 }

 Poller::~Poller()
 {
 	memset(&_v_att, 0, sizeof(_v_att));
 	memset(&_v_att_sp, 0, sizeof(_v_att_sp));
 	memset(&_v_rates_sp, 0, sizeof(_v_rates_sp));
 	memset(&_manual, 0, sizeof(_manual));
 	memset(&_v_control_mode, 0, sizeof(_v_control_mode));
 	memset(&_actuators, 0, sizeof(_actuators));
 	memset(&_armed, 0, sizeof(_armed));
 	memset(&_vehicle_status, 0, sizeof(_vehicle_status));
 	memset(&_rc_chan, 0, sizeof(_rc_chan));
 	memset(&_local_pos, 0, sizeof(_local_pos));
 	memset(&_sens, 0, sizeof(_sens));
 	memset(&_track, 0, sizeof(_track));
 	memset(&_heli_state, 0, sizeof(_heli_state));
 	memset(&_flight_mode, 0, sizeof(_flight_mode));
 	memset(&_local_pos_sp, 0, sizeof(_local_pos_sp));
 	memset(&_ftap_safety, 0, sizeof(_ftap_safety));
 	memset(&_heli_mission_mode, 0, sizeof(_heli_mission_mode));
 	memset(&_tether_attach, 0, sizeof(_tether_attach));

 }

/*Subscribe to all things we're going to poll. Add as desired. */ 

 void 
 Poller::do_subscriptions() 
 {
 	_v_att_sp_sub = orb_subscribe(ORB_ID(custom_attitude_setpoint));
 	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
 	_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
 	_v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
 	_params_sub = orb_subscribe(ORB_ID(parameter_update));
 	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
 	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
 	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
 	_rc_channel_sub = orb_subscribe(ORB_ID(rc_channels));
 	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
 	_sens_sub = orb_subscribe(ORB_ID(sensor_combined));
 	_track_sub = orb_subscribe(ORB_ID(track_setpoint));
 	_heli_state_sub = orb_subscribe(ORB_ID(heli_state));
 	_flight_mode_sub = orb_subscribe(ORB_ID(heli_flight_mode));
 	_local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_custom_position_setpoint));
 	_ftap_safety_sub = orb_subscribe(ORB_ID(ftap_safety));
 	_heli_mission_mode_sub = orb_subscribe(ORB_ID(heli_mission_mode));
 	_tether_attach_sub = orb_subscribe(ORB_ID(tether_attach));
 }

 void 
 Poller::poll_all()
 {
 	parameter_update_poll();
 	vehicle_control_mode_poll();
 	arming_status_poll();
 	vehicle_manual_poll();
 	vehicle_status_poll();
 	local_position_poll();
	rc_channels_poll(); //For the switch on the control state
	vehicle_attitude_setpoint_poll();
	vehicle_rates_setpoint_poll();
	local_position_setpoint_poll();
	vehicle_attitude_poll();
	sensor_combined_poll();
	track_setpoint_poll();
	heli_state_poll();
	flight_mode_poll();
	ftap_safety_poll();
	heli_mission_mode_poll();
	tether_attach_poll();
}

bool 
Poller::ftap_safety_poll() {

	bool updated; 
	/* Check if parameters have changed */
	orb_check(_ftap_safety_sub, &updated); 

	if (updated) {
		orb_copy(ORB_ID(ftap_safety), _ftap_safety_sub, &_ftap_safety);
		return true;
	}
	return false;

}

bool 
Poller::flight_mode_poll() {

	bool updated; 
	/* Check if parameters have changed */
	orb_check(_flight_mode_sub, &updated); 

	if (updated) {
		orb_copy(ORB_ID(heli_flight_mode), _flight_mode_sub, &_flight_mode);
		return true;
	}
	return false;


}

bool
Poller::local_position_setpoint_poll() 
{
	bool updated;


	/* Check if parameters have changed */
	orb_check(_local_pos_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_custom_position_setpoint), _local_pos_sp_sub, &_local_pos_sp);
		return true;
	}
	return false;
}


bool
Poller::heli_state_poll()
{
	bool updated;


	/* Check if parameters have changed */
	orb_check(_heli_state_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(heli_state), _heli_state_sub, &_heli_state);
		return true;
	}
	return false;
}


bool
Poller::track_setpoint_poll()
{
	bool updated;


	/* Check if parameters have changed */
	orb_check(_track_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(track_setpoint), _track_sub, &_track);
		return true;
	}
	return false;
}

bool
Poller::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _params_sub, &_param_update);
		return true;
	}
	return false;
}


bool
Poller::vehicle_control_mode_poll()
{
	bool updated;

	/* Check if vehicle control mode has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
		return true;
	}
	return false;
}


bool
Poller::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual);
		return true;
	}
	return false;
}

bool
Poller::vehicle_attitude_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_att_sp_sub, &updated);

	if (updated) {
		//orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
		orb_copy(ORB_ID(custom_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
		return true;
	}
	return false;
}


bool
Poller::vehicle_rates_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
		return true;
	}
	return false;
}


bool
Poller::arming_status_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
		return true;
	}
	return false;
}


bool
Poller::vehicle_status_poll()
{
	/* check if there is new status information */
	bool updated;
	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
		return true;
	}
	return false;
}

bool
Poller::vehicle_attitude_poll()
{
	bool updated;
	orb_check(_v_att_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
		return true;
	}
	return false;
}
bool
Poller::rc_channels_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_rc_channel_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(rc_channels), _rc_channel_sub, &_rc_chan);
		return true;
	}
	return false;
}

bool
Poller::local_position_poll()
{
	bool updated;
	orb_check(_local_pos_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
		return true;
	}
	return false;
}

bool
Poller::sensor_combined_poll() 
{
	bool updated;
	orb_check(_sens_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(sensor_combined), _sens_sub, &_sens);
		return true;
	}
	return false;
}

bool
Poller::heli_mission_mode_poll()
{
	bool updated;
	orb_check(_heli_mission_mode_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(heli_mission_mode),_heli_mission_mode_sub, &_heli_mission_mode);
		return true;
	}
	return false;
}

bool
Poller::tether_attach_poll()
{
	bool updated;
	orb_check(_tether_attach_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(tether_attach), _tether_attach_sub, &_tether_attach);
		return true;
	}
	return false;
}

