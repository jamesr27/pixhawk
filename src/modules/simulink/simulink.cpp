/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2015
 *
 ****************************************************************************/

/**
 * @file simulink.cpp
 *
 * @brief Module for running hardware in the loop. 
 *
 */
#include <inttypes.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <poll.h>
#include <nuttx/config.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <poll.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <stdio.h>
#include <stdbool.h>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <errno.h>
#include <limits.h>
#include <nuttx/serial/serial.h>
#include <math.h>
#include <lib/mathlib/mathlib.h>
#include <px4_posix.h>
#include <geo/geo.h>
//#include <mavlink/mavlink_log.h>

#include <time.h>
#include <sys/time.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/custom_attitude_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/test_values.h>
//#include <uORB/topics/track_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/differential_pressure.h>

#include "my_serial.h"
#include "autopilot_if.h"

//WE GENERATE DIFFERENT HEADERS TO DO THIS!
//#include <v1.0/simulink/mavlink.h>
#include <v1.0/bronberg/mavlink.h>
 
extern "C" __EXPORT int simulink_main(int argc, char *argv[]);

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;	
static orb_advert_t _gps_pub = nullptr;
static orb_advert_t _sensors_pub = nullptr;
static orb_advert_t _diff_pres_pub = nullptr;

//static int _act_sub = 0;
static int _att_sub = 0;
static int _v_att_sub = 0;
static int _test_sub = 0;
static int _rc_sub = 0;
static int _sensors_sub = 0;
static int _pos_sub = 0;
//static int _track_sub = 0;
static int _manual_sub = 0;

static int counter = 0;
static uint64_t previous_timestamp;

static int _mavlink_fd = 0;

static const float mg2ms2 = CONSTANTS_ONE_G / 1000.0f;

/** <Extract mavlink information and write it to the appropriate orbs. */ 

bool handle_mavlink_messages(Autopilot_Interface &api);

/** <Assign sensor data to appropriate ORB */ 

void assign_sensors(mavlink_hil_sensor_t sensor_message, uint64_t timestamp);

/** <Assign GPS data to appropriate ORB */ 

void assign_gps(mavlink_hil_gps_t gps_message, uint64_t timestamp);

/** <We're going to poll and write here */ 
void write_hil_controls(Autopilot_Interface &api);

/** <Helper to extract port information */

void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate);

/** <Main function. Comment out things here depending on what you need to do. */ 

int simulink_thread_main(int argc, char *argv[]);

/** Helper for correct usage of the main function */ 

static void usage(const char *reason);

/*Helper function to parse the command line for the device/baud*/

void write_hil_controls(Autopilot_Interface &api)
{
	struct rc_channels_s _rc_chan;
	memset(&_rc_chan, 0, sizeof(_rc_chan));

	struct vehicle_attitude_s _v_att;
	memset(&_v_att, 0, sizeof(_v_att));

	struct custom_attitude_setpoint_s _att_s;
	memset(&_att_s, 0, sizeof(_att_s));

	struct test_values_s _test_val;
	memset(&_test_val, 0, sizeof(test_values_s));

	struct sensor_combined_s _hil_sensors;
	memset(&_hil_sensors, 0, sizeof(_hil_sensors));

	//struct actuator_controls_0_s _actuators;
	//memset(&_actuators, 0, sizeof(_actuators));

	struct vehicle_local_position_s _pos;
	memset(&_pos, 0, sizeof(_pos));

//	struct track_setpoint_s _track;
//	memset(&_track, 0, sizeof(_track));

	struct manual_control_setpoint_s _manual;
	memset(&_manual, 0, sizeof(_manual));

	
	while (not thread_should_exit) {
		
		if(handle_mavlink_messages(api)) {
	

			orb_copy(ORB_ID(rc_channels), _rc_sub, &_rc_chan);

			orb_copy(ORB_ID(custom_attitude_setpoint), _att_sub, &_att_s);
			
			orb_copy(ORB_ID(test_values), _test_sub, &_test_val);

			orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);

			// orb_copy(ORB_ID(actuator_controls_0), _act_sub, &_actuators);

			//orb_copy(ORB_ID(track_setpoint), _track_sub, &_track);
			
			orb_copy(ORB_ID(vehicle_local_position), _pos_sub, &_pos);

			orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
			//orb_copy(ORB_ID(sensor_combined), _sensors_sub, &_hil_sensors);


			counter++;

			// James checks: See if we get data out of the orbs
			//printf("simwrite: %0.3f %0.3f %0.3f\n",(double)_pos.x,(double)_pos.y,(double)_pos.z);




			mavlink_simulink_t sim;
			sim.time_usec = hrt_absolute_time();
			sim.counter = counter;
			sim.stick_input0 =  _rc_chan.channels[0];
			sim.stick_input1 = _rc_chan.channels[1];
			sim.stick_input2 = _rc_chan.channels[2];
			sim.stick_input3 = _rc_chan.channels[3];
//			sim.stick_input0 =  _manual.y;
//			sim.stick_input1 = _manual.x;
//			sim.stick_input2 = _manual.r;
//			sim.stick_input3 = _manual.z;
		
			sim.stick_input4 = _rc_chan.channels[6];
			sim.pos_x = _pos.x;
			sim.pos_y = _pos.y;
			sim.pos_z = _pos.z;
			sim.v_x  = _pos.vx;
			sim.v_y  = _pos.vy;
			sim.v_z  = _pos.vz;
			sim.roll  = _v_att.roll;
			sim.pitch = _v_att.pitch;
			sim.yaw  = _v_att.yaw;
			sim.rollspeed = _v_att.rollspeed;
			sim.pitchspeed = _v_att.pitchspeed;
			sim.yawspeed = _v_att.yawspeed;
			sim.signal_1 = _rc_chan.channels[4];
			sim.signal_2 = _rc_chan.channels[5];
			sim.signal_3 = _rc_chan.channels[6];
			sim.probe_1 = _test_val.value1;
			sim.probe_2 = _test_val.value2;
			sim.probe_3 = _test_val.value3;
			sim.probe_4 = _test_val.value4;
			sim.probe_5 = 0;
			sim.probe_6 = 0;
			api.write_sim(sim);
	 		usleep(100);
		 } else {
	 		usleep(100);
		}
	}
}

void assign_sensors(mavlink_hil_sensor_t sensor_message, uint64_t timestamp) 
{
	//mavlink_log_info(_mavlink_fd, "We're in here\n");
	/* sensor combined */
	struct sensor_combined_s hil_sensors;
	memset(&hil_sensors, 0, sizeof(hil_sensors));

	// Differential pressure data
	struct differential_pressure_s _diff_pres;
	memset(&_diff_pres, 0, sizeof(_diff_pres));

	hil_sensors.timestamp = timestamp;

//	hil_sensors.gyro_raw[0] = sensor_message.xgyro * 1000.0f;
//	hil_sensors.gyro_raw[1] = sensor_message.ygyro * 1000.0f;
//	hil_sensors.gyro_raw[2] = sensor_message.zgyro * 1000.0f;
	hil_sensors.gyro_rad[0] = sensor_message.xgyro;
	hil_sensors.gyro_rad[1] = sensor_message.ygyro;
	hil_sensors.gyro_rad[2] = sensor_message.zgyro;
//	hil_sensors.gyro_timestamp[0] = timestamp;

//	hil_sensors.accelerometer_raw[0] = sensor_message.xacc / mg2ms2;
//	hil_sensors.accelerometer_raw[1] = sensor_message.yacc / mg2ms2;
//	hil_sensors.accelerometer_raw[2] = sensor_message.zacc / mg2ms2;
	hil_sensors.accelerometer_m_s2[0] = sensor_message.xacc;
	hil_sensors.accelerometer_m_s2[1] = sensor_message.yacc;
	hil_sensors.accelerometer_m_s2[2] = sensor_message.zacc;
//	hil_sensors.accelerometer_mode[0] = 0; // TODO what is this?
//	hil_sensors.accelerometer_range_m_s2[0] = 32.7f; // int16
//	hil_sensors.accelerometer_timestamp[0] = timestamp;

	// James prints
	//printf("Accels: %0.3f %0.3f %0.3f\n",(double)sensor_message.xacc,(double)sensor_message.yacc,(double)sensor_message.zacc);

//	hil_sensors.adc_voltage_v[0] = 0.0f;
//	hil_sensors.adc_voltage_v[1] = 0.0f;
//	hil_sensors.adc_voltage_v[2] = 0.0f;

//	hil_sensors.magnetometer_raw[0] = sensor_message.xmag * 1000.0f;
//	hil_sensors.magnetometer_raw[1] = sensor_message.ymag * 1000.0f;
//	hil_sensors.magnetometer_raw[2] = sensor_message.zmag * 1000.0f;
	hil_sensors.magnetometer_ga[0] = sensor_message.xmag;
	hil_sensors.magnetometer_ga[1] = sensor_message.ymag;
	hil_sensors.magnetometer_ga[2] = sensor_message.zmag;
//	hil_sensors.magnetometer_range_ga[0] = 32.7f; // int16
//	hil_sensors.magnetometer_mode[0] = 0; // TODO what is this
//	hil_sensors.magnetometer_cuttoff_freq_hz[0] = 50.0f;
//	hil_sensors.magnetometer_timestamp[0] = timestamp;

//	hil_sensors.baro_pres_mbar[0] = sensor_message.abs_pressure;
	hil_sensors.baro_alt_meter = sensor_message.pressure_alt;
	hil_sensors.baro_temp_celcius = sensor_message.temperature;
//	hil_sensors.baro_timestamp[0] = timestamp;

//	hil_sensors.differential_pressure_pa[0] = sensor_message.diff_pressure * 1e2f; //from hPa to Pa
	//hil_sensors.differential_pressure_filtered_pa[0] = sensor_message.differential_pressure_pa[0];
//	hil_sensors.differential_pressure_timestamp[0] = timestamp;

	/* publish combined sensor topic */
	if (_sensors_pub == nullptr) {
		_sensors_pub = orb_advertise(ORB_ID(sensor_combined), &hil_sensors);

	} else {
		orb_publish(ORB_ID(sensor_combined), _sensors_pub, &hil_sensors);
	}

	// Do the differential pressure publishing
	_diff_pres.differential_pressure_raw_pa = sensor_message.diff_pressure * 1e2f;
	_diff_pres.differential_pressure_filtered_pa = _diff_pres.differential_pressure_raw_pa;
	_diff_pres.temperature = hil_sensors.baro_temp_celcius;

	if(_diff_pres_pub != nullptr) {
		orb_publish(ORB_ID(differential_pressure), _diff_pres_pub, &_diff_pres);
	} else {
		_diff_pres_pub = orb_advertise(ORB_ID(differential_pressure), &_diff_pres);
	}

}

void assign_gps(mavlink_hil_gps_t gps_message, uint64_t timestamp) 
{
	struct vehicle_gps_position_s hil_gps;
	memset(&hil_gps, 0, sizeof(hil_gps));

	//hil_gps.timestamp_time = timestamp;
	hil_gps.time_utc_usec = gps_message.time_usec;

	//hil_gps.timestamp_position = timestamp;
	hil_gps.lat = gps_message.lat;
	hil_gps.lon = gps_message.lon;
	hil_gps.alt = gps_message.alt;
	hil_gps.eph = (float)gps_message.eph * 1e-2f; // from cm to m
	hil_gps.epv = (float)gps_message.epv * 1e-2f; // from cm to m

	//hil_gps.timestamp_variance = timestamp;
	hil_gps.s_variance_m_s = 5.0f;

	//hil_gps.timestamp_velocity = timestamp;
	hil_gps.vel_m_s = (float)gps_message.vel * 1e-2f; // from cm/s to m/s
	hil_gps.vel_n_m_s = gps_message.vn * 1e-2f; // from cm to m
	hil_gps.vel_e_m_s = gps_message.ve * 1e-2f; // from cm to m
	hil_gps.vel_d_m_s = gps_message.vd * 1e-2f; // from cm to m
	hil_gps.vel_ned_valid = true;
	hil_gps.cog_rad = _wrap_pi(gps_message.cog * M_DEG_TO_RAD_F * 1e-2f);

	hil_gps.fix_type = gps_message.fix_type;
	hil_gps.satellites_used = gps_message.satellites_visible;  //TODO: rename mavlink_hil_gps_t sats visible to used?

	if (_gps_pub == nullptr) {
		_gps_pub = orb_advertise(ORB_ID(vehicle_gps_position), &hil_gps);

	} else {
		orb_publish(ORB_ID(vehicle_gps_position), _gps_pub, &hil_gps);
	}	
}



// If I'm in HIL mode can I actually overwrite the sensor data? 
bool
handle_mavlink_messages(Autopilot_Interface &api) 
{
	if(api.current_messages.new_message) {
		switch(api.last_id())
		{
			case MAVLINK_MSG_ID_HIL_SENSOR:
			{	
				uint64_t this_timestamp =  api.current_messages.time_stamps.hil_sensor;
				if(previous_timestamp != this_timestamp) {
					//printf("Here in sensors\n");
					assign_sensors(api.current_messages.hil_sensor, this_timestamp);
					previous_timestamp = this_timestamp;
					return true;
				}
				break;
			}

			case MAVLINK_MSG_ID_HIL_GPS:
			{
				uint64_t this_timestamp =  api.current_messages.time_stamps.hil_gps;
				if(previous_timestamp != this_timestamp) {
					//printf("Here in gps\n");
					assign_gps(api.current_messages.hil_gps, this_timestamp);
					previous_timestamp = this_timestamp;
					return true;
				}
				break;
			}
		}  
	}
	return false;
}



void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

	// string for command line usage
	const char *commandline_usage = "usage: simulink start -d <devicename> -b <baudrate>";

	// Read input arguments
	for (int i = 0; i < argc; i++) { // argv[0] is "mavlink"

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			return;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf("%s\n",commandline_usage);
				return;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf("%s\n",commandline_usage);
				return;
			}
		}

	}

	return;
}

/* Function defining usage for the daemon part*/ 
static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: simulink {start|stop|status} [-p <additional params>]\n\n");
}

/*Daemon starting app. Resends the arguments to the main thread after "start|stop|status" */

int 
simulink_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("simulink",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX,
						 4000,
						 simulink_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

/*Main thread here. */ 

int 
simulink_thread_main(int argc, char *argv[]){

	_mavlink_fd = px4_open("dev/mavlink", 0);
	char *uart_name = (char*)"/dev/ttyACM0"; //USB
	//char *uart_name = (char*)"/dev/ttyS1"; //Defaults to the radio on Telem 1.
	
	int baudrate = 57600;
	thread_running= true;

	// do the parse, will throw an int if it fails
	parse_commandline(argc, argv, uart_name, baudrate);

	/* Initialize the helper classes to write/read the port */
	Serial_Port serial_port(uart_name, baudrate);
	Autopilot_Interface autopilot_interface(&serial_port);
	

	serial_port.start();
	autopilot_interface.start();
	
//	_act_sub = orb_subscribe(ORB_ID(actuator_controls_0));
	_rc_sub = orb_subscribe(ORB_ID(rc_channels));
	_att_sub = orb_subscribe(ORB_ID(custom_attitude_setpoint));
	_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_test_sub =  orb_subscribe(ORB_ID(test_values));
	_sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
	_pos_sub  = orb_subscribe(ORB_ID(vehicle_local_position));
//	_track_sub = orb_subscribe(ORB_ID(track_setpoint));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	write_hil_controls(autopilot_interface);

	autopilot_interface.stop();
	serial_port.stop();
	return 0;
}
