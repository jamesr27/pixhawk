// James Reeves.
// A simple app to check whether data exists outside of the simulink app.
// Once started it will print useful stuff to the terminal window.

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
#include <uORB/topics/control_state.h>
#include <uORB/topics/airspeed.h>

extern "C" __EXPORT int data_tester_main(int argc, char *argv[]);

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;


static int _sensors_sub = 0;
static int _gps_pos_sub = 0;
static int _control_state_sub = 0;
static int _differential_pressure_sub = 0;
static int _airspeed_sub = 0;

// Prototypes
int data_tester_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

// Functions




int
data_tester_thread_main(int argc, char *argv[]){
	// Initialise stuff
	struct sensor_combined_s _hil_sensors;
	memset(&_hil_sensors, 0, sizeof(_hil_sensors));

	struct vehicle_gps_position_s _hil_gps;
	memset(&_hil_gps, 0, sizeof(_hil_gps));

	struct control_state_s _control_state;
	memset(&_control_state, 0, sizeof(_control_state));

	struct differential_pressure_s _differential_pressure;
	memset(&_differential_pressure, 0, sizeof(_differential_pressure));

	struct airspeed_s _airspeed;
	memset(&_airspeed, 0, sizeof(_airspeed));

	// Subscribe to orbs
	_sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
	_gps_pos_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	_control_state_sub = orb_subscribe(ORB_ID(control_state));
	_differential_pressure_sub = orb_subscribe(ORB_ID(differential_pressure));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));

	// Main thread.
	while(!thread_should_exit){
		orb_copy(ORB_ID(sensor_combined), _sensors_sub, &_hil_sensors);
		orb_copy(ORB_ID(vehicle_gps_position), _gps_pos_sub, &_hil_gps);
		orb_copy(ORB_ID(control_state), _control_state_sub, &_control_state);
		orb_copy(ORB_ID(differential_pressure),_differential_pressure_sub, &_differential_pressure);
		orb_copy(ORB_ID(airspeed), _airspeed_sub, &_airspeed);

		// Print data to terminal
		//printf("Sensors: a: %0.3f %0.3f %0.3f m: %0.3f t: %0.3f\n",(double)_hil_sensors.accelerometer_m_s2[0],(double)_hil_sensors.accelerometer_m_s2[1],(double)_hil_sensors.accelerometer_m_s2[2],(double)_hil_sensors.magnetometer_ga[0],(double)_hil_sensors.baro_temp_celcius);
		//printf("SensTime: gi %0.3f ai %0.3f ar %0.3f mr %0.3f\n",(double)_hil_sensors.gyro_integral_dt,(double)_hil_sensors.accelerometer_integral_dt,(double)_hil_sensors.accelerometer_timestamp_relative,(double)_hil_sensors.magnetometer_timestamp_relative);
		printf("GPS: lat %0.3f lon %0.3f alt %0.3f\n",(double)_hil_gps.lat,(double)_hil_gps.lon,(double)_hil_gps.alt);
		printf("Control State: as %0.3f xa %0.3f zp %0.3f\n",(double)_control_state.airspeed,(double)_control_state.x_acc,(double)_control_state.z_pos);
		//printf("dpress: %0.3f %0.3f %0.3f\n",(double)_differential_pressure.differential_pressure_filtered_pa,(double)_differential_pressure.differential_pressure_raw_pa,(double)_differential_pressure.temperature);
		printf("airspeed: %0.3f %0.3f %0.3f\n",(double)_airspeed.true_airspeed_m_s,(double)_airspeed.true_airspeed_unfiltered_m_s,(double)_airspeed.indicated_airspeed_m_s);

		usleep(500000);
	}
	return 0;
}


int data_tester_main(int argc, char *argv[])
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
		daemon_task = px4_task_spawn_cmd("data_tester",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX-50,
						 1000,
						 data_tester_thread_main,
						 nullptr);
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

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: data_tester {start|stop|status} [-p <additional params>]\n\n");
}
