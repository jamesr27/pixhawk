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
#include <mavlink/mavlink_log.h>
#include <platforms/px4_defines.h>
#include <uORB/topics/safety.h>
#include <heli_lib/rotations/rotations.h>
#include <heli_lib/digital_filter/digital_filter.h>
#include <heli_lib/c_signal/c_signal.h>
#include <heli_lib/poller/poller.h>

extern "C" __EXPORT int heli_flight_mode_control_main(int argc, char *argv[]);

class HelicopterFlightModeControl
{
public:
	/**
	 * Constructor
	 */
	HelicopterFlightModeControl();

	/**
	 * Destructor, also kills task.
	 */
	~HelicopterFlightModeControl();


	void 		flight_mode(float dt);
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

	heli_flight_mode_s 	_flight_mode;
	orb_advert_t _heli_flight_mode_pub;

		
};