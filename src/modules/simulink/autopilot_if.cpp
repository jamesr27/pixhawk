/****************************************************************************
 *
 *  Arjun Bhargava
 *	Otherlab 2015
 *
 ****************************************************************************/

/**
 * @file autopilot_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and receiving commands to an autopilot via MAVlink
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "autopilot_if.h"


// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_Interface::
Autopilot_Interface(Serial_Port *serial_port_)
{
	// initialize attributes
	_control_updated = false;
	_sim_updated = false;
	write_count = 0;

	reading_status = 0;      // whether the read thread is running
	time_to_exit   = false;  // flag to signal thread exit

	read_tid  = 0; // read thread id

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

	serial_port = serial_port_; // serial port management object

}

Autopilot_Interface::
~Autopilot_Interface()
{}


// ------------------------------------------------------------------------------
//   Update Setpoint
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
update_controls(mavlink_hil_controls_t controls)
{
	_current_controls = controls;
	_control_updated = true;
}

void
Autopilot_Interface::
update_simulink(mavlink_simulink_t sim)
{
	_current_sim = sim;
	_sim_updated= true;
}


// ------------------------------------------------------------------------------
//  Know the last received message.
// ------------------------------------------------------------------------------

uint8_t 
Autopilot_Interface::
last_id()
{	
	return _last_msgid;
}

// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_messages()
{
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;
	current_messages.new_message = false;

	// Blocking wait for new data
	while ( not received_all and not time_to_exit )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = serial_port->read_message(message);
		//printf("Got through here\n");
		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{
			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;

			// Handle Message ID
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_PARAM_SET: 
				{
					printf("THIS END WORKS.\n");
					break;

				}

				case MAVLINK_MSG_ID_HIL_GPS: 
				{
					//printf("MAVLINK_MSG_ID_HIL_GPS\n");
					mavlink_msg_hil_gps_decode(&message, &(current_messages.hil_gps));
					current_messages.time_stamps.hil_gps = get_time_usec();
					this_timestamps.hil_gps = current_messages.time_stamps.hil_gps;
					_last_msgid = message.msgid;
					current_messages.new_message = true;
					break;
				}

				case MAVLINK_MSG_ID_HIL_SENSOR:
				{
					//printf("MAVLINK_MSG_ID_HIL_SENSOR\n");
					mavlink_msg_hil_sensor_decode(&message, &(current_messages.hil_sensor));
					current_messages.time_stamps.hil_sensor = get_time_usec();
					this_timestamps.hil_sensor = current_messages.time_stamps.hil_sensor;
					_last_msgid = message.msgid;
					current_messages.new_message = true;
					break;
				}

				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					//printf("MAVLINK_MSG_ID_HEARTBEAT %d\n", mavlink_msg_heartbeat_get_mavlink_version(&message));
					break;
				}

				default:
				{
					printf("Warning, did not handle message id %i\n", message.msgid);
					break;
				}

			} // end: switch msgid

		} // end: if read message

		// Check for receipt of all items
		received_all = this_timestamps.hil_sensor || this_timestamps.hil_gps;

	} // end: while not received all

	return;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
write_message(mavlink_message_t message)
{
	// do the write
	int len = serial_port->write_message(message);

	// book keep
	write_count++;

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Write controls Message
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_controls()
{
	// pull from position target
	mavlink_hil_controls_t hilc = _current_controls;

	mavlink_message_t message; 
	mavlink_msg_hil_controls_encode(1, 1, &message, &hilc);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message);

	// check the write
	if (len == 0 )
		fprintf(stderr,"WARNING: could not send HIL_CONTROLS \n");

	return;
}

void
Autopilot_Interface::
write_sim(mavlink_simulink_t sim)
{
	// pull from position target
	//mavlink_simulink_t sim = _current_sim;

	mavlink_message_t message; 
	mavlink_msg_simulink_encode(1, 1, &message, &sim);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message);

	// check the write
	if (len == 0 )
		fprintf(stderr,"WARNING: could not send SIMULINK \n");

	return;
}

// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start()
{
	int result;

	// --------------------------------------------------------------------------
	//   CHECK SERIAL PORT
	// --------------------------------------------------------------------------

	if (serial_port->status == 0 ) // SERIAL_PORT_OPEN
	{
		fprintf(stderr,"ERROR: serial port not open\n");
		exit(1);
	}

	// --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------

	printf("START READ/WRITE THREAD \n");

	result = pthread_create( &read_tid, NULL, &start_autopilot_interface_readWrite_thread, this);
	if ( result ) exit(1);

	// now we're reading messages
	printf("\n");


	// Done!
	return;

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(read_tid ,NULL);

	// now the read and write threads are closed
	printf("\n");

	// still need to close the serial_port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_readWrite_thread()
{

	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		readWrite_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Read/Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
readWrite_thread()
{
	reading_status = true;

	while ( not time_to_exit )
	{
		read_messages();
		usleep(300); // Read batches at 100Hz
		
		// if(_control_updated) {
		//  	write_controls();
		// 	_control_updated = false;
		 	
		// }

		// if(_sim_updated)
		// {
		// 	write_sim();
		//  	_sim_updated = false;
		// }
	}

	reading_status = false;


	return;
}
// End Autopilot_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_autopilot_interface_readWrite_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_readWrite_thread();

	// done!
	return NULL;
}


