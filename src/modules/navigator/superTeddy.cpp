// James Reeves. Super Teddy waypoint/mission helper class.
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <geo/geo.h>

#include <uORB/uORB.h>
#include <navigator/navigation.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vtol_vehicle_status.h>

#include "navigator.h"
#include "superTeddy.h"



