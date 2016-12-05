float32 vx			 # stick position in x direction -1..1
				 # in general corresponds to forward/back motion or pitch of vehicle,
				 # in general a positive value means forward or negative pitch and
				 # a negative value means backward or positive pitch
float32 vy			 # stick position in y direction -1..1
				 # in general corresponds to right/left motion or roll of vehicle,
				 # in general a positive value means right or positive roll and
				 # a negative value means left or negative roll
float32 vz			 # throttle stick position 0..1
				 # in general corresponds to up/down motion or thrust of vehicle,
				 # in general the value corresponds to the demanded throttle by the user,
				 # if the input is used for setting the setpoint of a vertical position
				 # controller any value > 0.5 means up and any value < 0.5 means down
float32 vr			 # yaw stick/twist positon, -1..1
				 # in general corresponds to the righthand rotation around the vertical
				 # (downwards) axis of the vehicle

float32 p1		#We have other options if we want them

float32 p2

float32 p3

float32 p4

