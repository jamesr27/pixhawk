uint8 windState
uint8 flightState
uint8 armingState
float32 xWaypoint
float32 yWaypoint
float32 heading

#Wind state, as determined by pilots. 
uint8 LOW_WIND = 0
uint8 HIGH_WIND = 1
uint8 EMERGENCY_WIND = 2

#Flight modes. Kept simple. 
uint8 TAKEOFF = 0
uint8 LAND = 1
uint8 LOITER = 2
uint8 HOVER = 3
uint8 EMERGENCY = 4
uint8 GROUNDED = 5

#Arming state is mapped onto similar things as the Pixhawk default
uint8 STANDBY = 0
uint8 PREARMED = 1
uint8 ARMED = 2

