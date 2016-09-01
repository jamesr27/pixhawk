
#uint64 timestamp				# in microseconds since system start, is set whenever the writing thread stores new data


float32 u
float32 v
float32 w

float32 pos_x
float32 pos_y
float32 pos_z

float32 roll_body				# body angle in NED frame
float32 pitch_body				# body angle in NED frame
float32 yaw_body				# body angle in NED frame
float32 coll

float32 roll_v					#Velocities from the sticks
float32 pitch_v
float32 yaw_v
float32 coll_v

float32 roll_ax					#Acceleration from the sticks
float32 pitch_ax				
float32 yaw_ax					
float32 coll_ax

float32 collective_sp
float32 head_speed				# Head speed signal. We'll filter and limit this to prevent the helicopter from spinning up too uncontrollably
