
px4_add_module(
	MODULE modules__heli_lib
#	MAIN heli_lib
	PRIORITY "SCHED_PRIORITY_MAX-30"
	STACK_MAIN 1000
	STACK_MAX 1200
	COMPILE_FLAGS -Os
	SRCS
		rotations/rotations.cpp 
		digital_filter/digital_filter.cpp
		c_signal/c_signal.cpp 
		poller/poller.cpp 
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
