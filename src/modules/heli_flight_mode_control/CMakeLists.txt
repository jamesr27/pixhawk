
px4_add_module(
	MODULE modules__heli_flight_mode_control
	MAIN heli_flight_mode_control
	PRIORITY "SCHED_PRIORITY_MAX-30"
	STACK_MAIN 1000
	STACK_MAX 1200
	COMPILE_FLAGS -Os
	SRCS
		heli_flight_mode_control.cpp 
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
