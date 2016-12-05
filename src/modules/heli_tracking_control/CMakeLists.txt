
px4_add_module(
	MODULE modules__heli_tracking_control
	MAIN heli_tracking_control
	PRIORITY "SCHED_PRIORITY_MAX-50"
	STACK_MAIN 1000
	STACK_MAX 1200
	COMPILE_FLAGS -Os
	SRCS
		heli_tracking_control.cpp
		heli_tracking_control_params.c
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
