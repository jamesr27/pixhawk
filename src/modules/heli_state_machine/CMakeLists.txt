
px4_add_module(
	MODULE modules__heli_state_machine
	MAIN heli_state_machine
	PRIORITY "SCHED_PRIORITY_MAX-80"
	STACK_MAIN 1000
	STACK_MAX 1200
	COMPILE_FLAGS -Os
	SRCS
		heli_state_machine.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
