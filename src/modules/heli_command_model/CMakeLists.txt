
px4_add_module(
	MODULE modules__heli_command_model
	MAIN heli_command_model
	PRIORITY "SCHED_PRIORITY_MAX-30"
	STACK_MAIN 1000
	STACK_MAX 1200
	COMPILE_FLAGS -Os
	SRCS
		heli_command_model.cpp 
		heli_command_model_params.c
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
