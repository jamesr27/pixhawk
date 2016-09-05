
px4_add_module(
	MODULE modules__data_tester
	MAIN data_tester
	PRIORITY "SCHED_PRIORITY_MAX-50"
	STACK_MAIN 1000
	STACK_MAX 1200
	COMPILE_FLAGS -Os
	SRCS
		data_tester.cpp 
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
