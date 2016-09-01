############################################################################
#
#   Arjun Bhargava
#	Otherlab 2015
#
############################################################################

#
# Mavlink Reader application to eventually hook up to gimbal controller
#

MODULE_COMMAND	= simulink

SRCS		= simulink.cpp \
		  my_serial.cpp\
		  autopilot_if.cpp
		  
INCLUDE_DIRS	 += $(MAVLINK_SRC)/include/mavlink

EXTRACXXFLAGS	= -Wno-attributes -Wno-packed

EXTRACFLAGS	= -Wno-packed

