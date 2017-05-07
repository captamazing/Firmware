############################################################################
#
# Copyright (c) 2017 LAOSAAC
#
############################################################################

include_directories(../../../../lib/DriverFramework/framework/include)

px4_add_module(
	MODULE platforms__posix__drivers__ursa_rc_in
	MAIN ursa_rc_in
	SRCS
		ursa_rc_in.cpp
	DEPENDS
		platforms__common
		df_driver_framework
	)

# vim: set noet fenc=utf-8 ff=unix ft=cmake :

