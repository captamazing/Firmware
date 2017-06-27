############################################################################
#
#   Copyright (c) 2017 LAOSAAC
#
############################################################################

include_directories(../../../../lib/DriverFramework/drivers)

px4_add_module(
	MODULE platforms__posix__drivers__ursa_rcin
	MAIN ursa_rcin
	SRCS
		ursa_rcin.cpp
	DEPENDS
		platforms__common
		df_driver_framework
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
