############################################################################
#
#   Copyright (c) 2017 LAOSAAC
#
############################################################################

include_directories(../../../../lib/DriverFramework/drivers)

px4_add_module(
	MODULE platforms__posix__drivers__df_srf08_wrapper
	MAIN df_srf08_wrapper
	SRCS
		df_srf08_wrapper.cpp
	DEPENDS
		platforms__common
		df_driver_framework
		df_srf08
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
