############################################################################
#
#   Copyright (c) 2017 LAOSAAC
#
############################################################################

include_directories(../../../../lib/DriverFramework/drivers)

px4_add_module(
	MODULE platforms__posix__drivers__df_ads1115_wrapper
	MAIN df_ads1115_wrapper
	SRCS
		df_ads1115_wrapper.cpp
	DEPENDS
		platforms__common
		df_driver_framework
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
