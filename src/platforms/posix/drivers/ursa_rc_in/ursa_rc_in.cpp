/****************************************************************************
 *
 *   Copyright (C) 2017 LAOSAAC
 *
 ****************************************************************************/

#include <string.h>
#include "DriverFramework.hpp"
#include "ursa_rc_in.hpp"

using namespace DriverFramework;

#define POW2(_x) ((_x) * (_x))


int RC_IN::rc_in_init()
{

	return 0;
}

int RC_IN::start()
{
	return 0;
}

int ADS1115::stop()
{

	return 0;
}

void ADS1115::_measure()
{
	
}

int ADS1115::devRead(void *buf, size_t count)
{	
	return 0;
}