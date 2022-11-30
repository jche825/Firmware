/**
 * @file drv_arduino_tacho.h
 *
 * Arduino tachometer interface.
 *
 * @author Junyi Chen
 */

#ifndef _DRV_ARDUINO_TACHO_H
#define _DRV_ARDUINO_TACHO_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

static constexpr const char*    ARDUTACHO_BASE_DEV_PATH	    = "/dev/ardutacho";

#endif /* _DRV_ARDUINO_TACHO_H */