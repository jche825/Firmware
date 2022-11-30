/**
 * @file ardutacho.cpp
 * @author Junyi Chen
 *
 * Interface for arduino tachometer.
 */

#include "ArduinoTacho.h"
#include <board_config.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <cstdlib>
#include <string.h>
#include <stdio.h>
#include <platforms/px4_getopt.h>

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

static constexpr struct ardutacho_bus_option {
	const char *devname;
	uint8_t busnum;
} bus_options[] = {
#ifdef PX4_I2C_BUS_EXPANSION
	{ "/dev/ardutacho_ext", PX4_I2C_BUS_EXPANSION },
#endif
#ifdef PX4_I2C_BUS_EXPANSION1
	{ "/dev/ardutacho_ext1", PX4_I2C_BUS_EXPANSION1 },
#endif
#ifdef PX4_I2C_BUS_EXPANSION2
	{ "/dev/ardutacho_ext2", PX4_I2C_BUS_EXPANSION2 },
#endif
#ifdef PX4_I2C_BUS_ONBOARD
	{ "/dev/ardutacho_int", PX4_I2C_BUS_ONBOARD },
#endif
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int ardutacho_main(int argc, char *argv[]);


/*
 * Local functions in support of shell commands
 */
namespace ardutacho
{

ArduinoTacho *drv_instance = nullptr;

int start(int argc, char *argv[]);
void stop();
void reset();
void info();
void usage();

// Start the driver
int
start(int argc, char *argv[])
{
    int fd, ret;

    // Check if driver instance already initialised
    if (drv_instance != nullptr) {
        PX4_INFO("already started");
        return PX4_OK;
    }

    // Initialise driver
	// Check all I2C buses for device
	for (uint8_t i = 0; i < (sizeof(bus_options) / sizeof(bus_options[0])); i++) {
		drv_instance = new ArduinoTacho(bus_options[i].devname, bus_options[i].busnum);

		if (!drv_instance) {
        	PX4_ERR("Failed to instantiate ArduinoTacho");
        	return PX4_ERROR;
    	}

		if (drv_instance->init() == PX4_OK) {
			break;
		}

		PX4_ERR("failed to initialise ArduinoTacho on busnum=%u", bus_options[i].busnum);
		delete drv_instance;
		drv_instance = nullptr;
	}

	if (!drv_instance) {
        PX4_ERR("No ArduinoTacho found");
        return PX4_ERROR;
    }
    
    fd = open(drv_instance->get_dev_name(), O_RDONLY);

    if (fd == PX4_ERROR) {
        PX4_ERR("Error opening fd");
        goto fail;
    }

    ret = ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT);
    close(fd);

    if (ret != PX4_OK) {
        PX4_ERR("pollrate fail");
        goto fail;
    }

    return PX4_OK;

fail:
    stop();
    return PX4_ERROR;

} // start

// Stop the driver
void
stop()
{
    if (drv_instance != nullptr) {
        delete drv_instance;
        drv_instance = nullptr;
    } else {
        PX4_ERR("ArduTacho driver not running");
    }

} // stop

// Reset
void
reset()
{
	if (!drv_instance) {
		PX4_ERR("ArduTacho driver not running");
		return;
	}

	int fd = open(drv_instance->get_dev_name(), O_RDONLY);

	if (fd < 0) {
		PX4_ERR("Error opening fd");
		return;
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		goto error;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		goto error;
	}

error:
	close(fd);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (drv_instance == nullptr) {
		PX4_ERR("ArduTacho driver not running");
		return;
	}

	printf("state @ %p\n", drv_instance);
	drv_instance->print_info();
}

void
usage()
{
	PX4_INFO("missing command: try 'start', 'stop', 'reset', 'info'");
}

} // namespace ardutacho

int
ardutacho_main(int argc, char *argv[])
{
	int myoptind = 1;

	if (argc > myoptind) {
		const char *verb = argv[myoptind];

		if (!strcmp(verb, "start")) {
			// Start/load the driver
			return ardutacho::start(argc, argv);
		} else if (!strcmp(verb, "stop")) {
			// Stop the driver
			ardutacho::stop();
		} else if (!strcmp(verb, "reset")) {
			// Reset the driver
			ardutacho::reset();
		} else if (!strcmp(verb, "info") || !strcmp(verb, "status")) {
			// Print driver information.
			ardutacho::info();
		} else {
			ardutacho::usage();
			return PX4_ERROR;
		}
		return PX4_OK;
	}
		
	PX4_ERR("unrecognised command, try 'start', 'stop', 'reset' or 'info'");
	return PX4_ERROR;
}