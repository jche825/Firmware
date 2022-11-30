/*
 * Created by Junyi Chen on 14/11/22.
 */

#include "ArduinoTacho.h"
#include <px4_defines.h>
#include <semaphore.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <string.h>
#include <stdio.h>
#include <drivers/drv_hrt.h>

ArduinoTacho::ArduinoTacho(const char *path, int bus, const char *name, uint16_t address, uint32_t frequency, int conversion_interval) :
    I2C(name, path, bus, address, frequency),
    _conversion_interval(conversion_interval),
    _measure_ticks(0),
    _reports(nullptr),
    _class_instance(-1),
    _orb_class_instance(-1),
    _actuator_outputs_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "ardutacho_read")),
	_comms_errors(perf_alloc(PC_COUNT, "ardutacho_comms_errors"))
{
    // up the retries since the device may miss measurement attempts
	_retries = 3;
    
    /* Initialise _work as 0 */
	memset(&_work, 0, sizeof(_work));
}

ArduinoTacho::~ArduinoTacho()
{
    /* make sure we are truly inactive */
    stop();

    /* Free existing reports */
    if (_reports != nullptr) {
        delete _reports;
    }

    if (_actuator_outputs_topic != nullptr) {
        orb_unadvertise(_actuator_outputs_topic);
    }

    if (_class_instance != -1) {
		unregister_class_devname(ARDUTACHO_BASE_DEV_PATH, _class_instance);
	}

    // Free perf counters with perf_free(counter);
    perf_free(_sample_perf);
    perf_free(_comms_errors);
}

int
ArduinoTacho::init()
{
    // do I2C init (and probe) first
    if (I2C::init() != PX4_OK) {
        return PX4_ERROR;
    }

    // Allocate report buffer
    _reports = new ringbuffer::RingBuffer(2, sizeof(actuator_outputs_s));

    if (_reports == nullptr) {
		return PX4_ERROR;
	}

    _class_instance = register_class_devname(ARDUTACHO_BASE_DEV_PATH);

    // Establish publish handle
	struct actuator_outputs_s ao_report;
    memset(&ao_report, 0, sizeof(ao_report));

    _actuator_outputs_topic = orb_advertise_multi(ORB_ID(actuator_outputs), &ao_report,
				 &_orb_class_instance, ORB_PRIO_HIGH);

    if (_actuator_outputs_topic == nullptr) {
		DEVICE_LOG("failed to create actuator_outputs object. Did you start uOrb?");
	}

    // Probe device
    if (probe() != PX4_OK) {
        return PX4_ERROR;
    }

    return PX4_OK;
}

int
ArduinoTacho::probe()
{
    uint8_t cmd = 0;
    int ret = transfer(&cmd, 1, nullptr, 0);

    if (ret != PX4_OK) {
        perf_count(_comms_errors);
		DEVICE_DEBUG("i2c::transfer returned %d", ret);
        return PX4_ERROR;
    }
    
    return PX4_OK;
}

int
ArduinoTacho::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(_conversion_interval);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();

					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					int ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(_conversion_interval)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			ATOMIC_ENTER;

			if (!_reports->resize(arg)) {
				ATOMIC_LEAVE;
				return -ENOMEM;
			}

			ATOMIC_LEAVE;

			return OK;
		}

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}


int
ArduinoTacho::collect()
{
    int ret = -EIO;
    perf_begin(_sample_perf);
    
    /* read from the sensor */
    rpmUnion val;
    
    ret = transfer(nullptr, 0, &val.rpmByte[0], sizeof(val.rpmByte));

	if (ret != PX4_OK) {
		DEVICE_DEBUG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

    // Prepare report
    struct actuator_outputs_s report;
    memset(&report, 0, sizeof(report));
    report.timestamp = hrt_absolute_time();
    report.noutputs = N_PROPS;
    /* Store RPM values to output */
	for (size_t i = 0; i < report.noutputs; i++) {
		report.output[i] = val.rpmUInt[i];
	}

    // Publish report to topic
    if (_actuator_outputs_topic != nullptr) {
        orb_publish(ORB_ID(actuator_outputs), _actuator_outputs_topic, &report);
    }
	
    // Push report to buffer
    _reports->force(&report);

    // Notify anyone waiting for data
    poll_notify(POLLIN);

    ret = PX4_OK;

    perf_end(_sample_perf);
    return ret;
}

void
ArduinoTacho::start()
{
	/* reset the report ring and state machine */
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, 
            &_work, 
            (worker_t)&ArduinoTacho::cycle_trampoline, 
            this, 
            1);
}

void
ArduinoTacho::stop()
{
    work_cancel(HPWORK, &_work);
}

void
ArduinoTacho::cycle_trampoline(void *arg)
{
	ArduinoTacho *dev = (ArduinoTacho *)arg;

	dev->cycle();
}

void
ArduinoTacho::cycle()
{
	/* Collect results */
	if (collect() != PX4_OK) {
		DEVICE_DEBUG("collection error");
		/* if error restart the measurement state machine */
		start();
		return;
	}

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&ArduinoTacho::cycle_trampoline,
		   this,
		   _measure_ticks);
}

const char
*ArduinoTacho::get_dev_name()
{
	return get_devname();
}

void
ArduinoTacho::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

