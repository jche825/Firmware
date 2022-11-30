/*
 * @author  Junyi Chen <jche825@aucklanduni.ac.nz>
 * @date    14/11/22.
 */

#include <px4_workqueue.h>

#include <board_config.h>

#include <perf/perf_counter.h>

#include <drivers/drv_arduino_tacho.h>
#include <drivers/device/device.h>
#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_outputs.h>

static constexpr uint8_t        ARDUTACHO_I2C_ADDR      = 0x6F;
static constexpr uint32_t       ARDUTACHO_I2C_CLOCK     = 400000;
static constexpr const char*    ARDUTACHO_DEV_NAME      = "ArduinoTacho";

// TODO: Remove hardcoded values
static constexpr uint8_t        N_PROPS                 = 8;
static constexpr unsigned int   ARDUTACHO_POLL_TIME     = 100000;    // In microseconds (=10Hz)

union rpmUnion {
    unsigned int rpmUInt[N_PROPS];
    uint8_t rpmByte[sizeof(int) * N_PROPS];
};

class ArduinoTacho : public device::I2C
{
public:
    ArduinoTacho(const char *path, 
            int bus,
            const char *name = ARDUTACHO_DEV_NAME, 
            uint16_t address = ARDUTACHO_I2C_ADDR, 
            uint32_t frequency = ARDUTACHO_I2C_CLOCK, 
            int conversion_interval = ARDUTACHO_POLL_TIME);
    virtual     ~ArduinoTacho();
    int         init() override;
    int         ioctl(device::file_t *filp, int cmd, unsigned long arg) override;
    const char  *get_dev_name();
    void		print_info();

protected:
private:
    // Interval between sensor measurements in microseconds
    int                     _conversion_interval;
    int				        _measure_ticks;

    // Data buffers
	work_s				    _work;
    ringbuffer::RingBuffer  *_reports;

    // Instance ID variables
	int				        _class_instance;
    int				        _orb_class_instance;

    // uORB topic link
    orb_advert_t            _actuator_outputs_topic;

    // Counters
    perf_counter_t          _sample_perf;
    perf_counter_t          _comms_errors;

    /* 
    * Check for the presence/availability of the device on the bus.
    */
    int             probe() override;

    /* Collect data from sensor */
    int             collect();

    /*
     *  STATE MACHINE FUNCTIONS:
     */

    /* Initialise automatic measurement state machine and start it */
    void            start();

    /* Cycle measurement state machine */
    static void     cycle_trampoline(void *arg);
    void            cycle();

    /* Stop automatic measurement state machine */
    void            stop();


};


