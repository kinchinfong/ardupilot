/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <utility>

#include <AP_HAL/AP_HAL.h>

#include <GCS_MAVLink/GCS.h>

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>

#include "AP_InertialSensor_SC7U22.h"

/* Registers and bits definitions */
#define SC7U22_REG_WHO_AM_I         0x01
#define     SC7U22_CHIP_ID          0x6A
#define SC7U22_REG_COM_CONF         0x04
#define     SC7U22_COM_CONF_DEFAULT 0x50
#define     SC7U22_COM_CONF_BOOT    0x80
#define SC7U22_REG_DATA_STAT        0x0B
#define SC7U22_REG_ACC_XH           0x0C
#define SC7U22_REG_ACC_XL           0x0D
#define SC7U22_REG_GYR_XH           0x12
#define SC7U22_REG_GYR_XL           0x13
#define SC7U22_REG_FIFO_CFG0        0x1C
#define SC7U22_REG_FIFO_CFG1        0x1D
#define SC7U22_REG_FIFO_CFG2        0x1E
#define SC7U22_REG_FIFO_STAT0       0x1F
#define SC7U22_REG_FIFO_STAT1       0x20
#define SC7U22_REG_FIFO_DATA        0x21
#define SC7U22_REG_TEMP_H           0x22
#define SC7U22_REG_TEMP_L           0x23
#define SC7U22_REG_ACC_CONF         0x40
#define SC7U22_REG_ACC_RANGE        0x41
#define SC7U22_REG_GYR_CONF         0x42
#define SC7U22_REG_GYR_RANGE        0x43
#define SC7U22_REG_FIFO_DOWNS       0x45
#define SC7U22_REG_SOFT_RST         0x4A
#define SC7U22_REG_PWR_CTRL         0x7D
#define SC7U22_REG_SEG_SEL          0x7F

/* ACC_RANGE register bits */
#define SC7U22_ACC_RANGE_2G         0x00
#define SC7U22_ACC_RANGE_4G         0x01
#define SC7U22_ACC_RANGE_8G         0x02
#define SC7U22_ACC_RANGE_16G        0x03

/* GYR_RANGE register bits */
#define SC7U22_GYR_RANGE_2000DPS    0x00
#define SC7U22_GYR_RANGE_1000DPS    0x01
#define SC7U22_GYR_RANGE_500DPS     0x02
#define SC7U22_GYR_RANGE_250DPS     0x03
#define SC7U22_GYR_RANGE_125DPS     0x04

/* FIFO enable bits */
#define SC7U22_FIFO_TEMP_EN         0x08
#define SC7U22_FIFO_ACC_EN          0x04
#define SC7U22_FIFO_GYR_EN          0x02
#define SC7U22_FIFO_HEADER_EN       0x01

/* PWR_CTRL bits */
#define SC7U22_ACC_EN               0x04
#define SC7U22_GYR_EN               0x02
#define SC7U22_TEMP_EN              0x08

/* COM_CONF bits */
#define SC7U22_ADDR_AUTO            0x10
#define SC7U22_BDU                  0x40

/* Soft reset command */
#define SC7U22_CMD_SOFTRESET        0xA5

/* SPI mode unlock command (from vendor driver) */
#define SC7U22_SPI_UNLOCK_CMD       0x66

/* Special register segment 1 for I2C disable */
#define SC7U22_SEG_SEL_SPECIAL1     0x83
#define SC7U22_REG_I2C_DISABLE      0x6F
#define SC7U22_I2C_DISABLE_VAL      0x04

/* Sample rate */
#define SC7U22_BACKEND_SAMPLE_RATE  1600

/* Datasheet says power-up time is about 50ms, but vendor code uses 200ms after reset */
#define SC7U22_POWERUP_DELAY_MSEC   50
#define SC7U22_SOFTRESET_DELAY_MSEC 200

/* Maximum number of samples to read from FIFO at once */
#define SC7U22_MAX_FIFO_SAMPLES     8

#define SC7U22_READ_FLAG            0x80
#define SC7U22_HARDWARE_INIT_MAX_TRIES 5

extern const AP_HAL::HAL& hal;

/*
 * FIFO data format (without header mode):
 * According to vendor driver and datasheet register description,
 * data order is: ACC_X, ACC_Y, ACC_Z, GYR_X, GYR_Y, GYR_Z
 */
struct PACKED RawData {
    struct {
        be16_t x;
        be16_t y;
        be16_t z;
    } accel;  // ACC data comes first (offset 0-5)
    struct {
        be16_t x;
        be16_t y;
        be16_t z;
    } gyro;   // GYR data comes second (offset 6-11)
};

AP_InertialSensor_SC7U22::AP_InertialSensor_SC7U22(AP_InertialSensor &imu,
                                                   AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                   enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , _rotation(rotation)
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_SC7U22::probe(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    auto sensor = NEW_NOTHROW AP_InertialSensor_SC7U22(imu, std::move(dev), rotation);

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->_init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_InertialSensor_Backend *
AP_InertialSensor_SC7U22::probe(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    auto sensor = NEW_NOTHROW AP_InertialSensor_SC7U22(imu, std::move(dev), rotation);

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->_init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_SC7U22::start()
{
    bool r;

    _dev->get_semaphore()->take_blocking();

    r = _configure_accel();
    if (!r) {
        AP_HAL::panic("SC7U22: Unable to configure accelerometer");
    }

    r = _configure_gyro();
    if (!r) {
        AP_HAL::panic("SC7U22: Unable to configure gyroscope");
    }

    r = _configure_fifo();
    if (!r) {
        AP_HAL::panic("SC7U22: Unable to configure FIFO");
    }

    _dev->get_semaphore()->give();

    if (!_imu.register_accel(_accel_instance, SC7U22_BACKEND_SAMPLE_RATE, _dev->get_bus_id_devtype(DEVTYPE_INS_SC7U22)) ||
        !_imu.register_gyro(_gyro_instance, SC7U22_BACKEND_SAMPLE_RATE, _dev->get_bus_id_devtype(DEVTYPE_INS_SC7U22))) {
        return;
    }

    // setup sensor rotations from probe()
    set_gyro_orientation(_gyro_instance, _rotation);
    set_accel_orientation(_accel_instance, _rotation);

    /* Call _poll_data() at 1kHz */
    _dev->register_periodic_callback(1000,
        FUNCTOR_BIND_MEMBER(&AP_InertialSensor_SC7U22::_poll_data, void));
}

bool AP_InertialSensor_SC7U22::update()
{
    update_accel(_accel_instance);
    update_gyro(_gyro_instance);
    return true;
}

bool AP_InertialSensor_SC7U22::_configure_accel()
{
    bool r;

    // Set accelerometer to high performance mode, 1600Hz ODR
    // ACC_FILTER_PERF=1 (high performance), ACC_ODR=1100 (1600Hz), ACC_BWP=010 (NORM_AVG4)
    r = _dev->write_register(SC7U22_REG_ACC_CONF, 0xAC);
    if (!r) {
        return false;
    }
    hal.scheduler->delay(1);

    // Set accelerometer range to ±16G
    r = _dev->write_register(SC7U22_REG_ACC_RANGE, SC7U22_ACC_RANGE_16G);
    if (!r) {
        return false;
    }
    hal.scheduler->delay(1);

    // Calculate scale for ±16G range with 16-bit resolution
    // Sensitivity: 0.488 mg/LSB for ±16G
    _accel_scale = GRAVITY_MSS * 16.0f / 32768.0f;

    return true;
}

bool AP_InertialSensor_SC7U22::_configure_gyro()
{
    bool r;

    // Set gyroscope to high performance mode, 1600Hz ODR
    // GYR_FILTER_PERF=1, GYR_NOISE_PERF=1, GYR_ODR=1100 (1600Hz), GYR_BWP=10
    r = _dev->write_register(SC7U22_REG_GYR_CONF, 0xCC);
    if (!r) {
        return false;
    }
    hal.scheduler->delay(1);

    // Set gyroscope range to ±2000dps
    r = _dev->write_register(SC7U22_REG_GYR_RANGE, SC7U22_GYR_RANGE_2000DPS);
    if (!r) {
        return false;
    }
    hal.scheduler->delay(1);

    // Calculate scale for ±2000dps range with 16-bit resolution
    // Sensitivity: 61 mdps/LSB for ±2000dps
    _gyro_scale = radians(2000.0f) / 32768.0f;

    return true;
}

bool AP_InertialSensor_SC7U22::_configure_fifo()
{
    bool r;

    // Enable FIFO for accelerometer and gyroscope, no header
    r = _dev->write_register(SC7U22_REG_FIFO_CFG0, SC7U22_FIFO_ACC_EN | SC7U22_FIFO_GYR_EN);
    if (!r) {
        DEV_PRINTF("SC7U22: Unable to enable FIFO\n");
        return false;
    }
    hal.scheduler->delay(1);

    // Set FIFO to Stream mode (0x02 << 4) with watermark at 12 bytes (1 sample)
    r = _dev->write_register(SC7U22_REG_FIFO_CFG1, (0x02 << 4) | 0x00);
    if (!r) {
        DEV_PRINTF("SC7U22: Unable to configure FIFO mode\n");
        return false;
    }
    hal.scheduler->delay(1);

    // Set FIFO watermark threshold (lower 8 bits) - 96 bytes = 8 samples
    r = _dev->write_register(SC7U22_REG_FIFO_CFG2, 0x30);
    if (!r) {
        DEV_PRINTF("SC7U22: Unable to set FIFO watermark\n");
        return false;
    }
    hal.scheduler->delay(1);

    // Configure FIFO downsampling - no downsampling, filtered data
    r = _dev->write_register(SC7U22_REG_FIFO_DOWNS, 0x88);
    if (!r) {
        DEV_PRINTF("SC7U22: Unable to configure FIFO downsampling\n");
        return false;
    }
    hal.scheduler->delay(1);

    return true;
}

void AP_InertialSensor_SC7U22::_read_fifo()
{
    struct RawData raw_data[SC7U22_MAX_FIFO_SAMPLES];
    uint16_t num_bytes;
    uint16_t excess;
    uint8_t num_samples = 0;
    bool r = true;

    static_assert(sizeof(raw_data) <= 100, "Too big to keep on stack");

    // Read FIFO length
    uint8_t fifo_stat[2];
    r = _dev->read_registers(SC7U22_REG_FIFO_STAT0, fifo_stat, 2);
    if (!r) {
        goto read_fifo_end;
    }

    // Check if FIFO is empty
    if (fifo_stat[0] & 0x40) {  // EMPTY bit
        goto read_fifo_end;
    }

    // Extract FIFO sample count (in bytes, each entry is 2 bytes)
    num_bytes = ((fifo_stat[0] & 0x0F) << 8) | fifo_stat[1];
    num_bytes *= 2;  // Convert to actual bytes

    if (!num_bytes) {
        goto read_fifo_end;
    }

read_fifo_read_data:
    if (num_bytes > sizeof(raw_data)) {
        excess = num_bytes - sizeof(raw_data);
        num_bytes = sizeof(raw_data);
    } else {
        excess = 0;
    }

    r = _dev->read_registers(SC7U22_REG_FIFO_DATA,
                             (uint8_t *)raw_data,
                             num_bytes);
    if (!r) {
        goto read_fifo_end;
    }

    /* Read again just once */
    if (excess && num_samples) {
        DEV_PRINTF("SC7U22: dropping %u samples from fifo\n",
                            (uint8_t)(excess / sizeof(struct RawData)));
        // Reset FIFO by switching to bypass and back to stream mode
        _dev->write_register(SC7U22_REG_FIFO_CFG1, 0x00);
        hal.scheduler->delay_microseconds(100);
        _dev->write_register(SC7U22_REG_FIFO_CFG1, (0x02 << 4));
        excess = 0;
    }

    num_samples = num_bytes / sizeof(struct RawData);
    for (uint8_t i = 0; i < num_samples; i++) {
        // SC7U22 FIFO format: ACC_X, ACC_Y, ACC_Z, GYR_X, GYR_Y, GYR_Z
        Vector3f accel{(float)(int16_t)be16toh(raw_data[i].accel.x),
                       (float)(int16_t)be16toh(raw_data[i].accel.y),
                       (float)(int16_t)be16toh(raw_data[i].accel.z)};
        Vector3f gyro{(float)(int16_t)be16toh(raw_data[i].gyro.x),
                      (float)(int16_t)be16toh(raw_data[i].gyro.y),
                      (float)(int16_t)be16toh(raw_data[i].gyro.z)};

        gyro.rotate(_rotation);
        accel.rotate(_rotation);

        gyro *= _gyro_scale;
        accel *= _accel_scale;

        _rotate_and_correct_gyro(_gyro_instance, gyro);
        _rotate_and_correct_accel(_accel_instance, accel);

        _notify_new_gyro_raw_sample(_gyro_instance, gyro);
        _notify_new_accel_raw_sample(_accel_instance, accel);
    }

    if (excess) {
        num_bytes = excess;
        goto read_fifo_read_data;
    }

read_fifo_end:
    if (!r) {
        DEV_PRINTF("SC7U22: error on reading FIFO\n");
    }
}

void AP_InertialSensor_SC7U22::_poll_data()
{
    _read_fifo();
}

bool AP_InertialSensor_SC7U22::_hardware_init()
{
    bool ret = false;

    hal.scheduler->delay(SC7U22_POWERUP_DELAY_MSEC);

    _dev->get_semaphore()->take_blocking();

    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    /*
     * SPI mode initialization: disable I2C interface
     * This is required for proper SPI communication according to vendor driver.
     * Reference: SL_SC7U22_AOI_Angle_Driver.c lines 57-70
     * 
     * Note: For SPI devices, the I2C interface must be disabled first.
     * This sequence writes to special register segment 0x83 to disable I2C.
     */
    // Step 0-1: Enter normal register segment
    _dev->write_register(SC7U22_REG_SEG_SEL, 0x00);
    hal.scheduler->delay(1);

    // Step 0-2: Write unlock command to enable special register access
    _dev->write_register(SC7U22_REG_SOFT_RST, SC7U22_SPI_UNLOCK_CMD);
    hal.scheduler->delay(1);

    // Step 0-3: Enter special register segment 1 (0x83)
    _dev->write_register(SC7U22_REG_SEG_SEL, SC7U22_SEG_SEL_SPECIAL1);
    hal.scheduler->delay(1);

    // Step 0-4: Disable I2C interface
    _dev->write_register(SC7U22_REG_I2C_DISABLE, SC7U22_I2C_DISABLE_VAL);
    hal.scheduler->delay(1);

    // Step 0-5: Exit special register segment
    _dev->write_register(SC7U22_REG_SEG_SEL, 0x00);
    hal.scheduler->delay(1);

    // Step 0-6: Clear unlock command
    _dev->write_register(SC7U22_REG_SOFT_RST, 0x00);
    hal.scheduler->delay(1);

    for (unsigned i = 0; i < SC7U22_HARDWARE_INIT_MAX_TRIES; i++) {
        uint8_t v;

        // Step 1: Exit special register segment (ensure we're in normal register space)
        ret = _dev->write_register(SC7U22_REG_SEG_SEL, 0x00);
        if (!ret) {
            continue;
        }
        hal.scheduler->delay(1);

        // Step 2: Power down all sensors first (as per vendor driver)
        ret = _dev->write_register(SC7U22_REG_PWR_CTRL, 0x00);
        if (!ret) {
            continue;
        }
        hal.scheduler->delay(10);

        // Step 3: Set Addr_Auto bit before reset (as per vendor driver)
        ret = _dev->write_register(SC7U22_REG_COM_CONF, SC7U22_ADDR_AUTO);
        if (!ret) {
            continue;
        }
        hal.scheduler->delay(1);

        // Step 4: Perform soft reset (write twice as per datasheet)
        ret = _dev->write_register(SC7U22_REG_SOFT_RST, SC7U22_CMD_SOFTRESET);
        if (!ret) {
            continue;
        }
        hal.scheduler->delay(1);

        ret = _dev->write_register(SC7U22_REG_SOFT_RST, SC7U22_CMD_SOFTRESET);
        if (!ret) {
            continue;
        }

        // Step 5: Wait for reset to complete (vendor driver uses 200ms)
        hal.scheduler->delay(SC7U22_SOFTRESET_DELAY_MSEC);

        // Step 6: Check chip ID
        ret = _dev->read_registers(SC7U22_REG_WHO_AM_I, &v, 1);
        if (!ret) {
            continue;
        }
        if (v != SC7U22_CHIP_ID) {
            DEV_PRINTF("SC7U22: unexpected chip ID 0x%02x\n", v);
            ret = false;
            continue;
        }

        // Step 7: Verify COM_CONF has reset to default value (0x50)
        ret = _dev->read_registers(SC7U22_REG_COM_CONF, &v, 1);
        if (!ret) {
            continue;
        }
        if (v != SC7U22_COM_CONF_DEFAULT) {
            DEV_PRINTF("SC7U22: COM_CONF not reset properly, got 0x%02x\n", v);
            ret = false;
            continue;
        }

        // Step 8: Configure COM_CONF - enable address auto-increment and block data update
        ret = _dev->write_register(SC7U22_REG_COM_CONF, SC7U22_ADDR_AUTO | SC7U22_BDU);
        if (!ret) {
            continue;
        }
        hal.scheduler->delay(1);

        // Step 9: Enable accelerometer, gyroscope and temperature sensor
        ret = _dev->write_register(SC7U22_REG_PWR_CTRL, SC7U22_ACC_EN | SC7U22_GYR_EN | SC7U22_TEMP_EN);
        if (!ret) {
            continue;
        }
        hal.scheduler->delay(10);

        // Step 10: Write PWR_CTRL again to ensure it takes effect (as per vendor driver)
        ret = _dev->write_register(SC7U22_REG_PWR_CTRL, SC7U22_ACC_EN | SC7U22_GYR_EN | SC7U22_TEMP_EN);
        if (!ret) {
            continue;
        }
        hal.scheduler->delay(10);

        DEV_PRINTF("SC7U22 initialized after %d retries\n", i+1);
        break;
    }

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    _dev->get_semaphore()->give();

    if (!ret) {
        DEV_PRINTF("SC7U22: failed to init\n");
    }

    return ret;
}

bool AP_InertialSensor_SC7U22::_init()
{
    _dev->set_read_flag(SC7U22_READ_FLAG);

    bool ret = _hardware_init();
    if (!ret) {
        DEV_PRINTF("SC7U22: failed to init\n");
    }

    return ret;
}
