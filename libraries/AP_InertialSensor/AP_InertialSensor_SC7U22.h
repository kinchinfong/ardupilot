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
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_HAL/I2CDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

#define SC7U22_DEFAULT_ROTATION ROTATION_NONE

class AP_InertialSensor_SC7U22 : public AP_InertialSensor_Backend {
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation=SC7U22_DEFAULT_ROTATION);

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                            enum Rotation rotation=SC7U22_DEFAULT_ROTATION);

    /**
     * Configure the sensors and start reading routine.
     */
    void start() override;

    bool update() override;

private:
    AP_InertialSensor_SC7U22(AP_InertialSensor &imu,
                             AP_HAL::OwnPtr<AP_HAL::Device> dev,
                             enum Rotation rotation);

    /**
     * Try to initialize this driver.
     *
     * Do sensor and other required initializations.
     *
     * @return true on success, false otherwise.
     */
    bool _init();

    /**
     * Try to perform initialization of the SC7U22 device.
     *
     * The device semaphore must be taken and released by the caller.
     *
     * @return true on success, false otherwise.
     */
    bool _hardware_init();

    /**
     * Configure accelerometer sensor. The device semaphore must already be
     * taken before calling this function.
     *
     * @return true on success, false otherwise.
     */
    bool _configure_accel();

    /**
     * Configure gyroscope sensor. The device semaphore must already be
     * taken before calling this function.
     *
     * @return true on success, false otherwise.
     */
    bool _configure_gyro();

    /**
     * Configure FIFO.
     *
     * @return true on success, false otherwise.
     */
    bool _configure_fifo();

    /**
     * Read samples from fifo.
     */
    void _read_fifo();

    /**
     * Poll data from the sensor.
     */
    void _poll_data();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    enum Rotation _rotation;

    float _accel_scale;
    float _gyro_scale;

    uint8_t _accel_instance;
    uint8_t _gyro_instance;
};

