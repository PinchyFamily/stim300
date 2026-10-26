# Stim 300
ROS 2 driver for the Sensonor STIM300 IMU over RS422 (921600 baud). Publishes `sensor_msgs/Imu` on `<namespace>/core/imu`.

## Running
```
ros2 launch stim300 stim300_driver.launch
```
or without the launch file, which leaves the sensor as it is:
```
ros2 run stim300 stim300_driver
```

Launch arguments (see `launch/stim300_driver.launch`):

| Argument | Default | Description |
|---|---|---|
| `robot_name` | `sam` | Namespace |
| `device` | `/dev/serial/by-id/usb-FTDI_USB-RS422_Cable_FT1JNN7R-if00-port0` | Serial device. This is the by-id path of one FTDI cable; use `device:=/dev/ttyUSB0` or your own path for another |
| `sample_rate` | `250` | Sensor sample rate in Hz |
| `lp_filter` | `66` | Sensor low-pass filter -3dB frequency in Hz |
| `save_to_flash` | `false` | Also store the settings in the sensor's flash |

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `device_name` | `/dev/ttyUSB0` | Serial device |
| `configure_sample_rate` | `0` | Sample rate to set on the sensor at start-up: 125, 250, 500, 1000 or 2000. `0` leaves the sensor as it is |
| `configure_lp_filter` | `0` | LP filter -3dB frequency to set on the sensor: 16, 33, 66, 131 or 262. Applies to gyro, accelerometer, inclinometer and aux. `0` leaves it as it is |
| `save_config_to_flash` | `false` | Store the settings in the sensor's flash. See [Where the settings live](#where-the-settings-live) |
| `frame_id` | `imu_link` | `header.frame_id` of the messages |
| `imu_topic` | `core/imu` | Topic name, relative to the namespace |
| `angular_velocity_covariance` | `[2.7474e-6, 2.7474e-6, 7.312e-6]` | Diagonal [x, y, z] of the gyro covariance |
| `linear_acceleration_covariance` | `[4.1915e-4, 4.1915e-4, 1.8995e-5]` | Diagonal [x, y, z] of the accelerometer covariance |
| `gravity` | `9.80665` | m/s², used to convert the accelerometer from g |
| `sample_rate` | `125` | Loop rate until the driver has read the sensor's configuration. After that the sensor's own rate is used. Set it equal to `configure_sample_rate` |

The default covariances were estimated in a pool test of another unit. Measure your own: record the unit sitting still and use the variance of each axis.

If the sensor refuses a setting, or does not answer, the node logs the sensor's reply and exits.

Not configurable through the node: the serial speed (fixed at 921600 baud, the sensor's standard), the datagram content and output units (the parser expects rate, acceleration and inclination, in angular rate and acceleration units), and the sensor's bias trim and g-compensation.

## Setting hints

### Sample rate
- One message is published per sensor sample, so the sample rate is the publishing rate.
- The unit ships at **2000 Hz**. The driver handles one datagram per loop and cannot keep up at that rate: with this driver, 2000 Hz gave about 1300 Hz that kept falling, together with `Sync buffer trimmed to prevent overflow` messages (lost data). 250 Hz was clean: 250.0 Hz with no gaps over 10 ms in a 60 s test.
- Only 250 Hz (works) and 2000 Hz (does not keep up) have been tested. 125, 500 and 1000 Hz have not.
- A rate can be refused (error `E007`) when the bit rate and datagram content cannot carry it.

### Low-pass filter
- The unit ships at **262 Hz**. At 250 Hz sampling the Nyquist frequency is 125 Hz, so a 262 Hz filter lets through frequencies that fold back into the band (aliasing). The sensor gives no warning about this combination.
- A rule of thumb is a cut-off of about a quarter of the sample rate or less. This is not from the datasheet, which points to Sensonor's application note TS1670, "Considerations related to sampling rate and filter settings".

| Sample rate | Filter to start from |
|---|---|
| 125 Hz | 33 Hz |
| 250 Hz | 66 Hz |
| 500 Hz | 131 Hz |
| 1000 Hz and up | 262 Hz |

- A lower cut-off removes more noise and vibration but adds delay. The datasheet gives the group delay at 262 Hz as 1.5 ms for the gyros, 6.5 ms for the accelerometers and 15 ms for the inclinometers (the driver builds the orientation from the inclinometer), and says the delay changes with the filter setting.
- The gyros are not delayed to match the accelerometers by default (the gyro output unit would have to be one of the "delayed" units). At 262 Hz that is about 5 ms between gyro and accelerometer data.

### Where the settings live
- With `save_config_to_flash: false` the settings are applied through the sensor's Service Mode every time the driver starts, and last until the sensor is powered off or reset. The sensor's stored settings are not changed.
- With `true` they are also written to the sensor's flash and survive a power cycle. The flash is rated for at least 10,000 saves, so do not leave this on in a launch file that runs often.

## Dependencies
Builds on ROS 2 Jazzy. Needs `sam_msgs` (and its `smarc_msgs`) from [smarc2](https://github.com/smarc-project/smarc2) `messages/`, which supply the default topic, frame and calibration service names. Tests: `colcon test --packages-select stim300`. `check_driver_stim300` has two known failures on this branch and on upstream.

## Serial port access
The user needs to be in the `dialout` group to open the device: `sudo usermod -aG dialout $USER`, then log in again.
