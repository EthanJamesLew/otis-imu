# OTIS-IMU

OTIS-IMU is a 9/6DoF sensor fusion processor that collects data from an accelerometer, gyroscope and magnetometer. The process has a uniform sampler that is able to collect sensor information in uniform time intervals. It supports a calibration mode that allows fast correction of errors resulting from external magnetic presence. The IMU also has several filtering modes, supporting the methods

* Unscented Kalman Filter (9 DoF)
* Complementary Filter (6 DoF)
* Mahony (6 DoF)
* Madgwick (9 DoF)

## Usage

First configure your environment variables

```
# env.bash

export IDL_PATH=<path to the esp_idl folder>
export PATH=$PATH:<path to your xtensa-esp32 toolchain>
```

Add the variables to your bash session

```
source env.bash
```

Then run

```
make
make flash
```

While uploading, hit the BOOT button on the esp32s devkit module.

## Hardware

The IMU process abstracts the sensor and processor into a HAL, so new sensors can be easily supported. For reference implementation,

* ESP32S-Devkit
* FXOS8700 Accelerometer/Magnetometer
* FXAS21002 Gyroscope











