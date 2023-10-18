# I2C MPU6886 4kHz IMU Library
This library is a program for sampling at 4kHz when connecting MPU6886 to I2C.

## Support Devices
I checked the operation with M5ATOM. Probably, if the microcomputer supports I2C speed of 400kHz, it will work. Change the I2C address to one that suits your environment.

## Usage
The low-pass filter has been canceled, so be careful of noise. * Please refer to the sample program for details.
examples/I2C_MPU6886_4kHz.ino
