#ifndef __I2C_MPU6886_4kHz_H__
#define __I2C_MPU6886_4kHz_H__

#include <Wire.h>

#define I2C_MPU6886_DEFAULT_ADDRESS 0x68

class I2C_MPU6886_4kHz
{
public:
  I2C_MPU6886_4kHz(uint8_t deviceAddress = I2C_MPU6886_DEFAULT_ADDRESS, TwoWire &i2cPort = Wire);

  int begin(void);

  uint8_t whoAmI();
  uint8_t accel_config = 2;
  uint8_t gyro_config = 3;

  void getAccel_binary(int16_t *ax_binary, int16_t *ay_binary, int16_t *az_binary);
  void getAccel_x_binary(int16_t *ax_binary);
  void getAccel_y_binary(int16_t *ay_binary);
  void getAccel_z_binary(int16_t *az_binary);
  void getGyro_binary(int16_t *gx_binary, int16_t *gy_binary, int16_t *gz_binary);
  void getGyro_x_binary(int16_t *gx_binary);
  void getGyro_y_binary(int16_t *gy_binary);
  void getGyro_z_binary(int16_t *gz_binary);

  void getAccel(float *ax, float *ay, float *az);
  void getAccel_x(float *ax);
  void getAccel_y(float *ay);
  void getAccel_z(float *az);
  void getGyro(float *gx, float *gy, float *gz);
  void getGyro_x(float *gx);
  void getGyro_y(float *gy);
  void getGyro_z(float *gz);

  void getTemp_binary(int16_t *t_binary);

  void getTemp(float *t);
  void disableGyro();
  void enableGyro();
  void disableAccel();
  void enableAccel();

  void resetDevice();
  void setSleepMode(bool mode);
  void setCycleMode(bool mode);
  void setGyroStandbyMode(bool mode);
  void disableTemperatureSensor(bool mode);

private:
  uint8_t readByte(uint8_t address);
  void writeByte(uint8_t address, uint8_t data);
  void bitOn(uint8_t address, uint8_t bit);
  void bitOff(uint8_t address, uint8_t bit);

  TwoWire *_i2cPort;
  int _deviceAddress;

  float _aRes;
  float _gRes;
};

#endif
