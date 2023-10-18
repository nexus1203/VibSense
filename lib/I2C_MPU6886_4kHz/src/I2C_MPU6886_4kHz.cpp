#include "I2C_MPU6886_4kHz.h"

I2C_MPU6886_4kHz::I2C_MPU6886_4kHz(uint8_t deviceAddress, TwoWire &i2cPort)
{
  _deviceAddress = deviceAddress;
  _i2cPort = &i2cPort;
}

uint8_t I2C_MPU6886_4kHz::readByte(uint8_t address)
{
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(address);
  _i2cPort->endTransmission();
  _i2cPort->requestFrom(_deviceAddress, 1);
  uint8_t val = _i2cPort->read();

  ESP_LOGD("MPU6886", "readByte(%02X) = %02X", address, val);
  return val;
}

void I2C_MPU6886_4kHz::writeByte(uint8_t address, uint8_t data)
{
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(address);
  _i2cPort->write(data);
  _i2cPort->endTransmission();
  ESP_LOGD("MPU6886", "writeByte(%02X) = %02X", address, data);
}

void I2C_MPU6886_4kHz::bitOn(uint8_t address, uint8_t bit)
{
  uint8_t add = address;
  uint8_t val = readByte(add) | bit;
  writeByte(add, val);
}

void I2C_MPU6886_4kHz::bitOff(uint8_t address, uint8_t bit)
{
  uint8_t add = address;
  uint8_t val = readByte(add) & ~bit;
  writeByte(add, val);
}

int I2C_MPU6886_4kHz::begin(void)
{
  // WHO_AM_I : IMU Check
  if (whoAmI() != 0x19)
  {
    return -1;
  }
  delay(1);

  // PWR_MGMT_1(0x6b)
  writeByte(0x6b, 0x00);
  delay(10);

  // PWR_MGMT_1(0x6b)
  writeByte(0x6b, 1 << 7);
  delay(10);

  // PWR_MGMT_1(0x6b)
  writeByte(0x6b, 1 << 0);
  delay(10);

  // ACCEL_CONFIG(0x1c) : +-8G
  // 0:2g 8:4g 10:8g 18:16g
  switch (accel_config)
  {
  case 0:
    _aRes = 2.0 / 32768.0;
    writeByte(0x1c, 0x00);
    break;
  case 1:
    _aRes = 4.0 / 32768.0;
    writeByte(0x1c, 0x08);
    break;
  case 3:
    _aRes = 16.0 / 32768.0;
    writeByte(0x1c, 0x18);
    break;
  default:
    _aRes = 8.0 / 32768.0;
    writeByte(0x1c, 0x10);
    break;
  }
  delay(1);

  // GYRO_CONFIG(0x1b) : +-2000dps
  // 0:250dps 8:500dps 10:1000dps 18:2000dps
  switch (gyro_config)
  {
  case 0:
    _gRes = 250.0 / 32768.0;
    writeByte(0x1b, 0x00);
    break;
  case 1:
    _gRes = 500.0 / 32768.0;
    writeByte(0x1b, 0x08);
    break;
  case 2:
    _gRes = 1000.0 / 32768.0;
    writeByte(0x1b, 0x10);
    break;
  default:
    _gRes = 2000.0 / 32768.0;
    writeByte(0x1b, 0x18);
  }
  delay(1);

  // CONFIG(0x1a) Lowpass filter settings
  writeByte(0x1a, 0x07);
  delay(1);

  // SMPLRT_DIV(0x19) Number of samples used for averaging (1 + set value)
  writeByte(0x19, 0x00);
  delay(1);

  // INT_ENABLE(0x38)
  writeByte(0x38, 0x00);
  delay(1);

  // ACCEL_CONFIG 2(0x1d) Lowpass filter bypass setting
  writeByte(0x1d, 0x08);
  delay(1);

  // USER_CTRL(0x6a)
  writeByte(0x6a, 0x00);
  delay(1);

  // FIFO_EN(0x23)
  writeByte(0x23, 0x00);
  delay(1);

  // INT_PIN_CFG(0x37)
  writeByte(0x37, 0x22);
  delay(1);

  // INT_ENABLE(0x38)
  writeByte(0x38, 0x01);
  delay(100);

  return 0;
}

uint8_t I2C_MPU6886_4kHz::whoAmI(void)
{
  return readByte(0x75);
}

void I2C_MPU6886_4kHz::getAccel_binary(int16_t *ax_binary, int16_t *ay_binary, int16_t *az_binary)
{
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(0x3b);
  _i2cPort->endTransmission();
  _i2cPort->requestFrom(_deviceAddress, 6);

  uint8_t val[6];
  for (int i = 0; i < 6; i++)
  {
    val[i] = _i2cPort->read();
  }

  *ax_binary = (int16_t)((val[0] << 8) | val[1]);
  *ay_binary = (int16_t)((val[2] << 8) | val[3]);
  *az_binary = (int16_t)((val[4] << 8) | val[5]);
}

void I2C_MPU6886_4kHz::getAccel(float *ax, float *ay, float *az)
{
  // float aRes = 8.0 / 32768.0;
  int16_t ax_binary;
  int16_t ay_binary;
  int16_t az_binary;
  getAccel_binary(&ax_binary, &ay_binary, &az_binary);
  *ax = ax_binary * _aRes;
  *ay = ay_binary * _aRes;
  *az = az_binary * _aRes;
}

void I2C_MPU6886_4kHz::getAccel_x_binary(int16_t *ax_binary)
{
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(0x3b);
  _i2cPort->endTransmission();
  _i2cPort->requestFrom(_deviceAddress, 2);
  uint8_t val1 = _i2cPort->read();
  uint8_t val2 = _i2cPort->read();
  *ax_binary = (int16_t)((val1 << 8) | val2);
}

void I2C_MPU6886_4kHz::getAccel_x(float *ax)
{
  // float aRes = 8.0 / 32768.0;
  int16_t ax_binary;
  getAccel_x_binary(&ax_binary);
  *ax = ax_binary * _aRes;
}

void I2C_MPU6886_4kHz::getAccel_y_binary(int16_t *ay_binary)
{
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(0x3d);
  _i2cPort->endTransmission();
  _i2cPort->requestFrom(_deviceAddress, 2);
  uint8_t val1 = _i2cPort->read();
  uint8_t val2 = _i2cPort->read();
  *ay_binary = (int16_t)((val1 << 8) | val2);
}

void I2C_MPU6886_4kHz::getAccel_y(float *ay)
{
  // float aRes = 8.0 / 32768.0;
  int16_t ay_binary;
  getAccel_y_binary(&ay_binary);
  *ay = ay_binary * _aRes;
}

void I2C_MPU6886_4kHz::getAccel_z_binary(int16_t *az_binary)
{
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(0x3f);
  _i2cPort->endTransmission();
  _i2cPort->requestFrom(_deviceAddress, 2);
  uint8_t val1 = _i2cPort->read();
  uint8_t val2 = _i2cPort->read();
  *az_binary = (int16_t)((val1 << 8) | val2);
}

void I2C_MPU6886_4kHz::getAccel_z(float *az)
{
  // float aRes = 8.0 / 32768.0;
  int16_t az_binary;
  getAccel_z_binary(&az_binary);
  *az = az_binary * _aRes;
}

void I2C_MPU6886_4kHz::getGyro_binary(int16_t *gx_binary, int16_t *gy_binary, int16_t *gz_binary)
{
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(0x43);
  _i2cPort->endTransmission();
  _i2cPort->requestFrom(_deviceAddress, 6);

  uint8_t val[6];
  for (int i = 0; i < 6; i++)
  {
    val[i] = _i2cPort->read();
  }

  *gx_binary = (int16_t)((val[0] << 8) | val[1]);
  *gy_binary = (int16_t)((val[2] << 8) | val[3]);
  *gz_binary = (int16_t)((val[4] << 8) | val[5]);
}

void I2C_MPU6886_4kHz::getGyro(float *gx, float *gy, float *gz)
{
  // float gRes = 2000.0 / 32768.0;
  int16_t gx_binary;
  int16_t gy_binary;
  int16_t gz_binary;
  getGyro_binary(&gx_binary, &gy_binary, &gz_binary);
  *gx = gx_binary * _gRes;
  *gy = gy_binary * _gRes;
  *gz = gz_binary * _gRes;
}

void I2C_MPU6886_4kHz::getGyro_x_binary(int16_t *gx_binary)
{
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(0x43);
  _i2cPort->endTransmission();
  _i2cPort->requestFrom(_deviceAddress, 2);
  uint8_t val1 = _i2cPort->read();
  uint8_t val2 = _i2cPort->read();
  *gx_binary = (int16_t)((val1 << 8) | val2);
}

void I2C_MPU6886_4kHz::getGyro_x(float *gx)
{
  // float gRes = 2000.0 / 32768.0;
  int16_t gx_binary;
  getGyro_x_binary(&gx_binary);
  *gx = gx_binary * _gRes;
}

void I2C_MPU6886_4kHz::getGyro_y_binary(int16_t *gy_binary)
{
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(0x45);
  _i2cPort->endTransmission();
  _i2cPort->requestFrom(_deviceAddress, 2);
  uint8_t val1 = _i2cPort->read();
  uint8_t val2 = _i2cPort->read();
  *gy_binary = (int16_t)((val1 << 8) | val2);
}

void I2C_MPU6886_4kHz::getGyro_y(float *gy)
{
  // float gRes = 2000.0 / 32768.0;
  int16_t gy_binary;
  getGyro_y_binary(&gy_binary);
  *gy = gy_binary * _gRes;
}

void I2C_MPU6886_4kHz::getGyro_z_binary(int16_t *gz_binary)
{
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(0x47);
  _i2cPort->endTransmission();
  _i2cPort->requestFrom(_deviceAddress, 2);
  uint8_t val1 = _i2cPort->read();
  uint8_t val2 = _i2cPort->read();
  *gz_binary = (int16_t)((val1 << 8) | val2);
}

void I2C_MPU6886_4kHz::getGyro_z(float *gz)
{
  // float gRes = 2000.0 / 32768.0;
  int16_t gz_binary;
  getGyro_x_binary(&gz_binary);
  *gz = gz_binary * _gRes;
}

void I2C_MPU6886_4kHz::getTemp_binary(int16_t *t_binary)
{
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(0x41);
  _i2cPort->endTransmission();
  _i2cPort->requestFrom(_deviceAddress, 2);
  uint8_t val1 = _i2cPort->read();
  uint8_t val2 = _i2cPort->read();
  *t_binary = (int16_t)((val1 << 8) | val2);
}

void I2C_MPU6886_4kHz::getTemp(float *t)
{
  int16_t t_binary;
  getTemp_binary(&t_binary);
  *t = 25.0 + t_binary / 326.8;
}

void I2C_MPU6886_4kHz::disableGyro()
{
  uint8_t currentData = readByte(0x6C); // Read the current data from PWR_MGMT_2
  currentData |= 0b00000111;            // Set STBY_XG, STBY_YG, STBY_ZG bits to 1
  writeByte(0x6C, currentData);         // Write the modified data back to PWR_MGMT_2
}

void I2C_MPU6886_4kHz::enableGyro()
{
  uint8_t currentData = readByte(0x6C);
  currentData &= ~0b00000111; // Clear STBY_XG, STBY_YG, STBY_ZG bits to 0
  writeByte(0x6C, currentData);
}

void I2C_MPU6886_4kHz::disableAccel()
{
  uint8_t currentData = readByte(0x6C);
  currentData |= 0b00111000; // Set STBY_XA, STBY_YA, STBY_ZA bits to 1
  writeByte(0x6C, currentData);
}

void I2C_MPU6886_4kHz::enableAccel()
{
  uint8_t currentData = readByte(0x6C);
  currentData &= ~0b00111000; // Clear STBY_XA, STBY_YA, STBY_ZA bits to 0
  writeByte(0x6C, currentData);
}

void I2C_MPU6886_4kHz::resetDevice()
{
  uint8_t currentData = readByte(0x6B);
  currentData |= (1 << 7); // Set the DEVICE_RESET bit
  writeByte(0x6B, currentData);
  // This bit will auto-clear once reset is done
}

void I2C_MPU6886_4kHz::setSleepMode(bool mode)
{
  uint8_t currentData = readByte(0x6B);
  if (mode)
    currentData |= (1 << 6); // Set the SLEEP bit
  else
    currentData &= ~(1 << 6); // Clear the SLEEP bit
  writeByte(0x6B, currentData);
}

void I2C_MPU6886_4kHz::setCycleMode(bool mode)
{
  uint8_t currentData = readByte(0x6B);
  if (mode)
    currentData |= (1 << 5); // Set the CYCLE bit
  else
    currentData &= ~(1 << 5); // Clear the CYCLE bit
  writeByte(0x6B, currentData);
}

void I2C_MPU6886_4kHz::setGyroStandbyMode(bool mode)
{
  uint8_t currentData = readByte(0x6B);
  if (mode)
    currentData |= (1 << 4); // Set the GYRO_STANDBY bit
  else
    currentData &= ~(1 << 4); // Clear the GYRO_STANDBY bit
  writeByte(0x6B, currentData);
}

void I2C_MPU6886_4kHz::disableTemperatureSensor(bool mode)
{
  uint8_t currentData = readByte(0x6B);
  if (mode)
    currentData |= (1 << 3); // Set the TEMP_DIS bit
  else
    currentData &= ~(1 << 3); // Clear the TEMP_DIS bit
  writeByte(0x6B, currentData);
}