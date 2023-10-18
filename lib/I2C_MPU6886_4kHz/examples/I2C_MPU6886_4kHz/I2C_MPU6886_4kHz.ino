// I2C_MPU6886_4kHz sample program
// Sampling speed at I2C 400kHz
// 1-axis reading MAX 4KHz
// 3-axis simultaneous reading MAX 2875kHz
// 6-axis simultaneous reading MAX1440kHz

// Sample quantity
#define SAMPLE 2048

#include "I2C_MPU6886_4kHz.h"

I2C_MPU6886_4kHz imu(I2C_MPU6886_DEFAULT_ADDRESS, Wire1);

float sample_value[SAMPLE];    //For value storage
int16_t sample_binary[SAMPLE]; //For value storage

void value_read()
{
  // For value storage(float)
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float t;

  // 1-axis reading
  imu.getAccel_x(&ax);
  imu.getAccel_y(&ay);
  imu.getAccel_z(&az);
  imu.getGyro_x(&gx);
  imu.getGyro_y(&gy);
  imu.getGyro_z(&gz);
  imu.getTemp(&t);

  Serial.printf("%f,%f,%f,%f,%f,%f,%f\n", ax, ay, az, gx, gy, gz, t);

  // 3-axis reading
  imu.getAccel(&ax, &ay, &az);
  imu.getGyro(&gx, &gy, &gz);

  Serial.printf("%f,%f,%f,%f,%f,%f,%f\n", ax, ay, az, gx, gy, gz, t);
}

void binary_read()
{
  // For value storage (binary)
  int16_t ax_binary;
  int16_t ay_binary;
  int16_t az_binary;
  int16_t gx_binary;
  int16_t gy_binary;
  int16_t gz_binary;
  int16_t t_binary;

  imu.getAccel_x_binary(&ax_binary);
  imu.getAccel_y_binary(&ay_binary);
  imu.getAccel_z_binary(&az_binary);
  imu.getGyro_x_binary(&gx_binary);
  imu.getGyro_y_binary(&gy_binary);
  imu.getGyro_z_binary(&gz_binary);
  imu.getTemp_binary(&t_binary);

  Serial.print(ax_binary);
  Serial.print(F(","));
  Serial.print(ay_binary);
  Serial.print(F(","));
  Serial.print(az_binary);
  Serial.print(F(","));
  Serial.print(gx_binary);
  Serial.print(F(","));
  Serial.print(gy_binary);
  Serial.print(F(","));
  Serial.print(gz_binary);
  Serial.print(F(","));
  Serial.println(t_binary);

  imu.getAccel_binary(&ax_binary, &ay_binary, &az_binary);
  imu.getGyro_binary(&gx_binary, &gy_binary, &gz_binary);

  Serial.print(ax_binary);
  Serial.print(F(","));
  Serial.print(ay_binary);
  Serial.print(F(","));
  Serial.print(az_binary);
  Serial.print(F(","));
  Serial.print(gx_binary);
  Serial.print(F(","));
  Serial.print(gy_binary);
  Serial.print(F(","));
  Serial.print(gz_binary);
  Serial.print(F(","));
  Serial.println(t_binary);
}

void sampling()
{

  unsigned long start_time = micros(); // Keep start time

  // For numbers
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float t;

  // For binary
  int16_t ax_binary;
  int16_t ay_binary;
  int16_t az_binary;
  int16_t gx_binary;
  int16_t gy_binary;
  int16_t gz_binary;
  int16_t t_binary;

  for (int i = 0; i < SAMPLE; i++)
  {

    // 1-axis numerical value
    imu.getAccel_x(&ax);
    //imu.getAccel_y(&ay);
    //imu.getAccel_z(&az);
    //imu.getGyro_x(&gx);
    //imu.getGyro_y(&gy);
    //imu.getGyro_z(&gz);
    //imu.getTemp(&t);

    // 3-axis numerical value
    //imu.getAccel(&ax, &ay, &az);
    //imu.getGyro(&gx, &gy, &gz);

    // 1 axis binary
    //imu.getAccel_x_binary(&ax_binary);
    //imu.getAccel_y_binary(&ay_binary);
    //imu.getAccel_z_binary(&az_binary);
    //imu.getGyro_x_binary(&gx_binary);
    //imu.getGyro_y_binary(&gy_binary);
    //imu.getGyro_z_binary(&gz_binary);
    //imu.getTemp_binary(&t_binary);

    // 3-axis binary
    //imu.getAccel_binary(&ax_binary, &ay_binary, &az_binary);
    //imu.getGyro_binary(&gx_binary, &gy_binary, &gz_binary);

    // If you want to display the value, uncomment it and put the variable
    //sample_value[i] = ax;
    //sample_binary[i] = t_binary;
  }
  unsigned long end_time = micros(); // Keep the end time

  // For checking the sampling cycle
  Serial.print(F("Sampling frequency : "));
  Serial.println(1000000.0 / float(end_time - start_time) * SAMPLE);

  // Uncomment below to see the value
  for (int i = 0; i < SAMPLE; i++)
  {
    //Serial.println(sample_value[i], 4);
    //Serial.println(sample_binary[i]);
  }
}

void setup()
{
  Serial.begin(115200);

  delay(100);

  Wire1.begin(25, 21);    // I2C pin settings
  Wire1.setClock(400000); // Set the I2C speed to 400kHz

  // Sensor range setting (do before begin)
  // ACCEL_CONFIG
  // 0:2g 1:4g 2:8g 3:16g [default 2:8g]
  imu.accel_config = 2;
  // GYRO_CONFIG
  // 0:250dps 1:500dps 2:1000dps 3:2000dps [default 3:2000dps]
  imu.gyro_config = 3;

  imu.begin(); // Initialization of MPU6886
}

void loop()
{
  // Please uncomment the item you want to check
  value_read(); // Value reading
  //binary_read(); // Binary reading
  //sampling(); // Sampling speed check

  delay(100);
}
