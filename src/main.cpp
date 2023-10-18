#include <FS.h> //file system import must be imported first
#include <M5Core2.h>
#include <String.h>
#include <SPIFFS.h> //ESP32 file system import
#include <WiFiManager.h>
#include <math.h>
#include "time.h"
#include "AsyncUDP.h"
#include "I2C_MPU6886_4kHz.h"

#define SAMPLES 375 // number of samples to take in one cycle
#define MAX_RETRIES 3

#define CHUNK_SIZE 1028
#define HEADER "DATA_START"
#define FOOTER "DATA_END"
#define DATA_PORT 5255

// Semaphore declaration
SemaphoreHandle_t xSemaphore = NULL;

// acc data structure
struct AccData
{
  float x;
  float y;
  float z;
};
AccData acc;
// AccData buffer
AccData acc_buffer[SAMPLES];
AccData acc_buffer2[SAMPLES];

// imu
I2C_MPU6886_4kHz imu(I2C_MPU6886_DEFAULT_ADDRESS, Wire1);

// Process variable
bool START = false;
bool TIME_SET = false;

// UDP for remote client
AsyncUDP udp;

// IP address
IPAddress deviceIP;
IPAddress subnetMask;
IPAddress remoteIP;

// for timing
unsigned long last_time = millis();

// Wifi settings
String SSID = "";
String PASSWORD = "";

// fixed varriables
const char ACCESS_POINT[] = "VibSense";

// global varriables
WiFiManager wm;
bool WIFI_CONNECTED = false;
bool WIFI_STATE = false;
bool WIFI_SSID_EXIST = true;
// flag for saving data
bool shouldSaveConfig = false;

// battery variables
double battery_level = 0;
bool charge_state = 0;
int battery_level_last = 110;

// Time related variables
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 28800; // Taiwan GMT +8
const int daylightOffset_sec = 0;
RTC_TimeTypeDef RTCtime;
RTC_DateTypeDef RTCDate;
String Datetoday;
String Timenow;

// WiFi Parameters
char g_range[] = "";
WiFiManagerParameter GRangeSetup("ParaGRange", "G Range (2, 4, 8, 16)", g_range, 2);
String ParaGRange = "";

char remote_ip[] = "";
WiFiManagerParameter RemoteIPSetup("ParaRemoteIP", "Remote IP Address (For data transmission)", remote_ip, 18);
String ParaRemoteIP = "";

// Accelerometer parameters
int G_RANGE = 16;         // default g values
int SAMPLING_RATE = 3000; // default sampling rate

void setAccScale(int Scale)
{
  if (Scale == 2)
  {
    imu.accel_config = 0; // 2g
  }
  else if (Scale == 4)
  {
    imu.accel_config = 1; // 4g
  }
  else if (Scale == 8)
  {
    imu.accel_config = 2; // 8g
  }
  else if (Scale == 16)
  {
    imu.accel_config = 3; // 16g
  }
  else
  {
    imu.accel_config = 3; // 16g
  }
  Serial.println("Accel Scale Set @: " + String(imu.accel_config) + "g");
}

// safe char array to int
int safeString2Int(String data)
{
  // safely convert string to Integer
  // try and catch if the string is not a number
  int result = 0;
  try
  {
    result = data.toInt();
  }
  catch (const std::exception &e)
  {
    Serial.println("Error: " + String(e.what()));
  }
  return result;
}

int getGrange(String dataString)
{
  // safely convert string to Integer
  int data = safeString2Int(dataString);
  // default value +- 2g
  int result = 2;
  // if between 0 and 2 result = 2
  if (data >= 0 && data <= 2)
  {
    result = 2;
  }
  // between 2 and 4 then 4
  else if (data > 2 && data <= 4)
  {
    result = 4;
  }
  // between 4 and 8 then 8
  else if (data > 4 && data <= 8)
  {
    result = 8;
  }
  // between 8 and 16 then 16
  else if (data > 8 && data <= 16)
  {
    result = 16;
  }
  else
  {
    result = 2;
  }
  return result;
}

// Save and Read data from SPIFFS for LINE Notify
void save_data(String ParaGRange, String ParaRemoteIP)
{
  if (ParaGRange != "")
  {
    if (SPIFFS.exists("/grange.txt"))
    {
      SPIFFS.remove("/grange.txt");
    }
    File dataFile = SPIFFS.open("/grange.txt", "w");
    dataFile.print(ParaGRange);
    dataFile.close();
  }
  if (ParaRemoteIP != "")
  {
    if (SPIFFS.exists("/remoteip.txt"))
    {
      SPIFFS.remove("/remoteip.txt");
    }
    File dataFile2 = SPIFFS.open("/remoteip.txt", "w");
    dataFile2.print(ParaRemoteIP);
    dataFile2.close();
  }
}

// read data from SPIFFS for LINE Notify
void read_data()
{
  if (SPIFFS.exists("/grange.txt"))
  {
    File dataFile = SPIFFS.open("/grange.txt", "r");
    ParaGRange = dataFile.readString();
    dataFile.close();
  }
  else
  {
    ParaGRange = "16";
  }

  if (SPIFFS.exists("/remoteip.txt"))
  {
    File dataFile2 = SPIFFS.open("/remoteip.txt", "r");
    ParaRemoteIP = dataFile2.readString();
    dataFile2.close();
  }
  else
  {
    ParaRemoteIP = "";
  }

  G_RANGE = getGrange(ParaGRange);
  remoteIP.fromString(ParaRemoteIP);

  Serial.println("");
  Serial.println(":::::::Loaded Settings are as Follow:::::::");
  Serial.println("G Range: +-" + String(G_RANGE));
  Serial.println("Remote IP: " + String(ParaRemoteIP));
}

void send_udp_data()
{
  // send broadcast 3 times
  udp.broadcastTo("Play_sound", DATA_PORT);
}

// setting time for the machine
void setTime()
{
  Serial.println("Getting time");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("obtain time");
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    TIME_SET = false;
  }
  else
  {
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
    // set RTC date time
    RTCDate.Year = timeinfo.tm_year + 1900;
    RTCDate.Month = timeinfo.tm_mon + 1;
    RTCDate.Date = timeinfo.tm_mday;
    if (!M5.Rtc.SetDate(&RTCDate))
    {
      Serial.println("wrong date set!");
      TIME_SET = false;
    }
    else
    {
      Serial.println("Date set!");
      TIME_SET = true;
    }

    RTCtime.Hours = timeinfo.tm_hour;
    RTCtime.Minutes = timeinfo.tm_min;
    RTCtime.Seconds = timeinfo.tm_sec;
    if (!M5.Rtc.SetTime(&RTCtime))
    {
      Serial.println("wrong time set!");
      TIME_SET = false;
    }
    else
    {
      Serial.println("Time set!");
      TIME_SET = true;
    }
  }
}

String getRtcDate()
{
  RTC_DateTypeDef RTCDate;
  M5.Rtc.GetDate(&RTCDate);
  String Month;
  String Date;
  if (RTCDate.Month < 10)
  {
    Month = "0" + String(RTCDate.Month);
  }
  else
  {
    Month = String(RTCDate.Month);
  }
  if (RTCDate.Date < 10)
  {
    Date = "0" + String(RTCDate.Date);
  }
  else
  {
    Date = String(RTCDate.Date);
  }

  String date = String(RTCDate.Year) + "-" + Month + "-" + Date;
  return date;
}

String getRtcTime()
{
  RTC_TimeTypeDef RTCtime;
  M5.Rtc.GetTime(&RTCtime);
  String Hour;
  String Minute;
  if (RTCtime.Hours % 12 < 10)
  {
    if (RTCtime.Hours % 12 == 0)
    {
      Hour = "12";
    }
    else
    {
      Hour = "0" + String(RTCtime.Hours % 12);
    }
  }
  else
  {
    Hour = String(RTCtime.Hours % 12);
  }

  if (RTCtime.Minutes < 10)
  {
    Minute = "0" + String(RTCtime.Minutes);
  }
  else
  {
    Minute = String(RTCtime.Minutes);
  }

  String time = Hour + ":" + Minute;

  if (RTCtime.Hours >= 12)
    time += " PM";
  else
    time += " AM";
  return time;
}

void clear_recArea(int x, int y, int w, int h)
{
  // clears a rectangle area for skipping the clear display
  // this improves the display and avoids flickering
  M5.Lcd.fillRect(x, y, w, h, 0x0000);
}

bool batteryChange()
{
  battery_level = M5.Axp.GetBatteryLevel();
  if (abs(battery_level_last - battery_level) > 2)
  {
    battery_level_last = battery_level;
    return true;
  }
  else
  {
    return false;
  }
}

bool chargeChange()
{
  bool charging = M5.Axp.isCharging();
  if (charging != charge_state)
  {
    charge_state = charging;
    return true;
  }
  else
  {
    return false;
  }
}

void process_btnA()
{
  while (true)
  {
    M5.update();
    if (M5.BtnA.wasPressed())
    {
      M5.Lcd.clear(BLACK);
      break;
    }
    vTaskDelay(50);
  }
}

bool process_okMenu()
{
  bool flag = false;

  M5.Lcd.drawPngFile(SPIFFS, "/check.png", 40, 240 - 33);
  M5.Lcd.drawPngFile(SPIFFS, "/remove.png", 320 - 40 - 32 - 17, 240 - 33);
  while (true)
  {
    M5.update();
    if (M5.BtnA.wasPressed())
    {

      flag = true;
      break;
    }
    else if (M5.BtnC.wasPressed())
    {
      flag = false;
      break;
    }
    vTaskDelay(100);
  }
  return flag;
}

void draw_ok()
{
  // draw above the button B
  M5.Lcd.drawPngFile(SPIFFS, "/check.png", 40, 240 - 33);
}

void draw_cancel()
{
  // draw above the button C
  M5.Lcd.drawPngFile(SPIFFS, "/remove.png", 320 - 40 - 32 - 17, 240 - 33);
}

void update_batteryStatus()
{
  // check if the charging state is change to redraw the charging status
  if (chargeChange())
  {
    if (charge_state)
    {
      clear_recArea(320 - 32 - 27, 6, 24, 32);
      M5.Lcd.drawPngFile(SPIFFS, "/charge.png", 320 - 32 - 27, 6);
    }
    else
    {
      clear_recArea(320 - 32 - 27, 6, 24, 32);
    }
  }
  if (batteryChange())
  {
    if (battery_level >= 99.2)
    {
      clear_recArea(320 - 32 - 27, 6, 24, 32);
      M5.Lcd.drawPngFile(SPIFFS, "/checkm.png", 320 - 32 - 27, 6); // draw charged icon
    }
    else if (battery_level >= 70)
    {
      clear_recArea(320 - 33, 2, 32, 32);
      M5.Lcd.drawPngFile(SPIFFS, "/full_battery.png", 320 - 33, 2);
    }
    else if (battery_level < 70 && battery_level >= 30)
    {
      clear_recArea(320 - 33, 2, 32, 32);
      M5.Lcd.drawPngFile(SPIFFS, "/half_battery.png", 320 - 33, 2);
    }
    else if (battery_level < 30 && battery_level >= 10)
    {
      clear_recArea(320 - 33, 2, 32, 32);
      M5.Lcd.drawPngFile(SPIFFS, "/low_battery.png", 320 - 33, 2);
    }
    else // battery_level < 10
    {
      clear_recArea(320 - 33, 2, 32, 32);
      M5.Lcd.drawPngFile(SPIFFS, "/very_low_battery.png", 320 - 33, 2);
    }

    // charging current management
    if (battery_level < 90)
    {
      M5.Axp.SetCHGCurrent(M5.Axp.kCHG_450mA); // Set LDO2 current to 450mA.
    }
    else
    {
      M5.Axp.SetCHGCurrent(M5.Axp.kCHG_190mA); // Set LDO2 current to 190mA. if charge is more than 90%
    }
  }
  // Serial.print("Battery level: ");
  // Serial.println(battery_level);
  // Serial.print("Charge state: ");
  // Serial.println(charge_state);
  // Serial.print("last battery level: ");
  // Serial.println(battery_level_last);
  clear_recArea(320 - 33 - 27 - 30, 0, 32, 24);
  M5.Lcd.drawCentreString(String(battery_level, 0) + "%", 320 - 33 - 27 - 15, 6, 2);
}

// callback notifying us of the need to save config
void saveConfigCallback()
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void process_configPortal()
{
  while (true)
  {
    M5.update();
    wm.process();
    draw_ok();
    if (M5.BtnA.wasPressed())
    {
      wm.stopConfigPortal();
      vTaskDelay(100);
      M5.Lcd.clear(BLACK);
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.drawCentreString(String("SSID:").c_str(), 160, 120, 4);
      M5.Lcd.drawCentreString(wm.getWiFiSSID().c_str(), 160, 150, 4);

      ParaGRange = GRangeSetup.getValue();
      ParaRemoteIP = RemoteIPSetup.getValue();

      save_data(ParaGRange, ParaRemoteIP);
      vTaskDelay(2000);

      for (int i = 0; i < 3; i++)
      {
        M5.Lcd.clear(BLACK);
        M5.Lcd.setTextColor(WHITE);
        M5.Lcd.drawCentreString("Restarting", 160, 110, 4);
        M5.Lcd.drawCentreString(String(3 - i), 160, 135, 4);
        vTaskDelay(1000);
      }
      ESP.restart();
    }

    vTaskDelay(250);
  }
}

void WifiSetup()
{

  wm.setDarkMode(true); // dark mode for better visibility on phone
  // wm.setConfigPortalTimeout(300); // 5 minutes
  M5.Lcd.clear(BLACK);
  M5.Lcd.setTextColor(WHITE);
  vTaskDelay(50);
  // M5.Lcd.drawCentreString("Configure WiFi", 160, 160, 3);
  M5.Lcd.drawPngFile(SPIFFS, "/wifi_setup.png", 96, 36);
  if (process_okMenu())
  {
    M5.Lcd.clear(BLACK);

    M5.Lcd.drawPngFile(SPIFFS, "/router.png", 128, 88);
    M5.Lcd.drawCentreString(ACCESS_POINT, 160, 160, 4);

    wm.setSaveConfigCallback(saveConfigCallback);

    wm.addParameter(&GRangeSetup);
    wm.addParameter(&RemoteIPSetup);

    wm.setConfigPortalBlocking(false);

    // wm.setConfigPortalTimeout(300);
    bool result = wm.startConfigPortal(ACCESS_POINT);
    process_configPortal();
  }
  else
  {
    M5.Lcd.clear(BLACK);
    M5.Lcd.drawCentreString("Connecting to WiFi", 160, 120, 3);
    vTaskDelay(1200);
  }
}

void connectToWifi(int attempts)
{
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA); // Set WiFi mode to station.
  if (wm.getWiFiIsSaved())
  {
    Serial.println("WIFI is saved");
    SSID = wm.getWiFiSSID();
    PASSWORD = wm.getWiFiPass();
    M5.Lcd.drawPngFile(SPIFFS, "/wifi.png", 128, 88);
    for (int i = 0; i < attempts; i++)
    {
      M5.update();

      char connecting_msg[100];
      sprintf(connecting_msg, "Connecting to %s: %i", SSID.c_str(), i);
      M5.Lcd.drawCentreString(connecting_msg, 160, 160, 2);
      M5.Lcd.drawCentreString("Please wait...", 160, 180, 2);
      WiFi.disconnect(true);
      WiFi.mode(WIFI_STA); // Set WiFi mode to station.
      vTaskDelay(200);
      WiFi.begin(SSID.c_str(), PASSWORD.c_str());
      vTaskDelay(1000);
      if (WiFi.status() == WL_CONNECTED)
      {
        WIFI_CONNECTED = true;
        // update the RTC time with NTP time
        Serial.println("Connected!");
        Serial.println("Getting time");
        setTime();
        break;
      }
    }

    M5.Lcd.clear(BLACK);
    // process the connection attempt
    if (WIFI_CONNECTED)
    {
      M5.Lcd.drawPngFile(SPIFFS, "/connection.png", 128, 88);
      M5.Lcd.drawCentreString("Connected!", 160, 138, 4);
    }
    else
    {
      M5.Lcd.drawPngFile(SPIFFS, "/no_signal.png", 128, 88);
      M5.Lcd.drawCentreString("Failed to connect!", 160, 138, 4);
    }

    draw_ok();
    process_btnA();
  }
  else
  {

    M5.Lcd.drawPngFile(SPIFFS, "/no_signal.", 128, 88);
    M5.Lcd.drawCentreString("No WiFi Setting!", 160, 138, 4);
    WIFI_SSID_EXIST = false;
    draw_ok();
    // wait for user to skip!
    process_btnA();
  }
}

void connectToWiFiOnce()
{
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  vTaskDelay(50);
  WiFi.begin(SSID.c_str(), PASSWORD.c_str());
  vTaskDelay(1000);
  if (WiFi.status() == WL_CONNECTED)
  {
    WIFI_CONNECTED = true;
  }
}

void checkWiFi()
{

  if (WiFi.status() != WL_CONNECTED)
  {
    WIFI_CONNECTED = false;
  }
  else
  {
    WIFI_CONNECTED = true;
  }

  if (WIFI_STATE != WIFI_CONNECTED)
  {
    WIFI_STATE = WIFI_CONNECTED;
    if (WIFI_CONNECTED)
    {
      clear_recArea(0, 0, 100, 35);
      M5.Lcd.drawPngFile(SPIFFS, "/wifi_signal.png", 1, 2);
    }
    else
    {
      clear_recArea(0, 0, 100, 35);
      M5.Lcd.drawPngFile(SPIFFS, "/no_wifi.png", 1, 2);
    }
  }
  if (WIFI_CONNECTED && !TIME_SET)
  {
    setTime();
  }
}

IPAddress getBroadcastAddress(IPAddress ip, IPAddress subnetMask)
{
  IPAddress broadcast;
  for (int i = 0; i < 4; i++)
  {
    broadcast[i] = ip[i] | (~subnetMask[i] & 255);
  }
  return broadcast;
}

void GUI_task(void *pvParameters)
{
  START = true;
  for (;;)
  {
    // put your main code here, to run repeatedly:
    M5.update(); // Update M5Core2.

    // Serial.println(analogRead(EOG_PIN));

    update_batteryStatus();
    checkWiFi();
    if (!WIFI_CONNECTED && WIFI_SSID_EXIST)
    {
      connectToWiFiOnce();
    }

    // get device IP
    deviceIP = WiFi.localIP();
    subnetMask = WiFi.subnetMask();

    Datetoday = getRtcDate();
    Timenow = getRtcTime();
    clear_recArea(160 - 40, 5, 80, 45);
    M5.Lcd.drawCentreString(Datetoday, 160, 10, 2);
    M5.Lcd.drawCentreString(Timenow, 160, 25, 2);
    if (WIFI_CONNECTED)
    {
      M5.Lcd.drawCentreString("Working!", 160, 100, 4);
    }
    else
    {
      // clear
      // clear_recArea(160 - 80, 120 - 20, 160, 40);
      M5.Lcd.drawCentreString("No WiFi!", 160, 100, 4);
    }
    // set the G rane and Sampling rate on screen
    String rangeG = "G Range: " + String(G_RANGE);
    rangeG += " g";
    M5.Lcd.drawCentreString(rangeG, 160, 130, 4);
    String IPad = "Device IP: " + String(deviceIP[0]) + "." + String(deviceIP[1]) + "." + String(deviceIP[2]) + "." + String(deviceIP[3]);
    M5.Lcd.drawCentreString(IPad, 160, 160, 2);

    String RemIP = "Remote IP: " + String(remoteIP[0]) + "." + String(remoteIP[1]) + "." + String(remoteIP[2]) + "." + String(remoteIP[3]);
    M5.Lcd.drawCentreString(RemIP, 160, 180, 2);

    vTaskDelay(1000);
  }
}

// Function to send a packet with retries
// Function to send a packet with retries
bool send_with_retries(uint8_t *data, size_t size)
{
  if (remoteIP == IPAddress(0, 0, 0, 0))
  {
    Serial.println("Invalid remote IP address.");
    return false; // Invalid IP address
  }

  for (int retry = 0; retry < MAX_RETRIES; retry++)
  {
    if (udp.writeTo(data, size, remoteIP, DATA_PORT) != 0)
    {
      return true; // Successfully sent
    }
    vTaskDelay(8 / portTICK_PERIOD_MS);
  }
  return false; // Failed to send after MAX_RETRIES
}

void send_UDP_chunks()
{
  uint8_t *data_ptr = (uint8_t *)acc_buffer2;
  size_t data_size = sizeof(acc_buffer2);
  size_t bytes_sent = 0;

  // Send HEADER
  if (!send_with_retries((uint8_t *)HEADER, strlen(HEADER)))
  {
    Serial.println("Failed to send HEADER after multiple retries!");
    return;
  }

  while (bytes_sent < data_size)
  {
    size_t bytes_to_send = (data_size - bytes_sent > CHUNK_SIZE) ? CHUNK_SIZE : data_size - bytes_sent;

    if (!send_with_retries(data_ptr + bytes_sent, bytes_to_send))
    {
      // Failed to send chunk after retries
      Serial.println("Failed to send data chunk after multiple retries!");
      return;
    }

    bytes_sent += bytes_to_send;
    vTaskDelay(5 / portTICK_PERIOD_MS); // Delay to avoid flooding the network
  }

  // Send FOOTER
  if (!send_with_retries((uint8_t *)FOOTER, strlen(FOOTER)))
  {
    Serial.println("Failed to send FOOTER after multiple retries!");
    return;
  }
}

void UDP_task(void *pvParameters)
{
  const TickType_t xMaxWait = 165 / portTICK_PERIOD_MS;

  for (;;)
  {
    // Wait for the semaphore to be given by the Acc_task
    if ((xSemaphoreTake(xSemaphore, xMaxWait) == pdTRUE) && WIFI_CONNECTED)
    {

      send_UDP_chunks();
    }
    else
    {
      Serial.println("Failed to get semaphore in time! Acc_task may be lagging.");
    }
  }
}

void Acc_task(void *pvParameters)
{
  Wire1.begin(21, 22);    // I2C pin settings
  Wire1.setClock(400000); // Set the I2C speed to 400kHz
  setAccScale(G_RANGE);
  // imu.resetDevice(); // reset the imu
  // imu.enableGyro(); // enable gyro

  imu.begin();

  // imu.setGyroStandbyMode(true); // set gyro to standby mode
  // imu.disableGyro();            // disable gyro
  // imu.disableTemperatureSensor(true);
  vTaskDelay(100);

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 125 / portTICK_PERIOD_MS;
  // unsigned long CurrentTime = millis();

  // bool flip = false;
  for (;;)
  {
    // Serial.println(START);
    unsigned long start_time2 = micros(); // Keep start time
    if (START)
    {
      unsigned long start_time = micros(); // Keep start time

      for (int i = 0; i < SAMPLES; i++)
      {
        imu.getAccel(&acc.x, &acc.y, &acc.z);
        acc_buffer[i] = acc;
        // Serial.println("X: " + String(acc.x) + " Y: " + String(acc.y) + " Z: " + String(acc.z));
      }
      // copy the data to the second buffer
      memcpy(acc_buffer2, acc_buffer, sizeof(acc_buffer));
      xSemaphoreGive(xSemaphore); // Signal that buffer2 is filled and ready for transmission.

      unsigned long end_time = micros(); // Keep the end time

      // For checking the sampling cycle
      // Serial.print(F("Sampling frequency : "));
      // Serial.println(1000000.0 / float(end_time - start_time) * SAMPLES);
    }
    // For checking the sampling cycle
    unsigned long end_time2 = micros(); // Keep the end time
    // Serial.print(F("::::::::::::CYCLE TIME::::::::::::::: "));
    // Serial.println(float(end_time2 - start_time2));
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup()
{
  M5.begin(); // Init M5Core2.
  Serial.begin(115200);

  M5.Axp.SetCHGCurrent(M5.Axp.kCHG_450mA); // Set LDO2 current to 450mA.
  SPIFFS.begin(true);

  // // Start WiFisetup
  M5.Lcd.clear(WHITE);
  M5.Lcd.drawPngFile(SPIFFS, "/vibration.png", 48, 88);
  M5.lcd.drawRect(0, 219, 320, 10, GREEN);
  for (int i = 0; i < 32; i++)
  {
    M5.update();
    if (M5.BtnB.wasPressed())
    {
      WifiSetup();
    }
    M5.Lcd.fillRect(0, 219, 32 + (32 * i), 10, GREEN);
    vTaskDelay(200);
  }

  read_data();

  M5.Lcd.clear(BLACK);

  connectToWifi(10);
  vTaskDelay(1000);

  xSemaphore = xSemaphoreCreateBinary(); // Initialize the Semaphore
  if (xSemaphore == NULL)
  {
    Serial.println("Failed to create semaphore!");
    // Handle this appropriately, maybe halt the system or indicate error via LEDs
  }

  xTaskCreatePinnedToCore(
      Acc_task,
      "Acc_task",
      10000,
      NULL,
      10,
      NULL,
      1);
  xTaskCreatePinnedToCore(
      GUI_task,
      "GUI_task",
      10000,
      NULL,
      1,
      NULL,
      0);

  xTaskCreatePinnedToCore(
      UDP_task,
      "UDP_task",
      10000,
      NULL,
      1,
      NULL,
      0);
}

void loop()
{
  yield();
}
