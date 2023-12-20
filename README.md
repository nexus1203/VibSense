# VibSense :zap:

VibSense is a wireless vibration sensor solution designed for real-time monitoring and analysis. Built around the M5stack-core and powered by the MPU6886 sensor, it operates at a sampling rate of 3kHz and is backed by RTOS for efficient multitasking.

:star: **Key Features**:
- **Wireless Configuration**: Utilizes WiFiManager for easy setup of WiFi credentials and device configurations.
- **Intuitive Setup Mode**: Access setup mode with the center button during startup, enabling you to connect to an AP named "VibSense" for configuration.
- **High-Frequency Sampling**: Achieves a sampling rate of 3000Hz, transmitting vibration data every 125ms.
- **Universal Compatibility**: Designed to work seamlessly with mobile devices such as Android and iPhone.
- **UDP Transmission**: Uses unicast UDP on Port 5255 for data transfer, ensuring real-time performance.

## :gear: Configuration

During the startup of the device, press the center button to enter the setup mode. The device will then initiate an Access Point (AP) named `VibSense`. Connect to this AP using a mobile device to configure:

- WiFi SSID
- RemoteIP 
- Accelerometer Sensitivity

After configuration, the device will transfer the vibration data to the specified `remoteIP`. Ensure both devices (VibSense and `remoteIP`) are on the same WiFi network for successful data transmission.
![image](https://github.com/nexus1203/VibSense/assets/70212520/f5d16c68-e28d-43dd-a81f-7e6c611ba036)

## :rocket: Installation & Setup

1. **Filesystem Build & Upload**:
   ```bash
   # Build the filesystem
   platformio run --target buildfs

   # Upload to M5stack-core2
   platformio run --target uploadfs

2. **Code Build & Upload:88
  ```bash
  # Build the code
  platformio run
   ```

# Upload to M5stack-core2
   ```bash
   platformio run --target upload
   ```

:computer: User Interface
For a comprehensive user-interface to visualize and interpret the vibration data, please visit [pyVibSense](https://github.com/nexus1203/pyVibSense/tree/main).

:bulb: Tips
Ensure a stable power source for the M5stack-core during the upload process.
For any issues during setup or usage, please raise an issue in the GitHub repository.


:memo: License
This project is licensed under the MIT License. See LICENSE for more details.

