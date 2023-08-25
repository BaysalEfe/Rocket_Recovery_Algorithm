# Rocket Recovery Telemetry System

This Arduino sketch implements a rocket telemetry system that gathers data from various sensors including GPS, pressure, temperature, and IMU (MPU6050). The collected data is transmitted over LoRa communication for remote monitoring and analysis. The sketch also includes a recovery algorithm based on altitude and sensor readings.

## Features

- Acquires data from GPS, pressure sensor, temperature sensor, and MPU6050 IMU sensor.
- Transmits sensor data over LoRa communication using LoRa_E22 library.
- Implements a recovery algorithm based on altitude and sensor data.
- Monitors rocket altitude, latitude, longitude, pressure, temperature, angles, acceleration, and gyroscopic readings.
- Provides LED and buzzer indications for specific conditions.

## Components

- [TinyGPS++](https://github.com/mikalhart/TinyGPSPlus): Library for parsing GPS data.
- [SoftwareSerial](https://www.arduino.cc/en/Reference/softwareSerial): Software-based serial communication.
- [LoRa_E22](https://github.com/soonuse/LoRa_E22): Library for LoRa communication using E22 modules.
- [LPS](https://github.com/pololu/lps-arduino): Library for LPS25H pressure sensor.
- [Wire](https://www.arduino.cc/en/reference/wire): I2C communication library.
- [MPU6050_tockn](https://github.com/tockn/MPU6050_tockn): Library for MPU6050 IMU sensor.

## Usage

1. Connect the required hardware components: GPS module, pressure sensor, MPU6050 IMU, LoRa module, LEDs, and buzzer.
2. Upload this sketch to an Arduino board.
3. Ensure all the sensor connections are correct.
4. Monitor the serial output to view sensor readings and status messages.
5. Customize the recovery algorithm and LED/buzzer indications based on your requirements.

Feel free to contribute to the project by improving the code, adding more features, or enhancing the recovery algorithm.

## License

This project is licensed under the [MIT License](LICENSE).

## Contact

mehmetefebaysal06@gmail.com

