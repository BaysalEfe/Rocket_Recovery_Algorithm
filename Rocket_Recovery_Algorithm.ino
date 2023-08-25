
//------------- LIBRARIES -------------//
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "LoRa_E22.h"
#include <LPS.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

//------------- VARIABLES -------------//
static const int RXPin = 28, TXPin = 29;
static const uint32_t GPSBaud = 9600;
float X_angle, Y_angle, Z_angle, ground_altitude, previous_altitude, altitude_difference, altitude;
float loop_count, main_parachute, drogue_parachute, burnout_altitude = 1100;
bool altitude_check = 0, angle_check = 0, altitude2_check = 0, burnout_altitude_check = 0;
int teensy_led = 13, led = 27, buzzer = 26, status = 1;
float gps_altitude, latitude, longitude, pressure, temperature, angleX, angleY, angleZ, accX, accY, accZ, gyroX, gyroY, gyroZ;
float altitude_counter = 0;

//------------- LIBRARY USAGE -------------//
TinyGPSPlus gps;
MPU6050 mpu6050(Wire);
LPS ps;
SoftwareSerial portgps(RXPin, TXPin);
SoftwareSerial portlora(34, 35);
LoRa_E22 e22ttl(&portlora);
#define M0 7
#define M1 6
#define first_separation A10
#define second_separation A11

//------------- RF COMMUNICATION STRUCTURE -------------//
typedef struct {
  float L_Loop;
  float L_gps_altitude;
  float L_altitude;
  float L_latitude;
  float L_longitude;
  float L_pressure;
  float L_temperature;
  float L_angleX;
  float L_angleY;
  float L_angleZ;
  float L_accX;
  float L_accY;
  float L_accZ;
  float L_gyroX;
  float L_gyroY;
  float L_gyroZ;
  float L_status;
} Signal;

Signal data;

void setup() {
  // Initialize communication systems
  portgps.begin(GPSBaud);
  Serial.begin(9600);
  Wire.begin();
  e22ttl.begin();

  // Set input/output pins
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(teensy_led, OUTPUT);
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);

  // Initialize LPS25H sensor
  if (!ps.init()) {
    Serial.println("LPS25H sensor is not working.");
    while (1);
  }
  ps.enableDefault();

  // Initialize MPU IMU sensor
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  portgps.listen();
  digitalWrite(teensy_led, HIGH);

  // Acquire LPS25H data
  float pressure = ps.readPressureMillibars();
  float altitude = ps.pressureToAltitudeMeters(pressure);
  temperature = ps.readTemperatureC();

  // Acquire MPU IMU sensor data
  angleX = mpu6050.getAngleX();
  angleY = mpu6050.getAngleY();
  angleZ = mpu6050.getAngleZ();
  accX = mpu6050.getAccX();
  accY = mpu6050.getAccY();
  accZ = mpu6050.getAccZ();
  gyroX = mpu6050.getGyroX();
  gyroY = mpu6050.getGyroY();
  gyroZ = mpu6050.getGyroZ();

  // Acquire GPS data
  gps_altitude = gps.altitude.meters();

  if (gps.altitude.isValid()) {
    mpu6050.update();

    Serial.print("GPS Rocket altitude: ");
    Serial.println(gps_altitude);
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.print("\tAltitude: ");
    Serial.print(altitude);
    Serial.print("\tTemperature: ");
    Serial.println(temperature);
    Serial.print("accX: ");
    Serial.print(accX);
    Serial.print("\taccY: ");
    Serial.print(accY);
    Serial.print("\taccZ: ");
    Serial.println(accY);
    Serial.print("gyroX: ");
    Serial.print(gyroX);
    Serial.print("\tgyroY: ");
    Serial.print(gyroY);
    Serial.print("\tgyroZ: ");
    Serial.println(gyroZ);
    Serial.print("angleX: ");
    Serial.print(angleX);
    Serial.print("\tangleY: ");
    Serial.print(angleY);
    Serial.print("\tangleZ: ");
    Serial.println(angleZ);
    Serial.print("Status: ");
    Serial.println(status);
    Serial.print("Altitude difference: ");
    Serial.println(altitude_difference);
    Serial.print("angle_check: ");
    Serial.println(angle_check);
    Serial.print("altitude_check: ");
    Serial.println(altitude_check);
    Serial.print("Loop Count: ");
    Serial.println(loop_count);

    // Prepare data for RF packet
    data.L_Loop = loop_count;
    data.L_gps_altitude = gps_altitude;
    data.L_altitude = altitude;
    data.L_latitude = gps.location.lat(), 6;
    data.L_longitude = gps.location.lng(), 6;
    data.L_pressure = pressure;
    data.L_temperature = temperature;
    data.L_angleX = angleX;
    data.L_angleY = angleY;
    data.L_angleZ = angleZ;
    data.L_accX = accX;
    data.L_accY = accY;
    data.L_accZ = accZ;
    data.L_gyroX = gyroX;
    data.L_gyroY = gyroY;
    data.L_gyroZ = gyroZ;
    data.L_status = status;

    // Recovery algorithm
    if (loop_count < 5) {
      digitalWrite(buzzer, HIGH);
      digitalWrite(led, HIGH);
    }

    if (loop_count == 5) {
      ground_altitude = altitude;
      digitalWrite(buzzer, LOW);
      digitalWrite(led, LOW);
    }

    if (loop_count > 5) {
      altitude = altitude - ground_altitude;
      altitude_difference = altitude - previous_altitude;

      if (altitude_difference <= -5) {
        altitude_counter = altitude_counter + 1;
      } else if (altitude_difference > -5) {
        altitude_counter = altitude_counter - 1;
      }

      if (altitude_counter < 0) {
        altitude_counter = 0;
      }

      if (altitude_counter > 2) {
        altitude_check = true;
      
      }

    }

  }

}
