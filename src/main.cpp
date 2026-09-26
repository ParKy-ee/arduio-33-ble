#include <Arduino.h>

#ifdef IMU_REV2
#include <Arduino_BMI270_BMM150.h>
#else
#include <Arduino_LSM9DS1.h>
#endif

#ifndef SENSOR_POINT
#define SENSOR_POINT chest
#endif

#define STRINGIFY_VALUE(value) #value
#define STRINGIFY(value) STRINGIFY_VALUE(value)

// Arduino IMU APIs report acceleration in g and angular speed in deg/s.
constexpr float kGravity = 9.80665f;
constexpr float kRadiansPerDegree = 0.017453292519943295f;
constexpr uint32_t kSamplePeriodMs = 10;  // Target 100 Hz.

uint32_t nextSampleMs = 0;
uint32_t sequence = 0;
uint32_t nextSensorStatusMs = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {
  }

  Serial.print("# starting IMU for point ");
  Serial.println(STRINGIFY(SENSOR_POINT));

  if (!IMU.begin()) {
    while (true) {
      Serial.println("# ERROR: IMU.begin() failed; check board revision and selected environment");
      delay(1000);
    }
  }

  Serial.println("# IMU initialized");
  Serial.println("sensor_id,boot_ms,seq,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z");
  nextSampleMs = millis();
  nextSensorStatusMs = nextSampleMs;
}

void loop() {
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - nextSampleMs) < 0) {
    return;
  }
  nextSampleMs = now + kSamplePeriodMs;

  const bool accelerationReady = IMU.accelerationAvailable();
  const bool gyroscopeReady = IMU.gyroscopeAvailable();
  if (!accelerationReady || !gyroscopeReady) {
    if (static_cast<int32_t>(now - nextSensorStatusMs) >= 0) {
      Serial.print("# waiting for sensors: accel=");
      Serial.print(accelerationReady ? "ready" : "no data");
      Serial.print(" gyro=");
      Serial.println(gyroscopeReady ? "ready" : "no data");
      nextSensorStatusMs = now + 1000;
    }
    return;
  }

  float ax, ay, az, gx, gy, gz;
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);

  Serial.print(STRINGIFY(SENSOR_POINT));
  Serial.print(',');
  Serial.print(now);
  Serial.print(',');
  Serial.print(sequence++);
  Serial.print(',');
  Serial.print(ax * kGravity, 6);
  Serial.print(',');
  Serial.print(ay * kGravity, 6);
  Serial.print(',');
  Serial.print(az * kGravity, 6);
  Serial.print(',');
  Serial.print(gx * kRadiansPerDegree, 6);
  Serial.print(',');
  Serial.print(gy * kRadiansPerDegree, 6);
  Serial.print(',');
  Serial.print(gz * kRadiansPerDegree, 6);
  Serial.println();
}
