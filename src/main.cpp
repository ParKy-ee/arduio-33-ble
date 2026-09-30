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

// Arduino IMU APIs report acceleration in g, angular speed in deg/s,
// and magnetic field in uT.
constexpr float kGravity = 9.80665f;
constexpr float kRadiansPerDegree = 0.017453292519943295f;
constexpr uint16_t kGyroCalibrationSamples = 200;

uint32_t sequence = 0;
uint32_t nextSensorStatusMs = 0;
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

void calibrateGyroscope() {
  Serial.println("# keep IMU still: calibrating gyro bias");

  float sumX = 0.0f;
  float sumY = 0.0f;
  float sumZ = 0.0f;
  uint16_t samples = 0;
  uint32_t nextStatusMs = millis() + 1000;

  while (samples < kGyroCalibrationSamples) {
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      float ax, ay, az, gx, gy, gz;
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);
      sumX += gx;
      sumY += gy;
      sumZ += gz;
      ++samples;
    } else {
      delay(1);
    }

    if (static_cast<int32_t>(millis() - nextStatusMs) >= 0) {
      Serial.print("# gyro calibration samples: ");
      Serial.println(samples);
      nextStatusMs = millis() + 1000;
    }
  }

  gyroBiasX = sumX / samples;
  gyroBiasY = sumY / samples;
  gyroBiasZ = sumZ / samples;
  Serial.println("# gyro calibration complete");
}

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

  calibrateGyroscope();

  Serial.println("# IMU initialized");
  Serial.println("sensor_id,boot_ms,seq,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z");
  nextSensorStatusMs = millis();
}

void loop() {
  const uint32_t now = millis();
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

  float mx, my, mz;
  const bool magneticFieldReady = IMU.magneticFieldAvailable();
  if (magneticFieldReady) {
    IMU.readMagneticField(mx, my, mz);
  }

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
  Serial.print((gx - gyroBiasX) * kRadiansPerDegree, 6);
  Serial.print(',');
  Serial.print((gy - gyroBiasY) * kRadiansPerDegree, 6);
  Serial.print(',');
  Serial.print((gz - gyroBiasZ) * kRadiansPerDegree, 6);
  Serial.print(',');
  if (magneticFieldReady) {
    Serial.print(mx, 6);
    Serial.print(',');
    Serial.print(my, 6);
    Serial.print(',');
    Serial.print(mz, 6);
  } else {
    // Preserve the three magnetometer CSV columns when no new sample is ready.
    Serial.print(",,");
  }
  Serial.println();
}
