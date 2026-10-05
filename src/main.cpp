#include "webpage.h"
#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <math.h>

constexpr uint8_t I2C_SDA_PIN = 8;
constexpr uint8_t I2C_SCL_PIN = 9;
constexpr uint8_t MPU_ADDR = 0x68;
constexpr char SENSOR_ID[] = "wrist";
constexpr uint32_t SAMPLE_INTERVAL_MS = 50; // 20 Hz
constexpr uint16_t CALIBRATION_SAMPLES = 40; // 2 seconds of steady data
constexpr uint16_t STATUS_SAMPLES = 10;      // 0.5 seconds before a status change
constexpr float WRIST_LIMIT_DEG = 12.0f;    // Prototype setting, not a clinical limit
constexpr float GRAVITY_MS2 = 9.80665f;

// Attach to the back of the hand: +Y toward the fingers, +X across the knuckles,
// and +Z away from the skin. Check the sign with a gentle wrist bend before use.
constexpr float FLEXION_SIGN = -1.0f;

WebServer server(80);

struct ImuSample {
  uint32_t bootMs = 0;
  uint32_t seq = 0;
  float ax = 0, ay = 0, az = 0; // g, kept for the existing webpage
  float gx = 0, gy = 0, gz = 0; // rad/s, kept for the existing webpage
  float tiltDeg = 0;
  float accelNormG = 0;
  float gyroNormDps = 0;
  bool steady = false;
};

enum class Posture { Uncalibrated, Calibrating, Moving, Normal, Flexion, Extension };

ImuSample sample;
Posture posture = Posture::Uncalibrated;
Posture pendingPosture = Posture::Uncalibrated;
uint16_t pendingCount = 0;
uint16_t calibrationCount = 0;
float calibrationSin = 0;
float calibrationCos = 0;
float neutralTiltDeg = 0;
float wristFlexDeg = 0;
bool calibrated = false;
bool calibrating = false;
bool sensorReady = false;
bool lastReadOk = false;
uint32_t sequence = 0;

const char *postureName(Posture value) {
  switch (value) {
    case Posture::Calibrating: return "calibrating";
    case Posture::Moving: return "moving";
    case Posture::Normal: return "normal";
    case Posture::Flexion: return "wrist_flexion";
    case Posture::Extension: return "wrist_extension";
    default: return "uncalibrated";
  }
}

bool writeMpuRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

bool initializeMpu() {
  // Set the ranges used by the conversion factors below.
  return writeMpuRegister(0x6B, 0x00) && // Wake
         writeMpuRegister(0x1C, 0x00) && // Accelerometer +/-2 g
         writeMpuRegister(0x1B, 0x00);   // Gyroscope +/-250 deg/s
}

int16_t readI16() {
  const uint16_t high = static_cast<uint8_t>(Wire.read());
  const uint16_t low = static_cast<uint8_t>(Wire.read());
  return static_cast<int16_t>((high << 8) | low);
}

float angleDifferenceDeg(float angle, float reference) {
  const float radians = (angle - reference) * DEG_TO_RAD;
  return atan2f(sinf(radians), cosf(radians)) * RAD_TO_DEG;
}

bool readMpu(uint32_t seq) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Start of accel / temperature / gyro register block
  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom(static_cast<uint8_t>(MPU_ADDR), static_cast<size_t>(14), true) != 14) {
    return false;
  }

  const int16_t rawAx = readI16();
  const int16_t rawAy = readI16();
  const int16_t rawAz = readI16();
  (void)readI16(); // Temperature is not used for posture detection.
  const int16_t rawGx = readI16();
  const int16_t rawGy = readI16();
  const int16_t rawGz = readI16();

  sample.bootMs = millis(); // Sensor read time rather than HTTP response time
  sample.seq = seq;
  sample.ax = rawAx / 16384.0f;
  sample.ay = rawAy / 16384.0f;
  sample.az = rawAz / 16384.0f;
  sample.gx = rawGx / 131.0f * DEG_TO_RAD;
  sample.gy = rawGy / 131.0f * DEG_TO_RAD;
  sample.gz = rawGz / 131.0f * DEG_TO_RAD;
  sample.accelNormG = sqrtf(sample.ax * sample.ax + sample.ay * sample.ay +
                            sample.az * sample.az);
  sample.gyroNormDps = sqrtf(sample.gx * sample.gx + sample.gy * sample.gy +
                             sample.gz * sample.gz) * RAD_TO_DEG;
  sample.steady = sample.accelNormG >= 0.90f &&
                  sample.accelNormG <= 1.10f &&
                  sample.gyroNormDps <= 10.0f;
  // Gravity on Y/Z estimates bending about the sensor X axis.
  sample.tiltDeg = atan2f(sample.ay, sample.az) * RAD_TO_DEG;
  return true;
}

void updatePosture() {
  if (calibrating) {
    posture = Posture::Calibrating;
    if (!sample.steady) {
      calibrationCount = 0;
      calibrationSin = 0;
      calibrationCos = 0;
      return;
    }
    const float radians = sample.tiltDeg * DEG_TO_RAD;
    calibrationSin += sinf(radians);
    calibrationCos += cosf(radians);
    if (++calibrationCount >= CALIBRATION_SAMPLES) {
      neutralTiltDeg = atan2f(calibrationSin, calibrationCos) * RAD_TO_DEG;
      calibrated = true;
      calibrating = false;
      wristFlexDeg = 0;
      pendingCount = 0;
      posture = Posture::Normal;
      Serial.print("# wrist_neutral_tilt_deg=");
      Serial.println(neutralTiltDeg, 2);
    }
    return;
  }

  if (!calibrated) {
    posture = Posture::Uncalibrated;
    return;
  }

  wristFlexDeg = FLEXION_SIGN *
                 angleDifferenceDeg(sample.tiltDeg, neutralTiltDeg);
  if (!sample.steady) {
    posture = Posture::Moving;
    pendingCount = 0;
    return;
  }

  Posture candidate = Posture::Normal;
  if (wristFlexDeg >= WRIST_LIMIT_DEG) candidate = Posture::Flexion;
  if (wristFlexDeg <= -WRIST_LIMIT_DEG) candidate = Posture::Extension;
  if (candidate == pendingPosture) {
    if (pendingCount < STATUS_SAMPLES) ++pendingCount;
  } else {
    pendingPosture = candidate;
    pendingCount = 1;
  }
  if (pendingCount >= STATUS_SAMPLES) posture = candidate;
}

void printCaptureRow() {
  // Nine fields accepted by tools/capture.py. Serial acceleration is m/s^2.
  Serial.print(SENSOR_ID);
  Serial.print(','); Serial.print(sample.bootMs);
  Serial.print(','); Serial.print(sample.seq);
  Serial.print(','); Serial.print(sample.ax * GRAVITY_MS2, 6);
  Serial.print(','); Serial.print(sample.ay * GRAVITY_MS2, 6);
  Serial.print(','); Serial.print(sample.az * GRAVITY_MS2, 6);
  Serial.print(','); Serial.print(sample.gx, 6);
  Serial.print(','); Serial.print(sample.gy, 6);
  Serial.print(','); Serial.println(sample.gz, 6);
}

void handleRoot() {
  server.sendHeader("Content-Encoding", "gzip");
  server.send_P(200, "text/html", (const char *)webpage_html_gz,
                webpage_html_gz_len);
}

void handleData() {
  // Preserve ax...gz and t for the webpage; the extra fields are for logging.
  String json;
  json.reserve(330);
  json = "{\"sensor_id\":\"";
  json += SENSOR_ID;
  json += "\",\"seq\":" + String(sample.seq);
  json += ",\"sample_ms\":" + String(sample.bootMs);
  json += ",\"ax\":" + String(sample.ax, 4);
  json += ",\"ay\":" + String(sample.ay, 4);
  json += ",\"az\":" + String(sample.az, 4);
  json += ",\"gx\":" + String(sample.gx, 5);
  json += ",\"gy\":" + String(sample.gy, 5);
  json += ",\"gz\":" + String(sample.gz, 5);
  json += ",\"t\":" + String(sample.bootMs / 1000.0f, 3);
  json += ",\"tilt_deg\":" + String(sample.tiltDeg, 2);
  json += ",\"neutral_tilt_deg\":" + String(neutralTiltDeg, 2);
  json += ",\"wrist_flex_deg\":" + String(wristFlexDeg, 2);
  json += ",\"calibration_samples\":" + String(calibrationCount);
  json += ",\"accel_norm_g\":" + String(sample.accelNormG, 3);
  json += ",\"gyro_norm_dps\":" + String(sample.gyroNormDps, 2);
  json += ",\"steady\":";
  json += (sample.steady ? "true" : "false");
  json += ",\"calibrated\":";
  json += (calibrated ? "true" : "false");
  json += ",\"valid\":";
  json += (lastReadOk ? "true" : "false");
  json += ",\"status\":\"";
  json += postureName(posture);
  json += "\"}";
  server.send(200, "application/json", json);
}

void handleCalibrate() {
  if (!sensorReady) {
    server.send(503, "application/json", "{\"error\":\"imu_not_ready\"}");
    return;
  }
  calibrated = false;
  calibrating = true;
  calibrationCount = 0;
  calibrationSin = 0;
  calibrationCos = 0;
  pendingCount = 0;
  wristFlexDeg = 0;
  posture = Posture::Calibrating;
  Serial.println("# calibration_started: hold the wrist neutral and still for 2 seconds");
  server.send(202, "application/json",
              "{\"status\":\"calibrating\",\"instruction\":\"Hold the wrist neutral and still for 2 seconds, then check /data\"}");
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  sensorReady = initializeMpu();
  Serial.println(sensorReady ? "# MPU6050 ready" : "# MPU6050 not found");
  Serial.println("sensor_id,boot_ms,seq,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z");

  delay(1000);
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  delay(100);
  WiFi.softAP("ESP32-IMU", nullptr, 1, 0, 4);
  delay(1000);
  Serial.print("# AP IP address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/calibrate", HTTP_GET, handleCalibrate);
  server.begin();
  Serial.println("# Open /calibrate while holding the wrist neutral, then read /data");
}

void loop() {
  server.handleClient();
  static uint32_t lastReadMs = 0;
  if (millis() - lastReadMs < SAMPLE_INTERVAL_MS) return;
  lastReadMs = millis();

  if (!sensorReady) {
    static uint32_t lastRetryMs = 0;
    if (millis() - lastRetryMs >= 1000) {
      lastRetryMs = millis();
      sensorReady = initializeMpu();
    }
    return;
  }

  ++sequence;
  lastReadOk = readMpu(sequence);
  if (!lastReadOk) return;
  updatePosture();
  printCaptureRow();
}
