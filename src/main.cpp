#include "webpage.h"
#include <Arduino.h>
#include <Preferences.h>
#include <stdlib.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <math.h>

#ifndef SENSOR_POINT_ID
#define SENSOR_POINT_ID 0
#endif
#ifndef SENSOR_SIDE_ID
#define SENSOR_SIDE_ID 0
#endif

constexpr uint8_t I2C_SDA_PIN = 8;
constexpr uint8_t I2C_SCL_PIN = 9;
constexpr uint8_t MPU_ADDR = 0x68;
constexpr uint32_t SAMPLE_INTERVAL_MS = 50;
constexpr uint16_t CALIBRATION_SAMPLES = 40;
constexpr uint32_t CALIBRATION_TIMEOUT_MS = 10000;
constexpr uint32_t CALIBRATION_MAX_GAP_MS = 120;
constexpr uint8_t CALIBRATION_METHOD_VERSION = 1;
constexpr uint16_t STATUS_SAMPLES = 10;
constexpr float WRIST_LIMIT_DEG = 12.0f;
constexpr float GRAVITY_MS2 = 9.80665f;
constexpr float RAD_TO_DEG_F = 57.29577951308232f;
constexpr float DEG_TO_RAD_F = 0.017453292519943295f;

const char *const MOUNT_SITES[] = {
    "wrist", "right_forearm", "left_forearm", "right_upper_arm",
    "left_upper_arm", "chest", "neck", "head", "lumbar", "pelvis",
    "left_thigh", "right_thigh", "left_shin", "right_shin", "shoulder",
    "elbow", "other"};
constexpr uint8_t MOUNT_SITE_COUNT = sizeof(MOUNT_SITES) / sizeof(MOUNT_SITES[0]);
const char *const SIDES[] = {"unknown", "left", "right"};
constexpr uint8_t SIDE_COUNT = sizeof(SIDES) / sizeof(SIDES[0]);
uint8_t activeSiteId = SENSOR_POINT_ID;

const char *mountSite() {
  return MOUNT_SITES[(activeSiteId < MOUNT_SITE_COUNT)
                         ? activeSiteId
                         : MOUNT_SITE_COUNT - 1];
}
const char *sensorSide() {
  if (activeSiteId == 2 || activeSiteId == 4 || activeSiteId == 10 || activeSiteId == 12) return "left";
  if (activeSiteId == 1 || activeSiteId == 3 || activeSiteId == 11 || activeSiteId == 13) return "right";
  if (activeSiteId != SENSOR_POINT_ID) return "unknown";
  return SIDES[(SENSOR_SIDE_ID >= 0 && SENSOR_SIDE_ID < SIDE_COUNT) ? SENSOR_SIDE_ID : 0];
}

WebServer server(80);
Preferences preferences;
String sensorUid;
uint32_t sequenceNumber = 0;
uint32_t bootId = 0;
uint32_t lastReadMs = 0;
bool sensorReady = false;
bool lastReadOk = false;

struct ImuSample {
  uint32_t bootMs = 0;
  uint32_t seq = 0;
  float ax = 0, ay = 0, az = 0; // raw accelerometer in g
  float gx = 0, gy = 0, gz = 0; // raw gyroscope in rad/s
  float accelNormG = 0;
  float gyroNormDps = 0;
  float rollDeg = 0, pitchDeg = 0;
  float correctedGx = 0, correctedGy = 0, correctedGz = 0;
  float relativeRollDeg = 0, relativePitchDeg = 0;
  bool steady = false;
};

struct CalibrationProfile {
  uint32_t schema = 3;
  uint16_t siteId = 0;
  uint8_t valid = 0;
  uint8_t methodVersion = CALIBRATION_METHOD_VERSION;
  char uid[20] = {};
  char calibrationId[40] = {};
  float neutralRollDeg = 0;
  float neutralPitchDeg = 0;
  float gyroBias[3] = {};
  uint32_t completedBootMs = 0;
  uint32_t completedBootId = 0;
};

struct CalibrationTask {
  bool active = false;
  uint16_t count = 0;
  uint32_t startedMs = 0;
  uint32_t previousSampleMs = 0;
  float rollSin = 0, rollCos = 0;
  float pitchSin = 0, pitchCos = 0;
  float gyroSum[3] = {};
  String status = "idle";
  String message = "ยังไม่ได้เริ่ม calibration";
};

ImuSample sample;
CalibrationProfile calibration;
CalibrationTask calibrationTask;
enum class FirmwarePose { Uncalibrated, Calibrating, Moving, Normal, Flexion, Extension, UnsupportedSite };
FirmwarePose firmwarePose = FirmwarePose::Uncalibrated;
FirmwarePose pendingPose = FirmwarePose::Uncalibrated;
uint16_t pendingPoseCount = 0;
float wristFlexDeg = 0;
uint32_t lastCalibrationCounter = 0;

constexpr size_t LIVE_HISTORY_CAPACITY = 64;
constexpr size_t LIVE_BATCH_LIMIT = 32;
struct LiveHistoryEntry {
  ImuSample sample;
  uint32_t bootId = 0;
  uint8_t mountSiteId = 0;
  bool valid = false;
  bool calibrated = false;
  bool relativeValid = false;
  uint8_t calibrationMethodVersion = 0;
  uint32_t calibrationTimeBootMs = 0;
  uint32_t calibrationBootId = 0;
  FirmwarePose pose = FirmwarePose::Uncalibrated;
  float wristFlex = 0;
  char sensorUid[20] = {};
  char calibrationId[40] = {};
  char side[8] = {};
};
LiveHistoryEntry liveHistory[LIVE_HISTORY_CAPACITY];
size_t liveHistoryNext = 0;
size_t liveHistoryCount = 0;

bool activeCalibrationValid() {
  return calibration.valid && calibration.siteId == activeSiteId && sensorUid.equals(calibration.uid);
}

float angleDifferenceDeg(float angle, float reference) {
  const float radians = (angle - reference) * DEG_TO_RAD_F;
  return atan2f(sinf(radians), cosf(radians)) * RAD_TO_DEG_F;
}

const char *firmwarePoseName(FirmwarePose value) {
  switch (value) {
    case FirmwarePose::Calibrating: return "calibrating";
    case FirmwarePose::Moving: return "moving";
    case FirmwarePose::Normal: return "normal";
    case FirmwarePose::Flexion: return "wrist_flexion";
    case FirmwarePose::Extension: return "wrist_extension";
    case FirmwarePose::UnsupportedSite: return "threshold_not_configured_for_site";
    default: return "uncalibrated";
  }
}

LiveHistoryEntry currentLiveHistoryEntry() {
  LiveHistoryEntry entry;
  entry.sample = sample;
  entry.bootId = bootId;
  entry.mountSiteId = activeSiteId;
  entry.valid = lastReadOk;
  entry.calibrated = activeCalibrationValid();
  entry.relativeValid = entry.calibrated && activeSiteId == 0;
  entry.calibrationMethodVersion = entry.calibrated ? calibration.methodVersion : 0;
  entry.calibrationTimeBootMs = entry.calibrated ? calibration.completedBootMs : 0;
  entry.calibrationBootId = entry.calibrated ? calibration.completedBootId : 0;
  entry.pose = firmwarePose;
  entry.wristFlex = wristFlexDeg;
  sensorUid.toCharArray(entry.sensorUid, sizeof(entry.sensorUid));
  if (entry.calibrated) {
    snprintf(entry.calibrationId, sizeof(entry.calibrationId), "%s", calibration.calibrationId);
  }
  String side = sensorSide();
  side.toCharArray(entry.side, sizeof(entry.side));
  return entry;
}

void rememberLiveSample() {
  liveHistory[liveHistoryNext] = currentLiveHistoryEntry();
  liveHistoryNext = (liveHistoryNext + 1) % LIVE_HISTORY_CAPACITY;
  if (liveHistoryCount < LIVE_HISTORY_CAPACITY) ++liveHistoryCount;
}

String liveSampleJson(const LiveHistoryEntry &entry) {
  const ImuSample &value = entry.sample;
  String json;
  json.reserve(850);
  json = "{\"sensor_uid\":\"" + String(entry.sensorUid) + "\",\"sensor_id\":\"" + String(entry.sensorUid) + "\"";
  json += ",\"mount_site\":\"" + String(MOUNT_SITES[(entry.mountSiteId < MOUNT_SITE_COUNT) ? entry.mountSiteId : MOUNT_SITE_COUNT - 1]) + "\"";
  json += ",\"mount_site_id\":" + String(entry.mountSiteId) + ",\"side\":\"" + String(entry.side) + "\"";
  json += ",\"seq\":" + String(value.seq) + ",\"sample_ms\":" + String(value.bootMs) + ",\"boot_id\":" + String(entry.bootId);
  json += ",\"ax\":" + String(value.ax, 6) + ",\"ay\":" + String(value.ay, 6) + ",\"az\":" + String(value.az, 6);
  json += ",\"gx\":" + String(value.gx, 7) + ",\"gy\":" + String(value.gy, 7) + ",\"gz\":" + String(value.gz, 7);
  json += ",\"gx_corrected\":" + String(value.correctedGx, 7) + ",\"gy_corrected\":" + String(value.correctedGy, 7) + ",\"gz_corrected\":" + String(value.correctedGz, 7);
  json += ",\"t\":" + String(value.bootMs / 1000.0f, 3);
  json += ",\"accel_norm_g\":" + String(value.accelNormG, 4) + ",\"gyro_norm_dps\":" + String(value.gyroNormDps, 3);
  json += ",\"roll_deg\":" + String(value.rollDeg, 3) + ",\"pitch_deg\":" + String(value.pitchDeg, 3);
  json += ",\"relative_roll_deg\":" + String(value.relativeRollDeg, 3) + ",\"relative_pitch_deg\":" + String(value.relativePitchDeg, 3);
  json += ",\"wrist_flex_deg\":" + String(entry.wristFlex, 3);
  json += ",\"relative_angle_valid\":" + String(entry.relativeValid ? "true" : "false");
  json += ",\"relative_orientation_valid\":" + String(entry.calibrated ? "true" : "false");
  json += ",\"calibration_id\":\"" + String(entry.calibrationId) + "\"";
  json += ",\"calibration_method_version\":" + String(entry.calibrationMethodVersion);
  json += ",\"calibration_time_boot_ms\":" + String(entry.calibrationTimeBootMs);
  json += ",\"calibration_boot_id\":" + String(entry.calibrationBootId);
  json += ",\"calibrated\":" + String(entry.calibrated ? "true" : "false");
  json += ",\"steady\":" + String(value.steady ? "true" : "false");
  json += ",\"valid\":" + String(entry.valid ? "true" : "false");
  json += ",\"status\":\"" + String(firmwarePoseName(entry.pose)) + "\"}";
  return json;
}

bool writeMpuRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

bool initializeMpu() {
  return writeMpuRegister(0x6B, 0x00) && // wake
         writeMpuRegister(0x1C, 0x00) && // accelerometer +/-2g
         writeMpuRegister(0x1B, 0x00);   // gyro +/-250 deg/s
}

int16_t readI16() {
  const uint16_t high = static_cast<uint8_t>(Wire.read());
  const uint16_t low = static_cast<uint8_t>(Wire.read());
  return static_cast<int16_t>((high << 8) | low);
}

bool readMpu(uint32_t seq) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom(static_cast<uint8_t>(MPU_ADDR), static_cast<size_t>(14), true) != 14) return false;

  const int16_t rawAx = readI16(), rawAy = readI16(), rawAz = readI16();
  (void)readI16();
  const int16_t rawGx = readI16(), rawGy = readI16(), rawGz = readI16();
  sample.bootMs = millis();
  sample.seq = seq;
  sample.ax = rawAx / 16384.0f;
  sample.ay = rawAy / 16384.0f;
  sample.az = rawAz / 16384.0f;
  sample.gx = rawGx / 131.0f * DEG_TO_RAD_F;
  sample.gy = rawGy / 131.0f * DEG_TO_RAD_F;
  sample.gz = rawGz / 131.0f * DEG_TO_RAD_F;
  sample.accelNormG = sqrtf(sample.ax * sample.ax + sample.ay * sample.ay + sample.az * sample.az);
  sample.gyroNormDps = sqrtf(sample.gx * sample.gx + sample.gy * sample.gy + sample.gz * sample.gz) * RAD_TO_DEG_F;
  sample.rollDeg = atan2f(sample.ay, sample.az) * RAD_TO_DEG_F;
  sample.pitchDeg = atan2f(-sample.ax, sqrtf(sample.ay * sample.ay + sample.az * sample.az)) * RAD_TO_DEG_F;
  const bool calibratedHere = activeCalibrationValid();
  sample.correctedGx = sample.gx - (calibratedHere ? calibration.gyroBias[0] : 0.0f);
  sample.correctedGy = sample.gy - (calibratedHere ? calibration.gyroBias[1] : 0.0f);
  sample.correctedGz = sample.gz - (calibratedHere ? calibration.gyroBias[2] : 0.0f);
  sample.relativeRollDeg = calibratedHere ? angleDifferenceDeg(sample.rollDeg, calibration.neutralRollDeg) : 0.0f;
  sample.relativePitchDeg = calibratedHere ? angleDifferenceDeg(sample.pitchDeg, calibration.neutralPitchDeg) : 0.0f;
  const float steadyGx = calibratedHere ? sample.correctedGx : sample.gx;
  const float steadyGy = calibratedHere ? sample.correctedGy : sample.gy;
  const float steadyGz = calibratedHere ? sample.correctedGz : sample.gz;
  const float steadyRate = sqrtf(steadyGx * steadyGx + steadyGy * steadyGy + steadyGz * steadyGz) * RAD_TO_DEG_F;
  sample.steady = sample.accelNormG >= 0.90f && sample.accelNormG <= 1.10f && steadyRate <= 10.0f;
  return true;
}

void clearCandidate() {
  calibrationTask.count = 0;
  calibrationTask.rollSin = calibrationTask.rollCos = 0;
  calibrationTask.pitchSin = calibrationTask.pitchCos = 0;
  for (float &value : calibrationTask.gyroSum) value = 0;
}

void finishCalibration(const char *status, const char *message) {
  calibrationTask.active = false;
  calibrationTask.status = status;
  calibrationTask.message = message;
  calibrationTask.previousSampleMs = 0;
}

void updateCalibration(bool readOk) {
  if (!calibrationTask.active) return;
  if (millis() - calibrationTask.startedMs > CALIBRATION_TIMEOUT_MS) {
    finishCalibration("timeout", "หมดเวลา: baseline เดิมยังคงถูกใช้งาน");
    return;
  }
  if (!readOk) {
    clearCandidate();
    calibrationTask.previousSampleMs = 0;
    calibrationTask.status = "collecting";
    calibrationTask.message = "อ่าน IMU ไม่สำเร็จ กำลังเริ่มนับช่วงนิ่งใหม่";
    return;
  }
  if (calibrationTask.previousSampleMs != 0 &&
      sample.bootMs - calibrationTask.previousSampleMs > CALIBRATION_MAX_GAP_MS) {
    clearCandidate();
  }
  calibrationTask.previousSampleMs = sample.bootMs;
  if (!sample.steady) {
    clearCandidate();
    calibrationTask.status = "collecting";
    calibrationTask.message = "ตรวจพบการขยับ: อยู่นิ่งในท่ากลางเพื่อเริ่มนับใหม่";
    return;
  }
  const float roll = sample.rollDeg * DEG_TO_RAD_F;
  const float pitch = sample.pitchDeg * DEG_TO_RAD_F;
  calibrationTask.rollSin += sinf(roll); calibrationTask.rollCos += cosf(roll);
  calibrationTask.pitchSin += sinf(pitch); calibrationTask.pitchCos += cosf(pitch);
  calibrationTask.gyroSum[0] += sample.gx;
  calibrationTask.gyroSum[1] += sample.gy;
  calibrationTask.gyroSum[2] += sample.gz;
  ++calibrationTask.count;
  calibrationTask.status = "collecting";
  calibrationTask.message = "กำลังเก็บตัวอย่างท่ากลางต่อเนื่อง";
  if (calibrationTask.count < CALIBRATION_SAMPLES) return;

  CalibrationProfile candidate = {};
  candidate.schema = 3;
  candidate.siteId = activeSiteId;
  candidate.valid = 1;
  candidate.methodVersion = CALIBRATION_METHOD_VERSION;
  sensorUid.toCharArray(candidate.uid, sizeof(candidate.uid));
  candidate.neutralRollDeg = atan2f(calibrationTask.rollSin, calibrationTask.rollCos) * RAD_TO_DEG_F;
  candidate.neutralPitchDeg = atan2f(calibrationTask.pitchSin, calibrationTask.pitchCos) * RAD_TO_DEG_F;
  candidate.completedBootMs = millis();
  candidate.completedBootId = bootId;
  for (uint8_t axis = 0; axis < 3; ++axis) candidate.gyroBias[axis] = calibrationTask.gyroSum[axis] / calibrationTask.count;
  preferences.begin("imu_cal", false);
  lastCalibrationCounter = preferences.getUInt("counter", 0) + 1;
  preferences.putUInt("counter", lastCalibrationCounter);
  preferences.end();
  snprintf(candidate.calibrationId, sizeof(candidate.calibrationId), "cal-%s-%lu",
           sensorUid.substring(sensorUid.length() > 8 ? sensorUid.length() - 8 : 0).c_str(),
           static_cast<unsigned long>(lastCalibrationCounter));
  preferences.begin("imu_cal", false);
  const size_t saved = preferences.putBytes("profile", &candidate, sizeof(candidate));
  preferences.end();
  if (saved != sizeof(candidate)) {
    finishCalibration("save_error", "บันทึก baseline ไม่สำเร็จ: baseline เดิมยังคงถูกใช้");
    return;
  }
  calibration = candidate;
  wristFlexDeg = 0;
  pendingPoseCount = 0;
  finishCalibration("complete", "สำเร็จ: ใช้ baseline ใหม่แล้ว");
  Serial.print("# calibration_id="); Serial.println(calibration.calibrationId);
}

void loadCalibration() {
  activeSiteId = (SENSOR_POINT_ID >= 0 && SENSOR_POINT_ID < MOUNT_SITE_COUNT)
                     ? static_cast<uint8_t>(SENSOR_POINT_ID)
                     : static_cast<uint8_t>(MOUNT_SITE_COUNT - 1);
  preferences.begin("imu_cfg", true);
  const uint8_t savedSite = preferences.getUChar("mount", activeSiteId);
  preferences.end();
  if (savedSite < MOUNT_SITE_COUNT) activeSiteId = savedSite;
  preferences.begin("imu_cal", true);
  if (preferences.getBytesLength("profile") == sizeof(CalibrationProfile)) {
    CalibrationProfile saved = {};
    preferences.getBytes("profile", &saved, sizeof(saved));
    if (saved.schema == 3 && saved.valid && sensorUid.equals(saved.uid)) calibration = saved;
  }
  preferences.end();
  if (!calibration.valid) return;
  preferences.begin("imu_cal", true);
  lastCalibrationCounter = preferences.getUInt("counter", 0);
  preferences.end();
}

void refreshCalibratedSample() {
  const bool calibratedHere = activeCalibrationValid();
  sample.correctedGx = sample.gx - (calibratedHere ? calibration.gyroBias[0] : 0.0f);
  sample.correctedGy = sample.gy - (calibratedHere ? calibration.gyroBias[1] : 0.0f);
  sample.correctedGz = sample.gz - (calibratedHere ? calibration.gyroBias[2] : 0.0f);
  sample.relativeRollDeg = calibratedHere ? angleDifferenceDeg(sample.rollDeg, calibration.neutralRollDeg) : 0.0f;
  sample.relativePitchDeg = calibratedHere ? angleDifferenceDeg(sample.pitchDeg, calibration.neutralPitchDeg) : 0.0f;
}

void updateFirmwarePose() {
  if (calibrationTask.active) { firmwarePose = FirmwarePose::Calibrating; return; }
  if (!activeCalibrationValid()) { firmwarePose = FirmwarePose::Uncalibrated; return; }
  // Only the known wrist mounting profile has a defined flexion axis/sign.
  if (activeSiteId != 0) { firmwarePose = FirmwarePose::UnsupportedSite; return; }
  wristFlexDeg = -sample.relativeRollDeg;
  if (!sample.steady) {
    firmwarePose = FirmwarePose::Moving;
    pendingPoseCount = 0;
    return;
  }
  FirmwarePose candidate = FirmwarePose::Normal;
  if (wristFlexDeg >= WRIST_LIMIT_DEG) candidate = FirmwarePose::Flexion;
  if (wristFlexDeg <= -WRIST_LIMIT_DEG) candidate = FirmwarePose::Extension;
  if (candidate == pendingPose) {
    if (pendingPoseCount < STATUS_SAMPLES) ++pendingPoseCount;
  } else { pendingPose = candidate; pendingPoseCount = 1; }
  if (pendingPoseCount >= STATUS_SAMPLES) firmwarePose = candidate;
}

void printCaptureRow() {
  Serial.print(sensorUid); Serial.print(','); Serial.print(sample.bootMs); Serial.print(','); Serial.print(sample.seq);
  Serial.print(','); Serial.print(sample.ax, 6); Serial.print(','); Serial.print(sample.ay, 6); Serial.print(','); Serial.print(sample.az, 6);
  Serial.print(','); Serial.print(sample.gx, 7); Serial.print(','); Serial.print(sample.gy, 7); Serial.print(','); Serial.print(sample.gz, 7);
  const bool calibratedHere = activeCalibrationValid();
  Serial.print(','); Serial.print(calibratedHere ? calibration.calibrationId : "");
  Serial.print(','); Serial.print(calibratedHere ? "true" : "false");
  Serial.print(','); Serial.print(firmwarePoseName(firmwarePose));
  Serial.print(','); Serial.print(sample.rollDeg, 4); Serial.print(','); Serial.print(sample.pitchDeg, 4);
  Serial.print(','); Serial.print(sample.correctedGx, 7); Serial.print(','); Serial.print(sample.correctedGy, 7); Serial.print(','); Serial.print(sample.correctedGz, 7);
  Serial.print(','); Serial.print(sample.relativeRollDeg, 4); Serial.print(','); Serial.print(sample.relativePitchDeg, 4);
  Serial.print(','); Serial.print(calibratedHere && activeSiteId == 0 ? "true" : "false");
  Serial.print(','); Serial.print(calibratedHere ? "true" : "false");
  Serial.print(','); Serial.print(calibratedHere ? calibration.methodVersion : 0);
  Serial.print(','); Serial.print(calibratedHere ? calibration.completedBootMs : 0);
  Serial.print(','); Serial.println(calibratedHere ? calibration.completedBootId : 0);
}

void handleRoot() {
  server.sendHeader("Content-Encoding", "gzip");
  server.send_P(200, "text/html", reinterpret_cast<const char *>(webpage_html_gz), webpage_html_gz_len);
}

void handleData() {
  server.send(lastReadOk ? 200 : 503, "application/json", liveSampleJson(currentLiveHistoryEntry()));
}

void handleSampleBatch() {
  const uint32_t requestedBootId = server.hasArg("boot_id")
                                       ? static_cast<uint32_t>(strtoul(server.arg("boot_id").c_str(), nullptr, 10))
                                       : 0;
  const bool sameBoot = server.hasArg("boot_id") && requestedBootId == bootId;
  const uint32_t afterSeq = sameBoot && server.hasArg("after_seq")
                                ? static_cast<uint32_t>(strtoul(server.arg("after_seq").c_str(), nullptr, 10))
                                : 0;
  const size_t oldestIndex = (liveHistoryNext + LIVE_HISTORY_CAPACITY - liveHistoryCount) % LIVE_HISTORY_CAPACITY;
  const size_t firstOffset = liveHistoryCount > LIVE_BATCH_LIMIT ? liveHistoryCount - LIVE_BATCH_LIMIT : 0;
  const uint32_t firstBatchSeq = liveHistoryCount
                                     ? liveHistory[(oldestIndex + firstOffset) % LIVE_HISTORY_CAPACITY].sample.seq
                                     : 0;
  const size_t latestIndex = liveHistoryCount
                                 ? (liveHistoryNext + LIVE_HISTORY_CAPACITY - 1) % LIVE_HISTORY_CAPACITY
                                 : 0;
  const uint32_t latestSeq = liveHistoryCount ? liveHistory[latestIndex].sample.seq : 0;
  const bool dropped = sameBoot && afterSeq > 0 && liveHistoryCount &&
                       firstBatchSeq > afterSeq + 1 && latestSeq > afterSeq;

  String json;
  json.reserve(16000);
  json = "{\"boot_id\":" + String(bootId) + ",\"latest_seq\":" + String(latestSeq) +
         ",\"dropped\":" + String(dropped ? "true" : "false") + ",\"samples\":[";
  bool first = true;
  for (size_t offset = firstOffset; offset < liveHistoryCount; ++offset) {
    const size_t index = (oldestIndex + offset) % LIVE_HISTORY_CAPACITY;
    const LiveHistoryEntry &entry = liveHistory[index];
    if (sameBoot && entry.sample.seq <= afterSeq) continue;
    if (!entry.valid) continue;
    if (!first) json += ',';
    json += liveSampleJson(entry);
    first = false;
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleCalibrationPage() {
  String page = R"CAL(<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>IMU Calibration</title><style>body{font:16px system-ui;max-width:600px;margin:32px auto;padding:16px;background:#101820;color:#edf5f8}main{padding:22px;background:#182631;border-radius:14px}label{display:grid;gap:8px}select,button{padding:12px;border:1px solid #38515e;border-radius:8px;background:#0d151e;color:#edf5f8;font:inherit}button{background:#287c69;cursor:pointer}button:disabled{opacity:.5;cursor:default}progress{width:100%;height:18px}small{color:#afc0c9}</style><main><h1>Set IMU neutral baseline</h1><p>Device: <b id="uid">?</b></p><p>Side: <b id="side">?</b></p><label>Physical sensor mount position<select id="site">{{SITE_OPTIONS}}</select></label><p>Place the IMU at the selected body position in the neutral pose. Hold still while 40 samples are collected (about 2 seconds).</p><button id="start">Start calibration</button><p><progress id="progress" max="40" value="0"></progress></p><p id="status">Reading device status?</p><p><small>Gyroscope bias and neutral orientation are stored only after a successful calibration. Motion, read failures, or sample gaps restart the still interval. The previous profile stays stored until success. The time below is device uptime because the ESP has no real-time clock.</small></p></main><script>const $=id=>document.getElementById(id);async function poll(){try{const r=await fetch('/api/calibration/status',{cache:'no-store'}),d=await r.json();$('uid').textContent=d.sensor_uid;$('side').textContent=d.side;$('site').value=String(d.mount_site_id);$('site').disabled=d.status==='collecting';$('progress').value=d.samples;$('start').disabled=d.status==='collecting';$('status').textContent=d.message+' ('+d.samples+'/'+d.required_samples+') ? '+d.mount_site+' ? baseline: '+(d.calibrated?d.calibration_id:'not calibrated for this position')+(d.calibrated?' ? method v'+d.calibration_method_version+' ? uptime '+d.calibration_time_boot_ms+' ms / boot '+d.calibration_boot_id:'');}catch(e){$('status').textContent='Could not read status: '+e.message;}}$('start').onclick=async()=>{try{const r=await fetch('/api/calibration/start',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({mount_site:$('site').value})}),d=await r.json();$('status').textContent=d.message||d.error;await poll();}catch(e){$('status').textContent='Could not start: '+e.message;}};poll();setInterval(poll,250);</script></html>)CAL";
  page.replace("<h1>Set IMU neutral baseline</h1>",
               "<h1>Set IMU position and neutral baseline</h1>");
  page.replace("Place the IMU at the selected body position in the neutral pose. Hold still while 40 samples are collected (about 2 seconds).",
               "Choose the IMU's physical mounting position here. The position is saved when calibration starts. Move the IMU to that body location, hold a neutral pose, and keep still while 40 samples are collected.");
  page.replace("const $=id=>document.getElementById(id);async function poll()",
               "const $=id=>document.getElementById(id);let siteDirty=false;$('site').addEventListener('change',()=>{siteDirty=true;});async function poll()");
  page.replace("$('site').value=String(d.mount_site_id);",
               "if(!siteDirty||d.status==='collecting')$('site').value=String(d.mount_site_id);");
  page.replace("$('status').textContent=d.message||d.error;await poll();",
               "$('status').textContent=d.message||d.error;if(r.ok)siteDirty=false;await poll();");
  String options;
  for (uint8_t index = 0; index < MOUNT_SITE_COUNT; ++index) {
    options += "<option value=\"" + String(index) + "\">" + String(MOUNT_SITES[index]) + "</option>";
  }
  page.replace("{{SITE_OPTIONS}}", options);
  server.send(200, "text/html; charset=utf-8", page);
}

void handleCalibrationStart() {
  if (!sensorReady) {
    server.send(503, "application/json", "{\"error\":\"imu_not_ready\",\"message\":\"IMU is not ready\"}");
    return;
  }
  if (calibrationTask.active) {
    server.send(409, "application/json", "{\"error\":\"already_calibrating\"}");
    return;
  }

  uint8_t selectedSite = activeSiteId;
  if (server.hasArg("mount_site")) {
    const String requestedText = server.arg("mount_site");
    const int requestedSite = requestedText.toInt();
    if (requestedText != String(requestedSite) || requestedSite < 0 || requestedSite >= MOUNT_SITE_COUNT) {
      server.send(400, "application/json", "{\"error\":\"invalid_mount_site\"}");
      return;
    }
    selectedSite = static_cast<uint8_t>(requestedSite);
  }
  if (selectedSite != activeSiteId) {
    preferences.begin("imu_cfg", false);
    const size_t written = preferences.putUChar("mount", selectedSite);
    preferences.end();
    if (written != sizeof(uint8_t)) {
      server.send(500, "application/json", "{\"error\":\"mount_site_save_failed\"}");
      return;
    }
    activeSiteId = selectedSite;
  }

  calibrationTask = CalibrationTask{};
  calibrationTask.active = true;
  calibrationTask.startedMs = millis();
  calibrationTask.status = "collecting";
  calibrationTask.message = "Hold still in the neutral pose";
  server.send(202, "application/json",
              "{\"status\":\"collecting\",\"message\":\"Hold still in the neutral pose\"}");
}

void handleCalibrationStatus() {
  const bool calibratedHere = activeCalibrationValid();
  String json = "{\"sensor_uid\":\"" + sensorUid + "\",\"mount_site\":\"" + String(mountSite()) +
                "\",\"mount_site_id\":" + String(activeSiteId) + ",\"side\":\"" + String(sensorSide()) + "\"";
  json += ",\"status\":\"" + calibrationTask.status + "\",\"message\":\"" + calibrationTask.message + "\"";
  json += ",\"samples\":" + String(calibrationTask.count) + ",\"required_samples\":" + String(CALIBRATION_SAMPLES);
  json += ",\"calibrated\":" + String(calibratedHere ? "true" : "false");
  json += ",\"calibration_id\":\"" + String(calibratedHere ? calibration.calibrationId : "") + "\"";
  json += ",\"calibration_method_version\":" + String(calibratedHere ? calibration.methodVersion : 0);
  json += ",\"calibration_time_boot_ms\":" + String(calibratedHere ? calibration.completedBootMs : 0);
  json += ",\"calibration_boot_id\":" + String(calibratedHere ? calibration.completedBootId : 0);
  json += ",\"relative_angle_valid\":" + String(calibratedHere && activeSiteId == 0 ? "true" : "false") + "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  sensorReady = initializeMpu();
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.softAP("ESP32-IMU", nullptr, 1, 0, 4);
  sensorUid = WiFi.macAddress();
  sensorUid.replace(":", "");
  sensorUid.toLowerCase();
  bootId = esp_random();
  loadCalibration();
  Serial.println("# sensor_uid,boot_ms,seq,accel_x_g,accel_y_g,accel_z_g,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,calibration_id,calibrated,firmware_status,roll_deg,pitch_deg,corrected_gyro_x,corrected_gyro_y,corrected_gyro_z,relative_roll_deg,relative_pitch_deg,relative_angle_valid,relative_orientation_valid,calibration_method_version,calibration_time_boot_ms,calibration_boot_id");
  Serial.print("# sensor_uid="); Serial.println(sensorUid);
  Serial.print("# mount_site="); Serial.println(mountSite());
  Serial.print("# IMU "); Serial.println(sensorReady ? "ready" : "not found");
  Serial.print("# AP IP address: "); Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/samples", HTTP_GET, handleSampleBatch);
  server.on("/calibrate", HTTP_GET, handleCalibrationPage);
  server.on("/calibate", HTTP_GET, []() { server.sendHeader("Location", "/calibrate", true); server.send(302, "text/plain", "Redirecting to /calibrate"); });
  server.on("/api/calibration/start", HTTP_POST, handleCalibrationStart);
  server.on("/api/calibration/status", HTTP_GET, handleCalibrationStatus);
  server.begin();
}

void loop() {
  server.handleClient();
  const uint32_t now = millis();
  if (now - lastReadMs < SAMPLE_INTERVAL_MS) return;
  lastReadMs = now;
  if (!sensorReady) {
    static uint32_t lastRetryMs = 0;
    if (now - lastRetryMs >= 1000) { lastRetryMs = now; sensorReady = initializeMpu(); }
    updateCalibration(false);
    lastReadOk = false;
    return;
  }
  ++sequenceNumber;
  lastReadOk = readMpu(sequenceNumber);
  if (!lastReadOk) {
    updateCalibration(false);
    return;
  }
  updateCalibration(true);
  refreshCalibratedSample();
  updateFirmwarePose();
  rememberLiveSample();
  printCaptureRow();
}
