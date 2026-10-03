#include "webpage.h" // ไฟล์เว็บที่บีบอัดแล้ว (ดึงมาจากไฟล์ index2.html ของคุณ)
#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>

#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define MPU_ADDR 0x68 // I2C address of the MPU sensor

// ตัวแปรเก็บค่าดิบและค่าทางฟิสิกส์
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float ax, ay, az, gx, gy, gz, temp_c;

// สร้าง WebServer พอร์ต 80
WebServer server(80);

void handleRoot() {
  // ส่งหน้าเว็บออกไปแบบบีบอัด (gzip) เพื่อให้ทำงานไวขึ้น
  server.sendHeader("Content-Encoding", "gzip");
  server.send_P(200, "text/html", (const char *)webpage_html_gz,
                webpage_html_gz_len);
}

void handleData() {
  // สร้าง JSON คืนค่าเซ็นเซอร์ทั้งหมด
  String json = "{";
  json += "\"ax\":" + String(ax, 3) + ",";
  json += "\"ay\":" + String(ay, 3) + ",";
  json += "\"az\":" + String(az, 3) + ",";
  json += "\"gx\":" + String(gx, 3) + ",";
  json += "\"gy\":" + String(gy, 3) + ",";
  json += "\"gz\":" + String(gz, 3) + ",";
  json += "\"t\":" + String(millis() / 1000.0, 3); // วินาที
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);

  // หน่วงเวลา 3 วินาทีเต็มๆ เพื่อให้ Serial Monitor เปิดและจับภาพหน้าจอทันเผื่อบอร์ดรีบูท
  delay(3000);
  Serial.println("\n\n=================================");
  Serial.println("--- Starting MPU & Access Point ---");
  Serial.println("=================================");

  // เริ่มต้น I2C
  Serial.println("[1] Initializing I2C...");
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000); // 100kHz
  delay(100);

  // ปลุก MPU ให้ตื่น
  Serial.println("[2] Waking up MPU6050...");
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  Serial.println("    MPU is ready.");

  // ลองเว้นระยะก่อนเปิด WiFi ป้องกันไฟกระชาก (Brownout)
  delay(1000);

  // ตั้งค่า Access Point
  Serial.println("[3] Starting WiFi Access Point (AP)...");
  
  WiFi.disconnect(true);
  delay(100);
  
  WiFi.mode(WIFI_AP);
  // ลดกำลังส่ง WiFi ลงเพื่อป้องกันไฟตก (Brownout) ที่พบบ่อยในบอร์ด SuperMini
  WiFi.setTxPower(WIFI_POWER_8_5dBm); 
  delay(100);
  
  // กำหนด Channel 1 อย่างชัดเจน, รหัสผ่านไม่มี, ไม่ซ่อน (0), รับได้สูงสุด 4 เครื่อง
  WiFi.softAP("ESP32-IMU", NULL, 1, 0, 4); 

  delay(1000);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("    AP IP address: ");
  Serial.println(IP);

  // เริ่ม Web Server
  Serial.println("[4] Starting Web Server...");
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  Serial.println(
      "    HTTP server started! Connect to WiFi and visit the IP above.");
}

void loop() {
  // ให้ Server คอยตอบสนอง
  server.handleClient();

  // ควรอ่านเซ็นเซอร์ประมาณ 20 ครั้งต่อวินาที (ทุกๆ 50ms)
  static unsigned long lastRead = 0;
  if (millis() - lastRead > 50) {
    lastRead = millis();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, true);

    if (Wire.available() == 14) {
      AcX = Wire.read() << 8 | Wire.read();
      AcY = Wire.read() << 8 | Wire.read();
      AcZ = Wire.read() << 8 | Wire.read();
      Tmp = Wire.read() << 8 | Wire.read();
      GyX = Wire.read() << 8 | Wire.read();
      GyY = Wire.read() << 8 | Wire.read();
      GyZ = Wire.read() << 8 | Wire.read();

      // Accelerometer (g)
      ax = AcX / 16384.0;
      ay = AcY / 16384.0;
      az = AcZ / 16384.0;

      // Gyroscope แปลงจาก deg/s เป็น rad/s (เว็บต้องการ rad/s เพื่อนำไปคำนวณ)
      gx = (GyX / 131.0) * (PI / 180.0);
      gy = (GyY / 131.0) * (PI / 180.0);
      gz = (GyZ / 131.0) * (PI / 180.0);
    }
  }
}
