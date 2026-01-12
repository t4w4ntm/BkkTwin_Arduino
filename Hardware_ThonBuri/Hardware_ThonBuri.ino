#include <WiFi.h>
#include "time.h"
#include "ThingSpeak.h"
#include "HardwareSerial.h"

// ====== เพิ่มสำหรับ SHT31 ======
#include <Wire.h>
#include <Adafruit_SHT31.h>
Adafruit_SHT31 sht31 = Adafruit_SHT31();
float shtTempC = NAN;

// ================== ข้อมูลการเชื่อมต่อของคุณ ==================
const char* mySSID = "Teacher@SNR";
const char* myPASSWORD = "snr@12345678@";
unsigned long myChannelNumber = 3027679;
const char * myWriteAPIKey = "OBMYI01Y33A9MH37";
// ==========================================================

// --- การตั้งค่า NTP ---
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 7 * 3600; // UTC+7
const int   daylightOffset_sec = 0;

HardwareSerial pmsSerial(2);

// --- ตัวแปรสำหรับเก็บค่าฝุ่น ---
int pm1 = 0;
int pm2_5 = 0;
int pm10 = 0;

WiFiClient client;

// ====== สถานะ/เงื่อนไขการตรวจจับเซนเซอร์ ======
bool pmsPresent = false;
bool shtPresent = false;

unsigned long lastPmsOkMillis = 0;
const unsigned long PMS_MISS_TIMEOUT = 10UL * 1000;

int shtConsecutiveNan = 0;
const int SHT_NAN_LIMIT = 3;
unsigned long lastStatusPrint = 0;

// ====== ส่งทุก 25 วินาที ======
const unsigned long SEND_INTERVAL_MS = 25UL * 1000;
unsigned long lastSendMillis = 0;

// --- ฟังก์ชัน prototypes ---
bool readPMValues();
void connectWiFi();
void printLocalTime();
void readSHT31();
void updateSensorPresenceFromPMS(bool ok);
void updateSensorPresenceFromSHT(bool ok);
void checkPMSHeartbeat();
void sendToThingSpeak();

// ================== ฟังก์ชันต่างๆ ==================
void readSHT31() {
  Wire.beginTransmission(0x44);
  uint8_t i2cErr = Wire.endTransmission();

  float t = sht31.readTemperature();
  if (i2cErr == 0 && !isnan(t)) {
    shtTempC = t;
    updateSensorPresenceFromSHT(true);
  } else {
    updateSensorPresenceFromSHT(false);
  }
}

void updateSensorPresenceFromSHT(bool ok) {
  if (ok) {
    if (!shtPresent) Serial.println("[SHT31] RE-ATTACHED (กลับมาตอบสนอง)");
    shtPresent = true;
    shtConsecutiveNan = 0;
  } else {
    shtConsecutiveNan++;
    if (shtConsecutiveNan >= SHT_NAN_LIMIT && shtPresent) {
      shtPresent = false;
      Serial.println("[SHT31] DETACHED (ไม่ตอบสนอง/อ่านไม่ได้ต่อเนื่อง)");
    }
  }
}

void updateSensorPresenceFromPMS(bool ok) {
  if (ok) {
    if (!pmsPresent) Serial.println("[PMS] RE-ATTACHED (เริ่มได้รับเฟรมถูกต้อง)");
    pmsPresent = true;
    lastPmsOkMillis = millis();
  }
}

void checkPMSHeartbeat() {
  if (pmsPresent && (millis() - lastPmsOkMillis > PMS_MISS_TIMEOUT)) {
    pmsPresent = false;
    Serial.println("[PMS] DETACHED (ไม่มีเฟรมถูกต้องนานเกินกำหนด)");
  }
}

bool readPMValues() {
  while (pmsSerial.available() && pmsSerial.peek() != 0x42) { pmsSerial.read(); }
  if (pmsSerial.available() >= 32) {
    byte buffer[32];
    pmsSerial.readBytes(buffer, 32);
    if (buffer[0] == 0x42 && buffer[1] == 0x4D) {
      pm1   = buffer[10] * 256 + buffer[11];
      pm2_5 = buffer[12] * 256 + buffer[13];
      pm10  = buffer[14] * 256 + buffer[15];
      updateSensorPresenceFromPMS(true);
      return true;
    }
  }
  return false;
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);

  Serial.print("Connecting to WiFi...");
  WiFi.begin(mySSID, myPASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - start > 20000) {
      Serial.println("\nWiFi connect timeout, retrying...");
      WiFi.disconnect(true);
      delay(500);
      WiFi.begin(mySSID, myPASSWORD);
      start = millis();
    }
  }
  Serial.println("\nWiFi connected!");
}

void printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  char timeString[50];
  strftime(timeString, sizeof(timeString), "%A, %B %d %Y %H:%M:%S", &timeinfo);
  Serial.print("Current Time (Thailand): ");
  Serial.println(timeString);
}

void sendToThingSpeak() {
  Serial.println(">> Sending data to ThingSpeak (Field1 PM2.5, Field4 Temp + status)...");

  ThingSpeak.setField(2, pm2_5);

  if (!isnan(shtTempC)) {
    ThingSpeak.setField(5, shtTempC);
  }

  String statusMsg = String("PMS=") + (pmsPresent ? "OK" : "MISSING")
                   + " | SHT31=" + (shtPresent ? "OK" : "MISSING");
  ThingSpeak.setStatus(statusMsg);

  int httpCode = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

  if (httpCode == 200) {
    Serial.println("ThingSpeak: OK (200)");
  } else {
    Serial.printf("ThingSpeak: FAIL (HTTP %d)\n", httpCode);
    client.stop(); // เคลียร์ socket ค้าง

    // ถ้าโดน -401 บ่อย แปลว่า server "ไม่ยอมรับจุดข้อมูล"
    // (พบบ่อย: ส่งถี่เกิน/มีตัวอื่นส่งร่วม/คีย์ไม่ตรง)
  }
}

void setup() {
  Serial.begin(115200);

  // PMS5003: RX/TX ของ MCU ใช้ 21/22 ตามของคุณ
  pmsSerial.begin(9600, SERIAL_8N1, 21, 22);

  Wire.begin(19, 18);
  if (!sht31.begin(0x44)) {
    shtPresent = false;
    Serial.println("SHT31 not found at 0x44. Check wiring (SDA=19, SCL=18).");
  } else {
    shtPresent = true;
    Serial.println("SHT31 initialized (I2C 0x44).");
  }

  connectWiFi();

  Serial.println("Configuring time from NTP server...");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  ThingSpeak.begin(client);

  unsigned long start = millis();
  while (millis() - start < 2000) {
    if (readPMValues()) break;
    delay(50);
  }
  if (pmsPresent) Serial.println("[PMS] INITIAL OK (พบเฟรมข้อมูล)");
  else Serial.println("[PMS] INITIAL MISSING (ยังไม่พบเฟรม ลองตรวจสาย/ไฟ)");

  // รอครบ interval ก่อนส่งครั้งแรก เพื่อลดโอกาสโดน -401 ตอนบูต
  lastSendMillis = millis();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  readSHT31();

  bool gotPM = readPMValues();
  if (gotPM) {
    Serial.printf("PMS Read: PM1.0=%d, PM2.5=%d, PM10=%d | ", pm1, pm2_5, pm10);
  }

  checkPMSHeartbeat();

  if (millis() - lastStatusPrint > 5000) {
    lastStatusPrint = millis();
    Serial.printf("[STATUS] PMS=%s | SHT31=%s | T=%.2f°C\n",
                  pmsPresent ? "OK" : "MISSING",
                  shtPresent ? "OK" : "MISSING",
                  isnan(shtTempC) ? -999.0 : shtTempC);
  }

  unsigned long now = millis();
  if (now - lastSendMillis >= SEND_INTERVAL_MS) {
    lastSendMillis = now;
    sendToThingSpeak();
  }

  delay(1000);
}
