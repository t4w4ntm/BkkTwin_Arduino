#include <WiFi.h>
#include "time.h"
#include "ThingSpeak.h"
#include "HardwareSerial.h"

// ====== เพิ่มสำหรับ SHT31 ======
#include <Wire.h>
#include <Adafruit_SHT31.h>
Adafruit_SHT31 sht31 = Adafruit_SHT31();
float shtTempC = NAN; // จะส่งไป Field 4

// ================== ข้อมูลการเชื่อมต่อของคุณ ==================
const char* mySSID = "ZernZ_2G";
const char* myPASSWORD = "WaritZern12345";
unsigned long myChannelNumber = 3192372;
const char * myWriteAPIKey = "Z2D5YK92NWM760ZK";
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
const unsigned long PMS_MISS_TIMEOUT = 10UL * 1000; // ถ้าไม่มีเฟรมถูกต้องเกิน 10 วินาที ถือว่าหลุด

int shtConsecutiveNan = 0;
const int SHT_NAN_LIMIT = 3; // ถ้าอ่าน NAN ต่อเนื่อง >= 3 ครั้ง ถือว่าหลุด
unsigned long lastStatusPrint = 0;

// ====== ส่งทุก 20 วินาทีด้วย millis() ======
const unsigned long SEND_INTERVAL_MS = 20UL * 1000; // 20 วินาที
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
void readSHT31(){
  // ping I2C แบบเร็ว (ช่วยจับกรณีหลุดฮาร์ดแวร์)
  Wire.beginTransmission(0x44);
  uint8_t i2cErr = Wire.endTransmission();

  // อ่านค่าอุณหภูมิ (°C)
  float t = sht31.readTemperature();

  if (i2cErr == 0 && !isnan(t)) {
    shtTempC = t;
    updateSensorPresenceFromSHT(true);
  } else {
    updateSensorPresenceFromSHT(false);
  }
}

void updateSensorPresenceFromSHT(bool ok){
  if (ok) {
    if (!shtPresent) {
      Serial.println("[SHT31] RE-ATTACHED (กลับมาตอบสนอง)");
    }
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

void updateSensorPresenceFromPMS(bool ok){
  if (ok) {
    if (!pmsPresent) {
      Serial.println("[PMS] RE-ATTACHED (เริ่มได้รับเฟรมถูกต้อง)");
    }
    pmsPresent = true;
    lastPmsOkMillis = millis();
  }
}

void checkPMSHeartbeat(){
  if (pmsPresent) {
    if (millis() - lastPmsOkMillis > PMS_MISS_TIMEOUT) {
      pmsPresent = false;
      Serial.println("[PMS] DETACHED (ไม่มีเฟรมถูกต้องนานเกินกำหนด)");
    }
  }
}

bool readPMValues() {
  // กวาดทิ้งจนเจอ header 0x42
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
  Serial.print("Connecting to WiFi...");
  WiFi.begin(mySSID, myPASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    // กันค้างยาวเกินไป
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

  // Field 1: PM2.5
  ThingSpeak.setField(1, pm2_5);

  // Field 4: Temperature
  if (!isnan(shtTempC)) {
    ThingSpeak.setField(2, shtTempC);
  } else {
    // ถ้าอยากให้ส่ง 0 แทน ให้ปลดคอมเมนต์บรรทัดนี้
    // ThingSpeak.setField(4, 0);
  }

  // Status
  String statusMsg = String("PMS=") + (pmsPresent ? "OK" : "MISSING")
                   + " | SHT31=" + (shtPresent ? "OK" : "MISSING");
  ThingSpeak.setStatus(statusMsg);

  int httpCode = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if (httpCode == 200) {
    Serial.println("ThingSpeak: OK (200)");
  } else {
    Serial.printf("ThingSpeak: FAIL (HTTP %d)\n", httpCode);
  }
}

void setup() {
  Serial.begin(115200);

  // PMS5003: RX/TX ของ MCU ใช้ 21/22 ตามของคุณ
  pmsSerial.begin(9600, SERIAL_8N1, 21, 22);

  // ====== เริ่มต้น I2C และ SHT31 (SDA=19, SCL=18) ======
  Wire.begin(19, 18);
  if (!sht31.begin(0x44)) {
    shtPresent = false;
    Serial.println("SHT31 not found at 0x44. Check wiring (SDA=19, SCL=18).");
  } else {
    shtPresent = true;
    Serial.println("SHT31 initialized (I2C 0x44).");
  }
  // ======================================================

  connectWiFi();

  Serial.println("Configuring time from NTP server...");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  ThingSpeak.begin(client);

  // ====== พยายามตรวจจับ PMS ตั้งแต่เริ่มต้น (ภายใน ~2 วินาที) ======
  unsigned long start = millis();
  while (millis() - start < 2000) {
    if (readPMValues()) break;
    delay(50);
  }
  if (pmsPresent) {
    Serial.println("[PMS] INITIAL OK (พบเฟรมข้อมูล)");
  } else {
    Serial.println("[PMS] INITIAL MISSING (ยังไม่พบเฟรม ลองตรวจสาย/ไฟ)");
  }

  // ====== ตั้งเวลาเริ่มส่ง: ให้ส่งครั้งแรกหลังจาก 20 วิ หรือจะให้ส่งทันทีได้ ======
  // ถ้าอยากให้ "ส่งทันที" ตอนเริ่ม ให้ใช้: lastSendMillis = millis() - SEND_INTERVAL_MS;
  lastSendMillis = millis() - SEND_INTERVAL_MS;
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  // อ่าน SHT31 ทุกวินาที
  readSHT31();

  // อ่าน PMS ทุกวินาที
  bool gotPM = readPMValues();
  if (gotPM) {
    Serial.printf("PMS Read: PM1.0=%d, PM2.5=%d, PM10=%d | ", pm1, pm2_5, pm10);
  }

  checkPMSHeartbeat();

  // แสดงสถานะทุก ~5 วินาที
  if (millis() - lastStatusPrint > 5000) {
    lastStatusPrint = millis();
    Serial.printf("[STATUS] PMS=%s | SHT31=%s | T=%.2f°C\n",
                  pmsPresent ? "OK" : "MISSING",
                  shtPresent ? "OK" : "MISSING",
                  isnan(shtTempC) ? -999.0 : shtTempC);
  }

  // ====== ส่งข้อมูลทุก 20 วินาทีด้วย millis() ======
  unsigned long now = millis();
  if (now - lastSendMillis >= SEND_INTERVAL_MS) {
    // กัน drift: เลื่อนฐานเวลาไปทีละช่วง (ทำให้รอบนิ่งกว่า)
    lastSendMillis += SEND_INTERVAL_MS;

    // ถ้า loop หน่วงหนักมาก ๆ จนข้ามหลายช่วง ให้ sync ฐานใหม่ (กันส่งถี่ติดกัน)
    if (now - lastSendMillis >= SEND_INTERVAL_MS) {
      lastSendMillis = now;
    }

    sendToThingSpeak();
  }

  delay(1000);
}
