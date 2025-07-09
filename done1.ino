#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <HardwareSerial.h>

// ========== WIFI & FIREBASE ==========
#define WIFI_SSID "hamba allah"
#define WIFI_PASSWORD "sijisampesatus"
#define API_KEY "AIzaSyBuxgx9kY13NG7q0jDkUbbyDUTlkF3Ntu4"
#define DATABASE_URL "https://web-air-default-rtdb.asia-southeast1.firebasedatabase.app/"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

bool signupOK = false;
unsigned long sendDataPrevMillis = 0;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 3600, 60000); // UTC+7

// ========== SUHU - DS18B20 ==========
#define ONE_WIRE_BUS 27
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// ========== TDS Sensor ==========
#define TDS_PIN 34
#define VREF 3.3
#define SCOUNT 30
int analogBuffer[SCOUNT];
int analogBufferIndex = 0;
float averageVoltage = 0;
float tdsValue = 0;

// ========== pH Sensor ==========
#define PH_PIN 35
float voltage, phValue;

// ========== DO Sensor ==========
#define DO_PIN 32
#define DO_VREF 3300
#define DO_ADC_RES 4095
#define READ_TEMP 25
#define CAL1_V 1600
#define CAL1_T 25
#define CAL2_V 1300
#define CAL2_T 15

const uint16_t DO_Table[41] = {
  14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
  11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
  9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
  7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410
};

int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c) {
  uint16_t V_saturation = (uint32_t)CAL1_V + 35 * temperature_c - CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
}

// ========== Ultrasonik RS485 ==========
HardwareSerial mySerial(1); // RX: GPIO16, TX: GPIO17
const byte slaveAddress = 0x01;
const byte functionCode = 0x03;
const byte startAddressHigh = 0x01;
const byte startAddressLow = 0x00;
const byte registerCountHigh = 0x00;
const byte registerCountLow = 0x01;

// ========== MODBUS ==========
void constructModbusRequest(byte *frame) {
  frame[0] = slaveAddress;
  frame[1] = functionCode;
  frame[2] = startAddressHigh;
  frame[3] = startAddressLow;
  frame[4] = registerCountHigh;
  frame[5] = registerCountLow;
  uint16_t crc = calculateCRC(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = (crc >> 8) & 0xFF;
}

void sendModbusRequest(byte *frame, byte length) {
  for (byte i = 0; i < length; i++) {
    mySerial.write(frame[i]);
  }
}

void readModbusResponse(byte *frame, byte length) {
  for (byte i = 0; i < length; i++) {
    while (!mySerial.available());
    frame[i] = mySerial.read();
  }
}

uint16_t calculateCRC(byte *frame, byte length) {
  uint16_t crc = 0xFFFF;
  for (byte i = 0; i < length; i++) {
    crc ^= frame[i];
    for (byte j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, 16, 17);
  analogReadResolution(12);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) delay(300);

  timeClient.begin();
  sensors.begin();

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  if (Firebase.signUp(&config, &auth, "", "")) signupOK = true;
  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

// ========== LOOP ==========
void loop() {
  timeClient.update();
  String timestamp = timeClient.getFormattedTime();

  // ==== SUHU ====
  sensors.requestTemperatures();
  float suhu = sensors.getTempCByIndex(0);

  // ==== TDS ====
  int tdsADC = analogRead(TDS_PIN);
  float tdsVolt = tdsADC * VREF / 4095.0;
  tdsValue = (133.42 * tdsVolt * tdsVolt * tdsVolt - 255.86 * tdsVolt * tdsVolt + 857.39 * tdsVolt) * 0.5;

  // ==== pH ====
  int phADC = analogRead(PH_PIN);
  voltage = phADC * 3.3 / 4095.0;
  phValue = 7 + ((2.5 - voltage) / 0.18);

  // ==== DO ====
  int rawDO = analogRead(DO_PIN);
  int mvDO = (uint32_t)DO_VREF * rawDO / DO_ADC_RES * 1000;
  float DO = readDO(mvDO, READ_TEMP) / 1000.0;

  // ==== ULTRASONIK ====
  byte req[8];
  constructModbusRequest(req);
  sendModbusRequest(req, 8);
  delay(100);

  int distance = -1;
  if (mySerial.available()) {
    byte resp[7];
    readModbusResponse(resp, 7);
    distance = (resp[3] << 8) | resp[4];
    distance = distance / 10;
  }

  // ==== KIRIM KE FIREBASE ====
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 1000 || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();

    Firebase.RTDB.setFloat(&fbdo, "Sensor/Suhu/value", suhu);
    Firebase.RTDB.setString(&fbdo, "Sensor/Suhu/timestamp", timestamp);

    Firebase.RTDB.setFloat(&fbdo, "Sensor/TDS/value", tdsValue);
    Firebase.RTDB.setString(&fbdo, "Sensor/TDS/timestamp", timestamp);

    Firebase.RTDB.setFloat(&fbdo, "Sensor/pH/value", phValue);
    Firebase.RTDB.setString(&fbdo, "Sensor/pH/timestamp", timestamp);

    Firebase.RTDB.setFloat(&fbdo, "Sensor/DO/value", DO);
    Firebase.RTDB.setString(&fbdo, "Sensor/DO/timestamp", timestamp);

    if (distance != -1) {
      Firebase.RTDB.setInt(&fbdo, "Sensor/Distance/value", distance);
      Firebase.RTDB.setString(&fbdo, "Sensor/Distance/timestamp", timestamp);
    }
  }

  delay(100);
}
