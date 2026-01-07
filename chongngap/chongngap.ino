#define DEFAULT_MQTT_HOST "mqtt1.eoh.io"
#define ERA_AUTH_TOKEN "84f16e45-e807-412d-9451-695e98f46a96"

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ERaSimpleEsp8266.hpp>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const char ssid[] = "iPhone";
const char pass[] = "22222222";

#define WATER_PIN A0
#define PUMP_PIN   12
#define SERVO_PIN  13
#define BUZZER_PIN 14

const uint16_t BUZZER_FREQ_HZ = 2000;
const uint16_t BUZZER_ON_MS   = 120;
const uint16_t BUZZER_OFF_MS  = 120;

#define SERVO_CLOSE_ANGLE  90
#define SERVO_OPEN_ANGLE   0
#define SERVO_MOVE_DELAY_MS 1000

Servo myServo;
bool servoOpen = false;

#define RELAY_ACTIVE_LOW false
static inline void writeRelay(uint8_t pin, bool on) {
  if (RELAY_ACTIVE_LOW) digitalWrite(pin, on ? LOW : HIGH);
  else                  digitalWrite(pin, on ? HIGH : LOW);
}

unsigned long authUntilMs = 0;
static inline bool isAuthorized() { return millis() < authUntilMs; }

bool modeAuto = true;
bool pumpOn = false;
bool floodState = false;

const int WATER_ON  = 200;
const int WATER_OFF = 100;

float waterFiltered = 0.0f;

LiquidCrystal_I2C lcd(0x27, 16, 2);
String lastLine1 = "";
String lastLine2 = "";

bool eraReady = false;

void pushLog(const String& msg) {
  ERa.virtualWrite(V20, msg.c_str());
}

void lcdPrint2(const String& l1, const String& l2) {
  if (l1 == lastLine1 && l2 == lastLine2) return;
  lastLine1 = l1;
  lastLine2 = l2;
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(l1);
  lcd.setCursor(0, 1); lcd.print(l2);
}

int readWaterRaw() {
  return analogRead(WATER_PIN);
}

void setPump(bool on, const char* reason) {
  if (pumpOn == on) return;
  writeRelay(PUMP_PIN, on);
  pumpOn = on;
  ERa.virtualWrite(V11, pumpOn ? 1 : 0);
  pushLog(String("PUMP ") + (on ? "ON" : "OFF") + " (" + reason + ")");
}

static inline void safeDelayMs(uint16_t ms) {
  while (ms--) {
    delay(1);
    yield();
  }
}

void setServo(bool open, const char* reason) {
  if (servoOpen == open) return;
  myServo.attach(SERVO_PIN);
  myServo.write(open ? SERVO_OPEN_ANGLE : SERVO_CLOSE_ANGLE);
  safeDelayMs(SERVO_MOVE_DELAY_MS);
  myServo.detach();
  servoOpen = open;
  ERa.virtualWrite(V12, servoOpen ? 1 : 0);
  pushLog(String("SERVO ") + (open ? "OPEN" : "CLOSE") + " (" + reason + ")");
}

void buzzerAlarm(bool on) {
  static bool last = false;
  if (on == last) return;
  last = on;
  if (on) tone(BUZZER_PIN, BUZZER_FREQ_HZ);
  else    noTone(BUZZER_PIN);
}

bool floodByWaterSensor() {
  if (!floodState) return waterFiltered >= WATER_ON;
  else             return waterFiltered >  WATER_OFF;
}

void readSensorsAndPush() {
  int raw = readWaterRaw();
  if (waterFiltered <= 0.01f) waterFiltered = raw;
  else waterFiltered = 0.25f * raw + 0.75f * waterFiltered;
  ERa.virtualWrite(V2, (int)waterFiltered);
}

void autoControl() {
  bool floodNow = floodByWaterSensor();
  if (floodNow != floodState) {
    floodState = floodNow;
    pushLog(floodState ? "FLOOD: ENTER (AUTO)" : "FLOOD: EXIT (AUTO)");
  }
  setPump(floodState, "AUTO");
  setServo(floodState, "AUTO");
}

void updateUIAndAlarmFast() {
  if (!floodState) {
    lcdPrint2("TRANG THAI:", "AN TOAN");
    buzzerAlarm(false);
    return;
  }
  lcdPrint2("TRANG THAI:", "NGUY HIEM!");
  static unsigned long lastToggleMs = 0;
  static bool beepOn = false;
  unsigned long now = millis();
  unsigned long interval = beepOn ? BUZZER_ON_MS : BUZZER_OFF_MS;
  if (now - lastToggleMs >= interval) {
    lastToggleMs = now;
    beepOn = !beepOn;
    buzzerAlarm(beepOn);
  }
}

void timerTick100ms() {
  if (!eraReady) return;
  static unsigned long lastSensorMs = 0;
  static unsigned long lastUiMs = 0;
  unsigned long now = millis();
  if (now - lastUiMs >= 100) {
    lastUiMs = now;
    updateUIAndAlarmFast();
  }
  if (now - lastSensorMs >= 1000) {
    lastSensorMs = now;
    readSensorsAndPush();
    if (modeAuto) autoControl();
  }
}

ERA_CONNECTED() {
  eraReady = true;
  pushLog("ERa connected!");
  ERa.virtualWrite(V10, modeAuto ? 0 : 1);
  ERa.virtualWrite(V11, pumpOn ? 1 : 0);
  ERa.virtualWrite(V12, servoOpen ? 1 : 0);
  ERa.virtualWrite(V2, (int)waterFiltered);
  updateUIAndAlarmFast();
}

ERA_WRITE(V10) {
  modeAuto = (param.getInt() == 0);
  pushLog(modeAuto ? "MODE: AUTO" : "MODE: MANUAL");
  if (modeAuto) autoControl();
}

ERA_WRITE(V16) {
  const char* inp = param.getString();
  if (inp && strcmp(inp, "1234") == 0) {
    authUntilMs = millis() + 30000UL;
    pushLog("AUTH OK: 30s");
  } else {
    authUntilMs = 0;
    pushLog("AUTH FAIL");
  }
}

ERA_WRITE(V11) {
  if (modeAuto || !isAuthorized()) {
    ERa.virtualWrite(V11, pumpOn ? 1 : 0);
    return;
  }
  setPump(param.getInt() == 1, "MANUAL");
}

ERA_WRITE(V12) {
  if (modeAuto || !isAuthorized()) {
    ERa.virtualWrite(V12, servoOpen ? 1 : 0);
    return;
  }
  setServo(param.getInt() == 1, "MANUAL");
}

void setup() {
  pinMode(PUMP_PIN, OUTPUT);
  writeRelay(PUMP_PIN, false);
  pinMode(BUZZER_PIN, OUTPUT);
  buzzerAlarm(false);
  Wire.begin(4, 5);
  lcd.init();
  lcd.backlight();
  lcdPrint2("BOOT...", "WAIT ERA...");
  ERa.setPersistent(false);
  ERa.setScanWiFi(true);
  ERa.begin(ssid, pass);
  ERa.addInterval(100L, timerTick100ms);
}

void loop() {
  ERa.run();
}
