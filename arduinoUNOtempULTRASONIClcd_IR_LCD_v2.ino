#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DFRobot_MCP9808.h"

// ---------- LCD ----------
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------- MCP9808 ----------
#define I2C_ADDRESS  MCP9808_ADDRESS_7
DFRobot_MCP9808_I2C mcp9808(&Wire, I2C_ADDRESS);

// ---------- Ultrasonic ----------
const int trigPin = 11;   // HC-SR04 TRIG
const int echoPin = 12;   // HC-SR04 ECHO

// ---------- Timing ----------
unsigned long lastMs = 0;
const unsigned long periodMs = 200;   // 5 Hz αποστολή προς ESP32

// ---------- IR ToF received from ESP32 ----------
float ir_tof_ns = 0.0f;
bool haveIR = false;
String rxLine = "";

// ---------- Ultrasonic filtering ----------
const int US_SAMPLES = 5;             // 5 γρήγορες μετρήσεις ανά κύκλο
const unsigned long US_TIMEOUT_US = 30000;
const unsigned long US_BETWEEN_MS = 18; // μικρό κενό ανάμεσα στα pings

long readUltrasonicOnce_us() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  return pulseIn(echoPin, HIGH, US_TIMEOUT_US);
}

void sortLongs(long *arr, int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (arr[j] < arr[i]) {
        long t = arr[i];
        arr[i] = arr[j];
        arr[j] = t;
      }
    }
  }
}

long readUltrasonicFiltered_us() {
  long vals[US_SAMPLES];
  int n = 0;

  for (int i = 0; i < US_SAMPLES; i++) {
    long u = readUltrasonicOnce_us();
    if (u > 0) {
      vals[n++] = u;
    }
    if (i < US_SAMPLES - 1) delay(US_BETWEEN_MS);
  }

  if (n == 0) return 0;

  sortLongs(vals, n);

  // trimmed mean για πιο ήπιο jitter χωρίς να χάνουμε πολλές μετρήσεις
  if (n >= 5) {
    long s = vals[1] + vals[2] + vals[3];
    return lround((double)s / 3.0);
  }
  if (n == 4) {
    long s = vals[1] + vals[2];
    return lround((double)s / 2.0);
  }
  if (n == 3) {
    long s = vals[0] + vals[1] + vals[2];
    return lround((double)s / 3.0);
  }
  // αν έμειναν μόνο 1-2 valid δείγματα, παίρνουμε median/mean
  if (n == 2) {
    return lround(((double)vals[0] + (double)vals[1]) / 2.0);
  }
  return vals[0];
}


void parseESP32Line(const String& s) {
  // Αναμένουμε από ESP32 γραμμή: IR,2.7
  if (s.startsWith("IR,")) {
    float v = s.substring(3).toFloat();
    if (v > 0.0f) {
      ir_tof_ns = v;
      haveIR = true;
    }
  }
}

void readESP32Serial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      parseESP32Line(rxLine);
      rxLine = "";
    } else if (c != '\r') {
      if (rxLine.length() < 40) rxLine += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  lcd.init();
  lcd.backlight();

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);

  while (!mcp9808.begin()) {
    delay(500);
  }
  mcp9808.wakeUpMode();
  mcp9808.setResolution(RESOLUTION_0_125);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("UNO OK -> ESP32");
  delay(1200);
  lcd.clear();
}

void loop() {
  // Διαβάζει την τιμή IR που στέλνει το ESP32 προς το UNO
  readESP32Serial();

  if (millis() - lastMs < periodMs) return;
  lastMs = millis();

  float T = mcp9808.getTemperature();
  long us_us = readUltrasonicFiltered_us();

  // LCD
  // 1η γραμμή: θερμοκρασία αριστερά + IR ToF δεξιά
  lcd.setCursor(0, 0);
  lcd.print("                ");  // καθάρισμα γραμμής

  lcd.setCursor(0, 0);
  lcd.print("T=");
  lcd.print(T, 1);
  lcd.print("C");

  lcd.setCursor(10, 0);             // δεξιά στην 1η γραμμή
  lcd.print("IR ");
  if (haveIR) {
    lcd.print(ir_tof_ns, 1);        // π.χ. IR 2.7
  } else {
    lcd.print("---");
  }

  lcd.setCursor(0, 1);
  lcd.print("US=");
  lcd.print(us_us);
  lcd.print("us     ");

  // Send CSV to ESP32: "T,US_us\n"
  Serial.print(T, 3);
  Serial.print(",");
  Serial.println(us_us);
}
