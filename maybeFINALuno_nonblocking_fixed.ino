/*
  maybeFINALuno_nonblocking_fixed.ino

  Arduino UNO + HC-SR04 + AHT20 + VL53L0X + LCD I2C 16x2

  Τι κάνει:
  - Μετρά χρόνο υπερήχου HC-SR04 σε microseconds (us), χωρίς pulseIn().
  - Μετρά θερμοκρασία/υγρασία από AHT20 χωρίς delay()/while.
  - Μετρά απόσταση με VL53L0X και υπολογίζει χρόνο πτήσης φωτός πήγαινε-έλα σε ns.
  - Δείχνει στην LCD: χρόνο υπερήχου, θερμοκρασία και IR ToF.
  - Στέλνει στο Serial μόνο αριθμητικά δεδομένα με ; για Excel/LibreOffice.

  Συνδέσεις:
  HC-SR04 TRIG -> D13
  HC-SR04 ECHO -> D12
  LCD I2C      -> SDA/SCL
  AHT20 I2C    -> SDA/SCL, address 0x38
  VL53L0X I2C  -> SDA/SCL, όπως στο αρχικό maybeFINALuno: begin(0x50)

  Σημαντικό για μονάδες:
  - echo_us είναι σε microseconds (us), όχι ns.
  - ir_tof_ns είναι σε nanoseconds (ns).
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DFRobot_VL53L0X.h"

// -------------------- LCD / I2C sensors --------------------
LiquidCrystal_I2C lcd(0x27, 16, 2);
DFRobot_VL53L0X vl53;

// -------------------- HC-SR04 pins --------------------
const byte TRIG_PIN = 13;
const byte ECHO_PIN = 12;

// -------------------- HC-SR04 timing --------------------
const unsigned long US_PERIOD_MS = 120;      // κάθε πότε ξεκινά νέα μέτρηση HC-SR04
const unsigned long MIN_ECHO_US  = 200;      // κάτω από αυτό αγνοείται
const unsigned long MAX_ECHO_US  = 25000;    // περίπου μέχρι 4.3 m στον αέρα
const unsigned long AVG_WINDOW_MS = 8000;    // μέσος όρος 8 s, όπως η λογική του αρχικού

// -------------------- AHT20 raw I2C --------------------
const byte AHT20_ADDR = 0x38;
const unsigned long AHT_STARTUP_MS = 80;
const unsigned long AHT_MEASURE_MS = 90;
const unsigned long AHT_PERIOD_MS  = 2000;

// -------------------- VL53L0X --------------------
const unsigned long VL53_PERIOD_MS = 120;
const int VL53_MAX_VALID_MM = 2000;

// -------------------- LCD / Serial periods --------------------
const unsigned long INTRO_TIME_MS  = 1900;
const unsigned long LCD_PERIOD_MS  = 400;
const unsigned long SERIAL_PERIOD_MS = 500;

// -------------------- Constants --------------------
const double C_LIGHT = 299792458.0;           // m/s

// ============================================================
// Μεταβλητές HC-SR04
// ============================================================
enum HcsrState {
  HCSR_READY,
  HCSR_WAIT_HIGH,
  HCSR_WAIT_LOW
};

HcsrState hcsrState = HCSR_READY;
unsigned long lastHcsrMs = 0;
unsigned long waitStartUs = 0;
unsigned long echoStartUs = 0;
unsigned long echoUs = 0;
bool hcsrOk = false;

float echoSumUs = 0.0;
unsigned int echoCount = 0;
float echoAvgUs = 0.0;
bool echoAvgOk = false;
unsigned long avgStartMs = 0;

// ============================================================
// Μεταβλητές AHT20
// ============================================================
bool ahtInitSent = false;
bool ahtStarted = false;
bool ahtMeasuring = false;
bool ahtOk = false;
unsigned long ahtTimerMs = 0;
float temperatureC = 20.0;
float humidityRH = 50.0;

// ============================================================
// Μεταβλητές VL53L0X / IR ToF
// ============================================================
unsigned long lastVl53Ms = 0;
int vl53DistanceMm = -1;
float vl53DistanceCm = -1.0;
float roundTripCm = -1.0;
double irTofNs = 0.0;
bool vl53Ok = false;

// ============================================================
// Γενικές μεταβλητές
// ============================================================
unsigned long startMs = 0;
unsigned long lastLcdMs = 0;
unsigned long lastSerialMs = 0;
bool showIntro = true;

// ============================================================
// SETUP
// ============================================================
void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  Serial.begin(9600);
  Wire.begin();

  lcd.init();
  lcd.backlight();
  lcd.setCursor(1, 0);
  lcd.print(F(" Arduino Team "));
  lcd.setCursor(0, 1);
  lcd.print(F("Gel NeasKydonias"));

  // VL53L0X όπως στο αρχικό maybeFINALuno.ino
  vl53.begin(0x50);
  vl53.setMode(vl53.eContinuous, vl53.eHigh);
  vl53.start();

  startMs = millis();
  avgStartMs = millis();
  ahtTimerMs = millis();

  // Header για spreadsheet. Με ; για ελληνικό Excel/LibreOffice.
  Serial.println(F("elapsed_ms;vl53_mm;vl53_cm;roundtrip_cm;ir_tof_ns;echo_us;echo_avg8_us;temp_C;rh_pct;sound_speed_m_s;us_distance_cm"));
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  updateIntro();
  updateHcsr04();
  updateAht20();
  updateVl53();
  updateAverageWindow();
  updateSerial();
  updateLcd();
}

// ============================================================
// Αρχική οθόνη χωρίς delay()
// ============================================================
void updateIntro() {
  if (showIntro && millis() - startMs >= INTRO_TIME_MS) {
    showIntro = false;
    lcd.clear();
  }
}

// ============================================================
// HC-SR04 χωρίς pulseIn()
// Μόνο ο trigger παλμός έχει delayMicroseconds(10), δηλαδή 10 μs.
// Δεν υπάρχει blocking αναμονή για echo.
// ============================================================
void updateHcsr04() {
  unsigned long nowMs = millis();
  unsigned long nowUs = micros();

  switch (hcsrState) {
    case HCSR_READY:
      if (nowMs - lastHcsrMs >= US_PERIOD_MS) {
        lastHcsrMs = nowMs;

        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);

        waitStartUs = micros();
        hcsrState = HCSR_WAIT_HIGH;
      }
      break;

    case HCSR_WAIT_HIGH:
      if (digitalRead(ECHO_PIN) == HIGH) {
        echoStartUs = micros();
        hcsrState = HCSR_WAIT_LOW;
      } else if (nowUs - waitStartUs > MAX_ECHO_US) {
        hcsrOk = false;
        hcsrState = HCSR_READY;
      }
      break;

    case HCSR_WAIT_LOW:
      if (digitalRead(ECHO_PIN) == LOW) {
        echoUs = micros() - echoStartUs;
        hcsrOk = (echoUs >= MIN_ECHO_US && echoUs <= MAX_ECHO_US);

        if (hcsrOk) {
          echoSumUs += echoUs;
          echoCount++;
        }

        hcsrState = HCSR_READY;
      } else if (nowUs - echoStartUs > MAX_ECHO_US) {
        hcsrOk = false;
        hcsrState = HCSR_READY;
      }
      break;
  }
}

// ============================================================
// Μέσος όρος echo ανά 8 s
// ============================================================
void updateAverageWindow() {
  unsigned long nowMs = millis();

  if (nowMs - avgStartMs >= AVG_WINDOW_MS) {
    if (echoCount > 0) {
      echoAvgUs = echoSumUs / echoCount;
      echoAvgOk = true;
    } else {
      echoAvgUs = 0.0;
      echoAvgOk = false;
    }

    echoSumUs = 0.0;
    echoCount = 0;
    avgStartMs = nowMs;
  }
}

// ============================================================
// AHT20 χωρίς delay() και χωρίς while retry loop
// ============================================================
void updateAht20() {
  unsigned long nowMs = millis();

  // Περιμένουμε λίγο μετά την εκκίνηση.
  if (!ahtInitSent) {
    if (nowMs - ahtTimerMs < AHT_STARTUP_MS) return;

    Wire.beginTransmission(AHT20_ADDR);
    Wire.write(0xBE);   // init/calibrate
    Wire.write(0x08);
    Wire.write(0x00);
    ahtStarted = (Wire.endTransmission() == 0);

    ahtInitSent = true;
    ahtTimerMs = nowMs;
    return;
  }

  // Αν δεν απάντησε στην αρχικοποίηση, ξαναδοκιμάζουμε κάθε 2 s.
  if (!ahtStarted) {
    if (nowMs - ahtTimerMs >= AHT_PERIOD_MS) {
      ahtInitSent = false;
      ahtTimerMs = nowMs;
    }
    return;
  }

  // Αν έχει ξεκινήσει μέτρηση, περιμένουμε να ολοκληρωθεί χωρίς delay().
  if (ahtMeasuring) {
    if (nowMs - ahtTimerMs >= AHT_MEASURE_MS) {
      readAht20Data();
      ahtMeasuring = false;
      ahtTimerMs = nowMs;
    }
    return;
  }

  // Ξεκινάμε νέα μέτρηση κάθε AHT_PERIOD_MS.
  if (nowMs - ahtTimerMs >= AHT_PERIOD_MS) {
    Wire.beginTransmission(AHT20_ADDR);
    Wire.write(0xAC);   // trigger measurement
    Wire.write(0x33);
    Wire.write(0x00);

    if (Wire.endTransmission() == 0) {
      ahtMeasuring = true;
    } else {
      ahtOk = false;
    }

    ahtTimerMs = nowMs;
  }
}

void readAht20Data() {
  byte data[6];

  Wire.requestFrom(AHT20_ADDR, (byte)6);
  if (Wire.available() < 6) {
    ahtOk = false;
    return;
  }

  for (byte i = 0; i < 6; i++) {
    data[i] = Wire.read();
  }

  // Αν bit 7 = 1, ο αισθητήρας είναι ακόμη busy.
  if (data[0] & 0x80) {
    ahtOk = false;
    return;
  }

  unsigned long rawHum = ((unsigned long)data[1] << 12) |
                         ((unsigned long)data[2] << 4)  |
                         ((unsigned long)data[3] >> 4);

  unsigned long rawTemp = (((unsigned long)data[3] & 0x0F) << 16) |
                          ((unsigned long)data[4] << 8) |
                          (unsigned long)data[5];

  humidityRH = rawHum * 100.0 / 1048576.0;
  temperatureC = rawTemp * 200.0 / 1048576.0 - 50.0;

  if (humidityRH < 0.0) humidityRH = 0.0;
  if (humidityRH > 100.0) humidityRH = 100.0;

  ahtOk = true;
}

// ============================================================
// VL53L0X και υπολογισμός IR ToF
// ============================================================
void updateVl53() {
  unsigned long nowMs = millis();
  if (nowMs - lastVl53Ms < VL53_PERIOD_MS) return;
  lastVl53Ms = nowMs;

  int dmm = vl53.getDistance();

  if (dmm > 0 && dmm <= VL53_MAX_VALID_MM) {
    vl53DistanceMm = dmm;
    vl53DistanceCm = vl53DistanceMm / 10.0;

    // Το VL53L0X δίνει απόσταση μέχρι το αντικείμενο.
    // Για χρόνο φωτός πήγαινε-έλα, χρησιμοποιούμε 2 * απόσταση.
    roundTripCm = 2.0 * vl53DistanceCm;

    double roundTripM = roundTripCm / 100.0;
    irTofNs = (roundTripM / C_LIGHT) * 1e9;

    vl53Ok = true;
  } else {
    vl53Ok = false;
  }
}

// ============================================================
// Ταχύτητα ήχου με διόρθωση θερμοκρασίας/υγρασίας
// ============================================================
float soundSpeedMs() {
  return 331.3 + 0.606 * temperatureC + 0.0124 * humidityRH;
}

float ultrasonicDistanceCm() {
  if (!hcsrOk) return -1.0;

  float tofS = echoUs / 1000000.0;
  float distM = soundSpeedMs() * tofS / 2.0;
  return distM * 100.0;
}

// ============================================================
// Serial μόνο δεδομένα, χωρίς κείμενα που χαλάνε spreadsheet
// ============================================================
void updateSerial() {
  unsigned long nowMs = millis();
  if (nowMs - lastSerialMs < SERIAL_PERIOD_MS) return;
  lastSerialMs = nowMs;

  Serial.print(nowMs);
  Serial.print(';');

  if (vl53Ok) Serial.print(vl53DistanceMm);
  else Serial.print(F("NaN"));
  Serial.print(';');

  if (vl53Ok) Serial.print(vl53DistanceCm, 2);
  else Serial.print(F("NaN"));
  Serial.print(';');

  if (vl53Ok) Serial.print(roundTripCm, 2);
  else Serial.print(F("NaN"));
  Serial.print(';');

  if (vl53Ok) Serial.print(irTofNs, 6);
  else Serial.print(F("NaN"));
  Serial.print(';');

  if (hcsrOk) Serial.print(echoUs);
  else Serial.print(F("NaN"));
  Serial.print(';');

  if (echoAvgOk) Serial.print(echoAvgUs, 1);
  else Serial.print(F("NaN"));
  Serial.print(';');

  if (ahtOk) Serial.print(temperatureC, 2);
  else Serial.print(F("NaN"));
  Serial.print(';');

  if (ahtOk) Serial.print(humidityRH, 1);
  else Serial.print(F("NaN"));
  Serial.print(';');

  Serial.print(soundSpeedMs(), 2);
  Serial.print(';');

  float usCm = ultrasonicDistanceCm();
  if (usCm >= 0.0) Serial.println(usCm, 2);
  else Serial.println(F("NaN"));
}

// ============================================================
// LCD χωρίς συνεχές lcd.clear()
// ============================================================
void updateLcd() {
  if (showIntro) return;

  unsigned long nowMs = millis();
  if (nowMs - lastLcdMs < LCD_PERIOD_MS) return;
  lastLcdMs = nowMs;

  // Γραμμή 1: χρόνος υπερήχου σε us, όχι ns.
  lcd.setCursor(0, 0);
  lcd.print(F("US:"));
  if (hcsrOk) {
    printFixedWidthULong(echoUs, 5);
    lcd.print(F("us"));
  } else {
    lcd.print(F("-----us"));
  }

  lcd.print(F(" c"));
  lcd.print(soundSpeedMs(), 0);

  // καθάρισμα υπολοίπου γραμμής αν μείνουν παλιοί χαρακτήρες
  lcd.print(F("  "));

  // Γραμμή 2: θερμοκρασία + IR ToF σε ns.
  lcd.setCursor(0, 1);
  lcd.print(F("T:"));
  if (ahtOk) {
    lcd.print(temperatureC, 1);
    lcd.print(F("C"));
  } else {
    lcd.print(F("--.-C"));
  }

  lcd.print(F(" IR"));
  if (vl53Ok) {
    lcd.print(irTofNs, 2);     // π.χ. 2.67 για 80 cm πήγαινε-έλα
  } else {
    lcd.print(F("--.--"));
  }
  lcd.print(F("n"));           // n = ns, για να χωράει στην 16x2 LCD

  lcd.print(F(" "));
}

// Τυπώνει ακέραιο σε σταθερό πλάτος, γεμίζοντας με κενά μπροστά.
void printFixedWidthULong(unsigned long value, byte width) {
  char buf[11];
  ultoa(value, buf, 10);

  byte len = strlen(buf);
  for (byte i = len; i < width; i++) {
    lcd.print(' ');
  }
  lcd.print(buf);
}
