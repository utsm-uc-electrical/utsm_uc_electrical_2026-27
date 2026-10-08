#include <WiFi.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_LSM6DSOX.h>
#include <TinyGPS++.h>
#include <stdarg.h>
#include <math.h>

#if defined(__has_include)
#if __has_include("webapp.h")
#include "webapp.h"
#define HAVE_WEBAPP 1
#endif
// PLAN CHANGE: the burn / coast strategy. plan.h is made by the planner app and sits beside this file
// and webapp.h. It holds PLAN_ID, PLAN_NAME and PLAN_JSON. The board only STORES and SERVES the plan
// (the phone reads /plan.json and does all the matching), so a new strategy is a new plan.h and a
// re-upload; webapp.h does not change. With no plan.h the board behaves exactly as before.
#if __has_include("plan.h")
#include "plan.h"
#define HAVE_PLAN 1
#endif
#endif

#ifdef HAVE_PLAN
#define PLAN_ID_STR PLAN_ID
#else
#define PLAN_ID_STR "none"
#endif

#define AP_SSID      "UTSM-Telemetry"
#define AP_PASS      "utsm2027"
#define SAMPLE_HZ    10
#define MAX_SSE      4
#define SD_LOGGING   1
#define HAVE_JOULEMETER 1
#define IMU_HZ       100   // IMU CHANGE: log rate, 50 or 100 samples per second (see the IMU block further down)

#define CAPTIVE_PORTAL 0

static_assert(IMU_HZ == 50 || IMU_HZ == 100, "IMU_HZ must be 50 or 100");

static const int PIN_GPS_RX = 16, PIN_GPS_TX = 17;
static const int PIN_PPS     = 4;

static const int PIN_LORA_CS = 5;
static const int PIN_SD_CS   = 2;

static const int PIN_I2C_SDA = 21;
static const int PIN_I2C_SCL = 22;

#define I2C_HZ          100000
#define I2C_TIMEOUT_MS  20

#define INA226_ADDR     0x40
#define JM_HAVE_CURRENT 0
#define JM_R_SHUNT      0.002f
#define JM_I_MAX        40.0f
#define JM_V_MAX        36.0f

#define INA_REG_CONFIG  0x00
#define INA_REG_SHUNT   0x01
#define INA_REG_BUS     0x02
#define INA_REG_CAL     0x05
#define INA_REG_MANUF   0xFE
#define INA_REG_DIE     0xFF

#if JM_HAVE_CURRENT
#define JM_CONFIG     0x4527
#else
#define JM_CONFIG     0x4526
#endif

// ============================================================================================
// IMU SETTINGS.  Search this file for "IMU CHANGE" to find every place that may need editing.
// Nothing here is tied to a particular mounting yet, so check these once the board's position
// in the car is known.
//
// 1) AXES (depends on mounting).  The log and the dashboard use the chip's OWN axes, unchanged
//    (the CSV header says cal_applied=0).  Which chip axis points forward / left / up depends on
//    how the board sits.  Easiest option: leave the sketch alone and swap or negate columns in the
//    analysis script.  To log in vehicle axes instead, edit imuUnits() and imuSelfCheck().
//    To find the mapping: with the board level, the axis reading about +9.8 m/s^2 points up.
//
// 2) RANGE.  Now +/-4 g accel and +/-250 dps gyro.  If accel values sit flat at 39.2 m/s^2 (4 g)
//    the range is clipping, so go up.  To change a range, change all THREE of these together:
//      a) setAccelRange() / setGyroRange() in setup()
//      b) the matching scale constant just below
//      c) the "accel_range=4g . gyro_range=250dps" text in openLog(), which appears TWICE
//    If the ACCEL (a) and (b) do not match, imuSelfCheck() notices and switches the IMU off.
//    Nothing checks the GYRO scale, so for a gyro range change double-check (a), (b) and (c) by hand.
//
//      accel range              ACC_MPS2_PER_LSB            gyro range               GYR_DPS_PER_LSB
//      LSM6DS_ACCEL_RANGE_2_G   (0.000061f * 9.80665f)      LSM6DS_GYRO_RANGE_125_DPS   0.004375f
//      LSM6DS_ACCEL_RANGE_4_G   (0.000122f * 9.80665f)      LSM6DS_GYRO_RANGE_250_DPS   0.00875f
//      LSM6DS_ACCEL_RANGE_8_G   (0.000244f * 9.80665f)      LSM6DS_GYRO_RANGE_500_DPS   0.0175f
//      LSM6DS_ACCEL_RANGE_16_G  (0.000488f * 9.80665f)      LSM6DS_GYRO_RANGE_1000_DPS  0.035f
//                                                           LSM6DS_GYRO_RANGE_2000_DPS  0.070f
//
// 3) I2C ADDRESS.  0x6A when the chip's SDO pin is low, 0x6B when it is high.  If it is 0x6B,
//    change IMU_ADDR below AND change imu.begin_I2C() in setup() to imu.begin_I2C(IMU_ADDR).
//
// 4) RATE.  Set IMU_HZ at the top (50 or 100).  The chip's own data rate follows it automatically.
//
// 5) OFFSETS.  accOff[] further down.
//
// Leave IMU_QUEUE_LEN, IMU_FRESH_MS and IMU_REG_OUT alone: they do not depend on the mounting.
// ============================================================================================
#define IMU_ADDR           0x6A   // IMU CHANGE: 0x6B if SDO is high (see item 3 above)
#define IMU_QUEUE_LEN      256
#define IMU_FRESH_MS       250
#define IMU_REG_OUT        0x22
#define ACC_MPS2_PER_LSB   (0.000122f * 9.80665f)   // IMU CHANGE: must match the accel range in setup() (table above)
#define GYR_DPS_PER_LSB    0.00875f   // IMU CHANGE: must match the gyro range in setup() (table above)
#define IMU_ODR            (IMU_HZ > 52 ? LSM6DS_RATE_104_HZ : LSM6DS_RATE_52_HZ)

#define UTC_MAX_AGE_US     5000000LL
#define PPS_MIN_PERIOD_US  900000LL
#define UTC_SECOND_OFFSET  0

struct ImuRaw { int64_t us; int16_t v[6]; };

static inline bool isNan(float v) { return v != v; }

static void appendf(char *buf, size_t sz, size_t *n, const char *fmt, ...)
  __attribute__((format(printf, 4, 5)));
static void appendf(char *buf, size_t sz, size_t *n, const char *fmt, ...) {
  if (sz == 0 || *n >= sz - 1) return;
  va_list ap;
  va_start(ap, fmt);
  int w = vsnprintf(buf + *n, sz - *n, fmt, ap);
  va_end(ap);
  if (w < 0) return;
  *n += (size_t)w;
  if (*n > sz - 1) *n = sz - 1;
}

int  sseCount();
void cfgSave();
String helloJson();
bool jmBegin();
void i2cScan();
void sendText(WiFiClient &c, const char *status, const char *type, const String &body);

Adafruit_LSM6DSOX imu;
TinyGPSPlus       gps;
HardwareSerial    GPSSerial(2);
WiFiServer        server(80);
DNSServer         dns;
Preferences       prefs;
File              logFile;
File              imuFile;

WiFiClient sse[MAX_SSE];
bool       imuOk = false, sdOk = false, imuLogOk = false;
uint16_t   seq = 0;
uint32_t   lastSample = 0;
bool       runArmed = false;
uint32_t   runArmT = 0;
uint32_t   bootId = 0;

// IMU CHANGE: accel zero-offsets (m/s^2) in the chip's OWN axes (X, Y, Z of the chip, not the car).
// These are NOT subtracted from the data.  They are only printed in the CSV header so the analysis
// can apply them.  Get the numbers from the serial 'c' flip test, then paste them here.
// The offset belongs to the chip, so it stays valid when the board is remounted.
// If you remap the axes in imuUnits(), these stay in chip order; say so in the header (see openLog).
float accOff[3] = { 0, 0, 0 };

struct Cfg { float bandLo, bandHi, budgetWh, eventKm, leaveM, returnM, minLapS; };
Cfg cfg = { NAN, NAN, NAN, 16.09f, 40, 25, 20 };

void cfgLoad() {
  prefs.begin("utsm", true);
  cfg.bandLo   = prefs.getFloat("bandLo",   NAN);
  cfg.bandHi   = prefs.getFloat("bandHi",   NAN);
  cfg.budgetWh = prefs.getFloat("budgetWh", NAN);
  cfg.eventKm  = prefs.getFloat("eventKm",  16.09f);
  cfg.leaveM   = prefs.getFloat("leaveM",   40);
  cfg.returnM  = prefs.getFloat("returnM",  25);
  cfg.minLapS  = prefs.getFloat("minLapS",  20);
  prefs.end();
}
void cfgSave() {
  prefs.begin("utsm", false);
  prefs.putFloat("bandLo", cfg.bandLo);     prefs.putFloat("bandHi", cfg.bandHi);
  prefs.putFloat("budgetWh", cfg.budgetWh); prefs.putFloat("eventKm", cfg.eventKm);
  prefs.putFloat("leaveM", cfg.leaveM);     prefs.putFloat("returnM", cfg.returnM);
  prefs.putFloat("minLapS", cfg.minLapS);
  prefs.end();
}

uint32_t bootIdAdvance() {
  if (!prefs.begin("utsm", false)) return 0;
  uint32_t id = prefs.getUInt("bootId", 0) + 1;
  size_t w = prefs.putUInt("bootId", id);
  prefs.end();
  return w ? id : 0;
}

static void boardIdStr(char *out, size_t sz) {
  uint64_t mac = ESP.getEfuseMac();
  snprintf(out, sz, "%02X%02X%02X%02X%02X%02X",
           (unsigned)(mac & 0xFF), (unsigned)((mac >> 8) & 0xFF),
           (unsigned)((mac >> 16) & 0xFF), (unsigned)((mac >> 24) & 0xFF),
           (unsigned)((mac >> 32) & 0xFF), (unsigned)((mac >> 40) & 0xFF));
}

static SemaphoreHandle_t i2cMux = NULL;

static bool i2cLock() {
  return i2cMux != NULL && xSemaphoreTake(i2cMux, pdMS_TO_TICKS(60)) == pdTRUE;
}
static void i2cUnlock() { xSemaphoreGive(i2cMux); }

bool i2cRead16(uint8_t addr, uint8_t reg, uint16_t *out) {
  if (!i2cLock()) return false;
  bool ok = false;
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission() == 0 &&
      Wire.requestFrom((uint8_t)addr, (uint8_t)2) == 2 &&
      Wire.available() >= 2) {
    uint16_t hi = (uint8_t)Wire.read();
    uint16_t lo = (uint8_t)Wire.read();
    *out = (uint16_t)((hi << 8) | lo);
    ok = true;
  }
  i2cUnlock();
  return ok;
}

bool i2cWrite16(uint8_t addr, uint8_t reg, uint16_t val) {
  if (!i2cLock()) return false;
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write((uint8_t)(val >> 8));
  Wire.write((uint8_t)(val & 0xFF));
  bool ok = Wire.endTransmission() == 0;
  i2cUnlock();
  return ok;
}

bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n) {
  if (!i2cLock()) return false;
  bool ok = false;
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission() == 0 &&
      Wire.requestFrom((uint8_t)addr, (uint8_t)n) == n &&
      (size_t)Wire.available() >= n) {
    for (size_t k = 0; k < n; k++) buf[k] = (uint8_t)Wire.read();
    ok = true;
  }
  i2cUnlock();
  return ok;
}

void i2cBusClear() {
  if (!i2cLock()) return;
  Wire.end();
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  pinMode(PIN_I2C_SCL, OUTPUT);
  digitalWrite(PIN_I2C_SCL, HIGH);
  for (int k = 0; k < 9; k++) {
    digitalWrite(PIN_I2C_SCL, LOW);  delayMicroseconds(5);
    digitalWrite(PIN_I2C_SCL, HIGH); delayMicroseconds(5);
    if (digitalRead(PIN_I2C_SDA) == HIGH) break;
  }

  pinMode(PIN_I2C_SDA, OUTPUT);
  digitalWrite(PIN_I2C_SDA, LOW);   delayMicroseconds(5);
  digitalWrite(PIN_I2C_SCL, HIGH);  delayMicroseconds(5);
  digitalWrite(PIN_I2C_SDA, HIGH);  delayMicroseconds(5);
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  pinMode(PIN_I2C_SCL, INPUT_PULLUP);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(I2C_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  i2cUnlock();
}

void i2cScan() {
  Serial.println(F("I2C scan 0x08-0x77:"));
  int found = 0;
  for (uint8_t a = 0x08; a <= 0x77; a++) {
    if (!i2cLock()) { Serial.println(F("  bus busy - try again")); return; }
    Wire.beginTransmission(a);
    bool hit = Wire.endTransmission() == 0;
    i2cUnlock();
    if (hit) {
      Serial.printf("  0x%02X", a);
      if (a == 0x6A || a == 0x6B)      Serial.print(F("   LSM6DSOX (IMU)"));
      else if (a >= 0x40 && a <= 0x4F) Serial.print(F("   INA226 address range"));
      Serial.println();
      found++;
    }
    delay(2);
  }
  if (!found) Serial.println(F("  nothing answered - check GND, pull-ups, power"));
}

static bool     jmOk      = false;
static uint8_t  jmFail    = 0;
static uint32_t jmRetryAt = 0;
static uint32_t jmMisses  = 0;

struct Jm { bool haveV, haveI; float v, i; } jm;

bool jmBegin() {
  uint16_t id;
  if (!i2cRead16(INA226_ADDR, INA_REG_MANUF, &id)) return false;
  if (id != 0x5449) return false;
  if (!i2cRead16(INA226_ADDR, INA_REG_DIE, &id)) return false;
  if ((id >> 4) != 0x226) return false;

  if (!i2cWrite16(INA226_ADDR, INA_REG_CONFIG, 0x8000)) return false;
  delay(2);
  if (!i2cWrite16(INA226_ADDR, INA_REG_CONFIG, JM_CONFIG)) return false;

#if JM_HAVE_CURRENT

  float currentLsb = JM_I_MAX / 32768.0f;
  float cal        = 0.00512f / (currentLsb * JM_R_SHUNT);
  if (cal >= 1.0f && cal <= 65535.0f) i2cWrite16(INA226_ADDR, INA_REG_CAL, (uint16_t)cal);
#endif

  return true;
}

void jmRead() {
  jm.haveV = jm.haveI = false;

#if HAVE_JOULEMETER
  if (!jmOk) {

    if ((int32_t)(millis() - jmRetryAt) < 0) return;
    jmRetryAt = millis() + 1000;
    jmOk = jmBegin();
    if (!jmOk) return;
    Serial.println(F("JM: INA226 back"));
    jmFail = 0;
  }

  uint16_t rawBus;
  bool okBus = i2cRead16(INA226_ADDR, INA_REG_BUS, &rawBus);

#if JM_HAVE_CURRENT
  uint16_t rawSh;
  bool okSh = i2cRead16(INA226_ADDR, INA_REG_SHUNT, &rawSh);
#endif

  bool alive = okBus && rawBus <= 0x7FFF;

  if (!alive) {
    jmMisses++;
    if (++jmFail >= 2) {
      Serial.printf("JM: no answer, clearing bus (misses=%lu)\n",
                    (unsigned long)jmMisses);
      i2cBusClear();
      jmOk = false;
      jmRetryAt = millis() + 1000;
    }
    return;
  }
  jmFail = 0;

  {
    float v = rawBus * 0.00125f;
    if (v <= JM_V_MAX * 1.05f) { jm.v = v; jm.haveV = true; }
  }

#if JM_HAVE_CURRENT

  if (okSh) {
    int16_t s = (int16_t)rawSh;
    if (s > -32760 && s < 32760) {
      float amps = (s * 0.0000025f) / JM_R_SHUNT;
      if (fabsf(amps) <= JM_I_MAX * 1.25f) { jm.i = amps; jm.haveI = true; }
    }
  }
#endif

  if (!jm.haveV) jmMisses++;
#endif
}

static portMUX_TYPE ppsMux = portMUX_INITIALIZER_UNLOCKED;
static volatile int64_t  ppsEdgeUs   = 0;
static volatile uint32_t ppsCount    = 0;
static volatile uint32_t ppsRejected = 0;

void IRAM_ATTR ppsIsr() {
  int64_t t = esp_timer_get_time();
  portENTER_CRITICAL_ISR(&ppsMux);
  if (ppsCount != 0 && t - ppsEdgeUs < PPS_MIN_PERIOD_US) ppsRejected = ppsRejected + 1;
  else { ppsEdgeUs = t; ppsCount = ppsCount + 1; }
  portEXIT_CRITICAL_ISR(&ppsMux);
}

static void ppsSnapshot(int64_t *edgeUs, uint32_t *count, uint32_t *rejected) {
  portENTER_CRITICAL(&ppsMux);
  *edgeUs = ppsEdgeUs; *count = ppsCount; *rejected = ppsRejected;
  portEXIT_CRITICAL(&ppsMux);
}

static int64_t daysFromCivil(int y, unsigned m, unsigned d) {
  y -= (m <= 2);
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return (int64_t)era * 146097 + (int64_t)doe - 719468;
}
static void civilFromDays(int64_t z, int *y, unsigned *m, unsigned *d) {
  z += 719468;
  const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  const unsigned doe = (unsigned)(z - era * 146097);
  const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const unsigned mp = (5 * doy + 2) / 153;
  *d = doy - (153 * mp + 2) / 5 + 1;
  *m = mp < 10 ? mp + 3 : mp - 9;
  *y = (int)yoe + (int)(era * 400) + (*m <= 2 ? 1 : 0);
}

static void utcIso(int64_t ms, char *out, size_t sz) {
  int64_t s = ms / 1000;
  int milli = (int)(ms % 1000);
  int y; unsigned mo, d;
  civilFromDays(s / 86400, &y, &mo, &d);
  int rem = (int)(s % 86400);
  snprintf(out, sz, "%04d-%02u-%02uT%02d:%02d:%02d.%03dZ",
           y, mo, d, rem / 3600, (rem % 3600) / 60, rem % 60, milli);
}

static bool     utcOk       = false;
static int64_t  utcAnchorUs = 0;
static int64_t  utcAnchorMs = 0;
static bool     utcHaveLast = false;
static int64_t  utcLastSec  = 0;
static uint32_t utcLastCnt  = 0;
static uint32_t utcRejects  = 0;
static int32_t  utcDelayMs  = -1;

void utcPoll() {
  if (!gps.date.isUpdated()) return;
  const int64_t nowUs = esp_timer_get_time();

  const int      yr = gps.date.year();
  const unsigned mo = gps.date.month(), dy = gps.date.day();
  const unsigned hh = gps.time.hour(), mi = gps.time.minute(), ss = gps.time.second();
  const unsigned cs = gps.time.centisecond();

  if (!gps.location.isValid() || gps.location.age() > 1000) return;

  if (cs != 0) return;

  if (yr < 2024 || yr > 2099 || mo < 1 || mo > 12 || dy < 1 || dy > 31 ||
      hh > 23 || mi > 59 || ss > 59) { utcRejects++; return; }

  int64_t edgeUs; uint32_t cnt, rej;
  ppsSnapshot(&edgeUs, &cnt, &rej);
  const int64_t sinceEdge = nowUs - edgeUs;
  if (cnt == 0 || sinceEdge < 0 || sinceEdge > 900000) { utcRejects++; return; }
  if (utcHaveLast && cnt == utcLastCnt) return;

  const int64_t sec = daysFromCivil(yr, mo, dy) * 86400LL +
                      (int64_t)hh * 3600 + (int64_t)mi * 60 + (int64_t)ss +
                      UTC_SECOND_OFFSET;

  bool consistent = utcHaveLast && (int64_t)(uint32_t)(cnt - utcLastCnt) == (sec - utcLastSec);
  if (utcHaveLast && !consistent) { utcRejects++; utcOk = false; }

  utcHaveLast = true; utcLastSec = sec; utcLastCnt = cnt;
  utcDelayMs  = (int32_t)(sinceEdge / 1000);
  if (consistent) {
    utcAnchorUs = edgeUs;
    utcAnchorMs = sec * 1000;
    utcOk = true;
  }
}

bool utcMsAt(int64_t us, int64_t *ms) {
  if (!utcOk) return false;
  int64_t d = us - utcAnchorUs;
  if (d > UTC_MAX_AGE_US || d < -UTC_MAX_AGE_US) return false;
  *ms = utcAnchorMs + (d >= 0 ? d + 500 : d - 500) / 1000;
  return true;
}

static uint32_t imuCount = 0;
static volatile uint32_t imuDrops = 0, imuMisses = 0;

void utcDump() {
  int64_t edgeUs, now = esp_timer_get_time(); uint32_t cnt, rej;
  ppsSnapshot(&edgeUs, &cnt, &rej);
  if (cnt) Serial.printf("PPS : %lu edges, %lu rejected as noise, newest %.3f s ago\n",
                         (unsigned long)cnt, (unsigned long)rej, (now - edgeUs) / 1e6);
  else     Serial.printf("PPS : no edges yet (%lu rejected) - needs a GPS fix, and GPIO%d wired\n",
                         (unsigned long)rej, PIN_PPS);
  Serial.printf("UTC : %s, %lu candidate anchors refused, last NMEA-after-edge delay %ld ms\n",
                utcOk ? "locked" : "NOT locked", (unsigned long)utcRejects, (long)utcDelayMs);
  int64_t ms;
  if (utcMsAt(now, &ms)) {
    char s[32]; utcIso(ms, s, sizeof s);
    Serial.printf("      now = %s   <- compare with a network-synced clock\n", s);
  } else Serial.println(F("      now = (no UTC)"));
  Serial.printf("BOOT: id %lu\n", (unsigned long)bootId);
  Serial.printf("IMU : %lu samples, %.1f Hz average, %lu missed reads, %lu dropped (queue full)\n",
                (unsigned long)imuCount, now > 0 ? imuCount / (now / 1e6) : 0.0,
                (unsigned long)imuMisses, (unsigned long)imuDrops);
}

static QueueHandle_t imuQ = NULL;
static volatile bool imuPaused = false;
static ImuRaw imuLatest;
static bool   imuHaveLatest = false;

// IMU CHANGE (mounting): this is the ONE place that decides which column is which.  It feeds both the
// SD log (data_NN.csv and imu_NN.csv) and the live dashboard packet, so they change together.
//   Input  s.v[3], s.v[4], s.v[5] = chip accel X, Y, Z   (raw counts)
//          s.v[0], s.v[1], s.v[2] = chip gyro  X, Y, Z   (raw counts)
//   Output out[0..2] = accel X, Y, Z in m/s^2            out[3..5] = gyro X, Y, Z in deg/s
// As written, the output axes are the chip's own axes.
//
// To log in vehicle axes instead, replace the loop below with explicit lines.  Use the SAME
// permutation and signs for accel and gyro, and never mirror (swapping two axes needs one sign flip,
// or the axes become left-handed and the gyro signs come out wrong).  Example for a chip mounted
// with chip +Y pointing forward, chip +X to the right, chip +Z up, logging X=forward, Y=left, Z=up:
//     out[0] =  s.v[4] * ACC_MPS2_PER_LSB;     // forward = chip +Y
//     out[1] = -s.v[3] * ACC_MPS2_PER_LSB;     // left    = chip -X
//     out[2] =  s.v[5] * ACC_MPS2_PER_LSB;     // up      = chip +Z
//     out[3] =  s.v[1] * GYR_DPS_PER_LSB;      // gyro rotated the same way
//     out[4] = -s.v[0] * GYR_DPS_PER_LSB;
//     out[5] =  s.v[2] * GYR_DPS_PER_LSB;
// If you do this, ALSO edit imuSelfCheck() below, and add the axis convention to the CSV header text
// in openLog() so a log file says which axes it used.
static void imuUnits(const ImuRaw &s, float *out) {
  for (int k = 0; k < 3; k++) {
    out[k]     = s.v[3 + k] * ACC_MPS2_PER_LSB;
    out[3 + k] = s.v[k]     * GYR_DPS_PER_LSB;
  }
}

void imuPollOnce(int64_t t) {
  static int64_t lastUs = 0;
  static uint8_t fail = 0;
  static int64_t clearAt = 0;
  if (imuPaused) return;
  if (lastUs != 0 && t - lastUs < (1000000LL / IMU_HZ) / 2) return;
  lastUs = t;

  uint8_t b[12];
  ImuRaw s;
  bool good = i2cReadBytes(IMU_ADDR, IMU_REG_OUT, b, sizeof b);
  if (good) {
    for (int k = 0; k < 6; k++)
      s.v[k] = (int16_t)(((uint16_t)b[2 * k + 1] << 8) | b[2 * k]);
    s.us = t;
    if (s.v[3] == 0 && s.v[4] == 0 && s.v[5] == 0) good = false;
  }
  if (!good) {
    imuMisses = imuMisses + 1;
    if (++fail >= 20 && t >= clearAt) {
      i2cBusClear();
      fail = 0;
      clearAt = t + 1000000;
    }
    return;
  }
  fail = 0;
  if (xQueueSend(imuQ, &s, 0) != pdTRUE) imuDrops = imuDrops + 1;
}

void imuTask(void *) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / IMU_HZ);
  for (;;) {
    vTaskDelayUntil(&last, period);
    imuPollOnce(esp_timer_get_time());
  }
}

// IMU CHANGE: this boot check compares imuUnits() with the driver's own chip-axis x/y/z reading.
// If you remap the axes in imuUnits(), apply the SAME remap to a.acceleration.x / .y / .z in the
// comparison below.  Otherwise it reports a mismatch and switches the IMU off (accel/gyro then
// get omitted).  It also catches an ACCEL range changed in setup() without its scale constant.
// It does not look at the gyro.
bool imuSelfCheck() {
  delay(60);
  sensors_event_t a, g, t;
  if (!i2cLock()) return false;
  imu.getEvent(&a, &g, &t);
  i2cUnlock();
  uint8_t b[12];
  if (!i2cReadBytes(IMU_ADDR, IMU_REG_OUT, b, sizeof b)) {
    Serial.println(F("IMU: raw burst read failed"));
    return false;
  }
  ImuRaw s;
  for (int k = 0; k < 6; k++) s.v[k] = (int16_t)(((uint16_t)b[2 * k + 1] << 8) | b[2 * k]);
  float u[6];
  imuUnits(s, u);
  float d = fmaxf(fabsf(u[0] - a.acceleration.x),
            fmaxf(fabsf(u[1] - a.acceleration.y), fabsf(u[2] - a.acceleration.z)));
  Serial.printf("IMU: raw decode vs driver, worst accel difference %.3f m/s^2\n", d);
  if (!(d < 1.5f)) {
    Serial.println(F("IMU: raw decode DISAGREES with the driver — IMU switched off, "
                     "accel/gyro will be OMITTED. Check IMU_REG_OUT and the scale constants."));
    return false;
  }
  return true;
}

void sendGzipPage(WiFiClient &c) {
#ifndef HAVE_WEBAPP
  sendText(c, "200 OK", "text/html",
           "<!doctype html><meta name=viewport content='width=device-width'>"
           "<h3>UTSM telemetry</h3>"
           "<p>The dashboard is not in this build: webapp.h was missing from the "
           "sketch folder when it was compiled.</p>"
           "<p>The live data feed is still running at <a href='/events'>/events</a>.</p>");
  return;
#else
  c.print(F("HTTP/1.1 200 OK\r\n"
            "Content-Type: text/html; charset=utf-8\r\n"
            "Content-Encoding: gzip\r\n"
            "Cache-Control: no-cache\r\n"
            "Connection: close\r\nContent-Length: "));
  c.print(INDEX_HTML_GZ_LEN);
  c.print(F("\r\n\r\n"));

  // PLAN CHANGE: same guard as sendPlan(): stop on a short write or after 5 s.
  const size_t CH = 1024;
  const uint32_t t0 = millis();
  for (size_t i = 0; i < INDEX_HTML_GZ_LEN && c.connected() && millis() - t0 < 5000; i += CH) {
    size_t n = min(CH, (size_t)(INDEX_HTML_GZ_LEN - i));
    if (c.write(INDEX_HTML_GZ + i, n) != n) break;
    delay(0);
  }
#endif
}

// PLAN CHANGE: serve the compiled-in plan. Same chunked write from flash as the dashboard page.
// 404 when this build has no plan.h, so the page can tell "no plan" from "plan failed to load".
void sendPlan(WiFiClient &c) {
#ifndef HAVE_PLAN
  sendText(c, "404 Not Found", "text/plain", "no plan in this firmware");
#else
  const size_t total = sizeof(PLAN_JSON) - 1;
  c.print(F("HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Cache-Control: no-cache\r\n"
            "Connection: close\r\nContent-Length: "));
  c.print((unsigned)total);
  c.print(F("\r\n\r\n"));

  // PLAN CHANGE: a phone that wanders out of range mid-download must not hold the whole loop (GPS,
  // SSE, SD) hostage: stop on a short write or after 3 s. The page retries by itself.
  const size_t CH = 1024;
  const uint32_t t0 = millis();
  for (size_t i = 0; i < total && c.connected() && millis() - t0 < 3000; i += CH) {
    size_t n = min(CH, (size_t)(total - i));
    if (c.write((const uint8_t *)PLAN_JSON + i, n) != n) break;
    delay(0);
  }
#endif
}

void sendText(WiFiClient &c, const char *status, const char *type, const String &body) {
  c.printf("HTTP/1.1 %s\r\nContent-Type: %s\r\nCache-Control: no-cache\r\n"
           "Connection: close\r\nContent-Length: %u\r\n\r\n",
           status, type, (unsigned)body.length());
  c.print(body);
}

String helloJson() {
  String s = "{\"hello\":1,\"name\":\"" AP_SSID "\",\"fw\":\"utsm_wifi_telemetry 1.3\"";
  s += ",\"chip\":\"ESP32-D0WD-V3\"";
  s += ",\"hz\":" + String(SAMPLE_HZ);
  s += ",\"imuHz\":" + String(IMU_HZ);
  if (bootId) s += ",\"boot\":" + String((unsigned long)bootId);
#ifdef HAVE_PLAN
  s += ",\"plan\":\"" PLAN_ID "\"";
#else
  s += ",\"plan\":null";
#endif
  s += ",\"heap\":" + String((unsigned)ESP.getFreeHeap());
  s += ",\"clients\":" + String(sseCount());
  s += sdOk ? ",\"sd\":\"logging\"" : ",\"sd\":null";
  s += jmOk ? ",\"jm\":\"ina226\"" : ",\"jm\":null";
  s += ",\"jmMiss\":" + String((unsigned long)jmMisses);
  s += ",\"cfg\":{";
  auto f = [](float v) { return isNan(v) ? String("null") : String(v, 2); };
  s += "\"bandLo\":"  + f(cfg.bandLo)   + ",\"bandHi\":"  + f(cfg.bandHi);
  s += ",\"budgetWh\":" + f(cfg.budgetWh) + ",\"eventKm\":" + f(cfg.eventKm);
  s += ",\"leaveM\":" + f(cfg.leaveM)   + ",\"returnM\":" + f(cfg.returnM);
  s += ",\"minLapS\":" + f(cfg.minLapS) + "}}";
  return s;
}

int sseCount() { int n = 0; for (int i = 0; i < MAX_SSE; i++) if (sse[i] && sse[i].connected()) n++; return n; }

void addSse(WiFiClient &c) {
  for (int i = 0; i < MAX_SSE; i++) {
    if (!sse[i] || !sse[i].connected()) {
      sse[i] = c;
      sse[i].print(F("HTTP/1.1 200 OK\r\n"
                     "Content-Type: text/event-stream\r\n"
                     "Cache-Control: no-cache\r\n"
                     "Connection: keep-alive\r\n"
                     "Access-Control-Allow-Origin: *\r\n"
                     "X-Accel-Buffering: no\r\n\r\n"));
      sse[i].print("retry: 1500\ndata: " + helloJson() + "\n\n");
      return;
    }
  }
  sendText(c, "503 Service Unavailable", "text/plain", "too many viewers");
  c.stop();
}

void sseBroadcast(const char *json) {
  for (int i = 0; i < MAX_SSE; i++) {
    if (sse[i] && sse[i].connected()) { sse[i].print("data: "); sse[i].print(json); sse[i].print("\n\n"); }
    else if (sse[i]) { sse[i].stop(); }
  }
}

String qparam(const String &q, const String &key) {
  int i = q.indexOf(key + "=");
  while (i > 0 && q[i - 1] != '&' && q[i - 1] != '?') i = q.indexOf(key + "=", i + 1);
  if (i < 0) return "";
  int s = i + key.length() + 1, e = q.indexOf('&', s);
  return q.substring(s, e < 0 ? q.length() : e);
}
float qfloat(const String &q, const String &key, float dflt) {
  String v = qparam(q, key);
  if (v.length() == 0) return dflt;
  if (v == "null" || v == "NaN") return NAN;
  return v.toFloat();
}

void handleCmd(WiFiClient &c, const String &q) {
  String r = qparam(q, "run");
  if (r == "start") {
    runArmed = true; runArmT = millis();
    if (sdOk && logFile) { logFile.printf("# RUN START millis=%lu\n", (unsigned long)runArmT); logFile.flush(); }
  } else if (r == "stop") {
    runArmed = false;
    if (sdOk && logFile) { logFile.printf("# RUN STOP millis=%lu\n", (unsigned long)millis()); logFile.flush(); }
  }
  if (qparam(q, "set").length()) {
    cfg.bandLo   = qfloat(q, "bandLo",   cfg.bandLo);
    cfg.bandHi   = qfloat(q, "bandHi",   cfg.bandHi);
    cfg.budgetWh = qfloat(q, "budgetWh", cfg.budgetWh);
    cfg.eventKm  = qfloat(q, "eventKm",  cfg.eventKm);
    cfg.leaveM   = qfloat(q, "leaveM",   cfg.leaveM);
    cfg.returnM  = qfloat(q, "returnM",  cfg.returnM);
    cfg.minLapS  = qfloat(q, "minLapS",  cfg.minLapS);
    cfgSave();
  }
  sendText(c, "200 OK", "application/json", helloJson());
}

void handleHttp() {
  WiFiClient c = server.available();
  if (!c) return;

  uint32_t t0 = millis();
  String line;
  while (c.connected() && millis() - t0 < 400) {
    if (!c.available()) { delay(1); continue; }
    char ch = c.read();
    if (ch == '\n') break;
    if (ch != '\r' && line.length() < 220) line += ch;
  }
  while (c.available() && millis() - t0 < 500) c.read();

  int sp1 = line.indexOf(' '), sp2 = line.indexOf(' ', sp1 + 1);
  if (sp1 < 0 || sp2 < 0) { c.stop(); return; }
  String url = line.substring(sp1 + 1, sp2);
  String path = url, query = "";
  int qm = url.indexOf('?');
  if (qm >= 0) { path = url.substring(0, qm); query = url.substring(qm + 1); }

  if (path == "/events") { addSse(c); return; }
  if (path == "/cmd")    { handleCmd(c, query); c.stop(); return; }
  if (path == "/plan.json") { sendPlan(c); c.stop(); return; }
  if (path == "/" || path == "/index.html") { sendGzipPage(c); c.stop(); return; }

#if CAPTIVE_PORTAL

  c.print(F("HTTP/1.1 302 Found\r\nLocation: http://192.168.4.1/\r\n"
            "Connection: close\r\nContent-Length: 0\r\n\r\n"));
#else

  if (path.indexOf("hotspot-detect") >= 0 || path.indexOf("ncsi") >= 0 ||
      path.indexOf("connecttest") >= 0)
    sendText(c, "200 OK", "text/html",
             "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>");
  else if (path.indexOf("generate_204") >= 0 || path.indexOf("gen_204") >= 0)
    c.print(F("HTTP/1.1 204 No Content\r\nContent-Length: 0\r\n"
              "Connection: close\r\n\r\n"));
  else
    sendText(c, "404 Not Found", "text/plain", "not found");
#endif
  c.stop();
}

int staRssi() {
  wifi_sta_list_t sta;
  if (esp_wifi_ap_get_sta_list(&sta) != ESP_OK || sta.num == 0) return 0;
  return sta.sta[0].rssi;
}

struct Sample { bool have; float ax, ay, az, gx, gy, gz; } smp;

void readImu() {
  smp.have = false;
  if (!imuHaveLatest) return;
  if (esp_timer_get_time() - imuLatest.us > (int64_t)IMU_FRESH_MS * 1000) return;
  float u[6];
  imuUnits(imuLatest, u);
  smp.ax = u[0]; smp.ay = u[1]; smp.az = u[2];
  smp.gx = u[3]; smp.gy = u[4]; smp.gz = u[5];
  smp.have = true;
}

void buildPacket(char *buf, size_t sz) {
  size_t n = 0;
  appendf(buf, sz, &n, "{\"t\":%lu,\"seq\":%u", (unsigned long)millis(), (unsigned)seq++);

  if (smp.have)
    appendf(buf, sz, &n,
      ",\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f",
      smp.ax, smp.ay, smp.az, smp.gx, smp.gy, smp.gz);

  if (gps.location.isValid() && gps.location.age() < 3000)
    appendf(buf, sz, &n, ",\"lat\":%.6f,\"lon\":%.6f",
            gps.location.lat(), gps.location.lng());
  if (gps.altitude.isValid()  && gps.altitude.age()  < 3000)
    appendf(buf, sz, &n, ",\"alt\":%.1f", gps.altitude.meters());
  if (gps.speed.isValid()     && gps.speed.age()     < 3000)
    appendf(buf, sz, &n, ",\"spd\":%.2f", gps.speed.kmph());
  if (gps.satellites.isValid())
    appendf(buf, sz, &n, ",\"sats\":%d", (int)gps.satellites.value());
  if (gps.hdop.isValid())
    appendf(buf, sz, &n, ",\"hdop\":%.1f", gps.hdop.hdop());

#if HAVE_JOULEMETER

  if (jm.haveV) appendf(buf, sz, &n, ",\"v\":%.3f", jm.v);
  if (jm.haveI) appendf(buf, sz, &n, ",\"i\":%.3f", jm.i);
#endif

  appendf(buf, sz, &n, ",\"run\":%d", runArmed ? 1 : 0);
  if (runArmed) appendf(buf, sz, &n, ",\"armT\":%lu", (unsigned long)runArmT);
  int r = staRssi();
  if (r) appendf(buf, sz, &n, ",\"rssi\":%d", r);
  appendf(buf, sz, &n, "}");
}

void logRow() {
#if SD_LOGGING
  if (!sdOk || !logFile) return;
  char row[224];
  size_t n = 0;
  const int64_t nowUs = esp_timer_get_time();
  appendf(row, sizeof row, &n, "%lu,", (unsigned long)(uint32_t)(nowUs / 1000));

  if (smp.have)
    appendf(row, sizeof row, &n, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,",
            smp.ax, smp.ay, smp.az, smp.gx, smp.gy, smp.gz);
  else
    appendf(row, sizeof row, &n, ",,,,,,");

  if (gps.location.isValid())
    appendf(row, sizeof row, &n, "%.6f,%.6f,", gps.location.lat(), gps.location.lng());
  else
    appendf(row, sizeof row, &n, "0.000000,0.000000,");

  if (gps.altitude.isValid()) appendf(row, sizeof row, &n, "%.1f,", gps.altitude.meters());
  else                        appendf(row, sizeof row, &n, ",");

  if (gps.speed.isValid())    appendf(row, sizeof row, &n, "%.2f,", gps.speed.kmph());
  else                        appendf(row, sizeof row, &n, ",");

  if (gps.satellites.isValid()) appendf(row, sizeof row, &n, "%d,", (int)gps.satellites.value());
  else                          appendf(row, sizeof row, &n, ",");

  if (jm.haveV) appendf(row, sizeof row, &n, "%.3f,", jm.v);
  else          appendf(row, sizeof row, &n, ",");

  if (jm.haveI) appendf(row, sizeof row, &n, "%.3f,", jm.i);
  else          appendf(row, sizeof row, &n, ",");

  int64_t utcMs;
  if (utcMsAt(nowUs, &utcMs)) appendf(row, sizeof row, &n, "%lld,", (long long)utcMs);
  else                        appendf(row, sizeof row, &n, ",");

  if (bootId) appendf(row, sizeof row, &n, "%lu", (unsigned long)bootId);

  logFile.println(row);
  static uint32_t lastFlush = 0;
  if (millis() - lastFlush > 2000) { logFile.flush(); lastFlush = millis(); }
#endif
}

void imuLogRow(const ImuRaw &s) {
#if SD_LOGGING
  if (!imuLogOk || !imuFile) return;
  float u[6];
  imuUnits(s, u);
  char row[128];
  size_t n = 0;
  appendf(row, sizeof row, &n, "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,",
          (unsigned long)(uint32_t)(s.us / 1000), u[0], u[1], u[2], u[3], u[4], u[5]);
  int64_t utcMs;
  if (utcMsAt(s.us, &utcMs)) appendf(row, sizeof row, &n, "%lld,", (long long)utcMs);
  else                       appendf(row, sizeof row, &n, ",");
  if (bootId) appendf(row, sizeof row, &n, "%lu", (unsigned long)bootId);
  imuFile.println(row);
  static uint32_t lastFlush = 0;
  if (millis() - lastFlush > 2000) { imuFile.flush(); lastFlush = millis(); }
#endif
}

void imuDrain() {
  if (!imuQ) return;
  ImuRaw s;
  while (xQueueReceive(imuQ, &s, 0) == pdTRUE) {
    imuLatest = s;
    imuHaveLatest = true;
    imuCount++;
    imuLogRow(s);
  }
}

void openLog() {
#if SD_LOGGING
  if (!SD.begin(PIN_SD_CS)) { Serial.println(F("SD: no card (live telemetry still runs)")); return; }
  SD.mkdir("/Elec_Test");
  char path[32], ipath[32];
  for (int i = 0; i < 100; i++) {
    snprintf(path,  sizeof path,  "/Elec_Test/data_%02d.csv", i);
    snprintf(ipath, sizeof ipath, "/Elec_Test/imu_%02d.csv",  i);
    if (!SD.exists(path) && !SD.exists(ipath)) break;
  }
  logFile = SD.open(path, FILE_WRITE);
  if (!logFile) { Serial.println(F("SD: could not open a log file")); return; }
  sdOk = true;

  char board[16];
  boardIdStr(board, sizeof board);
  logFile.printf("# utsm_wifi_telemetry 1.3 · board=%s", board);
  if (bootId) logFile.printf(" · boot_id=%lu", (unsigned long)bootId);
  else        logFile.print(" · boot_id=unknown");
  // IMU CHANGE: "accel_range=4g . gyro_range=250dps" below is plain text, not read from the setting.
  // Update it whenever the range changes in setup() (also in the imu_NN.csv header further down).
  // If you remap axes in imuUnits(), add e.g. " · axes=fwd_left_up" here so the file says so.
  logFile.printf(" · sample_hz=%d · imu_hz=%d · imu_file=%s · accel_range=4g · gyro_range=250dps"
                 " · cal_applied=0 · accel_offset_mps2=%.4f,%.4f,%.4f"
                 " · utc=gps_pps(offset_s=%d)"
                 " · jm=%s · jm_current=%d",
                 SAMPLE_HZ, IMU_HZ, ipath + 11, accOff[0], accOff[1], accOff[2],
                 UTC_SECOND_OFFSET, jmOk ? "ina226" : "none", JM_HAVE_CURRENT);
#if JM_HAVE_CURRENT

  logFile.printf(" · r_shunt_ohm=%.5f", JM_R_SHUNT);
#endif
  // PLAN CHANGE: which burn/coast plan this board carried when the file was made ("none" = no plan.h)
  logFile.printf(" · plan=%s", PLAN_ID_STR);
  logFile.print('\n');
  logFile.println(F("Millis,AccelX_mps2,AccelY_mps2,AccelZ_mps2,GyroX_dps,GyroY_dps,GyroZ_dps,"
                    "Latitude,Longitude,Altitude_m,Speed_kmh,Satellites,Volts_V,Amps_A,UtcMs,BootId"));
  logFile.flush();
  Serial.printf("SD: logging to %s\n", path);

  imuFile = SD.open(ipath, FILE_WRITE);
  if (!imuFile) { Serial.println(F("SD: could not open the IMU log (10 Hz log still runs)")); return; }
  imuLogOk = true;
  imuFile.printf("# utsm_wifi_telemetry 1.3 imu · board=%s", board);
  if (bootId) imuFile.printf(" · boot_id=%lu", (unsigned long)bootId);
  else        imuFile.print(" · boot_id=unknown");
  // IMU CHANGE: same as above: range text here, and the axes note if you remapped in imuUnits().
  imuFile.printf(" · imu_hz=%d · accel_range=4g · gyro_range=250dps · cal_applied=0"
                 " · accel_offset_mps2=%.4f,%.4f,%.4f · stamp=read_start(leads_conversion_0_to_%dms)"
                 " · utc=gps_pps(offset_s=%d) · plan=%s\n",
                 IMU_HZ, accOff[0], accOff[1], accOff[2], 1000 / (IMU_HZ > 52 ? 104 : 52) + 1,
                 UTC_SECOND_OFFSET, PLAN_ID_STR);
  imuFile.println(F("Millis,AccelX_mps2,AccelY_mps2,AccelZ_mps2,GyroX_dps,GyroY_dps,GyroZ_dps,UtcMs,BootId"));
  imuFile.flush();
  Serial.printf("SD: IMU log %s\n", ipath);
#endif
}

// IMU CHANGE: the flip test reads the chip's own axes straight from the driver, so its offsets are
// per chip axis and do not depend on the mounting.  Do it on the bench: upright, then upside-down.
void flipTest() {
  if (!imuOk) { Serial.println(F("no IMU")); return; }
  imuPaused = true;
  float up[3], down[3];
  auto grab = [&](float *o) {
    double s[3] = { 0, 0, 0 };
    for (int i = 0; i < 200; i++) {
      sensors_event_t a, g, t;
      if (i2cLock()) { imu.getEvent(&a, &g, &t); i2cUnlock(); }
      if (i >= 5) { s[0] += a.acceleration.x; s[1] += a.acceleration.y; s[2] += a.acceleration.z; }
      delay(10);
    }
    for (int k = 0; k < 3; k++) o[k] = s[k] / 195.0;
  };
  Serial.println(F("Flip test: hold UPRIGHT and still, then press Enter."));
  while (Serial.read() != '\n') delay(10);
  grab(up);
  Serial.println(F("Now INVERT the board, hold still, press Enter."));
  while (Serial.read() != '\n') delay(10);
  grab(down);
  Serial.println(F("Paste these into accOff[]:"));
  for (int k = 0; k < 3; k++)
    Serial.printf("  ACC_OFF_%c = %.4f   (gravity %.4f)\n",
                  'X' + k, (up[k] + down[k]) / 2.0f, (up[k] - down[k]) / 2.0f);
  imuPaused = false;
}

void jmDump() {
  Serial.printf("JM: %s  misses=%lu  bus ceiling=%.1f V\n",
                jmOk ? "ok" : "absent", (unsigned long)jmMisses, JM_V_MAX);
  jmRead();
  if (jm.haveV) Serial.printf("  V = %.3f V\n", jm.v);
  else          Serial.println(F("  V = (no reading)"));
#if JM_HAVE_CURRENT
  Serial.printf("  shunt R = %.5f ohm, range +/-%.1f A\n",
                JM_R_SHUNT, 0.08192f / JM_R_SHUNT);
  if (jm.haveI) Serial.printf("  I = %.3f A\n", jm.i);
  else          Serial.println(F("  I = (no reading)"));
  if (jm.haveV && jm.haveI) Serial.printf("  P = %.2f W  (computed, not sent)\n", jm.v * jm.i);
#else
  Serial.println(F("  I = not measured (JM_HAVE_CURRENT 0 - no shunt wired)"));
  Serial.println(F("  P = no data, and correctly so: power needs both"));
#endif
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\nUTSM vehicle board — direct phone telemetry (1.3)"));

  i2cMux = xSemaphoreCreateMutex();
  imuQ   = xQueueCreate(IMU_QUEUE_LEN, sizeof(ImuRaw));

  pinMode(PIN_LORA_CS, OUTPUT); digitalWrite(PIN_LORA_CS, HIGH);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(I2C_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);

  // IMU CHANGE: address, range and data rate are set here.  If IMU_ADDR is 0x6B, use
  // imu.begin_I2C(IMU_ADDR).  A new range needs its scale constant changed too (table near the
  // top) and the range text in openLog().  The data rate follows IMU_HZ, so leave IMU_ODR alone.
  if (imu.begin_I2C()) {
    imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    imu.setAccelDataRate(IMU_ODR);
    imu.setGyroDataRate(IMU_ODR);
    imuOk = true;
    Serial.println(F("IMU: LSM6DSOX ok"));
  } else {
    Serial.println(F("IMU: not found — accel/gyro fields will be OMITTED, not zeroed"));
  }

  Wire.setClock(I2C_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);

  if (imuOk && !imuSelfCheck()) imuOk = false;

#if HAVE_JOULEMETER
  jmOk = jmBegin();
  if (jmOk) {
#if JM_HAVE_CURRENT
    Serial.printf("JM : INA226 at 0x%02X, volts + amps, R=%.5f ohm, range +/-%.1f A\n",
                  INA226_ADDR, JM_R_SHUNT, 0.08192f / JM_R_SHUNT);
    if (JM_I_MAX > 0.08192f / JM_R_SHUNT)
      Serial.println(F("JM : WARNING expected peak exceeds the shunt range — "
                       "current will blank at full scale, fit a smaller shunt"));
#else
    Serial.printf("JM : INA226 at 0x%02X, VOLTS ONLY (bus-only mode). "
                  "Current is not measured, so no amps are sent.\n", INA226_ADDR);
#endif
  } else {
    Serial.println(F("JM : no INA226 — v/i will be OMITTED, not zeroed. "
                     "Send 's' to scan the bus."));
  }
#endif

  GPSSerial.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

  pinMode(PIN_PPS, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(PIN_PPS), ppsIsr, RISING);

  bootId = bootIdAdvance();
  if (bootId) Serial.printf("BOOT: id %lu\n", (unsigned long)bootId);
  else        Serial.println(F("BOOT: counter could not be saved — BootId will be left EMPTY, not reused"));

  cfgLoad();
  openLog();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, strlen(AP_PASS) >= 8 ? AP_PASS : NULL);
  IPAddress ip = WiFi.softAPIP();
#if CAPTIVE_PORTAL
  dns.start(53, "*", ip);
#endif
  server.begin();
  server.setNoDelay(true);

  Serial.printf("AP  : %s  (password %s)\n", AP_SSID, strlen(AP_PASS) >= 8 ? AP_PASS : "<open>");
  Serial.printf("Open: http://%s/\n", ip.toString().c_str());
#ifndef HAVE_WEBAPP
  Serial.println(F("WEB : webapp.h was not found at build time - the page will be a placeholder. "
                   "Copy webapp.h into the sketch folder and rebuild."));
#endif
#ifdef HAVE_PLAN
  Serial.printf("PLAN: \"%s\"  id %s  (%u bytes, served at /plan.json)\n", PLAN_NAME, PLAN_ID,
                (unsigned)(sizeof(PLAN_JSON) - 1));
#else
  Serial.println(F("PLAN: none - no plan.h was in the sketch folder at build time, so the dashboard "
                   "uses the speed band / joulemeter cue. Add plan.h from the planner and rebuild."));
#endif
  Serial.println(F("Serial: 'c' flip test · 's' I2C scan · 'j' joulemeter readout · 'u' UTC/PPS/IMU check."));

  if (imuOk) {
    xTaskCreatePinnedToCore(imuTask, "imu", 6144, NULL, 2, NULL, 1);
    Serial.printf("IMU : %d Hz task running\n", IMU_HZ);
  }
}

void loop() {
#if CAPTIVE_PORTAL
  dns.processNextRequest();
#endif
  handleHttp();

  while (GPSSerial.available()) gps.encode(GPSSerial.read());
  utcPoll();
  imuDrain();

  if (Serial.available()) {
    int ch = Serial.peek();
    if      (ch == 'c') { Serial.read(); flipTest(); }
    else if (ch == 's') { Serial.read(); i2cScan(); }
    else if (ch == 'j') { Serial.read(); jmDump(); }
    else if (ch == 'u') { Serial.read(); utcDump(); }
    else                  Serial.read();
  }

  uint32_t now = millis();
  if (now - lastSample >= 1000 / SAMPLE_HZ) {
    lastSample = now;
    readImu();
    jmRead();
    char buf[384];
    buildPacket(buf, sizeof buf);
    sseBroadcast(buf);
    logRow();
  }
}
