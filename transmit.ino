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
#include <RadioLib.h>  // Arduino Library Manager: version 7.1.2
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


// ============================================================================
// GENERAL SETTINGS
// ============================================================================

#define AP_SSID      "UTSM-Telemetry"
#define AP_PASS      "utsm2027"

#define SAMPLE_HZ    10
#define MAX_SSE      4

// Supplied PCB ties SD CS to LORA_CS (GPIO5). REMOVE the SD card for radio
// testing; disabling software alone cannot deselect a card on that shared net.
// Rewire SD CS to GPIO2 before setting SD_LOGGING back to 1.
#define SD_LOGGING   0
#define HAVE_JOULEMETER 1

#define IMU_HZ       100

#define CAPTIVE_PORTAL 0


// ============================================================================
// LORA SETTINGS
//
// THESE MUST MATCH THE RECEIVER BOARD
// ============================================================================

#define LORA_FREQ_HZ       915000000UL
#define LORA_TX_HZ         5

#define LORA_SYNC_WORD     0xA7
#define LORA_TX_POWER_DBM  17

#define LORA_SF            7
#define LORA_BW_HZ         250000L
#define LORA_CR_DENOM      5


static_assert(
  IMU_HZ == 50 || IMU_HZ == 100,
  "IMU_HZ must be 50 or 100"
);

static_assert(
  LORA_TX_HZ > 0 &&
  LORA_TX_HZ <= SAMPLE_HZ,
  "LORA_TX_HZ must be 1..SAMPLE_HZ"
);


// ============================================================================
// PINS
// ============================================================================

static const int PIN_GPS_RX = 16;
static const int PIN_GPS_TX = 17;

static const int PIN_PPS = 4;


// Shared SPI bus

static const int PIN_SPI_SCK  = 18;
static const int PIN_SPI_MISO = 19;
static const int PIN_SPI_MOSI = 23;


// LoRa

static const int PIN_LORA_CS   = 5;
static const int PIN_LORA_RST  = 14;
static const int PIN_LORA_DIO1 = 12;
static const int PIN_LORA_BUSY = 25;
static const uint32_t PIN_LORA_RXEN = 13;
static const uint32_t PIN_LORA_TXEN = 15;
// Waveshare-linked ws_dev example uses TCXO_CTRL_1_7V (5 ms startup).
static const float LORA_TCXO_VOLTAGE = 1.7f;
static const bool LORA_DIO2_RF_SWITCH = false;
static const bool LORA_USE_LDO = false;

Module radioModule(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RST,
                   PIN_LORA_BUSY, SPI);
SX1262 radio(&radioModule);


// SD card

static const int PIN_SD_CS = 2;


// I2C

static const int PIN_I2C_SDA = 21;
static const int PIN_I2C_SCL = 22;


#define I2C_HZ          100000
#define I2C_TIMEOUT_MS  20


// ============================================================================
// INA226 / JOULEMETER
// ============================================================================

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
#define JM_CONFIG 0x4527
#else
#define JM_CONFIG 0x4526
#endif


// ============================================================================
// IMU SETTINGS
// ============================================================================

#define IMU_ADDR           0x6A

#define IMU_QUEUE_LEN      256

#define IMU_FRESH_MS       250

#define IMU_REG_OUT        0x22

#define ACC_MPS2_PER_LSB \
  (0.000122f * 9.80665f)

#define GYR_DPS_PER_LSB \
  0.00875f

#define IMU_ODR \
  (IMU_HZ > 52 ? LSM6DS_RATE_104_HZ : LSM6DS_RATE_52_HZ)


// ============================================================================
// UTC / PPS
// ============================================================================

#define UTC_MAX_AGE_US     5000000LL

#define PPS_MIN_PERIOD_US  900000LL

#define UTC_SECOND_OFFSET  0


// ============================================================================
// STRUCTURES
// ============================================================================

struct ImuRaw {

  int64_t us;

  int16_t v[6];
};


static inline bool isNan(float v) {

  return v != v;
}


// ============================================================================
// APPENDF
// ============================================================================

static void appendf(
  char *buf,
  size_t sz,
  size_t *n,
  const char *fmt,
  ...
)
__attribute__((format(printf, 4, 5)));


static void appendf(
  char *buf,
  size_t sz,
  size_t *n,
  const char *fmt,
  ...
) {

  if (
    sz == 0 ||
    *n >= sz - 1
  )
    return;


  va_list ap;

  va_start(
    ap,
    fmt
  );


  int w =
    vsnprintf(
      buf + *n,
      sz - *n,
      fmt,
      ap
    );


  va_end(ap);


  if (w < 0)
    return;


  *n += (size_t)w;


  if (*n > sz - 1)
    *n = sz - 1;
}


// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

int sseCount();

void cfgSave();

String helloJson();

bool jmBegin();

void i2cScan();

void sendText(
  WiFiClient &c,
  const char *status,
  const char *type,
  const String &body
);


// ============================================================================
// DEVICES
// ============================================================================

Adafruit_LSM6DSOX imu;

TinyGPSPlus gps;

HardwareSerial GPSSerial(2);

WiFiServer server(80);

DNSServer dns;

Preferences prefs;

File logFile;

File imuFile;


WiFiClient sse[MAX_SSE];


// ============================================================================
// GLOBAL STATE
// ============================================================================

bool imuOk = false;

bool sdOk = false;

bool imuLogOk = false;


uint16_t seq = 0;

uint32_t lastSample = 0;


bool runArmed = false;

uint32_t runArmT = 0;


uint32_t bootId = 0;


// LoRa

bool loraOk = false;

uint32_t lastLoraTx = 0;

uint32_t loraPackets = 0;

uint32_t loraBusySkips = 0;
uint32_t loraErrors = 0;
bool loraTxActive = false;
uint32_t loraTxStarted = 0;
static const uint32_t LORA_TX_TIMEOUT_MS = 1000;


// ============================================================================
// IMU OFFSET
// ============================================================================

float accOff[3] = {
  0,
  0,
  0
};


// ============================================================================
// CONFIG
// ============================================================================

struct Cfg {

  float bandLo;

  float bandHi;

  float budgetWh;

  float eventKm;

  float leaveM;

  float returnM;

  float minLapS;
};


Cfg cfg = {

  NAN,
  NAN,
  NAN,

  16.09f,

  40,

  25,

  20
};


// ============================================================================
// CONFIG LOAD
// ============================================================================

void cfgLoad() {

  prefs.begin(
    "utsm",
    true
  );


  cfg.bandLo =
    prefs.getFloat(
      "bandLo",
      NAN
    );


  cfg.bandHi =
    prefs.getFloat(
      "bandHi",
      NAN
    );


  cfg.budgetWh =
    prefs.getFloat(
      "budgetWh",
      NAN
    );


  cfg.eventKm =
    prefs.getFloat(
      "eventKm",
      16.09f
    );


  cfg.leaveM =
    prefs.getFloat(
      "leaveM",
      40
    );


  cfg.returnM =
    prefs.getFloat(
      "returnM",
      25
    );


  cfg.minLapS =
    prefs.getFloat(
      "minLapS",
      20
    );


  prefs.end();
}


// ============================================================================
// CONFIG SAVE
// ============================================================================

void cfgSave() {

  prefs.begin(
    "utsm",
    false
  );


  prefs.putFloat(
    "bandLo",
    cfg.bandLo
  );


  prefs.putFloat(
    "bandHi",
    cfg.bandHi
  );


  prefs.putFloat(
    "budgetWh",
    cfg.budgetWh
  );


  prefs.putFloat(
    "eventKm",
    cfg.eventKm
  );


  prefs.putFloat(
    "leaveM",
    cfg.leaveM
  );


  prefs.putFloat(
    "returnM",
    cfg.returnM
  );


  prefs.putFloat(
    "minLapS",
    cfg.minLapS
  );


  prefs.end();
}


// ============================================================================
// BOOT ID
// ============================================================================

uint32_t bootIdAdvance() {

  if (
    !prefs.begin(
      "utsm",
      false
    )
  )
    return 0;


  uint32_t id =
    prefs.getUInt(
      "bootId",
      0
    ) + 1;


  size_t written =
    prefs.putUInt(
      "bootId",
      id
    );


  prefs.end();


  return written
    ? id
    : 0;
}


// ============================================================================
// BOARD ID
// ============================================================================

static void boardIdStr(
  char *out,
  size_t sz
) {

  uint64_t mac =
    ESP.getEfuseMac();


  snprintf(
    out,
    sz,
    "%02X%02X%02X%02X%02X%02X",

    (unsigned)(mac & 0xFF),

    (unsigned)(
      (mac >> 8) & 0xFF
    ),

    (unsigned)(
      (mac >> 16) & 0xFF
    ),

    (unsigned)(
      (mac >> 24) & 0xFF
    ),

    (unsigned)(
      (mac >> 32) & 0xFF
    ),

    (unsigned)(
      (mac >> 40) & 0xFF
    )
  );
}


// ============================================================================
// I2C MUTEX
// ============================================================================

static SemaphoreHandle_t i2cMux = NULL;


static bool i2cLock() {

  return
    i2cMux != NULL &&
    xSemaphoreTake(
      i2cMux,
      pdMS_TO_TICKS(60)
    ) == pdTRUE;
}


static void i2cUnlock() {

  xSemaphoreGive(
    i2cMux
  );
}


// ============================================================================
// I2C READ 16
// ============================================================================

bool i2cRead16(
  uint8_t addr,
  uint8_t reg,
  uint16_t *out
) {

  if (!i2cLock())
    return false;


  bool ok = false;


  Wire.beginTransmission(addr);

  Wire.write(reg);


  if (
    Wire.endTransmission() == 0 &&

    Wire.requestFrom(
      (uint8_t)addr,
      (uint8_t)2
    ) == 2 &&

    Wire.available() >= 2
  ) {

    uint16_t hi =
      (uint8_t)Wire.read();


    uint16_t lo =
      (uint8_t)Wire.read();


    *out =
      (uint16_t)(
        (hi << 8) |
        lo
      );


    ok = true;
  }


  i2cUnlock();


  return ok;
}


// ============================================================================
// I2C WRITE 16
// ============================================================================

bool i2cWrite16(
  uint8_t addr,
  uint8_t reg,
  uint16_t val
) {

  if (!i2cLock())
    return false;


  Wire.beginTransmission(addr);

  Wire.write(reg);

  Wire.write(
    (uint8_t)(
      val >> 8
    )
  );

  Wire.write(
    (uint8_t)(
      val & 0xFF
    )
  );


  bool ok =
    Wire.endTransmission() == 0;


  i2cUnlock();


  return ok;
}


// ============================================================================
// I2C BYTE READ
// ============================================================================

bool i2cReadBytes(
  uint8_t addr,
  uint8_t reg,
  uint8_t *buf,
  size_t n
) {

  if (!i2cLock())
    return false;


  bool ok = false;


  Wire.beginTransmission(addr);

  Wire.write(reg);


  if (
    Wire.endTransmission() == 0 &&

    Wire.requestFrom(
      (uint8_t)addr,
      (uint8_t)n
    ) == n &&

    (size_t)Wire.available() >= n
  ) {

    for (
      size_t k = 0;
      k < n;
      k++
    ) {

      buf[k] =
        (uint8_t)Wire.read();
    }


    ok = true;
  }


  i2cUnlock();


  return ok;
}


// ============================================================================
// I2C BUS CLEAR
// ============================================================================

void i2cBusClear() {

  if (!i2cLock())
    return;


  Wire.end();


  pinMode(
    PIN_I2C_SDA,
    INPUT_PULLUP
  );


  pinMode(
    PIN_I2C_SCL,
    OUTPUT
  );


  digitalWrite(
    PIN_I2C_SCL,
    HIGH
  );


  for (
    int k = 0;
    k < 9;
    k++
  ) {

    digitalWrite(
      PIN_I2C_SCL,
      LOW
    );

    delayMicroseconds(5);


    digitalWrite(
      PIN_I2C_SCL,
      HIGH
    );

    delayMicroseconds(5);


    if (
      digitalRead(
        PIN_I2C_SDA
      ) == HIGH
    )
      break;
  }


  pinMode(
    PIN_I2C_SDA,
    OUTPUT
  );


  digitalWrite(
    PIN_I2C_SDA,
    LOW
  );

  delayMicroseconds(5);


  digitalWrite(
    PIN_I2C_SCL,
    HIGH
  );

  delayMicroseconds(5);


  digitalWrite(
    PIN_I2C_SDA,
    HIGH
  );

  delayMicroseconds(5);


  pinMode(
    PIN_I2C_SDA,
    INPUT_PULLUP
  );


  pinMode(
    PIN_I2C_SCL,
    INPUT_PULLUP
  );


  Wire.begin(
    PIN_I2C_SDA,
    PIN_I2C_SCL
  );


  Wire.setClock(
    I2C_HZ
  );


  Wire.setTimeOut(
    I2C_TIMEOUT_MS
  );


  i2cUnlock();
}


// ============================================================================
// I2C SCAN
// ============================================================================

void i2cScan() {

  Serial.println(
    F("I2C scan 0x08-0x77:")
  );


  int found = 0;


  for (
    uint8_t a = 0x08;
    a <= 0x77;
    a++
  ) {

    if (!i2cLock()) {

      Serial.println(
        F("bus busy")
      );

      return;
    }


    Wire.beginTransmission(a);


    bool hit =
      Wire.endTransmission() == 0;


    i2cUnlock();


    if (hit) {

      Serial.printf(
        "  0x%02X",
        a
      );


      if (
        a == 0x6A ||
        a == 0x6B
      ) {

        Serial.print(
          F(" LSM6DSOX")
        );
      }

      else if (
        a >= 0x40 &&
        a <= 0x4F
      ) {

        Serial.print(
          F(" INA226 range")
        );
      }


      Serial.println();


      found++;
    }


    delay(2);
  }


  if (!found) {

    Serial.println(
      F("nothing answered")
    );
  }
}


// ============================================================================
// JOULEMETER
// ============================================================================

static bool jmOk = false;

static uint8_t jmFail = 0;

static uint32_t jmRetryAt = 0;

static uint32_t jmMisses = 0;


struct Jm {

  bool haveV;

  bool haveI;

  float v;

  float i;
};


Jm jm;


// ============================================================================
// JOULEMETER INIT
// ============================================================================

bool jmBegin() {

  uint16_t id;


  if (
    !i2cRead16(
      INA226_ADDR,
      INA_REG_MANUF,
      &id
    )
  )
    return false;


  if (
    id != 0x5449
  )
    return false;


  if (
    !i2cRead16(
      INA226_ADDR,
      INA_REG_DIE,
      &id
    )
  )
    return false;


  if (
    (id >> 4) != 0x226
  )
    return false;


  if (
    !i2cWrite16(
      INA226_ADDR,
      INA_REG_CONFIG,
      0x8000
    )
  )
    return false;


  delay(2);


  if (
    !i2cWrite16(
      INA226_ADDR,
      INA_REG_CONFIG,
      JM_CONFIG
    )
  )
    return false;


#if JM_HAVE_CURRENT

  float currentLsb =
    JM_I_MAX / 32768.0f;


  float cal =
    0.00512f /
    (
      currentLsb *
      JM_R_SHUNT
    );


  if (
    cal >= 1.0f &&
    cal <= 65535.0f
  ) {

    i2cWrite16(
      INA226_ADDR,
      INA_REG_CAL,
      (uint16_t)cal
    );
  }

#endif


  return true;
}


// ============================================================================
// JOULEMETER READ
// ============================================================================

void jmRead() {

  jm.haveV = false;

  jm.haveI = false;


#if HAVE_JOULEMETER

  if (!jmOk) {

    if (
      (int32_t)(
        millis() -
        jmRetryAt
      ) < 0
    )
      return;


    jmRetryAt =
      millis() + 1000;


    jmOk =
      jmBegin();


    if (!jmOk)
      return;


    Serial.println(
      F("JM: INA226 back")
    );


    jmFail = 0;
  }


  uint16_t rawBus;


  bool okBus =
    i2cRead16(
      INA226_ADDR,
      INA_REG_BUS,
      &rawBus
    );


#if JM_HAVE_CURRENT

  uint16_t rawSh;


  bool okSh =
    i2cRead16(
      INA226_ADDR,
      INA_REG_SHUNT,
      &rawSh
    );

#endif


  bool alive =
    okBus &&
    rawBus <= 0x7FFF;


  if (!alive) {

    jmMisses++;


    if (++jmFail >= 2) {

      Serial.printf(
        "JM: no answer, misses=%lu\n",
        (unsigned long)jmMisses
      );


      i2cBusClear();


      jmOk = false;


      jmRetryAt =
        millis() + 1000;
    }


    return;
  }


  jmFail = 0;


  float v =
    rawBus *
    0.00125f;


  if (
    v <=
    JM_V_MAX *
    1.05f
  ) {

    jm.v = v;

    jm.haveV = true;
  }


#if JM_HAVE_CURRENT

  if (okSh) {

    int16_t s =
      (int16_t)rawSh;


    if (
      s > -32760 &&
      s < 32760
    ) {

      float amps =
        (
          s *
          0.0000025f
        ) /
        JM_R_SHUNT;


      if (
        fabsf(amps) <=
        JM_I_MAX *
        1.25f
      ) {

        jm.i = amps;

        jm.haveI = true;
      }
    }
  }

#endif


  if (!jm.haveV)
    jmMisses++;


#endif
}


// ============================================================================
// PPS / UTC
// ============================================================================

static portMUX_TYPE ppsMux =
  portMUX_INITIALIZER_UNLOCKED;


static volatile int64_t ppsEdgeUs = 0;

static volatile uint32_t ppsCount = 0;

static volatile uint32_t ppsRejected = 0;


// ============================================================================
// PPS ISR
// ============================================================================

void IRAM_ATTR ppsIsr() {

  int64_t t =
    esp_timer_get_time();


  portENTER_CRITICAL_ISR(
    &ppsMux
  );


  if (
    ppsCount != 0 &&
    t - ppsEdgeUs <
    PPS_MIN_PERIOD_US
  ) {

    ppsRejected++;
  }

  else {

    ppsEdgeUs = t;

    ppsCount++;
  }


  portEXIT_CRITICAL_ISR(
    &ppsMux
  );
}


// ============================================================================
// PPS SNAPSHOT
// ============================================================================

static void ppsSnapshot(
  int64_t *edgeUs,
  uint32_t *count,
  uint32_t *rejected
) {

  portENTER_CRITICAL(
    &ppsMux
  );


  *edgeUs = ppsEdgeUs;

  *count = ppsCount;

  *rejected = ppsRejected;


  portEXIT_CRITICAL(
    &ppsMux
  );
}


// ============================================================================
// UTC DATE CONVERSION
// ============================================================================

static int64_t daysFromCivil(
  int y,
  unsigned m,
  unsigned d
) {

  y -= (m <= 2);


  const int era =
    (
      y >= 0
      ? y
      : y - 399
    ) / 400;


  const unsigned yoe =
    (unsigned)(
      y -
      era * 400
    );


  const unsigned doy =
    (
      153 *
      (
        m > 2
        ? m - 3
        : m + 9
      ) +
      2
    ) /
    5 +
    d -
    1;


  const unsigned doe =
    yoe * 365 +
    yoe / 4 -
    yoe / 100 +
    doy;


  return
    (int64_t)era *
    146097 +
    (int64_t)doe -
    719468;
}


// ============================================================================

static void civilFromDays(
  int64_t z,
  int *y,
  unsigned *m,
  unsigned *d
) {

  z += 719468;


  const int64_t era =
    (
      z >= 0
      ? z
      : z - 146096
    ) /
    146097;


  const unsigned doe =
    (unsigned)(
      z -
      era *
      146097
    );


  const unsigned yoe =
    (
      doe -
      doe / 1460 +
      doe / 36524 -
      doe / 146096
    ) /
    365;


  const unsigned doy =
    doe -
    (
      365 * yoe +
      yoe / 4 -
      yoe / 100
    );


  const unsigned mp =
    (
      5 * doy + 2
    ) /
    153;


  *d =
    doy -
    (
      153 * mp + 2
    ) /
    5 +
    1;


  *m =
    mp < 10
    ? mp + 3
    : mp - 9;


  *y =
    (int)yoe +
    (int)(
      era *
      400
    ) +
    (
      *m <= 2
      ? 1
      : 0
    );
}


// ============================================================================
// UTC ISO
// ============================================================================

static void utcIso(
  int64_t ms,
  char *out,
  size_t sz
) {

  int64_t s =
    ms / 1000;


  int milli =
    (int)(
      ms % 1000
    );


  int y;

  unsigned mo;

  unsigned d;


  civilFromDays(
    s / 86400,
    &y,
    &mo,
    &d
  );


  int rem =
    (int)(
      s % 86400
    );


  snprintf(
    out,
    sz,
    "%04d-%02u-%02uT%02d:%02d:%02d.%03dZ",

    y,

    mo,

    d,

    rem / 3600,

    (rem % 3600) / 60,

    rem % 60,

    milli
  );
}


// ============================================================================
// UTC STATE
// ============================================================================

static bool utcOk = false;

static int64_t utcAnchorUs = 0;

static int64_t utcAnchorMs = 0;

static bool utcHaveLast = false;

static int64_t utcLastSec = 0;

static uint32_t utcLastCnt = 0;

static uint32_t utcRejects = 0;

static int32_t utcDelayMs = -1;


// ============================================================================
// UTC POLL
// ============================================================================

void utcPoll() {

  if (
    !gps.date.isUpdated()
  )
    return;


  const int64_t nowUs =
    esp_timer_get_time();


  const int yr =
    gps.date.year();


  const unsigned mo =
    gps.date.month();


  const unsigned dy =
    gps.date.day();


  const unsigned hh =
    gps.time.hour();


  const unsigned mi =
    gps.time.minute();


  const unsigned ss =
    gps.time.second();


  const unsigned cs =
    gps.time.centisecond();


  if (
    !gps.location.isValid() ||
    gps.location.age() > 1000
  )
    return;


  if (cs != 0)
    return;


  if (
    yr < 2024 ||
    yr > 2099 ||
    mo < 1 ||
    mo > 12 ||
    dy < 1 ||
    dy > 31 ||
    hh > 23 ||
    mi > 59 ||
    ss > 59
  ) {

    utcRejects++;

    return;
  }


  int64_t edgeUs;

  uint32_t cnt;

  uint32_t rej;


  ppsSnapshot(
    &edgeUs,
    &cnt,
    &rej
  );


  const int64_t sinceEdge =
    nowUs -
    edgeUs;


  if (
    cnt == 0 ||
    sinceEdge < 0 ||
    sinceEdge > 900000
  ) {

    utcRejects++;

    return;
  }


  if (
    utcHaveLast &&
    cnt == utcLastCnt
  )
    return;


  const int64_t sec =

    daysFromCivil(
      yr,
      mo,
      dy
    ) *
    86400LL +

    (int64_t)hh *
    3600 +

    (int64_t)mi *
    60 +

    (int64_t)ss +

    UTC_SECOND_OFFSET;


  bool consistent =

    utcHaveLast &&

    (int64_t)(
      (uint32_t)(
        cnt -
        utcLastCnt
      )
    ) ==

    (
      sec -
      utcLastSec
    );


  if (
    utcHaveLast &&
    !consistent
  ) {

    utcRejects++;

    utcOk = false;
  }


  utcHaveLast = true;

  utcLastSec = sec;

  utcLastCnt = cnt;


  utcDelayMs =
    (int32_t)(
      sinceEdge /
      1000
    );


  if (consistent) {

    utcAnchorUs =
      edgeUs;


    utcAnchorMs =
      sec *
      1000;


    utcOk = true;
  }
}


// ============================================================================
// UTC TIME LOOKUP
// ============================================================================

bool utcMsAt(
  int64_t us,
  int64_t *ms
) {

  if (!utcOk)
    return false;


  int64_t d =
    us -
    utcAnchorUs;


  if (
    d > UTC_MAX_AGE_US ||
    d < -UTC_MAX_AGE_US
  )
    return false;


  *ms =
    utcAnchorMs +
    (
      d >= 0
      ? d + 500
      : d - 500
    ) /
    1000;


  return true;
}


// ============================================================================
// IMU COUNTERS
// ============================================================================

static uint32_t imuCount = 0;

static volatile uint32_t imuDrops = 0;

static volatile uint32_t imuMisses = 0;


// ============================================================================
// UTC DEBUG
// ============================================================================

void utcDump() {

  int64_t edgeUs;

  int64_t now =
    esp_timer_get_time();


  uint32_t cnt;

  uint32_t rej;


  ppsSnapshot(
    &edgeUs,
    &cnt,
    &rej
  );


  if (cnt) {

    Serial.printf(
      "PPS: %lu edges, %lu rejected, newest %.3f s ago\n",

      (unsigned long)cnt,

      (unsigned long)rej,

      (
        now -
        edgeUs
      ) /
      1e6
    );
  }

  else {

    Serial.printf(
      "PPS: no edges yet, GPIO%d\n",
      PIN_PPS
    );
  }


  Serial.printf(
    "UTC: %s, rejects=%lu, delay=%ld ms\n",

    utcOk
      ? "locked"
      : "NOT locked",

    (unsigned long)utcRejects,

    (long)utcDelayMs
  );


  int64_t ms;


  if (
    utcMsAt(
      now,
      &ms
    )
  ) {

    char s[32];


    utcIso(
      ms,
      s,
      sizeof(s)
    );


    Serial.printf(
      "UTC now = %s\n",
      s
    );
  }


  Serial.printf(
    "BOOT: id %lu\n",
    (unsigned long)bootId
  );


  Serial.printf(
    "IMU: %lu samples, %lu missed, %lu dropped\n",

    (unsigned long)imuCount,

    (unsigned long)imuMisses,

    (unsigned long)imuDrops
  );
}


// ============================================================================
// IMU QUEUE
// ============================================================================

static QueueHandle_t imuQ = NULL;

static volatile bool imuPaused = false;

static ImuRaw imuLatest;

static bool imuHaveLatest = false;


// ============================================================================
// IMU CONVERSION
// ============================================================================

static void imuUnits(
  const ImuRaw &s,
  float *out
) {

  for (
    int k = 0;
    k < 3;
    k++
  ) {

    out[k] =
      s.v[3 + k] *
      ACC_MPS2_PER_LSB;


    out[3 + k] =
      s.v[k] *
      GYR_DPS_PER_LSB;
  }
}


// ============================================================================
// IMU POLL
// ============================================================================

void imuPollOnce(
  int64_t t
) {

  static int64_t lastUs = 0;

  static uint8_t fail = 0;

  static int64_t clearAt = 0;


  if (imuPaused)
    return;


  if (
    lastUs != 0 &&
    t - lastUs <
    (
      1000000LL /
      IMU_HZ
    ) /
    2
  )
    return;


  lastUs = t;


  uint8_t b[12];


  ImuRaw s;


  bool good =
    i2cReadBytes(
      IMU_ADDR,
      IMU_REG_OUT,
      b,
      sizeof(b)
    );


  if (good) {

    for (
      int k = 0;
      k < 6;
      k++
    ) {

      s.v[k] =
        (int16_t)(
          (
            (uint16_t)b[
              2 * k + 1
            ] << 8
          ) |
          b[
            2 * k
          ]
        );
    }


    s.us = t;


    if (
      s.v[3] == 0 &&
      s.v[4] == 0 &&
      s.v[5] == 0
    ) {

      good = false;
    }
  }


  if (!good) {

    imuMisses++;


    if (
      ++fail >= 20 &&
      t >= clearAt
    ) {

      i2cBusClear();


      fail = 0;


      clearAt =
        t +
        1000000;
    }


    return;
  }


  fail = 0;


  if (
    xQueueSend(
      imuQ,
      &s,
      0
    ) != pdTRUE
  ) {

    imuDrops++;
  }
}


// ============================================================================
// IMU TASK
// ============================================================================

void imuTask(void *) {

  TickType_t last =
    xTaskGetTickCount();


  const TickType_t period =
    pdMS_TO_TICKS(
      1000 /
      IMU_HZ
    );


  for (;;) {

    vTaskDelayUntil(
      &last,
      period
    );


    imuPollOnce(
      esp_timer_get_time()
    );
  }
}


// ============================================================================
// IMU SELF CHECK
// ============================================================================

bool imuSelfCheck() {

  delay(60);


  sensors_event_t a;

  sensors_event_t g;

  sensors_event_t t;


  if (!i2cLock())
    return false;


  imu.getEvent(
    &a,
    &g,
    &t
  );


  i2cUnlock();


  uint8_t b[12];


  if (
    !i2cReadBytes(
      IMU_ADDR,
      IMU_REG_OUT,
      b,
      sizeof(b)
    )
  ) {

    Serial.println(
      F("IMU: raw read failed")
    );


    return false;
  }


  ImuRaw s;


  for (
    int k = 0;
    k < 6;
    k++
  ) {

    s.v[k] =
      (int16_t)(
        (
          (uint16_t)b[
            2 * k + 1
          ] << 8
        ) |
        b[
          2 * k
        ]
      );
  }


  float u[6];


  imuUnits(
    s,
    u
  );


  float d =
    fmaxf(
      fabsf(
        u[0] -
        a.acceleration.x
      ),

      fmaxf(
        fabsf(
          u[1] -
          a.acceleration.y
        ),

        fabsf(
          u[2] -
          a.acceleration.z
        )
      )
    );


  Serial.printf(
    "IMU self-check difference %.3f m/s^2\n",
    d
  );


  if (!(d < 1.5f)) {

    Serial.println(
      F("IMU self-check failed")
    );


    return false;
  }


  return true;
}


// ============================================================================
// WEB PAGE
// ============================================================================

void sendGzipPage(
  WiFiClient &c
) {

#ifndef HAVE_WEBAPP

  sendText(
    c,
    "200 OK",
    "text/html",

    "<!doctype html>"
    "<meta name=viewport content='width=device-width'>"
    "<h3>UTSM telemetry</h3>"
    "<p>webapp.h missing.</p>"
    "<p>Live telemetry remains available at /events.</p>"
  );


  return;

#else

  c.print(
    F(
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: text/html; charset=utf-8\r\n"
      "Content-Encoding: gzip\r\n"
      "Cache-Control: no-cache\r\n"
      "Connection: close\r\n"
      "Content-Length: "
    )
  );


  c.print(
    INDEX_HTML_GZ_LEN
  );


  c.print(
    F("\r\n\r\n")
  );


  const size_t CH = 1024;


  const uint32_t t0 =
    millis();


  for (
    size_t i = 0;

    i < INDEX_HTML_GZ_LEN &&
    c.connected() &&
    millis() - t0 < 5000;

    i += CH
  ) {

    size_t n =
      min(
        CH,

        (size_t)(
          INDEX_HTML_GZ_LEN -
          i
        )
      );


    if (
      c.write(
        INDEX_HTML_GZ + i,
        n
      ) != n
    )
      break;


    delay(0);
  }

#endif
}


// ============================================================================
// SEND PLAN
// ============================================================================

void sendPlan(
  WiFiClient &c
) {

#ifndef HAVE_PLAN

  sendText(
    c,
    "404 Not Found",
    "text/plain",
    "no plan in this firmware"
  );

#else

  const size_t total =
    sizeof(PLAN_JSON) - 1;


  c.print(
    F(
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: application/json\r\n"
      "Cache-Control: no-cache\r\n"
      "Connection: close\r\n"
      "Content-Length: "
    )
  );


  c.print(
    (unsigned)total
  );


  c.print(
    F("\r\n\r\n")
  );


  const size_t CH = 1024;


  const uint32_t t0 =
    millis();


  for (
    size_t i = 0;

    i < total &&
    c.connected() &&
    millis() - t0 < 3000;

    i += CH
  ) {

    size_t n =
      min(
        CH,

        (size_t)(
          total -
          i
        )
      );


    if (
      c.write(
        (const uint8_t *)PLAN_JSON + i,
        n
      ) != n
    )
      break;


    delay(0);
  }

#endif
}


// ============================================================================
// SEND TEXT
// ============================================================================

void sendText(
  WiFiClient &c,
  const char *status,
  const char *type,
  const String &body
) {

  c.printf(
    "HTTP/1.1 %s\r\n"
    "Content-Type: %s\r\n"
    "Cache-Control: no-cache\r\n"
    "Connection: close\r\n"
    "Content-Length: %u\r\n"
    "\r\n",

    status,

    type,

    (unsigned)body.length()
  );


  c.print(body);
}


// ============================================================================
// HELLO JSON
// ============================================================================

String helloJson() {

  String s =

    "{\"hello\":1,"
    "\"name\":\"" AP_SSID "\","
    "\"fw\":\"utsm_wifi_telemetry 1.3\"";


  s +=
    ",\"chip\":\"ESP32-D0WD-V3\"";


  s +=
    ",\"hz\":" +
    String(SAMPLE_HZ);


  s +=
    ",\"imuHz\":" +
    String(IMU_HZ);


  if (bootId) {

    s +=
      ",\"boot\":" +
      String(
        (unsigned long)bootId
      );
  }


#ifdef HAVE_PLAN

  s +=
    ",\"plan\":\"" PLAN_ID "\"";

#else

  s +=
    ",\"plan\":null";

#endif


  s +=
    ",\"heap\":" +
    String(
      (unsigned)ESP.getFreeHeap()
    );


  s +=
    ",\"clients\":" +
    String(
      sseCount()
    );


  s +=
    sdOk
    ? ",\"sd\":\"logging\""
    : ",\"sd\":null";


  s +=
    jmOk
    ? ",\"jm\":\"ina226\""
    : ",\"jm\":null";


  s +=
    ",\"jmMiss\":" +
    String(
      (unsigned long)jmMisses
    );


  s +=
    loraOk
    ? ",\"lora\":\"tx\""
    : ",\"lora\":null";


  s +=
    ",\"loraTx\":" +
    String(
      (unsigned long)loraPackets
    );


  s +=
    ",\"loraBusy\":" +
    String(
      (unsigned long)loraBusySkips
    );


  s +=
    ",\"cfg\":{";


  auto f = [](float v) {

    return
      isNan(v)
      ? String("null")
      : String(
          v,
          2
        );
  };


  s +=
    "\"bandLo\":" +
    f(cfg.bandLo);


  s +=
    ",\"bandHi\":" +
    f(cfg.bandHi);


  s +=
    ",\"budgetWh\":" +
    f(cfg.budgetWh);


  s +=
    ",\"eventKm\":" +
    f(cfg.eventKm);


  s +=
    ",\"leaveM\":" +
    f(cfg.leaveM);


  s +=
    ",\"returnM\":" +
    f(cfg.returnM);


  s +=
    ",\"minLapS\":" +
    f(cfg.minLapS);


  s += "}}";


  return s;
}


// ============================================================================
// SSE
// ============================================================================

int sseCount() {

  int n = 0;


  for (
    int i = 0;
    i < MAX_SSE;
    i++
  ) {

    if (
      sse[i] &&
      sse[i].connected()
    ) {

      n++;
    }
  }


  return n;
}


// ============================================================================

void addSse(
  WiFiClient &c
) {

  for (
    int i = 0;
    i < MAX_SSE;
    i++
  ) {

    if (
      !sse[i] ||
      !sse[i].connected()
    ) {

      sse[i] = c;


      sse[i].print(
        F(
          "HTTP/1.1 200 OK\r\n"
          "Content-Type: text/event-stream\r\n"
          "Cache-Control: no-cache\r\n"
          "Connection: keep-alive\r\n"
          "Access-Control-Allow-Origin: *\r\n"
          "X-Accel-Buffering: no\r\n"
          "\r\n"
        )
      );


      sse[i].print(
        "retry: 1500\ndata: " +
        helloJson() +
        "\n\n"
      );


      return;
    }
  }


  sendText(
    c,
    "503 Service Unavailable",
    "text/plain",
    "too many viewers"
  );


  c.stop();
}


// ============================================================================

void sseBroadcast(
  const char *json
) {

  for (
    int i = 0;
    i < MAX_SSE;
    i++
  ) {

    if (
      sse[i] &&
      sse[i].connected()
    ) {

      sse[i].print(
        "data: "
      );


      sse[i].print(
        json
      );


      sse[i].print(
        "\n\n"
      );
    }

    else if (sse[i]) {

      sse[i].stop();
    }
  }
}


// ============================================================================
// QUERY PARAMETER
// ============================================================================

String qparam(
  const String &q,
  const String &key
) {

  int i =
    q.indexOf(
      key + "="
    );


  while (
    i > 0 &&
    q[i - 1] != '&' &&
    q[i - 1] != '?'
  ) {

    i =
      q.indexOf(
        key + "=",
        i + 1
      );
  }


  if (i < 0)
    return "";


  int s =
    i +
    key.length() +
    1;


  int e =
    q.indexOf(
      '&',
      s
    );


  return
    q.substring(
      s,
      e < 0
      ? q.length()
      : e
    );
}


// ============================================================================

float qfloat(
  const String &q,
  const String &key,
  float dflt
) {

  String v =
    qparam(
      q,
      key
    );


  if (
    v.length() == 0
  )
    return dflt;


  if (
    v == "null" ||
    v == "NaN"
  )
    return NAN;


  return v.toFloat();
}


// ============================================================================
// COMMAND HANDLER
// ============================================================================

void handleCmd(
  WiFiClient &c,
  const String &q
) {

  String r =
    qparam(
      q,
      "run"
    );


  if (
    r == "start"
  ) {

    runArmed = true;

    runArmT =
      millis();


    if (
      sdOk &&
      logFile
    ) {

      logFile.printf(
        "# RUN START millis=%lu\n",
        (unsigned long)runArmT
      );


      logFile.flush();
    }
  }

  else if (
    r == "stop"
  ) {

    runArmed = false;


    if (
      sdOk &&
      logFile
    ) {

      logFile.printf(
        "# RUN STOP millis=%lu\n",
        (unsigned long)millis()
      );


      logFile.flush();
    }
  }


  if (
    qparam(
      q,
      "set"
    ).length()
  ) {

    cfg.bandLo =
      qfloat(
        q,
        "bandLo",
        cfg.bandLo
      );


    cfg.bandHi =
      qfloat(
        q,
        "bandHi",
        cfg.bandHi
      );


    cfg.budgetWh =
      qfloat(
        q,
        "budgetWh",
        cfg.budgetWh
      );


    cfg.eventKm =
      qfloat(
        q,
        "eventKm",
        cfg.eventKm
      );


    cfg.leaveM =
      qfloat(
        q,
        "leaveM",
        cfg.leaveM
      );


    cfg.returnM =
      qfloat(
        q,
        "returnM",
        cfg.returnM
      );


    cfg.minLapS =
      qfloat(
        q,
        "minLapS",
        cfg.minLapS
      );


    cfgSave();
  }


  sendText(
    c,
    "200 OK",
    "application/json",
    helloJson()
  );
}


// ============================================================================
// HTTP
// ============================================================================

void handleHttp() {

  WiFiClient c =
    server.available();


  if (!c)
    return;


  uint32_t t0 =
    millis();


  String line;


  while (
    c.connected() &&
    millis() - t0 < 400
  ) {

    if (!c.available()) {

      delay(1);

      continue;
    }


    char ch =
      c.read();


    if (
      ch == '\n'
    )
      break;


    if (
      ch != '\r' &&
      line.length() < 220
    ) {

      line += ch;
    }
  }


  while (
    c.available() &&
    millis() - t0 < 500
  ) {

    c.read();
  }


  int sp1 =
    line.indexOf(' ');


  int sp2 =
    line.indexOf(
      ' ',
      sp1 + 1
    );


  if (
    sp1 < 0 ||
    sp2 < 0
  ) {

    c.stop();

    return;
  }


  String url =
    line.substring(
      sp1 + 1,
      sp2
    );


  String path =
    url;


  String query = "";


  int qm =
    url.indexOf('?');


  if (qm >= 0) {

    path =
      url.substring(
        0,
        qm
      );


    query =
      url.substring(
        qm + 1
      );
  }


  if (
    path == "/events"
  ) {

    addSse(c);

    return;
  }


  if (
    path == "/cmd"
  ) {

    handleCmd(
      c,
      query
    );


    c.stop();

    return;
  }


  if (
    path == "/plan.json"
  ) {

    sendPlan(c);

    c.stop();

    return;
  }


  if (
    path == "/" ||
    path == "/index.html"
  ) {

    sendGzipPage(c);

    c.stop();

    return;
  }


#if CAPTIVE_PORTAL

  c.print(
    F(
      "HTTP/1.1 302 Found\r\n"
      "Location: http://192.168.4.1/\r\n"
      "Connection: close\r\n"
      "Content-Length: 0\r\n"
      "\r\n"
    )
  );

#else

  if (
    path.indexOf(
      "hotspot-detect"
    ) >= 0 ||
    path.indexOf(
      "ncsi"
    ) >= 0 ||
    path.indexOf(
      "connecttest"
    ) >= 0
  ) {

    sendText(
      c,
      "200 OK",
      "text/html",
      "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>"
    );
  }

  else if (
    path.indexOf(
      "generate_204"
    ) >= 0 ||
    path.indexOf(
      "gen_204"
    ) >= 0
  ) {

    c.print(
      F(
        "HTTP/1.1 204 No Content\r\n"
        "Content-Length: 0\r\n"
        "Connection: close\r\n"
        "\r\n"
      )
    );
  }

  else {

    sendText(
      c,
      "404 Not Found",
      "text/plain",
      "not found"
    );
  }

#endif


  c.stop();
}


// ============================================================================
// WIFI RSSI
// ============================================================================

int staRssi() {

  wifi_sta_list_t sta;


  if (
    esp_wifi_ap_get_sta_list(
      &sta
    ) != ESP_OK ||
    sta.num == 0
  ) {

    return 0;
  }


  return
    sta.sta[0].rssi;
}


// ============================================================================
// LIVE SAMPLE
// ============================================================================

struct Sample {

  bool have;

  float ax;

  float ay;

  float az;

  float gx;

  float gy;

  float gz;
};


Sample smp;


// ============================================================================
// LORA PACKET FLAGS
// ============================================================================

enum : uint16_t {

  LORA_F_IMU      = 1u << 0,

  LORA_F_GPS_POS  = 1u << 1,

  LORA_F_ALT      = 1u << 2,

  LORA_F_SPEED    = 1u << 3,

  LORA_F_SATS     = 1u << 4,

  LORA_F_HDOP     = 1u << 5,

  LORA_F_VOLTS    = 1u << 6,

  LORA_F_AMPS     = 1u << 7,

  LORA_F_RUN      = 1u << 8,

  LORA_F_UTC      = 1u << 9
};


// ============================================================================
// LORA TELEMETRY PACKET
// ============================================================================

struct __attribute__((packed)) LoraTelemetry {

  uint16_t magic;

  uint8_t version;

  uint16_t flags;


  uint16_t seq;

  uint32_t millis32;


  float ax;

  float ay;

  float az;


  float gx;

  float gy;

  float gz;


  int32_t latE7;

  int32_t lonE7;


  float altM;

  float speedKmh;


  uint8_t sats;

  float hdop;


  float volts;

  float amps;


  uint32_t armT;

  uint32_t bootId;


  int64_t utcMs;
};


static_assert(
  sizeof(LoraTelemetry) < 255,
  "LoRa packet too large"
);


// ============================================================================
// LORA INIT
// ============================================================================

bool loraBegin() {
  pinMode(PIN_LORA_CS, OUTPUT);
  digitalWrite(PIN_LORA_CS, HIGH);
  pinMode(PIN_LORA_DIO1, INPUT);
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

  int16_t state = radio.begin(LORA_FREQ_HZ / 1000000.0f,
                              LORA_BW_HZ / 1000.0f, LORA_SF,
                              LORA_CR_DENOM, LORA_SYNC_WORD,
                              LORA_TX_POWER_DBM, 8,
                              LORA_TCXO_VOLTAGE, LORA_USE_LDO);
  if (state == RADIOLIB_ERR_NONE)
    state = radio.setDio2AsRfSwitch(LORA_DIO2_RF_SWITCH);
  if (state == RADIOLIB_ERR_NONE) {
    radio.setRfSwitchPins(PIN_LORA_RXEN, PIN_LORA_TXEN);
    state = radio.explicitHeader();
  }
  if (state == RADIOLIB_ERR_NONE) state = radio.setCRC(2);
  if (state == RADIOLIB_ERR_NONE) state = radio.invertIQ(false);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("LoRa: SX1262 initialization failed, RadioLib code %d\n", (int)state);
    return false;
  }
  loraTxActive = false;
  Serial.printf("LoRa: SX1262 TX ready %.3f MHz, SF%d, BW %ld, CR 4/%d, %d dBm, packet %u bytes\n",
                LORA_FREQ_HZ / 1000000.0, LORA_SF, (long)LORA_BW_HZ,
                LORA_CR_DENOM, LORA_TX_POWER_DBM, (unsigned)sizeof(LoraTelemetry));
  return true;
}

// Service async transmission every loop; no sensor sampling or SPI in an ISR.
void loraPoll() {
  if (!loraOk || !loraTxActive) return;
  const bool completed = digitalRead(PIN_LORA_DIO1) == HIGH;
  if (!completed && millis() - loraTxStarted < LORA_TX_TIMEOUT_MS) return;
  const int16_t state = radio.finishTransmit();
  loraTxActive = false;
  if (completed && state == RADIOLIB_ERR_NONE) {
    ++loraPackets;  // Radio TxDone, not an acknowledgment from the receiver.
  } else {
    ++loraErrors;
    Serial.printf("LoRa: TX %s, cleanup code %d\n",
                  completed ? "cleanup failed" : "timed out", (int)state);
  }
  if (state != RADIOLIB_ERR_NONE) loraOk = false;
}


// ============================================================================
// LORA TRANSMIT
// ============================================================================

void loraSendTelemetry(
  uint16_t packetSeq
) {

  if (!loraOk)
    return;


  uint32_t now =
    millis();


  if (
    now -
    lastLoraTx <
    (
      1000UL /
      LORA_TX_HZ
    )
  )
    return;


  if (
    loraTxActive
  ) {

    loraBusySkips++;

    return;
  }


  lastLoraTx = now;


  LoraTelemetry p = {};


  p.magic =
    0x5554;


  p.version =
    1;


  p.seq =
    packetSeq;


  p.millis32 =
    now;


  p.bootId =
    bootId;


  // IMU

  if (smp.have) {

    p.flags |=
      LORA_F_IMU;


    p.ax =
      smp.ax;


    p.ay =
      smp.ay;


    p.az =
      smp.az;


    p.gx =
      smp.gx;


    p.gy =
      smp.gy;


    p.gz =
      smp.gz;
  }


  // GPS position

  if (
    gps.location.isValid() &&
    gps.location.age() < 3000
  ) {

    p.flags |=
      LORA_F_GPS_POS;


    p.latE7 =
      (int32_t)llround(
        gps.location.lat() *
        10000000.0
      );


    p.lonE7 =
      (int32_t)llround(
        gps.location.lng() *
        10000000.0
      );
  }


  // Altitude

  if (
    gps.altitude.isValid() &&
    gps.altitude.age() < 3000
  ) {

    p.flags |=
      LORA_F_ALT;


    p.altM =
      gps.altitude.meters();
  }


  // Speed

  if (
    gps.speed.isValid() &&
    gps.speed.age() < 3000
  ) {

    p.flags |=
      LORA_F_SPEED;


    p.speedKmh =
      gps.speed.kmph();
  }


  // Satellites

  if (
    gps.satellites.isValid()
  ) {

    p.flags |=
      LORA_F_SATS;


    uint32_t satCount =
      gps.satellites.value();


    if (
      satCount > 255
    )
      satCount = 255;


    p.sats =
      (uint8_t)satCount;
  }


  // HDOP

  if (
    gps.hdop.isValid()
  ) {

    p.flags |=
      LORA_F_HDOP;


    p.hdop =
      gps.hdop.hdop();
  }


#if HAVE_JOULEMETER

  if (jm.haveV) {

    p.flags |=
      LORA_F_VOLTS;


    p.volts =
      jm.v;
  }


  if (jm.haveI) {

    p.flags |=
      LORA_F_AMPS;


    p.amps =
      jm.i;
  }

#endif


  // Run state

  if (runArmed) {

    p.flags |=
      LORA_F_RUN;


    p.armT =
      runArmT;
  }


  // UTC

  int64_t utc;


  if (
    utcMsAt(
      esp_timer_get_time(),
      &utc
    )
  ) {

    p.flags |=
      LORA_F_UTC;


    p.utcMs =
      utc;
  }


  // RadioLib copies the payload into the radio FIFO before returning.
  // DIO1 reports TxDone; loraPoll() releases the RF switch and counts it.
  const int16_t state = radio.startTransmit((const uint8_t *)&p, sizeof(p));
  if (state == RADIOLIB_ERR_NONE) {
    loraTxActive = true;
    loraTxStarted = millis();
  } else {
    ++loraErrors;
    const int16_t cleanup = radio.finishTransmit();
    if (cleanup != RADIOLIB_ERR_NONE) loraOk = false;
    Serial.printf("LoRa: start TX failed, RadioLib code %d\n", (int)state);
  }
}


// ============================================================================
// READ IMU LATEST
// ============================================================================

void readImu() {

  smp.have = false;


  if (
    !imuHaveLatest
  )
    return;


  if (
    esp_timer_get_time() -
    imuLatest.us >
    (int64_t)IMU_FRESH_MS *
    1000
  )
    return;


  float u[6];


  imuUnits(
    imuLatest,
    u
  );


  smp.ax = u[0];

  smp.ay = u[1];

  smp.az = u[2];


  smp.gx = u[3];

  smp.gy = u[4];

  smp.gz = u[5];


  smp.have = true;
}


// ============================================================================
// BUILD WIFI JSON PACKET
// ============================================================================

void buildPacket(
  char *buf,
  size_t sz,
  uint16_t packetSeq
) {

  size_t n = 0;


  appendf(
    buf,
    sz,
    &n,

    "{\"t\":%lu,\"seq\":%u",

    (unsigned long)millis(),

    (unsigned)packetSeq
  );


  if (smp.have) {

    appendf(
      buf,
      sz,
      &n,

      ",\"ax\":%.3f,"
      "\"ay\":%.3f,"
      "\"az\":%.3f,"
      "\"gx\":%.3f,"
      "\"gy\":%.3f,"
      "\"gz\":%.3f",

      smp.ax,
      smp.ay,
      smp.az,

      smp.gx,
      smp.gy,
      smp.gz
    );
  }


  if (
    gps.location.isValid() &&
    gps.location.age() < 3000
  ) {

    appendf(
      buf,
      sz,
      &n,

      ",\"lat\":%.6f,"
      "\"lon\":%.6f",

      gps.location.lat(),

      gps.location.lng()
    );
  }


  if (
    gps.altitude.isValid() &&
    gps.altitude.age() < 3000
  ) {

    appendf(
      buf,
      sz,
      &n,

      ",\"alt\":%.1f",

      gps.altitude.meters()
    );
  }


  if (
    gps.speed.isValid() &&
    gps.speed.age() < 3000
  ) {

    appendf(
      buf,
      sz,
      &n,

      ",\"spd\":%.2f",

      gps.speed.kmph()
    );
  }


  if (
    gps.satellites.isValid()
  ) {

    appendf(
      buf,
      sz,
      &n,

      ",\"sats\":%d",

      (int)gps.satellites.value()
    );
  }


  if (
    gps.hdop.isValid()
  ) {

    appendf(
      buf,
      sz,
      &n,

      ",\"hdop\":%.1f",

      gps.hdop.hdop()
    );
  }


#if HAVE_JOULEMETER

  if (jm.haveV) {

    appendf(
      buf,
      sz,
      &n,

      ",\"v\":%.3f",

      jm.v
    );
  }


  if (jm.haveI) {

    appendf(
      buf,
      sz,
      &n,

      ",\"i\":%.3f",

      jm.i
    );
  }

#endif


  appendf(
    buf,
    sz,
    &n,

    ",\"run\":%d",

    runArmed
      ? 1
      : 0
  );


  if (runArmed) {

    appendf(
      buf,
      sz,
      &n,

      ",\"armT\":%lu",

      (unsigned long)runArmT
    );
  }


  int r =
    staRssi();


  if (r) {

    appendf(
      buf,
      sz,
      &n,

      ",\"rssi\":%d",

      r
    );
  }


  appendf(
    buf,
    sz,
    &n,
    "}"
  );
}


// ============================================================================
// LOG MAIN ROW
// ============================================================================

void logRow() {

#if SD_LOGGING

  if (
    !sdOk ||
    !logFile
  )
    return;


  char row[224];


  size_t n = 0;


  const int64_t nowUs =
    esp_timer_get_time();


  appendf(
    row,
    sizeof(row),
    &n,

    "%lu,",

    (unsigned long)(
      (uint32_t)(
        nowUs /
        1000
      )
    )
  );


  if (smp.have) {

    appendf(
      row,
      sizeof(row),
      &n,

      "%.3f,%.3f,%.3f,"
      "%.3f,%.3f,%.3f,",

      smp.ax,
      smp.ay,
      smp.az,

      smp.gx,
      smp.gy,
      smp.gz
    );
  }

  else {

    appendf(
      row,
      sizeof(row),
      &n,
      ",,,,,,"
    );
  }


  if (
    gps.location.isValid()
  ) {

    appendf(
      row,
      sizeof(row),
      &n,

      "%.6f,%.6f,",

      gps.location.lat(),

      gps.location.lng()
    );
  }

  else {

    appendf(
      row,
      sizeof(row),
      &n,

      "0.000000,"
      "0.000000,"
    );
  }


  if (
    gps.altitude.isValid()
  ) {

    appendf(
      row,
      sizeof(row),
      &n,

      "%.1f,",

      gps.altitude.meters()
    );
  }

  else {

    appendf(
      row,
      sizeof(row),
      &n,
      ","
    );
  }


  if (
    gps.speed.isValid()
  ) {

    appendf(
      row,
      sizeof(row),
      &n,

      "%.2f,",

      gps.speed.kmph()
    );
  }

  else {

    appendf(
      row,
      sizeof(row),
      &n,
      ","
    );
  }


  if (
    gps.satellites.isValid()
  ) {

    appendf(
      row,
      sizeof(row),
      &n,

      "%d,",

      (int)gps.satellites.value()
    );
  }

  else {

    appendf(
      row,
      sizeof(row),
      &n,
      ","
    );
  }


  if (jm.haveV) {

    appendf(
      row,
      sizeof(row),
      &n,

      "%.3f,",

      jm.v
    );
  }

  else {

    appendf(
      row,
      sizeof(row),
      &n,
      ","
    );
  }


  if (jm.haveI) {

    appendf(
      row,
      sizeof(row),
      &n,

      "%.3f,",

      jm.i
    );
  }

  else {

    appendf(
      row,
      sizeof(row),
      &n,
      ","
    );
  }


  int64_t utcMs;


  if (
    utcMsAt(
      nowUs,
      &utcMs
    )
  ) {

    appendf(
      row,
      sizeof(row),
      &n,

      "%lld,",

      (long long)utcMs
    );
  }

  else {

    appendf(
      row,
      sizeof(row),
      &n,
      ","
    );
  }


  if (bootId) {

    appendf(
      row,
      sizeof(row),
      &n,

      "%lu",

      (unsigned long)bootId
    );
  }


  logFile.println(row);


  static uint32_t lastFlush = 0;


  if (
    millis() -
    lastFlush >
    2000
  ) {

    logFile.flush();

    lastFlush =
      millis();
  }

#endif
}


// ============================================================================
// LOG IMU ROW
// ============================================================================

void imuLogRow(
  const ImuRaw &s
) {

#if SD_LOGGING

  if (
    !imuLogOk ||
    !imuFile
  )
    return;


  float u[6];


  imuUnits(
    s,
    u
  );


  char row[128];


  size_t n = 0;


  appendf(
    row,
    sizeof(row),
    &n,

    "%lu,"
    "%.3f,%.3f,%.3f,"
    "%.3f,%.3f,%.3f,",

    (unsigned long)(
      (uint32_t)(
        s.us /
        1000
      )
    ),

    u[0],
    u[1],
    u[2],

    u[3],
    u[4],
    u[5]
  );


  int64_t utcMs;


  if (
    utcMsAt(
      s.us,
      &utcMs
    )
  ) {

    appendf(
      row,
      sizeof(row),
      &n,

      "%lld,",

      (long long)utcMs
    );
  }

  else {

    appendf(
      row,
      sizeof(row),
      &n,
      ","
    );
  }


  if (bootId) {

    appendf(
      row,
      sizeof(row),
      &n,

      "%lu",

      (unsigned long)bootId
    );
  }


  imuFile.println(row);


  static uint32_t lastFlush = 0;


  if (
    millis() -
    lastFlush >
    2000
  ) {

    imuFile.flush();

    lastFlush =
      millis();
  }

#endif
}


// ============================================================================
// IMU DRAIN
// ============================================================================

void imuDrain() {

  if (!imuQ)
    return;


  ImuRaw s;


  while (
    xQueueReceive(
      imuQ,
      &s,
      0
    ) == pdTRUE
  ) {

    imuLatest =
      s;


    imuHaveLatest =
      true;


    imuCount++;


    imuLogRow(s);
  }
}


// ============================================================================
// OPEN SD LOGS
// ============================================================================

void openLog() {

#if SD_LOGGING

  if (
    !SD.begin(
      PIN_SD_CS
    )
  ) {

    Serial.println(
      F("SD: no card")
    );


    return;
  }


  SD.mkdir(
    "/Elec_Test"
  );


  char path[32];

  char ipath[32];


  for (
    int i = 0;
    i < 100;
    i++
  ) {

    snprintf(
      path,
      sizeof(path),
      "/Elec_Test/data_%02d.csv",
      i
    );


    snprintf(
      ipath,
      sizeof(ipath),
      "/Elec_Test/imu_%02d.csv",
      i
    );


    if (
      !SD.exists(path) &&
      !SD.exists(ipath)
    )
      break;
  }


  logFile =
    SD.open(
      path,
      FILE_WRITE
    );


  if (!logFile) {

    Serial.println(
      F("SD: could not open data log")
    );


    return;
  }


  sdOk = true;


  char board[16];


  boardIdStr(
    board,
    sizeof(board)
  );


  logFile.printf(
    "# utsm_wifi_telemetry 1.3 · board=%s",

    board
  );


  if (bootId) {

    logFile.printf(
      " · boot_id=%lu",

      (unsigned long)bootId
    );
  }

  else {

    logFile.print(
      " · boot_id=unknown"
    );
  }


  logFile.printf(
    " · sample_hz=%d"
    " · imu_hz=%d"
    " · imu_file=%s"
    " · accel_range=4g"
    " · gyro_range=250dps"
    " · cal_applied=0"
    " · accel_offset_mps2=%.4f,%.4f,%.4f"
    " · utc=gps_pps(offset_s=%d)"
    " · jm=%s"
    " · jm_current=%d",

    SAMPLE_HZ,

    IMU_HZ,

    ipath + 11,

    accOff[0],

    accOff[1],

    accOff[2],

    UTC_SECOND_OFFSET,

    jmOk
      ? "ina226"
      : "none",

    JM_HAVE_CURRENT
  );


#if JM_HAVE_CURRENT

  logFile.printf(
    " · r_shunt_ohm=%.5f",
    JM_R_SHUNT
  );

#endif


  logFile.printf(
    " · plan=%s",
    PLAN_ID_STR
  );


  logFile.print('\n');


  logFile.println(
    F(
      "Millis,"
      "AccelX_mps2,"
      "AccelY_mps2,"
      "AccelZ_mps2,"
      "GyroX_dps,"
      "GyroY_dps,"
      "GyroZ_dps,"
      "Latitude,"
      "Longitude,"
      "Altitude_m,"
      "Speed_kmh,"
      "Satellites,"
      "Volts_V,"
      "Amps_A,"
      "UtcMs,"
      "BootId"
    )
  );


  logFile.flush();


  Serial.printf(
    "SD: logging to %s\n",
    path
  );


  imuFile =
    SD.open(
      ipath,
      FILE_WRITE
    );


  if (!imuFile) {

    Serial.println(
      F("SD: could not open IMU log")
    );


    return;
  }


  imuLogOk = true;


  imuFile.printf(
    "# utsm_wifi_telemetry 1.3 imu · board=%s",

    board
  );


  if (bootId) {

    imuFile.printf(
      " · boot_id=%lu",

      (unsigned long)bootId
    );
  }

  else {

    imuFile.print(
      " · boot_id=unknown"
    );
  }


  imuFile.printf(
    " · imu_hz=%d"
    " · accel_range=4g"
    " · gyro_range=250dps"
    " · cal_applied=0"
    " · accel_offset_mps2=%.4f,%.4f,%.4f"
    " · stamp=read_start"
    " · utc=gps_pps(offset_s=%d)"
    " · plan=%s\n",

    IMU_HZ,

    accOff[0],

    accOff[1],

    accOff[2],

    UTC_SECOND_OFFSET,

    PLAN_ID_STR
  );


  imuFile.println(
    F(
      "Millis,"
      "AccelX_mps2,"
      "AccelY_mps2,"
      "AccelZ_mps2,"
      "GyroX_dps,"
      "GyroY_dps,"
      "GyroZ_dps,"
      "UtcMs,"
      "BootId"
    )
  );


  imuFile.flush();


  Serial.printf(
    "SD: IMU log %s\n",
    ipath
  );

#endif
}


// ============================================================================
// FLIP TEST
// ============================================================================

void flipTest() {

  if (!imuOk) {

    Serial.println(
      F("no IMU")
    );


    return;
  }


  imuPaused = true;


  float up[3];

  float down[3];


  auto grab = [&](float *o) {

    double s[3] = {
      0,
      0,
      0
    };


    for (
      int i = 0;
      i < 200;
      i++
    ) {

      sensors_event_t a;

      sensors_event_t g;

      sensors_event_t t;


      if (i2cLock()) {

        imu.getEvent(
          &a,
          &g,
          &t
        );


        i2cUnlock();
      }


      if (i >= 5) {

        s[0] +=
          a.acceleration.x;


        s[1] +=
          a.acceleration.y;


        s[2] +=
          a.acceleration.z;
      }


      delay(10);
    }


    for (
      int k = 0;
      k < 3;
      k++
    ) {

      o[k] =
        s[k] /
        195.0;
    }
  };


  Serial.println(
    F("Flip test: hold UPRIGHT and press Enter.")
  );


  while (
    Serial.read() != '\n'
  ) {

    delay(10);
  }


  grab(up);


  Serial.println(
    F("Invert board and press Enter.")
  );


  while (
    Serial.read() != '\n'
  ) {

    delay(10);
  }


  grab(down);


  Serial.println(
    F("Paste these into accOff[]:")
  );


  for (
    int k = 0;
    k < 3;
    k++
  ) {

    Serial.printf(
      "ACC_OFF_%c = %.4f   gravity %.4f\n",

      'X' + k,

      (
        up[k] +
        down[k]
      ) /
      2.0f,

      (
        up[k] -
        down[k]
      ) /
      2.0f
    );
  }


  imuPaused = false;
}


// ============================================================================
// JOULEMETER DEBUG
// ============================================================================

void jmDump() {

  Serial.printf(
    "JM: %s misses=%lu\n",

    jmOk
      ? "ok"
      : "absent",

    (unsigned long)jmMisses
  );


  jmRead();


  if (jm.haveV) {

    Serial.printf(
      "V = %.3f V\n",
      jm.v
    );
  }

  else {

    Serial.println(
      F("V = no reading")
    );
  }


#if JM_HAVE_CURRENT

  if (jm.haveI) {

    Serial.printf(
      "I = %.3f A\n",
      jm.i
    );
  }

  else {

    Serial.println(
      F("I = no reading")
    );
  }

#else

  Serial.println(
    F("Current measurement disabled")
  );

#endif
}


// ============================================================================
// SETUP
// ============================================================================

void setup() {

  Serial.begin(
    115200
  );


  delay(200);


  Serial.println();


  Serial.println(
    F("UTSM vehicle board")
  );


  // --------------------------------------------------------------------------
  // FreeRTOS resources
  // --------------------------------------------------------------------------

  i2cMux =
    xSemaphoreCreateMutex();


  imuQ =
    xQueueCreate(
      IMU_QUEUE_LEN,
      sizeof(ImuRaw)
    );


  // --------------------------------------------------------------------------
  // SPI
  // --------------------------------------------------------------------------

  pinMode(
    PIN_LORA_CS,
    OUTPUT
  );


  digitalWrite(
    PIN_LORA_CS,
    HIGH
  );


  pinMode(
    PIN_SD_CS,
    OUTPUT
  );


  digitalWrite(
    PIN_SD_CS,
    HIGH
  );


  SPI.begin(
    PIN_SPI_SCK,
    PIN_SPI_MISO,
    PIN_SPI_MOSI
  );


  // --------------------------------------------------------------------------
  // I2C
  // --------------------------------------------------------------------------

  Wire.begin(
    PIN_I2C_SDA,
    PIN_I2C_SCL
  );


  Wire.setClock(
    I2C_HZ
  );


  Wire.setTimeOut(
    I2C_TIMEOUT_MS
  );


  // --------------------------------------------------------------------------
  // IMU
  // --------------------------------------------------------------------------

  if (
    imu.begin_I2C()
  ) {

    imu.setAccelRange(
      LSM6DS_ACCEL_RANGE_4_G
    );


    imu.setGyroRange(
      LSM6DS_GYRO_RANGE_250_DPS
    );


    imu.setAccelDataRate(
      IMU_ODR
    );


    imu.setGyroDataRate(
      IMU_ODR
    );


    imuOk = true;


    Serial.println(
      F("IMU: LSM6DSOX OK")
    );
  }

  else {

    Serial.println(
      F("IMU: not found")
    );
  }


  Wire.setClock(
    I2C_HZ
  );


  Wire.setTimeOut(
    I2C_TIMEOUT_MS
  );


  if (
    imuOk &&
    !imuSelfCheck()
  ) {

    imuOk = false;
  }


  // --------------------------------------------------------------------------
  // Joulemeter
  // --------------------------------------------------------------------------

#if HAVE_JOULEMETER

  jmOk =
    jmBegin();


  if (jmOk) {

    Serial.printf(
      "JM: INA226 found at 0x%02X\n",
      INA226_ADDR
    );
  }

  else {

    Serial.println(
      F("JM: INA226 not found")
    );
  }

#endif


  // --------------------------------------------------------------------------
  // GPS
  // --------------------------------------------------------------------------

  GPSSerial.begin(
    9600,
    SERIAL_8N1,
    PIN_GPS_RX,
    PIN_GPS_TX
  );


  // --------------------------------------------------------------------------
  // PPS
  // --------------------------------------------------------------------------

  pinMode(
    PIN_PPS,
    INPUT_PULLDOWN
  );


  attachInterrupt(
    digitalPinToInterrupt(
      PIN_PPS
    ),
    ppsIsr,
    RISING
  );


  // --------------------------------------------------------------------------
  // Boot ID
  // --------------------------------------------------------------------------

  bootId =
    bootIdAdvance();


  if (bootId) {

    Serial.printf(
      "BOOT: id %lu\n",
      (unsigned long)bootId
    );
  }

  else {

    Serial.println(
      F("BOOT: counter unavailable")
    );
  }


  // --------------------------------------------------------------------------
  // Config + SD
  // --------------------------------------------------------------------------

  cfgLoad();


  openLog();


  // --------------------------------------------------------------------------
  // LoRa
  // --------------------------------------------------------------------------

  loraOk =
    loraBegin();


  if (!loraOk) {

    Serial.println(
      F("LoRa disabled")
    );
  }


  // --------------------------------------------------------------------------
  // WiFi AP
  // --------------------------------------------------------------------------

  WiFi.mode(
    WIFI_AP
  );


  WiFi.softAP(
    AP_SSID,

    strlen(AP_PASS) >= 8
      ? AP_PASS
      : NULL
  );


  IPAddress ip =
    WiFi.softAPIP();


#if CAPTIVE_PORTAL

  dns.start(
    53,
    "*",
    ip
  );

#endif


  server.begin();


  server.setNoDelay(
    true
  );


  Serial.printf(
    "AP: %s\n",
    AP_SSID
  );


  Serial.printf(
    "Open: http://%s/\n",
    ip.toString().c_str()
  );


#ifdef HAVE_PLAN

  Serial.printf(
    "PLAN: %s\n",
    PLAN_ID
  );

#else

  Serial.println(
    F("PLAN: none")
  );

#endif


  Serial.println(
    F(
      "Serial commands: "
      "c=flip test, "
      "s=I2C scan, "
      "j=joulemeter, "
      "u=UTC check"
    )
  );


  // --------------------------------------------------------------------------
  // IMU task
  // --------------------------------------------------------------------------

  if (imuOk) {

    xTaskCreatePinnedToCore(
      imuTask,
      "imu",
      6144,
      NULL,
      2,
      NULL,
      1
    );


    Serial.printf(
      "IMU: %d Hz task running\n",
      IMU_HZ
    );
  }
}


// ============================================================================
// LOOP
// ============================================================================

void loop() {

  loraPoll();

#if CAPTIVE_PORTAL

  dns.processNextRequest();

#endif


  // HTTP / dashboard

  handleHttp();


  // --------------------------------------------------------------------------
  // GPS
  // --------------------------------------------------------------------------

  while (
    GPSSerial.available()
  ) {

    gps.encode(
      GPSSerial.read()
    );
  }


  // UTC lock

  utcPoll();


  // IMU queue

  imuDrain();


  // --------------------------------------------------------------------------
  // Serial debug commands
  // --------------------------------------------------------------------------

  if (
    Serial.available()
  ) {

    int ch =
      Serial.peek();


    if (
      ch == 'c'
    ) {

      Serial.read();

      flipTest();
    }

    else if (
      ch == 's'
    ) {

      Serial.read();

      i2cScan();
    }

    else if (
      ch == 'j'
    ) {

      Serial.read();

      jmDump();
    }

    else if (
      ch == 'u'
    ) {

      Serial.read();

      utcDump();
    }

    else {

      Serial.read();
    }
  }


  // --------------------------------------------------------------------------
  // Main 10 Hz telemetry
  // --------------------------------------------------------------------------

  uint32_t now =
    millis();


  if (
    now -
    lastSample >=
    1000 /
    SAMPLE_HZ
  ) {

    lastSample =
      now;


    readImu();


    jmRead();


    // Same sequence number used by
    // WiFi + LoRa

    uint16_t packetSeq =
      seq++;


    // WiFi telemetry

    char buf[384];


    buildPacket(
      buf,
      sizeof(buf),
      packetSeq
    );


    sseBroadcast(
      buf
    );


    // LoRa telemetry

    loraSendTelemetry(
      packetSeq
    );


    // SD

    logRow();
  }
}