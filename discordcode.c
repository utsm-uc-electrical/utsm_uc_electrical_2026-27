/* ============================================================================
   UTSM vehicle board — direct phone telemetry over Wi-Fi
   ---------------------------------------------------------------------------
   Turns the Rev A board into its own access point and streams raw samples to
   any phone that joins it. No pit laptop, no LoRa link, no app install: the
   phone joins "UTSM-Telemetry", opens http://192.168.4.1 and gets the whole
   dashboard served out of this sketch's flash.

   Design notes, because two of these are load-bearing:

   1. NO EXTERNAL NETWORKING LIBRARIES. The HTTP server below is about 120
      lines of raw WiFiServer, and the live feed is Server-Sent Events — a
      plain HTTP response that is never closed. Telemetry only flows one way,
      so a WebSocket buys nothing here, and every ESPAsyncWebServer fork is one
      more thing that can fail to compile the night before a track day. As a
      bonus, browsers reconnect an EventSource by themselves, which is exactly
      what you want when a phone strapped to a tiller drops off the AP for a
      second at the far end of the circuit.

   2. A CHANNEL THE BOARD IS NOT MEASURING IS OMITTED FROM THE PACKET. It is
      never sent as 0. This is the whole reason the SD analyzer needed fixing
      in September: a "0 W" reading that is indistinguishable from "no
      joulemeter connected" is worse than no reading at all. Every optional
      field below is inside an `if`. Please keep it that way.

   The SD logger from utsm_datalogger is kept and unchanged in format, so the
   same flash gives you live telemetry on the phone AND log files the existing
   SD analyzer already opens.

   Board: ESP32-DEVKIT-V1 (ESP32-D0WD-V3). Arduino-ESP32 2.0.x / espressif32
   6.5.x — core 3.x changed APIs this sketch has not been checked against.
   ========================================================================== */

#include <WiFi.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_LSM6DSOX.h>
#include <TinyGPS++.h>

#include "webapp.h"     // INDEX_HTML_GZ / INDEX_HTML_GZ_LEN, built by tools/

/* ------------------------------------------------------------------ config */
#define AP_SSID      "UTSM-Telemetry"
#define AP_PASS      "utsm2027"      // >= 8 chars, or "" for an open network
#define SAMPLE_HZ    10
#define MAX_SSE      4               // phones that can watch at once
#define SD_LOGGING   1
#define HAVE_JOULEMETER 0            // 1 once v/i sensing is fitted — see §JM

/* Captive portal. On joining a network, phones probe for internet.

   At 1, every probe is redirected here, so the "sign in to network" sheet opens
   the dashboard by itself. That sounds like the friendlier option and it is not:
   confirmed on an iPhone 9 Sep 2026, once iOS flags a network as captive it
   stops routing ordinary traffic over it until the portal is satisfied — and
   dismissing the sheet does not satisfy it. Safari's request for 192.168.4.1
   then goes out over cellular, where that address means nothing, and the board
   never hears it. The sheet is also a cut-down browser that cannot Add to Home
   Screen, so the auto-popup costs you the thing you actually want.

   At 0 the board answers those probes the way a working internet connection
   would. iOS never flags the network, Safari routes over Wi-Fi normally, and
   Add to Home Screen works. The only loss is the automatic popup, so a teammate
   joining has to be told to open http://192.168.4.1/ themselves.

   0 is the default for that reason. If you switch back to 1, make the phone
   forget the network first — iOS caches its captive verdict per network. */
#define CAPTIVE_PORTAL 0

/* ------------------------------------------------------------------- pins */
// GPS: the Rev A schematic does not cross the UART over; the board is
// hot-wired. The ESP32 GPIO matrix can do the same job in software, which is
// what these arguments are — RX on 16, TX on 17.
static const int PIN_GPS_RX = 16, PIN_GPS_TX = 17;
static const int PIN_LORA_CS = 5;    // parked high: keeps the SX1262 off the SPI bus
static const int PIN_SD_CS   = 2;    // UNVERIFIED — read off the schematic's D2/CS0 net.
                                     // GPIO2 is also a strapping pin: if a card socket
                                     // holds it high at reset, uploads fail intermittently.

/* isnan() is a macro in C but not in C++, and which one a core gives you
   depends on whether <math.h> or <cmath> won. This is unambiguous everywhere. */
static inline bool isNan(float v) { return v != v; }

/* forward declarations — the Arduino builder generates these, a plain g++
   compile check does not */
int  sseCount();
void cfgSave();
String helloJson();

/* ------------------------------------------------------------------ state */
Adafruit_LSM6DSOX imu;
TinyGPSPlus       gps;
HardwareSerial    GPSSerial(2);
WiFiServer        server(80);
DNSServer         dns;
Preferences       prefs;
File              logFile;

WiFiClient sse[MAX_SSE];
bool       imuOk = false, sdOk = false;
uint16_t   seq = 0;
uint32_t   lastSample = 0;
bool       runArmed = false;
uint32_t   runArmT = 0;

// accelerometer offsets from the flip test; logged raw, corrected in analysis
float accOff[3] = { 0, 0, 0 };

struct Cfg { float bandLo, bandHi, budgetWh, eventKm, leaveM, returnM, minLapS; };
Cfg cfg = { NAN, NAN, NAN, 16.09f, 40, 25, 20 };

/* ============================================================================
   Config persistence. The vehicle owns the configuration, not the phone, so a
   second phone joining mid-session sees the same driving band and lap gate.
   NAN is the on-the-wire spelling of "not set" — a band or a budget that has
   never been decided must not arrive at the dashboard as a number.
   ========================================================================== */
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

/* ============================================================================
   HTTP. Hand-rolled and deliberately small: a request line, a couple of
   headers, three routes and a catch-all redirect for the captive portal.
   ========================================================================== */

void sendGzipPage(WiFiClient &c) {
  c.print(F("HTTP/1.1 200 OK\r\n"
            "Content-Type: text/html; charset=utf-8\r\n"
            "Content-Encoding: gzip\r\n"
            "Cache-Control: no-cache\r\n"
            "Connection: close\r\nContent-Length: "));
  c.print(INDEX_HTML_GZ_LEN);
  c.print(F("\r\n\r\n"));
  // Chunked so a slow phone cannot stall the sample loop behind one huge write.
  const size_t CH = 1024;
  for (size_t i = 0; i < INDEX_HTML_GZ_LEN && c.connected(); i += CH) {
    size_t n = min(CH, (size_t)(INDEX_HTML_GZ_LEN - i));
    c.write(INDEX_HTML_GZ + i, n);
    delay(0);                            // let WiFi/TCP run
  }
}

void sendText(WiFiClient &c, const char *status, const char *type, const String &body) {
  c.printf("HTTP/1.1 %s\r\nContent-Type: %s\r\nCache-Control: no-cache\r\n"
           "Connection: close\r\nContent-Length: %u\r\n\r\n",
           status, type, (unsigned)body.length());
  c.print(body);
}

/* One-time frame telling the phone who it is talking to and what the vehicle's
   stored configuration is. Sent the moment an EventSource attaches. */
String helloJson() {
  String s = "{\"hello\":1,\"name\":\"" AP_SSID "\",\"fw\":\"utsm_wifi_telemetry 1.0\"";
  s += ",\"chip\":\"ESP32-D0WD-V3\"";
  s += ",\"hz\":" + String(SAMPLE_HZ);
  s += ",\"heap\":" + String((unsigned)ESP.getFreeHeap());
  s += ",\"clients\":" + String(sseCount());
  s += sdOk ? ",\"sd\":\"logging\"" : ",\"sd\":null";
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

/* Query-string helper: returns "" when the key is absent, so a /cmd that
   mentions only some settings leaves the rest alone. */
String qparam(const String &q, const String &key) {
  int i = q.indexOf(key + "=");
  while (i > 0 && q[i - 1] != '&' && q[i - 1] != '?') i = q.indexOf(key + "=", i + 1);
  if (i < 0) return "";
  int s = i + key.length() + 1, e = q.indexOf('&', s);
  return q.substring(s, e < 0 ? q.length() : e);
}
float qfloat(const String &q, const String &key, float dflt) {
  String v = qparam(q, key);
  if (v.length() == 0) return dflt;          // key absent — keep what we had
  if (v == "null" || v == "NaN") return NAN; // explicitly cleared
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

  // Read the request line, then drain the headers. Bounded so a half-open
  // connection cannot hold the sample loop hostage.
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

  if (path == "/events") { addSse(c); return; }        // kept open, do not stop()
  if (path == "/cmd")    { handleCmd(c, query); c.stop(); return; }
  if (path == "/" || path == "/index.html") { sendGzipPage(c); c.stop(); return; }

#if CAPTIVE_PORTAL
  /* Everything else — including the probes iOS, Android and Windows fire on
     joining — is redirected to the app. That is what makes the "sign in to
     network" sheet open the dashboard by itself. */
  c.print(F("HTTP/1.1 302 Found\r\nLocation: http://192.168.4.1/\r\n"
            "Connection: close\r\nContent-Length: 0\r\n\r\n"));
#else
  /* Answer the probes the way a working internet connection would, so the phone
     stops treating this as a captive network: iOS wants exactly this body,
     Android wants a bare 204. Everything else gets a plain 404. */
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

/* ============================================================================
   Sampling
   ========================================================================== */

/* RSSI of the phone as the AP sees it — the only honest link-quality number on
   a direct Wi-Fi path, and the closest equivalent to the LoRa receiver's rssi. */
int staRssi() {
  wifi_sta_list_t sta;
  if (esp_wifi_ap_get_sta_list(&sta) != ESP_OK || sta.num == 0) return 0;
  return sta.sta[0].rssi;
}

/* The IMU is read ONCE per sample into here, so the packet and the SD row are
   the same measurement rather than two I2C reads a millisecond apart. */
struct Sample { bool have; float ax, ay, az, gx, gy, gz; } smp;

void readImu() {
  smp.have = false;
  if (!imuOk) return;
  sensors_event_t a, g, t;
  imu.getEvent(&a, &g, &t);
  smp.ax = a.acceleration.x; smp.ay = a.acceleration.y; smp.az = a.acceleration.z;
  smp.gx = g.gyro.x * RAD_TO_DEG; smp.gy = g.gyro.y * RAD_TO_DEG; smp.gz = g.gyro.z * RAD_TO_DEG;
  smp.have = true;
}

void buildPacket(char *buf, size_t sz) {
  size_t n = 0;
  n += snprintf(buf + n, sz - n, "{\"t\":%lu,\"seq\":%u", (unsigned long)millis(), (unsigned)seq++);

  if (smp.have)
    n += snprintf(buf + n, sz - n,
      ",\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f",
      smp.ax, smp.ay, smp.az, smp.gx, smp.gy, smp.gz);

  /* Each GPS field stands on its own validity. A module that has a fix but no
     speed solution sends lat/lon and no spd, rather than lat/lon and a zero. */
  if (gps.location.isValid() && gps.location.age() < 3000)
    n += snprintf(buf + n, sz - n, ",\"lat\":%.6f,\"lon\":%.6f",
                  gps.location.lat(), gps.location.lng());
  if (gps.altitude.isValid()  && gps.altitude.age()  < 3000)
    n += snprintf(buf + n, sz - n, ",\"alt\":%.1f", gps.altitude.meters());
  if (gps.speed.isValid()     && gps.speed.age()     < 3000)
    n += snprintf(buf + n, sz - n, ",\"spd\":%.2f", gps.speed.kmph());   // km/h — see PROTOCOL.md
  if (gps.satellites.isValid())
    n += snprintf(buf + n, sz - n, ",\"sats\":%d", (int)gps.satellites.value());
  if (gps.hdop.isValid())
    n += snprintf(buf + n, sz - n, ",\"hdop\":%.1f", gps.hdop.hdop());

#if HAVE_JOULEMETER
  /* §JM — the ONLY place v and i are emitted. Read the real sensor here and
     emit both keys, or neither. Do not add an `else` that sends zeros: the
     dashboard's energy cards are designed to disappear when these are absent,
     and a zero would be displayed as a measurement. */
  float volts = readVolts(), amps = readAmps();
  if (!isnan(volts) && !isnan(amps))
    n += snprintf(buf + n, sz - n, ",\"v\":%.2f,\"i\":%.3f", volts, amps);
#endif

  n += snprintf(buf + n, sz - n, ",\"run\":%d", runArmed ? 1 : 0);
  if (runArmed) n += snprintf(buf + n, sz - n, ",\"armT\":%lu", (unsigned long)runArmT);
  int r = staRssi();
  if (r) n += snprintf(buf + n, sz - n, ",\"rssi\":%d", r);
  snprintf(buf + n, sz - n, "}");
}

/* SD row in the utsm_datalogger 12-column format, so the existing SD analyzer
   opens these files unchanged. One deliberate difference: a sensor that is not
   present writes an EMPTY field, not a zero. Empty is how CSV spells "no
   measurement" and how pandas reads NaN; a zero would be a fabricated reading,
   which is the same bug this whole project just finished removing. Latitude and
   longitude keep the shared %.6f so a pre-fix row reads 0.000000, never
   0000000 — the strict-schema break found in the August bench QA. */
void logRow() {
#if SD_LOGGING
  if (!sdOk || !logFile) return;
  char row[192];
  size_t n = snprintf(row, sizeof row, "%lu,", (unsigned long)millis());
  if (smp.have)
    n += snprintf(row + n, sizeof row - n, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,",
                  smp.ax, smp.ay, smp.az, smp.gx, smp.gy, smp.gz);
  else
    n += snprintf(row + n, sizeof row - n, ",,,,,,");
  if (gps.location.isValid())
    n += snprintf(row + n, sizeof row - n, "%.6f,%.6f,", gps.location.lat(), gps.location.lng());
  else
    n += snprintf(row + n, sizeof row - n, "0.000000,0.000000,");
  if (gps.altitude.isValid()) n += snprintf(row + n, sizeof row - n, "%.1f,", gps.altitude.meters());
  else                        n += snprintf(row + n, sizeof row - n, ",");
  if (gps.speed.isValid())    n += snprintf(row + n, sizeof row - n, "%.2f,", gps.speed.kmph());
  else                        n += snprintf(row + n, sizeof row - n, ",");
  if (gps.satellites.isValid()) snprintf(row + n, sizeof row - n, "%d", (int)gps.satellites.value());
  logFile.println(row);
  static uint32_t lastFlush = 0;
  if (millis() - lastFlush > 2000) { logFile.flush(); lastFlush = millis(); }
#endif
}

void openLog() {
#if SD_LOGGING
  if (!SD.begin(PIN_SD_CS)) { Serial.println(F("SD: no card (live telemetry still runs)")); return; }
  SD.mkdir("/Elec_Test");
  char path[32];
  for (int i = 0; i < 100; i++) {
    snprintf(path, sizeof path, "/Elec_Test/data_%02d.csv", i);
    if (!SD.exists(path)) break;
  }
  logFile = SD.open(path, FILE_WRITE);
  if (!logFile) { Serial.println(F("SD: could not open a log file")); return; }
  sdOk = true;
  logFile.printf("# utsm_wifi_telemetry · sample_hz=%d · accel_range=4g · gyro_range=250dps"
                 " · cal_applied=0 · accel_offset_mps2=%.4f,%.4f,%.4f\n",
                 SAMPLE_HZ, accOff[0], accOff[1], accOff[2]);
  logFile.println(F("Millis,AccelX_mps2,AccelY_mps2,AccelZ_mps2,GyroX_dps,GyroY_dps,GyroZ_dps,"
                    "Latitude,Longitude,Altitude_m,Speed_kmh,Satellites"));
  logFile.flush();
  Serial.printf("SD: logging to %s\n", path);
#endif
}

/* Flip-test calibration, carried over from utsm_datalogger: capture upright,
   then inverted. A single orientation cannot tell a real mounting tilt from a
   pair of zero-g offsets — only the flip resolves it. */
void flipTest() {
  if (!imuOk) { Serial.println(F("no IMU")); return; }
  float up[3], down[3];
  auto grab = [&](float *o) {
    double s[3] = { 0, 0, 0 };
    for (int i = 0; i < 200; i++) {
      sensors_event_t a, g, t; imu.getEvent(&a, &g, &t);
      if (i >= 5) { s[0] += a.acceleration.x; s[1] += a.acceleration.y; s[2] += a.acceleration.z; }
      delay(10);
    }
    for (int k = 0; k < 3; k++) o[k] = s[k] / 195.0;      // first 5 samples discarded
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
}

/* ============================================================================
   setup / loop
   ========================================================================== */

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\nUTSM vehicle board — direct phone telemetry"));

  // Park the radio: LORA_CS high keeps the SX1262 off the shared SPI bus so the
  // SD card has it to itself. Never power the SX1262 without an antenna.
  pinMode(PIN_LORA_CS, OUTPUT); digitalWrite(PIN_LORA_CS, HIGH);

  Wire.begin();
  if (imu.begin_I2C()) {
    imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
    imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
    imuOk = true;
    Serial.println(F("IMU: LSM6DSOX ok"));
  } else {
    Serial.println(F("IMU: not found — accel/gyro fields will be OMITTED, not zeroed"));
  }

  GPSSerial.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

  cfgLoad();
  openLog();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, strlen(AP_PASS) >= 8 ? AP_PASS : NULL);
  IPAddress ip = WiFi.softAPIP();
#if CAPTIVE_PORTAL
  dns.start(53, "*", ip);            // every lookup lands on us, so probes hit /
#endif
  server.begin();
  server.setNoDelay(true);

  Serial.printf("AP  : %s  (password %s)\n", AP_SSID, strlen(AP_PASS) >= 8 ? AP_PASS : "<open>");
  Serial.printf("Open: http://%s/\n", ip.toString().c_str());
  Serial.println(F("Serial 'c' runs the accelerometer flip test."));
}

void loop() {
#if CAPTIVE_PORTAL
  dns.processNextRequest();
#endif
  handleHttp();

  while (GPSSerial.available()) gps.encode(GPSSerial.read());

  if (Serial.available() && Serial.peek() == 'c') { Serial.read(); flipTest(); }

  uint32_t now = millis();
  if (now - lastSample >= 1000 / SAMPLE_HZ) {
    lastSample = now;
    readImu();
    char buf[384];
    buildPacket(buf, sizeof buf);
    sseBroadcast(buf);
    logRow();
  }
}