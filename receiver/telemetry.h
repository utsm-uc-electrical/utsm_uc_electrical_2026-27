#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>

// Wire format v1 from transmit.ino. ESP32 sends little-endian integers and
// IEEE-754 32-bit floats. Keep field order, packing and flags identical.
enum : uint16_t {
  LORA_F_IMU = 1u << 0,
  LORA_F_GPS_POS = 1u << 1,
  LORA_F_ALT = 1u << 2,
  LORA_F_SPEED = 1u << 3,
  LORA_F_SATS = 1u << 4,
  LORA_F_HDOP = 1u << 5,
  LORA_F_VOLTS = 1u << 6,
  LORA_F_AMPS = 1u << 7,
  LORA_F_RUN = 1u << 8,
  LORA_F_UTC = 1u << 9
};

struct __attribute__((packed)) LoraTelemetry {
  uint16_t magic;
  uint8_t version;
  uint16_t flags;
  uint16_t seq;
  uint32_t millis32;
  float ax, ay, az;
  float gx, gy, gz;
  int32_t latE7, lonE7;
  float altM, speedKmh;
  uint8_t sats;
  float hdop;
  float volts, amps;
  uint32_t armT, bootId;
  int64_t utcMs;
};

static_assert(sizeof(float) == 4, "Protocol requires 32-bit floats");
static_assert(sizeof(LoraTelemetry) == 80, "Transmitter packet layout changed");
static_assert(offsetof(LoraTelemetry, utcMs) == 72, "Incorrect packet packing");

inline bool decodeTelemetry(const uint8_t *bytes, size_t size, LoraTelemetry &p) {
  if (size != sizeof(p)) return false;
  memcpy(&p, bytes, sizeof(p));
  if (p.magic != 0x5554 || p.version != 1 || (p.flags & ~0x03FFu)) return false;
  // Unavailable fields are ignored; their zero-filled bytes are not readings.
  if ((p.flags & LORA_F_IMU) &&
      (!isfinite(p.ax) || !isfinite(p.ay) || !isfinite(p.az) ||
       !isfinite(p.gx) || !isfinite(p.gy) || !isfinite(p.gz))) return false;
  if ((p.flags & LORA_F_GPS_POS) &&
      (p.latE7 < -900000000 || p.latE7 > 900000000 ||
       p.lonE7 < -1800000000 || p.lonE7 > 1800000000)) return false;
  if ((p.flags & LORA_F_ALT) && !isfinite(p.altM)) return false;
  if ((p.flags & LORA_F_SPEED) && !isfinite(p.speedKmh)) return false;
  if ((p.flags & LORA_F_HDOP) && !isfinite(p.hdop)) return false;
  if ((p.flags & LORA_F_VOLTS) && !isfinite(p.volts)) return false;
  if ((p.flags & LORA_F_AMPS) && !isfinite(p.amps)) return false;
  return true;
}

// Bounded formatter: never emit a truncated JSON record.
class TelemetryJson {
 public:
  TelemetryJson(char *buffer, size_t capacity) : buf(buffer), cap(capacity) {
    if (cap) buf[0] = '\0';
  }
  void append(const char *fmt, ...) {
    if (!ok || used >= cap) { ok = false; return; }
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf + used, cap - used, fmt, args);
    va_end(args);
    if (n < 0 || (size_t)n >= cap - used) { ok = false; return; }
    used += (size_t)n;
  }
  bool good() const { return ok; }
 private:
  char *buf;
  size_t cap, used = 0;
  bool ok = true;
};

inline bool telemetryJson(const LoraTelemetry &p, uint32_t rxMs, int rssi,
                          float snr, char *out, size_t capacity) {
  TelemetryJson j(out, capacity);
  j.append("{\"type\":\"telemetry\",\"t\":%lu,\"seq\":%u,\"boot\":%lu,"
           "\"flags\":%u,\"rxMs\":%lu,\"loraRssi\":%d,\"loraSnr\":%.2f",
           (unsigned long)p.millis32, (unsigned)p.seq, (unsigned long)p.bootId,
           (unsigned)p.flags, (unsigned long)rxMs, rssi, (double)snr);
  if (p.flags & LORA_F_IMU)
    j.append(",\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f",
             (double)p.ax, (double)p.ay, (double)p.az,
             (double)p.gx, (double)p.gy, (double)p.gz);
  if (p.flags & LORA_F_GPS_POS)
    j.append(",\"lat\":%.7f,\"lon\":%.7f", p.latE7 / 1e7, p.lonE7 / 1e7);
  if (p.flags & LORA_F_ALT) j.append(",\"alt\":%.1f", (double)p.altM);
  if (p.flags & LORA_F_SPEED) j.append(",\"spd\":%.2f", (double)p.speedKmh);
  if (p.flags & LORA_F_SATS) j.append(",\"sats\":%u", (unsigned)p.sats);
  if (p.flags & LORA_F_HDOP) j.append(",\"hdop\":%.1f", (double)p.hdop);
  if (p.flags & LORA_F_VOLTS) j.append(",\"v\":%.3f", (double)p.volts);
  if (p.flags & LORA_F_AMPS) j.append(",\"i\":%.3f", (double)p.amps);
  if ((p.flags & (LORA_F_VOLTS | LORA_F_AMPS)) == (LORA_F_VOLTS | LORA_F_AMPS))
    j.append(",\"powerW\":%.3f", (double)p.volts * (double)p.amps);
  j.append(",\"run\":%d", (p.flags & LORA_F_RUN) ? 1 : 0);
  if (p.flags & LORA_F_RUN) j.append(",\"armT\":%lu", (unsigned long)p.armT);
  if (p.flags & LORA_F_UTC) j.append(",\"utcMs\":%lld", (long long)p.utcMs);
  j.append("}");
  return j.good();
}
