"""Exercise receiver.ino with a simulated radio/serial port, not ESP32 hardware."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]

MOCK = r'''
#pragma once
#include <cassert>
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <string>
#define HIGH 1
#define INPUT 0
#define OUTPUT 1
#define RADIOLIB_NC 0xffffffffu
#define RADIOLIB_ERR_NONE 0
#define RADIOLIB_ERR_PACKET_TOO_LONG -4
#define RADIOLIB_ERR_CRC_MISMATCH -7
#define LOW 0
static uint32_t clockMs = 0;
static bool dio1 = false;
uint32_t millis() { return clockMs; }
void delay(unsigned n) { clockMs += n; }
void pinMode(int, int) {}
void digitalWrite(int, int) {}
int digitalRead(int) { return dio1; }
struct SerialMock {
  std::string output;
  void begin(int baud) { assert(baud == 115200); }
  explicit operator bool() const { return true; }
  void print(const char *s) { output += s; }
  void print(unsigned long n) { output += std::to_string(n); }
  void println(const char *s) { output += s; output += '\n'; }
  void printf(const char *fmt, ...) {
    char buf[2048];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    output += buf;
  }
} Serial;
struct SPIClass { void begin(int, int, int) {} } SPI;
struct Module {
  Module(int cs, int irq, int rst, int busy, SPIClass&) {
    assert(cs == 5 && irq == 12 && rst == 14 && busy == 25);
  }
};
struct SX1262 {
  explicit SX1262(Module *) {}
  int beginCalls = 0, startCalls = 0, readCalls = 0;
  int16_t beginResult = 0, restartResult = 0, readResult = 0;
  int16_t transmitResult = 0, finishResult = 0;
  int transmitCalls = 0, finishCalls = 0;
  size_t length = 80;
  uint8_t payload[255] = {};
  int16_t begin(float mhz, float khz, int sf, int cr, int sync, int, int preamble, float tcxo, bool ldo) {
    ++beginCalls;
    assert(mhz == 915.0f && khz == 250.0f && sf == 7 && cr == 5);
    assert(sync == 0xA7 && preamble == 8);
    assert(tcxo == 1.7f && !ldo);
    return beginResult;
  }
  int16_t setDio2AsRfSwitch(bool enabled) { assert(!enabled); return 0; }
  void setRfSwitchPins(uint32_t rx, uint32_t tx) { assert(rx == 13 && tx == 15); }
  int16_t explicitHeader() { return 0; }
  int16_t setCRC(int n) { assert(n == 2); return 0; }
  int16_t invertIQ(bool invert) { assert(!invert); return 0; }
  int16_t startReceive() { ++startCalls; dio1 = false; return restartResult; }
  int16_t standby() { return 0; }
  size_t getPacketLength() { return length; }
  int16_t readData(uint8_t *out, size_t n) {
    ++readCalls;
    assert(n == 80);
    memcpy(out, payload, n);
    return readResult;
  }
  float getRSSI() { return -75.0f; }
  float getSNR() { return 7.25f; }
  int16_t startTransmit(const uint8_t *bytes, size_t n) {
    ++transmitCalls;
    assert(n == 80);
    memcpy(payload, bytes, n);
    dio1 = false;
    return transmitResult;
  }
  int16_t finishTransmit() { ++finishCalls; dio1 = false; return finishResult; }
};
'''

HARNESS = r'''
#include "receiver.ino"
int main() {
  setup();
  assert(radioOk && radio.beginCalls == 1 && radio.startCalls == 1);
  loop();
  assert(received == 0 && rejected == 0 && radio.readCalls == 0);
  LoraTelemetry p = {};
  p.magic = 0x5554;
  p.version = 1;
  p.seq = 42;
  memcpy(radio.payload, &p, sizeof(p));
  dio1 = true;
  loop();
  assert(received == 1 && rejected == 0 && radio.startCalls == 2);
  assert(Serial.output.find("\"seq\":42") != std::string::npos);
  // CRC errors are rejected, but reception is restarted.
  radio.readResult = RADIOLIB_ERR_CRC_MISMATCH;
  dio1 = true;
  loop();
  assert(received == 1 && rejected == 1 && radioOk);
  assert(lastRadioError == RADIOLIB_ERR_CRC_MISMATCH);
  // Over/undersize input must not be read into an 80-byte buffer.
  const int reads = radio.readCalls;
  radio.length = 255;
  dio1 = true;
  loop();
  radio.length = 79;
  dio1 = true;
  loop();
  assert(rejected == 3 && radio.readCalls == reads && radioOk);
  // Restart failure triggers reinitialization after the retry interval.
  radio.length = 80;
  radio.readResult = 0;
  radio.restartResult = -2;
  dio1 = true;
  loop();
  assert(!radioOk && lastRadioError == -2);
  radio.restartResult = 0;
  clockMs += 2000;
  loop();
  assert(radioOk && radio.beginCalls == 2);
  assert(Serial.output.find("\"link\":false") != std::string::npos);
  // Initialization failure remains visible and retries successfully.
  radioOk = false;
  radio.beginResult = -2;
  clockMs += 2000;
  loop();
  assert(!radioOk && lastRadioError == -2);
  radio.beginResult = 0;
  clockMs += 2000;
  loop();
  assert(radioOk && lastRadioError == 0);
}
'''


class ReceiverRuntimeTests(unittest.TestCase):
    def test_receive_errors_and_recovery(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = pathlib.Path(tmp)
            (folder / "mock.h").write_text(MOCK)
            for header in ("Arduino.h", "SPI.h", "RadioLib.h"):
                (folder / header).write_text('#include "mock.h"\n')
            (folder / "runtime.cpp").write_text(HARNESS)
            exe = folder / "runtime"
            subprocess.run(["clang++", "-std=c++11", "-Wall", "-Wextra", "-Werror",
                            "-I", str(folder), "-I", str(ROOT / "receiver"),
                            str(folder / "runtime.cpp"), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    unittest.main()
