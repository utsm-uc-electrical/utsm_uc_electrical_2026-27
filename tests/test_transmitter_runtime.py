"""Compile actual TX functions with simulated sensors/radio; decode their bytes."""
import pathlib
import re
import subprocess
import tempfile
import unittest
from test_receiver_runtime import MOCK

ROOT = pathlib.Path(__file__).resolve().parents[1]

SENSORS = r'''
struct Value {
  bool valid = true;
  bool isValid() const { return valid; }
  uint32_t age() const { return 0; }
  double lat() const { return 43.6627; }
  double lng() const { return -79.3956; }
  double meters() const { return 105.5; }
  double kmph() const { return 25.25; }
  uint32_t value() const { return 12; }
  double hdop() const { return 0.8; }
};
struct { Value location, altitude, speed, satellites, hdop; } gps;
struct { bool have = true; float ax=1, ay=2, az=3, gx=4, gy=5, gz=6; } smp;
struct { bool haveV=true, haveI=false; float v=24.5, i=0; } jm;
uint32_t bootId=7, runArmT=10;
bool runArmed=true;
int64_t esp_timer_get_time() { return (int64_t)clockMs * 1000; }
bool utcMsAt(int64_t, int64_t *utc) { *utc = 1792238400123LL; return true; }
bool loraOk=false;
uint32_t lastLoraTx=0, loraPackets=0, loraBusySkips=0, loraErrors=0;
bool loraTxActive=false;
uint32_t loraTxStarted=0;
static const uint32_t LORA_TX_TIMEOUT_MS=1000;
'''

HARNESS = r'''
int main() {
  loraOk = loraBegin();
  assert(loraOk);
  clockMs = 200;
  loraSendTelemetry(42);
  assert(loraTxActive && radio.transmitCalls == 1 && loraPackets == 0);
  LoraTelemetry p;
  assert(decodeTelemetry(radio.payload, 80, p));
  assert(p.seq == 42 && p.latE7 == 436627000 && p.lonE7 == -793956000);
  assert(p.volts == 24.5f && (p.flags & LORA_F_AMPS) == 0);
  assert(p.utcMs == 1792238400123LL && p.bootId == 7 && p.armT == 10);
  clockMs = 400;
  loraSendTelemetry(44);
  assert(loraBusySkips == 1 && radio.transmitCalls == 1);
  dio1 = true;
  loraPoll();
  assert(!loraTxActive && loraPackets == 1 && radio.finishCalls == 1);
  loraSendTelemetry(44);
  assert(loraTxActive);
  clockMs = 1400;
  loraPoll();
  assert(!loraTxActive && loraErrors == 1 && loraPackets == 1 && loraOk);
  // Start failure must release the RF switch and must not count a packet.
  radio.transmitResult = -2;
  loraSendTelemetry(46);
  assert(!loraTxActive && loraErrors == 2 && loraPackets == 1);
  // Cleanup failure disables the radio instead of sending in an unknown state.
  clockMs = 1600;
  radio.finishResult = -2;
  loraSendTelemetry(48);
  assert(!loraOk);
}
'''


class TransmitterRuntimeTests(unittest.TestCase):
    def test_async_send_and_wire_compatibility(self):
        source = (ROOT / "transmit.ino").read_text()
        settings = '\n'.join(re.findall(r'^#define (?:LORA_\w+|HAVE_JOULEMETER)\s+[^\n]+', source, re.M))
        start = source.index('static const int PIN_SPI_SCK')
        end = source.index('SX1262 radio(&radioModule);') + len('SX1262 radio(&radioModule);')
        pins = source[start:end]
        start = source.index('bool loraBegin() {')
        end = source.index('// READ IMU LATEST', start)
        functions = source[start:end]
        program = '#include "mock.h"\n#include "telemetry.h"\n' + settings + '\n' + pins + '\n' + SENSORS + functions + HARNESS
        with tempfile.TemporaryDirectory() as tmp:
            folder = pathlib.Path(tmp)
            (folder / "mock.h").write_text(MOCK)
            (folder / "transmitter.cpp").write_text(program)
            exe = folder / "transmitter"
            subprocess.run(["clang++", "-std=c++11", "-Wall", "-Wextra", "-Werror",
                            "-I", str(folder), "-I", str(ROOT / "receiver"),
                            str(folder / "transmitter.cpp"), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    unittest.main()
