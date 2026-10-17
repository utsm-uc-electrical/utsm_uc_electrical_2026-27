"""Host-side protocol tests; run: python3 tests/test_receiver.py.

Compiles the actual decoder/formatter and compares its layout to transmit.ino.
This does not replace an ESP32 firmware build or an over-the-air test.
"""
import json
import pathlib
import re
import struct
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
WIRE = struct.Struct("<HBHHI6fii2fB3fIIq")


class ReceiverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        folder = pathlib.Path(cls.temp.name)
        transmitter = (ROOT / "transmit.ino").read_text()
        packet = re.search(r"struct __attribute__\(\(packed\)\) LoraTelemetry \{.*?\};",
                           transmitter, re.S).group()
        flags = re.search(r"enum : uint16_t \{\s*LORA_F_IMU.*?\};",
                          transmitter, re.S).group()
        fields = re.findall(r"\b(?:uint\d+_t|int\d+_t|float)\s+(\w+)\s*;", packet)
        assertions = '\n'.join(
            f'static_assert(offsetof(LoraTelemetry, {f}) == '
            f'offsetof(sender::LoraTelemetry, {f}), "{f} offset mismatch");'
            for f in fields)
        assertions += '\n' + '\n'.join(
            f'static_assert((unsigned){f} == (unsigned)sender::{f}, "Flag mismatch");'
            for f in re.findall(r'(LORA_F_\w+)\s*=', flags))
        source = folder / "decode.cpp"
        source.write_text('''#include "telemetry.h"
#include <iostream>
#include <iterator>
#include <vector>
namespace sender {
''' + packet + '\n' + flags + '\n}\n' + assertions + '''
static_assert(sizeof(LoraTelemetry) == sizeof(sender::LoraTelemetry), "Size mismatch");
int main(int argc, char **) {
  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(std::cin)), {});
  LoraTelemetry p;
  if (!decodeTelemetry(bytes.data(), bytes.size(), p)) {
    std::cout << "{\\"valid\\":false}";
    return 0;
  }
  char output[1536];
  if (!telemetryJson(p, 1234, -75, 7.25f, output, argc > 1 ? 8 : sizeof(output))) {
    std::cout << "{\\"overflow\\":true}";
    return 0;
  }
  std::cout << output;
}
''')
        cls.exe = folder / "decode"
        subprocess.run(["clang++", "-std=c++11", "-Wall", "-Wextra", "-Werror",
                        "-I", str(ROOT / "receiver"), str(source), "-o", str(cls.exe)],
                       check=True)

    def packet(self, **changes):
        values = dict(magic=0x5554, version=1, flags=0x3FF, seq=65535,
                      t=4294967295, ax=1.25, ay=-2.5, az=9.81,
                      gx=0.1, gy=-0.2, gz=0.3, lat=436627000,
                      lon=-793956000, alt=105.5, spd=25.25, sats=12,
                      hdop=0.8, v=24.5, i=2.0, armT=500, boot=7,
                      utcMs=1792238400123)
        values.update(changes)
        return WIRE.pack(*values.values())

    def decode(self, packet, *args):
        result = subprocess.run([str(self.exe), *args], input=packet,
                                capture_output=True, check=True)
        return json.loads(result.stdout, parse_constant=lambda x: self.fail(x))

    def test_full_packet(self):
        raw = self.packet()
        self.assertEqual(len(raw), 80)
        p = self.decode(raw)
        self.assertEqual(p["type"], "telemetry")
        self.assertAlmostEqual(p["lat"], 43.6627)
        self.assertAlmostEqual(p["lon"], -79.3956)
        self.assertEqual(p["powerW"], 49.0)
        self.assertEqual(p["seq"], 65535)
        self.assertEqual(p["t"], 4294967295)
        self.assertEqual(p["utcMs"], 1792238400123)
        self.assertEqual(p["loraRssi"], -75)
        self.assertEqual(p["run"], 1)

    def test_missing_sensors_are_absent(self):
        p = self.decode(self.packet(flags=0, ax=float("nan")))
        for key in ("ax", "lat", "alt", "spd", "sats", "hdop", "v", "i", "powerW", "armT", "utcMs"):
            self.assertNotIn(key, p)
        self.assertEqual(p["run"], 0)

    def test_current_disabled_matches_transmitter(self):
        p = self.decode(self.packet(flags=0x3FF & ~(1 << 7)))
        self.assertEqual(p["v"], 24.5)
        self.assertNotIn("i", p)
        self.assertNotIn("powerW", p)

    def test_invalid_packets(self):
        for packet in (b"", self.packet()[:-1], self.packet() + b"x",
                       self.packet(magic=0), self.packet(version=2),
                       self.packet(flags=0x400), self.packet(lat=900000001),
                       self.packet(lon=-1800000001), self.packet(ax=float("nan")),
                       self.packet(v=float("inf"))):
            with self.subTest(packet=packet.hex()):
                self.assertEqual(self.decode(packet), {"valid": False})

    def test_truncation_is_not_emitted(self):
        self.assertEqual(self.decode(self.packet(), "small"), {"overflow": True})

    def test_sequence_wrap_is_accepted(self):
        self.assertEqual(self.decode(self.packet(seq=0))["seq"], 0)

    def test_radio_settings_match(self):
        tx = (ROOT / "transmit.ino").read_text()
        rx = (ROOT / "receiver" / "receiver.ino").read_text()
        for setting in ("LORA_FREQ_HZ", "LORA_SYNC_WORD", "LORA_SF", "LORA_BW_HZ", "LORA_CR_DENOM"):
            a = re.search(r"#define\s+" + setting + r"\s+(0x[0-9A-Fa-f]+|[0-9]+)", tx).group(1)
            b = re.search(setting + r"\s*=\s*(0x[0-9A-Fa-f]+|[0-9]+)", rx).group(1)
            self.assertEqual(int(a, 0), int(b, 0), setting)

    def test_pins_match_supplied_schematics(self):
        pins = dict(PIN_SPI_SCK=18, PIN_SPI_MISO=19, PIN_SPI_MOSI=23,
                    PIN_LORA_CS=5, PIN_LORA_RST=14, PIN_LORA_DIO1=12,
                    PIN_LORA_BUSY=25, PIN_LORA_RXEN=13, PIN_LORA_TXEN=15)
        for file in (ROOT / "transmit.ino", ROOT / "receiver" / "receiver.ino"):
            text = file.read_text()
            for pin, expected in pins.items():
                self.assertEqual(int(re.search(pin + r"\s*=\s*(\d+)", text).group(1)),
                                 expected, f"{file.name}: {pin}")


if __name__ == "__main__":
    unittest.main()
