# UTSM LoRa USB receiver

Data path: car running `transmit.ino` → LoRa radio link → receiver LoRa module → SPI → receiver ESP32 → USB serial → computer.

Open **`receiver/receiver.ino`** as its own Arduino sketch. Keep `telemetry.h` beside it. Do not put the receiver sketch alongside the transmitter and other `.ino` files in one sketch folder: Arduino combines those files, causing duplicate `setup()` and `loop()` definitions.

## Hardware and wiring

The configuration now follows the supplied **MainSchemRX.SchDoc** receiver and **MainSchem.SchDoc** transmitter diagrams. Both use an **ESP32-DEVKIT-V1** and a **Waveshare CORE1262-868M / Core1262-HF SX1262 SPI module**. Pin numbers below are ESP32 GPIO numbers, not symbol/header pin numbers.

| Core1262 signal | ESP32 GPIO, both boards |
| --- | --- |
| CLK / SCK | 18 |
| MISO | 19 |
| MOSI | 23 |
| CS / NSS | 5 |
| RESET | 14 |
| DIO1 | 12 |
| BUSY | 25 |
| RXEN | 13 |
| TXEN | 15 |
| DIO2 | Unconnected |
| 3V3 / GND | 3.3 V / common ground |

Both sketches use **RadioLib 7.1.2**, disable DIO2 antenna-switch control, and use `setRfSwitchPins(13, 15)`. The Core1262 schematic and its RTC6603SP truth table give RXEN HIGH / TXEN LOW for receive, and the inverse for transmit. The module includes the DC-DC inductor, so `LORA_USE_LDO = false`.

The **1.7 V TCXO setting** comes from the example repository linked by Waveshare's Core1262 documentation: `SX126xIoTcxoInit()` uses `TCXO_CTRL_1_7V`, with 5 ms startup. RadioLib 7.1.2 also defaults to 5 ms TCXO startup. This replaces the earlier unverified 0 V crystal placeholder. Verify the installed module revision if its vendor documentation specifies a different voltage.

Sources: [Waveshare Core1262 documentation](https://www.waveshare.com/wiki/Core1262-868M), [module schematic](https://files.waveshare.com/upload/c/c1/CoreSX1262_Sch.pdf), [RF switch truth table, page 3](https://files.waveshare.com/upload/c/c6/Datasheet-RTC6603SP-RichWave.pdf), [Waveshare-linked oscillator setup](https://github.com/siuwahzhong/lorawan-library-for-pico/blob/ws_dev/src/boards/rp2040/sx126x-board.c).

## USB connection and hardware corrections

The receiver schematic intends this path:

`SX1262 -> SPI -> ESP32 -> UART0 -> CP2104 U2 -> USB-C J1 -> computer`

The firmware uses `Serial` at **115200 baud**. Select **ESP32 Dev Module** in Arduino IDE; this classic ESP32 uses a USB-to-UART bridge, not native USB CDC. Its development board also has an onboard CP2102 and Micro-USB socket.

**The supplied schematic has issues that can prevent USB-C serial from working.** See [HARDWARE_REVIEW.md](../HARDWARE_REVIEW.md) for exact UART, USB data, supply/sense, CC-resistor and reset/boot findings. In particular, UART directions appear swapped by net naming, USB-C B7/DN2 is unconnected, and the CP2104 power arrangement needs correction. Do not interpret successful software tests as confirmation that J1 works as wired. Check for contention between the two USB-UART bridge transmit outputs before using either serial interface.

## Build and run

1. Install **esp32 by Espressif Systems** and **RadioLib by Jan Gromes, version 7.1.2** in Arduino IDE.
2. Open `receiver/receiver.ino` as a separate sketch, select **ESP32 Dev Module** and the board's serial port, and compile/upload. Keep `telemetry.h` beside it.
3. For the car, place `transmit.ino` in its own `transmit` sketch folder with `webapp.h` (and optional `plan.h`). Install **Adafruit LSM6DS**, **Adafruit Unified Sensor**, **Adafruit BusIO**, and **TinyGPSPlus** as required by its includes. The root contains unrelated `.ino` sketches, so do not compile them all together.
4. **Remove the transmitter SD card or isolate its CS** before testing LoRa. The supplied transmitter drawing connects SD CS to `LORA_CS`/GPIO5. Firmware now defaults to `SD_LOGGING 0`; that does not electrically isolate an inserted SD card. To restore logging, separate SD CS onto GPIO2 (the existing `PIN_SD_CS` setting), then enable logging.
5. Resolve the USB-C hardware issues or use a verified serial path with bridge contention addressed. Open Serial Monitor at **115200 baud** and power the transmitter. Attach suitable 915 MHz antennas.
6. Telemetry arrives up to five times per second. No Wi-Fi connection or run-start command is needed.

Both ends use 915 MHz, sync word `0xA7`, SF7, 250 kHz bandwidth, coding rate 4/5, eight-symbol preamble, explicit header, CRC enabled and normal IQ. RadioLib converts the 8-bit sync word to the SX1262 register representation internally. `transmit.ino` now uses asynchronous SX1262 transmission with TxDone polling and a one-second timeout, preserving the existing packet format. Its `loraPackets` counter now counts TxDone completions, not receiver acknowledgments. Initialization or cleanup errors are printed to the transmitter serial port; a failed initialization/cleanup disables transmitter LoRa until reset.

## Computer output

Example telemetry record (fields depend on which sensors have valid readings):

```json
{"type":"telemetry","t":4200,"seq":42,"boot":7,"flags":72,"rxMs":5000,"loraRssi":-75,"loraSnr":7.25,"spd":25.25,"v":24.500,"run":0}
```

Applications can read the serial port line by line, parse JSON, and use records where `type` is `telemetry`. Startup emits `hello`; `status` records appear every two seconds. Ignore non-JSON bootloader output if the board emits it during reset.

| Field | Meaning / units |
| --- | --- |
| `t`, `rxMs` | Transmitter and receiver uptime in milliseconds; separate clocks |
| `seq`, `boot` | Transmitter sample sequence and boot ID (0 means unavailable) |
| `flags` | Original validity/run flags from the packet |
| `ax`, `ay`, `az` | Acceleration, m/s² |
| `gx`, `gy`, `gz` | Angular velocity, degrees/s |
| `lat`, `lon`, `alt` | Latitude/longitude in degrees; altitude in metres |
| `spd`, `sats`, `hdop` | Speed in km/h, satellite count, GPS HDOP |
| `v`, `i`, `powerW` | Volts, amps, calculated watts (only when both measurements exist) |
| `run`, `armT` | Run armed state and transmitter arm uptime in milliseconds |
| `utcMs` | GPS/PPS UTC milliseconds since Unix epoch, when available |
| `loraRssi`, `loraSnr` | Receiver radio signal strength in dBm and SNR in dB |

Unavailable sensor fields are omitted, not reported as zero. The current transmitter has `JM_HAVE_CURRENT 0`, so it supplies no current or calculated power. This receiver does not integrate energy consumption.

Status reports `radio` (initialized and receive mode started), `link` (radio is active and a valid packet arrived within two seconds), `received`, `rejected`, `radioError` (most recent RadioLib operation error; 0 means success), and `lastRxAgeMs` (`null` until the first valid packet). Initialization or receive restart failures are retried every two seconds. `rejected` counts invalid lengths, protocol/format failures, and read errors, including CRC failures reported by RadioLib; it cannot count packets never detected over the air. `received` is a local decode/output count, not confirmation that a computer saved the data.

The transmitter increments `seq` at 10 Hz but sends LoRa at up to 5 Hz, so gaps of two or more are possible without RF loss. This receiver deliberately does not infer a packet-loss percentage from sequence gaps. The protocol has no vehicle identity or acknowledgment; use one transmitter on these settings. USB backpressure and disconnection can lose live records; the receiver does not provide persistent storage or replay.

## Verification and troubleshooting

Run `PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v` from the repository root (requires Python 3 and `clang++`). The ten host tests compare every wire field offset and flag against `transmit.ino`, compare radio settings, and exercise binary decoding and JSON formatting. Simulated radio/serial tests compile the receiver and the transmitter radio functions with test doubles. They check reception, CRC errors, unexpected lengths, restart failures, recovery, asynchronous TX completion/timeout, and decoding of bytes produced by the actual transmitter packet-building function. Pin assignments are also checked against the supplied schematics. These are not full ESP32 firmware builds or over-the-air tests. The RadioLib 7.1.2 API signatures used were separately checked against the downloaded library source.

- `radio: false`: inspect `radioError`; check SPI wiring, reset, BUSY, supply, and the module-specific oscillator/RF-switch settings.
- Radio initializes but nothing arrives: check DIO1 wiring as well as the RF settings.
- `radio: true`, `link: false`: check transmitter startup reports LoRa ready, matching settings, antennas and range.
- `rejected` increases: check that both ends use the same 80-byte protocol version 1.
- No serial output: check board/port selection, USB data cable, and bridge/native USB configuration.

Bench check: verify telemetry arrives, stop the transmitter and confirm `link` becomes false after two seconds, then restart it and verify reception resumes. Sensor faults must omit affected fields. Hardware and USB behavior still need this test on the actual boards.

References: [RadioLib 7.1.2 SX1262 API](https://github.com/jgromes/RadioLib/blob/7.1.2/src/modules/SX126x/SX1262.h), [RadioLib SX126x implementation](https://github.com/jgromes/RadioLib/blob/7.1.2/src/modules/SX126x/SX126x.cpp), [Espressif USB CDC setup](https://docs.espressif.com/projects/arduino-esp32/en/latest/tutorials/cdc_dfu_flash.html).
