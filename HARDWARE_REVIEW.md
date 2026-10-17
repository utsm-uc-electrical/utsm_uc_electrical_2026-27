# Telemetry boards: schematic findings and firmware configuration

Reviewed the two user-supplied images titled `MainSchemRX.SchDoc` (receiver, date shown 8/05/2026) and `MainSchem.SchDoc` (transmitter, date shown 9/08/2026). Findings below describe the drawings; verify the PCB netlist and assembled board before reworking hardware. Native schematic/PCB files were not supplied or edited.

## Firmware changes made

- Both ends now use RadioLib 7.1.2 with SX1262 and the same existing 80-byte protocol.
- Both schematics map SCK/MISO/MOSI/CS/RESET/DIO1/BUSY to GPIO18/19/23/5/14/12/25, respectively.
- RXEN is GPIO13 and TXEN GPIO15. DIO2 is explicitly unconnected. The firmware disables DIO2 RF control and uses the two ESP32 control outputs.
- RX state is GPIO13 HIGH, GPIO15 LOW; TX state is GPIO13 LOW, GPIO15 HIGH. This follows the Core1262 internal schematic and RTC6603SP truth table (V1 = RX_EN, V2 = TX_EN; RF1 = radio TX, RF2 = radio RX). Some vendor prose is inconsistent; the circuit and switch truth table are the basis here.
- TCXO is configured for 1.7 V with RadioLib's 5 ms startup delay, matching the Waveshare-linked `ws_dev` example. DC-DC regulation is enabled; the Core1262 internal schematic includes its external inductor.
- Receiver JSON is emitted on `Serial`/UART0 at 115200 baud. There is no native USB CDC setup on this ESP32.
- Transmitter SD logging defaults to disabled because of the chip-select conflict below. An attached SD card must still be removed or electrically isolated.

Sources: [Core1262 documentation](https://www.waveshare.com/wiki/Core1262-868M), [Core1262 internal schematic](https://files.waveshare.com/upload/c/c1/CoreSX1262_Sch.pdf), [RTC6603SP truth table, page 3](https://files.waveshare.com/upload/c/c6/Datasheet-RTC6603SP-RichWave.pdf), [Waveshare-linked TCXO initialization](https://github.com/siuwahzhong/lorawan-library-for-pico/blob/ws_dev/src/boards/rp2040/sx126x-board.c), [RadioLib 7.1.2 SX1262 API](https://github.com/jgromes/RadioLib/blob/7.1.2/src/modules/SX126x/SX1262.h).

## Receiver: USB-C path and corrections

The intended path is ESP32 UART0 to CP2104 U2 to USB-C J1. The drawing confirms USB-C was intended for data, not just power, but does not establish a functioning implementation.

### 1. UART direction is crossed incorrectly by net labels

| Drawing connection | Problem | Required functional connection |
| --- | --- | --- |
| U2 TXD pin 21 -> `RX0` -> U_ESP1 TX0 pin 13 | Output connected to output | U2 TXD -> ESP32 RX0 / GPIO3 |
| U2 RXD pin 20 -> `TX0` -> U_ESP1 RX0 pin 12 | Input connected to input | ESP32 TX0 / GPIO1 -> U2 RXD |

The ESP32 symbol's RX0 terminal has the `TX0` net label, and its TX0 terminal has the `RX0` net label. Renaming labels consistently does not fix direction; verify actual pin-to-pin connectivity. Follow signal directions and GPIO numbers.

### 2. CP2104 supply and USB presence detection

U2 REGIN pin 7 and VIO pin 5 are shown on +3V3, but VDD pin 6 is left unconnected. CP2104's documented 3.0–3.6 V self-powered configuration ties **REGIN and VDD together** at that supply, with the specified decoupling. Do not simply feed its regulator input 3.3 V while leaving VDD unconnected.

U2 VBUS pin 8 is marked unconnected, and J1 VBUS contacts are also unconnected. Implement USB VBUS sensing according to the CP2104 self-powered application circuit, including its divider requirement when the board can be unpowered while USB is connected. As drawn, J1 does not supply board power; a separate regulated supply is required unless a proper USB-powered supply circuit is added.

Reference: [Silicon Labs CP2104 datasheet, section 9 and Figures 8–9](https://www.silabs.com/documents/public/data-sheets/cp2104.pdf).

### 3. USB-C data pair and CC resistors

J1 DP1/A6 and DP2/B6 both connect to `USBD+`. DN1/A7 connects to `USBD-`, but **DN2/B7 is marked unconnected**. Connect both D− contacts to `USBD-` as well, so flipping the connector does not remove the negative data connection.

R2/R3 on CC1/CC2 are specified as **10 kΩ** in the drawing. A USB-C sink/UFP uses an Rd pull-down of **5.1 kΩ nominal on each CC pin**. Check fitted values and correct both. CC detection and VBUS sensing cannot be repaired by ESP32 firmware.

Reference: [USB-IF USB Type-C specification](https://www.usb.org/sites/default/files/USB%20Type-C%20Spec%20R2.0%20-%20August%202019.pdf).

### 4. Two bridges share the development board's UART

The photographed development board already contains a CP2102 connected to UART0. Adding CP2104 TXD to the same ESP32 RX0 can put two actively driven bridge outputs on the same net, even if only one USB cable is plugged in. Verify bridge power/output states and provide isolation or select one bridge for UART0. Removing a USB cable is not proof of electrical isolation when the carrier powers the development board.

For initial serial tests, the onboard Micro-USB port is a known interface only after any conflicting carrier bridge connection has been isolated. A USB-C-to-Micro-USB data cable can connect it to a USB-C computer. The desired J1 USB-C path should be tested after the wiring fixes above.

### 5. Automatic programming/reset is not fully connected as drawn

Q2's emitter uses net `DTR2`, whereas U2 supplies `DTR`. The `BOOT` net from S1/Q1/R4 is not visibly connected to the ESP32 GPIO0 terminal on this sheet. Verify these against the design files; automatic download mode will not work if they remain separate/unconnected. The development board's own BOOT and EN controls can be used manually with a verified serial path.

## Transmitter: SD and radio share chip select

The SD CARD connector pin 2 is labeled **LORA_CS**, the same net as the Core1262 CS and ESP32 GPIO5. Independent SPI peripherals can share SCK/MOSI/MISO, but require independent chip selects. Otherwise both can respond to the same transactions.

For radio-only testing, **remove the SD card or isolate its CS**. The firmware sets `SD_LOGGING 0`, but that does not prevent a physically selected SD card from driving MISO. To restore SD logging, move SD CS to the intended `D2/CS0` / GPIO2 net, confirm the PCB rework, and set `SD_LOGGING 1`. `PIN_SD_CS` remains 2 for that corrected wiring.

## Bench validation after hardware checks

1. Confirm 3.3 V supply, common ground, mapped GPIO continuity, and appropriate 915 MHz antennas.
2. Confirm receiver USB enumeration and standalone `hello`/`status` JSON at 115200 baud before introducing LoRa traffic.
3. Verify receiver `radio: true` and transmitter `SX1262 TX ready` logs. A nonzero RadioLib error needs diagnosis; the host tests cannot detect wiring or power faults.
4. Verify packets arrive with changing sequence/timestamps and plausible sensor values. TX completion does not prove reception.
5. Turn off the transmitter: receiver `link` should become false after two seconds. Restart it and confirm recovery.
6. Once the USB-C circuit is corrected, test both connector orientations. Only re-enable SD logging after separating its CS.

Software validation: ten host tests cover the packet layout, radio parameters, schematic GPIO map, binary-to-JSON conversion, receiver recovery, and transmitter async success/timeout/error paths. Full ESP32 builds, USB enumeration and RF reception have not been tested on the physical boards.
