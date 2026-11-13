# STM32 CAN Firmware — UTSM 2026–2027

CAN firmware for the University of Toronto Supermileage Team's 2026–2027 Urban Concept vehicle.

## Repository Scope

This branch contains only the STM32 `Src` directory, documentation, and tests — **not a complete STM32CubeIDE project**.

To build and flash, copy `Src` into your local STM32L476RET6 project's `Core/Src`. Required headers, HAL/CMSIS drivers, CubeMX configuration, startup files, linker scripts, and flashing tools must be maintained locally.

## Functionality

Identical firmware runs on all boards:

- **PA8:** Input with pull-down and 30 ms debounce.
- **CAN:** Transmits `0x01` (HIGH) or `0x00` (LOW) on changes and every 100 ms.
- **PB5:** HIGH when any active remote board reports HIGH.
- **Timeout:** Remote states expire after 500 ms.
- **Diagnostics:** CAN transmission, reception, and errors available through the debugger.

Local PA8 does not directly control local PB5.

## Hardware Configuration

| Item | Configuration |
|---|---|
| MCU | STM32L476RET6 |
| Input | PA8 |
| CAN RX / TX | PB8 / PB9 (AF9) |
| LED | PB5 |
| CAN bitrate | 125 kbps |
| CAN timing | 16 MHz PCLK1, Prescaler 8, BS1 13 tq, BS2 2 tq, SJW 2 tq |

## Multi-Board Support

Defaults to **2 nodes**. For 4 nodes, update `Src/CAN/can_app.h`:

```c
#define CAN_APP_NODE_COUNT 4U
```

Flash identical firmware to all boards. CAN IDs are derived from STM32 unique IDs; verify they differ.

Connect all boards to CANH/CANL with common ground and 120 Ω termination at both physical bus ends only.

## Testing

See `Src/Codex_Can/` for test and debugger guides.

Run optional host tests:

```bash
python3 Src/Codex_Can/test_can_app.py
```

Host tests do not replace hardware validation.