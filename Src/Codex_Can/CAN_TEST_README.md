# STM32L476 CAN input/LED application: two nodes, expandable to four

This modifies the existing PB5_Blink project. Build once and flash the **same ELF** to both STM32L476RET6 boards. Neither board has a transmitter/receiver role. Each transmits its own PA8 and uses received peer data to drive PB5.

For the failed physical test, start with [CAN_DEBUGGER_GUIDE.md](CAN_DEBUGGER_GUIDE.md). It gives a stage-by-stage CubeIDE procedure and explains the new confirmed-TX diagnostics. The hardware root cause remains unverified without board measurements.

## Files and behavior

- `PB5_Blink.ioc`: CAN1 normal mode, 125 kbps, automatic retransmission and automatic bus-off management; PA8 input with pull-down; PB5 push-pull output; PB8/PB9 CAN1 AF9; PA13/PA14 reserved for Serial Wire debug; RX0 and SCE interrupts priority 5. HSI16 supplies SYSCLK and APB1. No external HSE/LSE is required.
- `Core/Src/main.c`: CubeMX clock/GPIO/CAN hardware initialization, PA8 sampling and 30 ms debounce, the two-call application loop, and fatal handling. Edits remain in USER CODE sections.
- `Core/Src/CAN/can_app.c`: UID-derived identifiers, filter/start, transmit scheduling/completion/retries, receive callbacks, per-peer state/timeouts, PB5 policy, and CAN diagnostics.
- `Core/Src/CAN/can_app.h`: public API, debugger declarations, and `CAN_APP_NODE_COUNT` (default 2). Both module files are in `Core/Src/CAN`; main includes `"CAN/can_app.h"`, and the module includes its sibling header. No compiler include-path changes are required.
- `Core/Src/stm32l4xx_hal_msp.c`: CAN1/GPIOB clocks, PB8/PB9 alternate function, CAN1_RX0_IRQn and CAN1_SCE_IRQn initialization.
- `Core/Src/stm32l4xx_it.c` and `Core/Inc/stm32l4xx_it.h`: CAN1_RX0_IRQHandler and CAN1_SCE_IRQHandler call HAL_CAN_IRQHandler(&hcan1).
- `Core/Inc/main.h`: exports hcan1; `stm32l4xx_hal_conf.h` enables the HAL CAN module. `Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_can.h` and `Src/stm32l4xx_hal_can.c` use the matching STM32Cube FW_L4 V1.18.2 driver.
- `.cproject`: debugger CPU clock updated to 16 MHz. Existing project name, linker scripts, SWD, and toolchain are retained.

The startup PA8 level is sampled and transmitted after CAN starts. Subsequent changes in either direction require 30 ms continuously at the new level. A heartbeat is queued every 100 ms after the last successful queue operation. The local input never sets PB5. In the default two-node build, valid peer data 1 sets PB5 and data 0 clears it; 500 ms without a valid peer frame clears PB5. With four nodes, PB5 is the OR of all active remote input states. Each peer expires independently after 500 ms, and later valid frames restore that peer automatically.

All elapsed-time comparisons tolerate HAL tick rollover. The RX callback processes at most three frames per interrupt. The timeout check briefly masks interrupts so an arriving frame cannot race with the LED clear. There is no HAL_Delay in the application loop and no application dynamic allocation.

## CAN timing

The original project used MSI at 4 MHz. This version uses factory-trimmed HSI16 directly, with PLL disabled and AHB/APB1/APB2 prescalers all 1:

```
SYSCLK = HCLK = PCLK1 = PCLK2 = 16,000,000 Hz
CAN clock = PCLK1 (the APB timer clock multiplier does not apply)
Prescaler = 8
Synchronization segment = 1 tq
BS1 = 13 tq
BS2 = 2 tq
SJW = 2 tq
Total = 1 + 13 + 2 = 16 tq
One tq = 8 / 16,000,000 = 0.5 us
One bit = 16 * 0.5 us = 8 us
Bitrate = 16,000,000 / (8 * 16) = 125,000 bit/s
Sample point = (1 + 13) / 16 = 87.5%
```

`canPclk1Hz` records HAL_RCC_GetPCLK1Freq(). MX_CAN1_Init checks it against the expected timing product before enabling CAN. A later clock edit that breaks this relationship enters Error_Handler rather than silently using the wrong bitrate.

HSI16 avoids dependence on unverified external crystals for this bench test. Its actual frequency varies with temperature, voltage, and manufacturing tolerance. The calculation above is nominal; HAL_RCC_GetPCLK1Freq reports the configured nominal frequency, not a physical frequency measurement. For a production network or wide temperature range, validate oscillator tolerance and bit timing, or use a validated/calibrated clock. See ST DS10198, table 59: https://www.st.com/resource/en/datasheet/stm32l476re.pdf

## Identification and filtering

HAL_GetUIDw0/1/2 reads all 96 UID bits. FNV-1a hashes the 12 bytes in fixed order, followed by an avalanche mix and a `0x1FFFFFFF` mask. The result is a legal 29-bit extended identifier, including zero if the hash produces it. The same binary derives a different identifier on almost all physical devices, so simultaneous different payloads normally arbitrate safely.

A 96-to-29-bit hash cannot guarantee uniqueness. For an approximately uniform hash, a particular pair has collision probability about 1 / 536,870,912. Before testing, compare `localCanId` on the two real boards in hexadecimal. They **must differ**. If equal, stop the test: self-ID rejection would discard the peer, and simultaneous differing payloads under one ID could cause CAN errors. A collision would require a revised shared identification/collision-resolution scheme; it cannot be ruled out by software hashing alone. `uidWords[0..2]` lets you check the underlying UIDs.

Filter bank 0 uses 32-bit identifier/mask mode and directs matches to FIFO0. The filter register places IDE at bit 2 and RTR at bit 1. Filter ID low = `0x0004`, mask low = `0x0006`, and both high halves are zero. Thus IDE must be 1 (extended), RTR must be 0 (data), and all identifier bits are ignored by the hardware filter. The receive callback then requires:

1. Extended data frame.
2. Identifier different from `localCanId`.
3. DLC exactly 1.
4. Byte 0 equal to 0 or 1.

Only accepted frames update the LED, `lastRxTick`, and `canRxCount`. Other payloads/IDs never refresh the timeout. The peer table has `CAN_APP_NODE_COUNT - 1` slots and learns IDs from valid arrivals. An unknown ID cannot evict an active peer when the table is full: it increments `canPeerTableFullCount` and `canIgnoredRxCount`. A timed-out slot can be reused. This is discovery on a small shared test bus, not authenticated pairing; received frames are never retransmitted.

## Expand from two nodes to four

1. In `Core/Src/CAN/can_app.h`, change only the network-size setting:

   ```c
   #define CAN_APP_NODE_COUNT 4U
   ```

   The count includes this board: 2 means one remote peer; 4 means three remote peers.
2. Refresh, clean, and rebuild in CubeIDE; flash the same resulting ELF to all boards. IDs still come from each STM32 UID. Compare all generated IDs for uniqueness; changing this setting does not change bitrate or pins.
3. Each board's PB5 turns on when ANY active remote PA8 is HIGH. LOW heartbeats from another peer do not cancel an active HIGH. A missing HIGH sender stops contributing after its own 500 ms timeout, even while other peers keep transmitting LOW heartbeats.
4. Inspect `canPeers[0..CAN_APP_MAX_PEERS-1]`: `canId`, `inputState`, `lastRxTick`, `rxCount`, `occupied`, and `active`. `canPeerCount` is the number of active peers. `canRxTimedOut=1` means there are no active peers; inspect each slot to identify one missing peer.
5. To use paired outputs later, change the peer selection in `CAN_UpdatePeerOutput()` in `can_app.c`; the current default intentionally uses any remote HIGH.
6. Keep termination on only the two physical bus ends: H5 closed enables R15 (120 ohms) on your schematic. Leave H5 open on intermediate boards. Keep CANH/CANL/GND continuous along the bus.

Main's application loop is now:

```c
ProcessInput();
CAN_App_Process(HAL_GetTick(), localInputState);
```

`CAN_App_Init(&hcan1, localInputState)` is called once after the CubeMX CAN hardware initialization. It configures the protocol, starts CAN, and attempts the initial state transmission. It returns HAL errors to `Error_Handler`. HAL callbacks exist only in `can_app.c`; keep the existing RX0/SCE IRQ handlers calling the HAL.

## Transmit failures and bus-off

Only one application frame can be outstanding. `canTxCount` records mailbox queueing; `canTxCompleteCount` records only hardware RQCP + TXOK. Completion is polled with TX interrupts disabled, and failed completions explicitly retry the latest state. `canLastTxStatus` saves the completion flags before they are cleared. A normal-mode TX success proves a CAN ACK, but does not prove application acceptance by the peer. Busy hardware mailboxes are retried at 10 ms intervals. HAL queue failures retry after 100 ms. A frame still pending after 100 ms gets one asynchronous ABRQ-only register write (avoiding TSR read-modify-write and its write-one-to-clear flag side effects); the loop waits for completion without blocking, then allows a 100 ms backoff after an unsuccessful completion and sends the newest local state. Pending changes are coalesced, so a disconnected peer cannot build a backlog of stale states.

Hardware automatic retransmission remains enabled while a frame is pending; retransmissions on the wire can therefore occur faster than the 100 ms application heartbeat when ACKs are missing. Old transmissions are bounded by the abort policy. No new frames are queued during bus-off. Automatic bus-off management (ABOM) handles the CAN recovery sequence in hardware after 128 occurrences of 11 recessive bits. The loop notices recovery and resumes after 100 ms without repeatedly restarting CAN.

RX overrun, error warning, error passive, and bus-off notifications are enabled. Per-retry last-error-code interrupts are omitted to avoid an ACK-error interrupt storm when a board is alone. `canEsr` continuously exposes the hardware error status, including LEC/TEC/REC. Observed nonzero LEC transitions are mapped to HAL error masks and latched in `canLastError`, including ACK errors (0x20). `canErrorCount` counts callback/API/polled-error observations, not each wire retry. `canStarted` and `canInitStage` expose startup progress, and `canRxCallbackCount` distinguishes callback entry from valid peer reception. Fatal initialization failures set `fatalError`, force PB5 low, and stop in Error_Handler.

## Build and regeneration

1. Open the existing PB5_Blink project in STM32CubeIDE. Refresh it (select project, F5) and reload externally changed editors if prompted.
2. Choose the Debug build configuration. Use Project > Clean, then Project > Build Project. Confirm the build console compiles `can_app.c`; the managed builder discovers the new file under the existing Core source entry and regenerates its makefiles. Do not compile only the old Debug makefile before refreshing. The ELF is `Debug/PB5_Blink.elf`. Release uses `Release/PB5_Blink.elf`.
3. Code has already been generated from the updated .ioc. Regeneration is not required before building. If you later regenerate with CubeMX, open this .ioc, keep **Keep User Code** enabled, and generate for the existing STM32CubeIDE project using STM32Cube FW_L4 V1.18.2. Do not replace it with a new project.
4. After regeneration, verify HSI16/PCLK1=16 MHz, prescaler 8, BS1 13, BS2 2, SJW 2, automatic retransmission/bus-off enabled, PA8 pull-down, and both CAN interrupts. Main integration remains in USER CODE regions; `can_app.c` and `can_app.h` are separate user-owned files. Keep the existing project files when regenerating. The present patch has not been regenerated through CubeMX; the earlier regeneration check predates it.

Native application behavior tests can be rerun on this Mac with:

```
python3 Core/Src/Codex_Can/test_can_app.py
```

They compile the actual `can_app.c` and main input functions against a small HAL mock, separately configured for both two and four nodes, and cover debounce/bounce, rising/falling edges, input/output independence, tick rollover, receive validation, timeout/recovery, initial/change/periodic transmissions, full mailboxes, HAL failure backoff, stale-frame abort, bus-off/recovery, UID hashing, filter/start configuration, full peer tables, per-peer timeout/recovery, and the multi-node OR output rule. They complement the ARM firmware build; they do not validate physical wiring, CAN signaling, or actual silicon behavior.

## Flash the identical firmware to both boards

1. Power Board A correctly. Connect ST-LINK SWDIO to PA13, SWCLK to PA14, and GND to board GND; connect target reference voltage and optionally NRST according to your ST-LINK wiring. Avoid connecting competing power sources.
2. In CubeIDE, create/use an STM32 C/C++ Application debug configuration for PB5_Blink, select ST-LINK and SWD, and point it to `Debug/PB5_Blink.elf`. Download it, then Resume. If debugging stops at main, CAN has not started yet.
3. End the debug session, move the ST-LINK target cable to Board B, and download the **same ELF file without edits or a board-specific rebuild**. Resume Board B and ensure both boards are running. Reset/power-cycle both for a standalone test.
4. You can alternatively use STM32CubeProgrammer: connect via ST-LINK/SWD, open that same ELF, download with verification, and run/reset each board. Keep BOOT0 in the normal main-flash boot configuration.

## Debugger variables

In the CubeIDE Debug perspective, open Window > Show View > Expressions (or Live Expressions), add the following names, and Resume. Use hexadecimal display for identifiers and error masks. Live Expressions availability/update behavior depends on the probe; ordinary Expressions can be inspected while halted. Do not leave a board halted while expecting it to communicate normally.

| Variable | Meaning / expected observation |
|---|---|
| `rawInputState` | Immediate PA8 reading, 0 disconnected and 1 jumpered to 3.3 V |
| `localInputState` | Debounced PA8 state; changes about 30 ms after an edge |
| `localCanId` | Local extended transmit ID; compare both boards before testing |
| `uidWords[0]`, `[1]`, `[2]` | Factory UID words |
| `canPclk1Hz` | Must be 16000000 |
| `canTxCount` | Frames accepted into a transmit mailbox; approximately 10/s when connected, plus input edges. This alone does not prove peer ACK/reception |
| `canTxCompleteCount`, `lastTxCompleteTick` | Confirmed successful normal-mode transmissions and latest completion time |
| `canLastTxStatus` | TSR snapshot at completion, before clearing RQCP/TXOK |
| `canInitStage`, `canStarted` | Must reach 6 and 1 respectively; see debugger guide for failure stages |
| `canRxCallbackCount` | All RX0 callback entries, including rejected frames |
| `lastTxTick` | Most recent successful queue timestamp |
| `canRxCount` | Accepted peer state frames; approximately 10/s with an idle connected peer |
| `lastReceivedCanId` | Should equal the other board's localCanId |
| `lastReceivedState` | Last accepted peer payload; retains its last value after timeout |
| `lastRxTick` | Time of the last accepted peer frame |
| `canRxTimedOut` | 1 when no tracked remote peers are active; 0 while at least one peer is active |
| `canPeers[]` | Each peer ID, input, timestamp, receive count, occupied flag, and active flag |
| `canPeerCount`, `canPeerTableFullCount` | Number of active peers and number of frames dropped because no free/expired slot was available |
| `canErrorCount`, `canLastError`, `canLastEsr` | Error observations (callback/API/LEC transitions), cumulative HAL-compatible error bits, and latest error ESR |
| `canEsr` | Current ESR: BOFF bit 2, LEC bits 6:4, TEC bits 23:16, REC bits 31:24 |
| `canBusOff`, `canBusOffCount` | Current observed bus-off status and number of entries |
| `canTxFailureCount`, `canTxAbortCount`, `canTxAbortedCount` | Queue/completion failures, requested aborts, and completed aborts without TXOK |
| `canIgnoredRxCount` | Frames rejected by application validation after hardware filtering |
| `fatalError` | Must remain 0; 1 means initialization/timing check failed |

Pausing the CPU affects timeout timing and mailbox service, so test timeout behavior with both boards running or by removing power from the peer rather than relying on breakpoints.

## Hardware test

1. Power off both boards. Connect CANH to CANH, CANL to CANL, and GND to GND. Use a short twisted CANH/CANL pair and short stubs.
2. For this two-board bus, enable one 120 ohm terminator on **both** end boards. On your schematic, close H5 to connect R15 (120 ohms). With power off, measure roughly **60 ohms across CANH/CANL** for the connected bus. About 120 ohms suggests only one termination; roughly 40 ohms suggests three.
3. Check U3 (TCAN332D on the schematic) has 3.3 V on pin 3 and GND on pin 2 and that PB9 goes to TXD and PB8 to RXD. Check local decoupling, stable MCU supplies, and normal flash boot.
4. Flash the same ELF to both. Leave both PA8 jumpers disconnected. Power/reset both and let them run. Both PB5 outputs should be LOW.
5. Check different localCanId values, canPclk1Hz=16000000, canRxCount increasing on both, and lastReceivedCanId matching the other board. Use live reads or inspect one board briefly, then resume it.
6. Connect **Board A PA8 to its own 3.3 V rail through about 1 kohm**. After about 30 ms plus a CAN frame, Board B PB5 goes HIGH; A PB5 stays LOW. Remove the jumper: B PB5 goes LOW after debounce and transmission.
7. Repeat on Board B. Then set both PA8 inputs HIGH: both PB5 outputs go HIGH. Clear each separately to confirm independence.
8. To test timeout, first use A PA8 HIGH to make B PB5 HIGH. Remove power from A while leaving B powered and the bus wiring/termination intact. Within about 500 ms of the last valid received frame, B PB5 goes LOW. Restore A's power: B recovers automatically when A's valid heartbeat resumes. Repeat in reverse.

| A PA8 | B PA8 | A PB5 | B PB5 |
|---|---|---|---|
| LOW | LOW | LOW | LOW |
| HIGH | LOW | LOW | HIGH |
| LOW | HIGH | HIGH | LOW |
| HIGH | HIGH | HIGH | HIGH |

## Hardware points to check if it fails

- If the populated part is TCAN332: it is compatible with 3.3 V STM32 logic and Classical CAN at 125 kbps. On TCAN332, pins 5 and 8 are NC; it has no standby/enable pin. Confirm the populated part really is TCAN332 rather than a related part with different control pins. TI recommends a 100 nF ceramic bypass capacitor close to VCC/GND: https://www.ti.com/lit/ds/symlink/tcan332.pdf
- Two end terminators, common ground, correct CANH/CANL polarity, adequate power, and running firmware on both boards are necessary. A single running node generally cannot get CAN acknowledgments from an absent/unpowered node.
- Your schematic uses PB5 through R13 to the Q1 (2N7002) gate, with R14 as its pull-down. Q1 sinks D2 current through R12, so PB5 HIGH is the correct active-high LED command. D1 is the separate power LED. Check actual assembly if D2 does not follow PB5.
- Use the board's **3.3 V**, not 5 V, for PA8. The internal pull-down makes an unplugged input LOW. Long jumper leads or leakage can still cause noise; keep leads short. The 1 kohm series resistor limits accidental fault current.
- Check BOOT0, NRST, MCU VDD/VDDA and ground connections, soldering, and SWD connectivity. The supplied schematic has been reviewed; actual assembly and physical measurements remain unverified.
- HSI16 tolerance may need calibration or a validated external clock if errors persist, especially with temperature changes. Use a scope/CAN analyzer to verify nominal 8 us bits and decode 125 kbps extended data frames.

Current build/host-test verification is recorded in `BUILD_VERIFICATION.txt`. Physical communication and the success criterion still require the two-board hardware test above.
