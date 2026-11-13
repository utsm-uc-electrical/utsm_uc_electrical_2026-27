# Diagnose the failed two-board CAN test

The source review found the requested PA8/PB8/PB9/PB5 configuration, debounce,
125 kbps timing, receive filter, and RX0 interrupt path already present. The
supplied schematic confirms U3 TCAN332D, H5/R15 termination, and an active-high
PB5/Q1/D2 LED path. Physical measurements are still needed to establish the
failed test's root cause. Successful compilation is not hardware proof.

The update fixes missing transmission-result handling: a free mailbox was
previously retired without inspecting TXOK. `canTxCompleteCount` now increments
only for RQCP + TXOK, unsuccessful completions retry the latest state, and
missing-ACK/other ESR errors are latched even without per-retry interrupts.
Abort writes touch only ABRQ, avoiding the HAL TSR read-modify-write that can
clear write-one-to-clear completion flags. Completion flags are polled; TX
interrupts must remain disabled so the HAL handler cannot consume those flags.

## Module layout and node configuration

CAN application functions and HAL callbacks are now in `../CAN/can_app.c`, with
public declarations and `CAN_APP_NODE_COUNT` in `../CAN/can_app.h`. Main retains
CubeMX hardware initialization and PA8 debounce, then calls `CAN_App_Process()`.
Default node count 2 reserves one remote peer slot. Set it to 4 and flash the
same rebuilt binary on all four boards to reserve three remote slots. PB5 is
HIGH if any active remote input is HIGH; each peer expires independently after
500 ms. All original debug symbols are retained. For a missing individual peer,
inspect `canPeers[i].active` and `lastRxTick`; `canRxTimedOut` is the no-active-
peers summary, not an individual-node alarm. `canPeerTableFullCount` identifies
additional unknown senders rejected while all slots are live.

## 1. Build, flash, and check initialization

1. Refresh PB5_Blink in CubeIDE, select Debug, Clean, then Build Project.
   Confirm the console compiles the new `can_app.c`.
   Flash the newly built `Debug/PB5_Blink.elf` to both boards. Do not flash an
   ELF left over from before these edits. Resume past `main` on both boards.
2. Open Window > Show View > Expressions / Live Expressions. Add:

   ```text
   rawInputState
   localInputState
   canTxCount
   canTxCompleteCount
   canRxCount
   lastReceivedState
   localCanId
   canErrorCount
   canLastError
   canEsr
   canBusOff
   fatalError
   canInitStage
   canStarted
   canPclk1Hz
   canRxCallbackCount
   canIgnoredRxCount
   canTxFailureCount
   canTxAbortCount
   canTxAbortedCount
   canLastTxStatus
   canLastEsr
   lastTxCompleteTick
   lastReceivedCanId
   canRxTimedOut
   canPeers
   canPeerCount
   canPeerTableFullCount
   ```

3. Expect `fatalError=0`, `canInitStage=6`, `canStarted=1`, and
   `canPclk1Hz=16000000`. Compare `localCanId` in hexadecimal on both boards:
   the IDs must differ. A hash collision causes peer frames to be ignored.
4. If startup stops, break at `Error_Handler` and inspect the call stack,
   `hcan1.ErrorCode`, and `canInitStage`: 1 = clock/timing check, 2 = HAL init,
   3 = filter, 4 = notifications, 5 = start, 6 = fully running.
   Stage 0 means failure before the CAN initialization sequence.
5. In the peripheral/register view, CAN `MSR.INAK=0`, `MSR.SLAK=0`, and
   `BTR=0x011C0007` are expected in normal operation. This is prescaler 8,
   BS1 13 tq, BS2 2 tq, SJW 2 tq: 16 MHz / (8 * 16) = 125 kbps.

Use live reads while both CPUs run. Breakpoints perturb the bus, debounce,
timeout, and error counters. Resume promptly and reset both boards before
repeating a timing test. A stopped application cannot service its RX FIFO.

## 2. Isolate PA8 before investigating CAN

1. With Board A running and PA8 disconnected, expect `rawInputState=0` and
   `localInputState=0`. Connect PA8 to that board's own 3.3 V rail, preferably
   through about 1 kohm. Measure PA8 relative to that board's GND.
2. `rawInputState` must become 1 immediately when the main loop runs;
   `localInputState` must become 1 after at least 30 ms continuously HIGH.
   Live Expressions refresh may be too slow to show the 30 ms interval.
3. If raw stays 0, halt and inspect `GPIOA->IDR & 0x100`. If it is 0 with
   measured 3.3 V at the actual MCU PA8 pin, check pin identification, ground,
   soldering, and MCU power. Verify PA8 `MODER[17:16]=00` and
   `PUPDR[17:16]=10`. If IDR is 1 but raw stays 0 after resuming, check whether
   the CPU is in the main loop, `Error_Handler`, or a fault handler.
4. If raw changes but local does not, set a breakpoint in `ProcessInput`.
   Inspect `inputCandidate`, `inputCandidateTick`, and `uwTick`. Resume for
   more than 30 ms; repeated bounce resets the candidate timer. Check that
   SysTick is advancing. Do not single-step and expect wall-clock debounce.
5. After the debounced assignment, the next `CAN_App_Process()` call detects
   the change and sets `txPending`. Break in `CAN_SendState` in `can_app.c`
   only after confirming this path. The outgoing header must
   use `ExtId=localCanId`, IDE extended, RTR data, DLC 1, and `data[0]=1`.
   Repeat with the jumper removed: raw and then local must return to 0.

## 3. Distinguish queueing from successful CAN transmission

1. With both boards running, `canTxCount` and `canTxCompleteCount` should
   increase roughly ten times per second, plus input changes. A short lag of
   one frame is normal. Queue count increasing alone does not prove delivery.
2. If queued count increases but completion count stays zero, inspect
   `canLastError`, `canEsr`, `canLastEsr`, `canTxAbortCount`, and
   `canTxAbortedCount`. Pending frames are aborted after 100 ms and retried
   after a 100 ms backoff. Only one frame can be outstanding.
3. Decode live ESR with these debugger expressions:

   ```c
   (canEsr >> 4) & 7       /* LEC */
   (canEsr >> 16) & 255    /* TEC */
   (canEsr >> 24) & 255    /* REC */
   canEsr & 4             /* bus-off */
   ```

   LEC: 0 none, 1 stuff, 2 form, 3 ACK, 4 bit recessive, 5 bit dominant,
   6 CRC, 7 software marker. `canLastError & 0x20` latches an observed ACK
   error; `& 0x4` latches bus-off. History stays set until reset even after
   recovery. `canErrorCount` counts callbacks, API failures, and observed LEC
   transitions, not every failed wire attempt. Polling can miss brief errors.
   Missing ACK can leave the controller error-passive without bus-off, so
   `canBusOff=0` alone does not establish a healthy bus.
4. If queue count does not increase, inspect startup, `txPending`,
   `txOutstanding`, `txAbortRequested`, `txMailbox`, `hcan1.State`, and TSR
   TME bits. Completion uses the selected mailbox's RQCP/TXOK bits at bit
   offsets 0/1, 8/9, or 16/17. `canLastTxStatus` saves these before clearing.
   `canTxAbortCount` counts requests; `canTxAbortedCount` counts completed
   requests with no TXOK. A transmission can succeed while abort is requested.

## 4. Check the physical CAN path

1. Each STM32 needs a CAN transceiver: PB9 -> its TXD, PB8 <- its RXD. Connect
   transceiver CANH to CANH, CANL to CANL, and board GND to GND. MCU TX/RX
   pins are logic signals, not the differential CAN bus. Confirm transceiver
   U3 in the schematic is TCAN332D: pin 1 TXD, pin 4 RXD, pin 3 3.3 V,
   pin 2 GND, pin 6 CANL, pin 7 CANH. It has no standby/enable input.
2. Power off both boards and measure CANH-to-CANL resistance: two 120 ohm end
   terminators in parallel should give approximately 60 ohms. Check whether
   modules already contain termination before adding resistors. On your board,
   H5 closes the R15 termination path; enable it only at the bus ends.
3. With both boards powered and running, use a scope or analyzer to follow
   Board A PB9 -> A TXD -> differential CANH/CANL -> B RXD -> Board B PB8.
   Decode 125 kbps extended data frames, DLC 1, byte 0 = 0 or 1. Nominal bit
   time is 8 us. Verify ACK on the wire if completion never succeeds.
4. PB8/PB9 must both have `MODER=10` and AFRH nibbles 0/1 equal 9. The
   existing MSP code enables GPIOB/CAN1 clocks and configures both as AF9.
   Do not move pins or change the clock/bitrate to mask a wiring failure.

## 5. Isolate RX filter, interrupt, and validation

1. On Board B, watch `canRxCallbackCount`, `canIgnoredRxCount`, `canRxCount`.
   Confirm `lastReceivedCanId` equals A's `localCanId`.
2. If A's TX completes but B's callback count stays zero, set a breakpoint
   in `CAN1_RX0_IRQHandler`, then in `HAL_CAN_RxFifo0MsgPendingCallback`.
   A CAN ACK proves another controller received a valid frame electrically;
   it does not prove the acceptance filter or application accepted it.
3. Inspect FIFO0 `RF0R.FMP0`. If frames are pending but IRQ never executes,
   inspect NVIC CAN1_RX0 enable (IRQ 20), `PRIMASK`, `BASEPRI`, and IER
   FMPIE0 (bit 1). The ISR must call `HAL_CAN_IRQHandler(&hcan1)`.
4. If FIFO0 stays empty despite valid signaling at PB8, inspect filter bank 0:
   active, 32-bit mask mode, FIFO0, FR1=0x00000004 and FR2=0x00000006.
   This admits all extended data IDs and excludes standard/remote frames.
5. If callbacks increase but `canRxCount` does not, inspect the received
   header/data in the callback. Also check `canPeerTableFullCount` if the
   configured number of remote peers has been exceeded. It must be extended data, peer ID different
   from local ID, DLC exactly 1, payload 0 or 1. Invalid frames increment
   `canIgnoredRxCount`. Do not expect a standard-ID analyzer frame to pass.

## 6. Isolate PB5 and LED hardware

These single-peer steps apply to the default two-node configuration. With four
nodes, another active HIGH can legitimately keep PB5 HIGH after one peer sends LOW.

1. After a valid byte 1, B's `lastReceivedState=1`, `canRxCount` increments,
   `canRxTimedOut=0`, and the callback writes PB5 SET. A valid byte 0 writes
   RESET. PB5 is never set from its own board's PA8.
2. Inspect PB5 `MODER[11:10]=01`, `OTYPER[5]=0`, and `GPIOB->ODR & 0x20`.
   Measure PB5 voltage. If ODR is HIGH and the pad remains LOW, check loading,
   soldering, shorts, or external circuitry. If PB5 is 3.3 V but the LED is
   dark, inspect Q1, D2, R12/R13/R14, and their assembly. Your schematic routes
   PB5 through R13 to Q1 gate; Q1 sinks D2 current through R12. This is
   active-high. D1 is the separate power LED.
3. For an isolated output test, halt B after GPIO initialization and write
   `GPIOB->BSRR=0x20` using the debugger register view, then measure PB5.
   Write `GPIOB->BSRR=0x00200000` to clear it. This temporary test bypasses
   CAN and does not change firmware. Reset/resume both afterward.
4. The only PB5 writers in the application are startup LOW, valid RX state,
   peer-state/timeout output calculation, and fatal error LOW. If it turns on then goes off,
   watch `canRxTimedOut` and `lastRxTick` for missing heartbeats.

## Acceptance test

With the same updated ELF on both running boards: A PA8 LOW -> B PB5 LOW;
A PA8 HIGH -> B PB5 HIGH; remove jumper -> B PB5 LOW. Repeat in reverse and
with both inputs HIGH. On a healthy idle bench bus, expect about 30 ms debounce
plus transmission latency. After a disconnected/error condition, allow abort,
backoff, and hardware bus-off recovery before the newest state arrives.

Reference: [ST RM0351, bxCAN](https://www.st.com/resource/en/reference_manual/dm00083560.pdf),
[STM32L476 datasheet](https://www.st.com/resource/en/datasheet/DM00108832.pdf),
and [ST normal-mode CAN wiring guide](https://community.st.com/stm32-mcus-60/using-can-bxcan-in-normal-mode-with-stm32-microcontrollers-part-1-151183?fid=60&tid=151183).
