#!/usr/bin/env python3
"""Compile the actual can_app.c plus main.c input functions against a native HAL mock.
Requires Python 3 and a native C compiler (cc). Does not exercise hardware.
"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'main.c').read_text()
def region(name):
    return source.split(f'/* USER CODE BEGIN {name} */', 1)[1].split(f'/* USER CODE END {name} */', 1)[0]

mock = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define HAL_OK 0
typedef int HAL_StatusTypeDef;
#define HAL_ERROR 1
#define ENABLE 1U
#define DISABLE 0U
#define CAN_ID_EXT 4U
#define CAN_RTR_DATA 0U
#define CAN_RTR_REMOTE 2U
#define CAN_RX_FIFO0 0U
#define CAN_FILTERMODE_IDMASK 0U
#define CAN_FILTERSCALE_32BIT 1U
#define CAN_IT_RX_FIFO0_MSG_PENDING 1U
#define CAN_IT_RX_FIFO0_OVERRUN 2U
#define CAN_IT_ERROR_WARNING 4U
#define CAN_IT_ERROR_PASSIVE 8U
#define CAN_IT_BUSOFF 16U
#define CAN_IT_ERROR 32U
#define CAN_ESR_BOFF 4U
#define CAN_ESR_EWGF 1U
#define CAN_ESR_EPVF 2U
#define CAN_ESR_LEC 0x70U
#define CAN_ESR_LEC_Pos 4U
#define HAL_CAN_ERROR_EWG 1U
#define HAL_CAN_ERROR_EPV 2U
#define HAL_CAN_ERROR_BOF 4U
#define HAL_CAN_ERROR_STF 8U
#define HAL_CAN_ERROR_FOR 16U
#define HAL_CAN_ERROR_ACK 32U
#define HAL_CAN_ERROR_BR 64U
#define HAL_CAN_ERROR_BD 128U
#define HAL_CAN_ERROR_CRC 256U
#define HAL_CAN_ERROR_TX_ALST0 0x800U
#define HAL_CAN_ERROR_TX_TERR0 0x1000U
#define CAN_TX_MAILBOX0 1U
#define CAN_TX_MAILBOX1 2U
#define CAN_TSR_RQCP0 1U
#define CAN_TSR_TXOK0 2U
#define CAN_TSR_ALST0 4U
#define CAN_TSR_TERR0 8U
#define CAN_TSR_ABRQ0 128U
#define GPIO_PIN_8 256U
#define GPIO_PIN_5 32U
#define GPIO_PIN_SET 1U
#define GPIO_PIN_RESET 0U
#define GPIOA 0U
#define GPIOB 1U
typedef struct { uint32_t ESR, TSR; } Registers;
static Registers registers;
#define CAN1 (&registers)
typedef struct { Registers *Instance; } CAN_HandleTypeDef;
static CAN_HandleTypeDef hcan1 = { CAN1 };
typedef struct {
 uint32_t FilterBank,FilterMode,FilterScale,FilterIdHigh,FilterIdLow;
 uint32_t FilterMaskIdHigh,FilterMaskIdLow,FilterFIFOAssignment;
 uint32_t FilterActivation,SlaveStartFilterBank;
} CAN_FilterTypeDef;
typedef struct { uint32_t ExtId,IDE,RTR,DLC,TransmitGlobalTime; } CAN_TxHeaderTypeDef;
typedef struct { uint32_t ExtId,IDE,RTR,DLC; } CAN_RxHeaderTypeDef;
static uint32_t tick, input, led, irqmask, pendingMailbox, freeMailboxes;
static uint32_t addCalls, abortCalls, mockError, selectedMailbox;
static int addStatus;
static CAN_TxHeaderTypeDef sentHeader;
static uint8_t sentData;
static CAN_FilterTypeDef configuredFilter;
static uint32_t notifications, started;
static int filterStatus, notificationStatus, startStatus;
static uint32_t fakeUid[3] = {0x11223344,0x55667788,0x12345678};
static uint32_t HAL_GetUIDw0(void) { return fakeUid[0]; }
static uint32_t HAL_GetUIDw1(void) { return fakeUid[1]; }
static uint32_t HAL_GetUIDw2(void) { return fakeUid[2]; }
static uint32_t HAL_GetTick(void) { return tick; }
static uint32_t HAL_GPIO_ReadPin(uint32_t port, uint32_t pin) {
 assert(port==GPIOA && pin==GPIO_PIN_8); return input;
}
static void HAL_GPIO_WritePin(uint32_t port, uint32_t pin, uint32_t value) {
 assert(port==GPIOB && pin==GPIO_PIN_5); led=value;
}
static uint32_t __get_PRIMASK(void) { return irqmask; }
static void __disable_irq(void) { irqmask=1; }
static void __set_PRIMASK(uint32_t v) { irqmask=v; }
static int HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h, CAN_FilterTypeDef *f) {
 assert(h==&hcan1); configuredFilter=*f; return filterStatus;
}
static int HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h, uint32_t n) {
 assert(h==&hcan1); notifications=n; return notificationStatus;
}
static int HAL_CAN_Start(CAN_HandleTypeDef *h) {
 assert(h==&hcan1); started=1; return startStatus;
}
static uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *h) {
 assert(h==&hcan1); return mockError;
}
static uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *h, uint32_t m) {
 assert(h==&hcan1 && m==selectedMailbox); return pendingMailbox;
}
static uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *h) {
 assert(h==&hcan1); return freeMailboxes;
}
/* Model TSR W1C/abort writes; ABRQ is asynchronous, just like real hardware. */
static void write_tsr(uint32_t *reg, uint32_t bits) {
 assert(reg==&registers.TSR);
 for (uint32_t shift=0; shift<=16; shift+=8) {
   if (bits & (CAN_TSR_RQCP0 << shift)) *reg &= ~(15U << shift);
   if (bits & (CAN_TSR_ABRQ0 << shift)) ++abortCalls;
 }
}
#define WRITE_REG(reg, bits) write_tsr(&(reg), (bits))
static int HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h, CAN_TxHeaderTypeDef *hdr,
                               uint8_t *data, uint32_t *mailbox) {
 assert(h==&hcan1); ++addCalls; sentHeader=*hdr; sentData=data[0];
 if (addStatus==HAL_OK) {
   *mailbox=selectedMailbox; pendingMailbox=1U;
   uint32_t shift=(selectedMailbox==1) ? 0 : (selectedMailbox==2) ? 8 : 16;
   registers.TSR &= ~(15U << shift);
 }
 return addStatus;
}
static struct { CAN_RxHeaderTypeDef header; uint8_t data; } frames[8];
static uint32_t rxHead,rxTail;
static uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *h, uint32_t f) {
 assert(h==&hcan1 && f==CAN_RX_FIFO0); return rxTail-rxHead;
}
static int HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t f,
                               CAN_RxHeaderTypeDef *hdr, uint8_t *data) {
 assert(h==&hcan1 && f==CAN_RX_FIFO0 && rxHead<rxTail);
 *hdr=frames[rxHead].header; data[0]=frames[rxHead++].data; return HAL_OK;
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *h);
'''
tests = r'''
static void reset(void) {
 tick=input=led=irqmask=pendingMailbox=0; freeMailboxes=3;
 addCalls=abortCalls=mockError=0; addStatus=HAL_OK; selectedMailbox=1;
 registers.ESR=registers.TSR=0; rxHead=rxTail=0;
 notifications=started=0; filterStatus=notificationStatus=startStatus=HAL_OK;
 localInputState=rawInputState=lastReceivedState=canBusOff=0;
 canRxTimedOut=1; localCanId=0x12345; canPclk1Hz=0;
 canTxCount=canRxCount=canIgnoredRxCount=canErrorCount=canLastError=0;
 canTxCompleteCount=canTxAbortedCount=lastTxCompleteTick=canLastTxStatus=0;
 canInitStage=canRxCallbackCount=0; canStarted=1; canHandle=&hcan1;
 memset((void *)canPeers,0,sizeof(canPeers));
 canPeerCount=canPeerTableFullCount=0; txInputState=0;
 canLastEsr=canEsr=canTxFailureCount=canTxAbortCount=canBusOffCount=0;
 lastRxTick=lastTxTick=lastReceivedCanId=0; fatalError=0;
 inputCandidate=0; inputCandidateTick=0; txPending=1;
 txOutstanding=txAbortRequested=0;
 txMailbox=txQueuedTick=txAttemptTick=txRetryDelay=lastPolledLec=0;
}
static void complete(uint32_t result) {
 uint32_t shift=(selectedMailbox==1) ? 0 : (selectedMailbox==2) ? 8 : 16;
 registers.TSR=(CAN_TSR_RQCP0 | result) << shift;
 pendingMailbox=0;
}
static void receive(uint32_t ide,uint32_t rtr,uint32_t id,uint32_t dlc,uint8_t data) {
 assert(rxTail<8); frames[rxTail].header=(CAN_RxHeaderTypeDef){id,ide,rtr,dlc};
 frames[rxTail++].data=data; HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
}
static void test_debounce(void) {
 reset(); txPending=0;
 input=1; tick=1; ProcessInput();
 tick=20; input=0; ProcessInput();
 tick=21; input=1; ProcessInput();
 tick=50; ProcessInput(); assert(localInputState==0 && txPending==0);
 tick=51; ProcessInput(); assert(localInputState==1 && led==0);
 txPending=0; input=0; tick=52; ProcessInput();
 tick=81; ProcessInput(); assert(localInputState==1);
 tick=82; ProcessInput(); assert(localInputState==0);
 /* Unsigned tick rollover must not defeat debounce. */
 reset(); tick=UINT32_MAX-10U; input=1; ProcessInput();
 tick=18; ProcessInput(); assert(localInputState==0);
 tick=19; ProcessInput(); assert(localInputState==1);
}
static void test_receive_and_timeout(void) {
 reset(); tick=100;
 receive(CAN_ID_EXT,0,0x23456,1,1);
 assert(led==1 && canRxCount==1 && lastRxTick==100 && canRxTimedOut==0);
 assert(localInputState==0 && canTxCount==0 && lastReceivedCanId==0x23456);
 receive(CAN_ID_EXT,0,localCanId,1,0);
 receive(0,0,0x23456,1,0);
 receive(CAN_ID_EXT,CAN_RTR_REMOTE,0x23456,1,0);
 receive(CAN_ID_EXT,0,0x23456,0,0);
 receive(CAN_ID_EXT,0,0x23456,1,2);
 assert(led==1 && canRxCount==1 && canIgnoredRxCount==5 && lastRxTick==100);
 tick=599; CAN_UpdatePeerOutput(); assert(led==1);
 tick=600; CAN_UpdatePeerOutput(); assert(led==0 && canRxTimedOut==1 && irqmask==0);
 receive(CAN_ID_EXT,0,0x23456,1,1); assert(led==1 && canRxTimedOut==0);
 /* New frame after timeout recovers in either direction. */
 rxHead=rxTail=0; receive(CAN_ID_EXT,0,0x23456,1,0);
 assert(led==0 && lastReceivedState==0 && canRxCount==3);
 reset(); tick=UINT32_MAX-100U; receive(CAN_ID_EXT,0,1,1,1);
 tick=398; CAN_UpdatePeerOutput(); assert(led==1);
 tick=399; irqmask=1; CAN_UpdatePeerOutput(); assert(led==0 && irqmask==1);
}
static void test_transmit(void) {
 reset(); CAN_App_Process(tick,localInputState);
 assert(canTxCount==1 && sentData==0 && sentHeader.DLC==1);
 assert(sentHeader.IDE==CAN_ID_EXT && sentHeader.ExtId==localCanId && sentHeader.RTR==0);
 assert(canTxCompleteCount==0); complete(CAN_TSR_TXOK0);
 for(tick=1;tick<100;++tick) { ProcessInput(); CAN_App_Process(tick,localInputState); }
 assert(canTxCount==1 && canTxCompleteCount==1);
 ProcessInput(); CAN_App_Process(tick,localInputState); assert(canTxCount==2);
 /* A debounced edge schedules an immediate update between heartbeats. */
 complete(CAN_TSR_TXOK0); input=1; tick=101; ProcessInput();
 tick=131; ProcessInput(); CAN_App_Process(tick,localInputState);
 assert(canTxCount==3 && sentData==1 && led==0);
 /* Full mailboxes never call AddTxMessage or block the input processing. */
 reset(); freeMailboxes=0;
 for(tick=0;tick<50;++tick) CAN_App_Process(tick,localInputState);
 assert(addCalls==0 && txPending==1);
 freeMailboxes=3; tick=50; CAN_App_Process(tick,localInputState); assert(canTxCount==1);
 /* HAL failure backs off 100ms; no per-loop frame flood. */
 reset(); addStatus=HAL_ERROR;
 for(tick=0;tick<100;++tick) CAN_App_Process(tick,localInputState);
 assert(addCalls==1 && canTxFailureCount==1);
 tick=100; addStatus=HAL_OK; CAN_App_Process(tick,localInputState); assert(canTxCount==1);
 /* No ACK: only one outstanding frame, one asynchronous abort request. */
 reset(); CAN_App_Process(0,localInputState);
 for(tick=1;tick<250;++tick) { ProcessInput(); CAN_App_Process(tick,localInputState); }
 assert(canTxCount==1 && abortCalls==1 && canTxAbortCount==1);
 complete(0); tick=250; CAN_App_Process(tick,localInputState); assert(canTxCount==1);
 assert(canTxCompleteCount==0 && canTxAbortedCount==1);
 tick=349; CAN_App_Process(tick,localInputState); assert(canTxCount==1);
 tick=350; CAN_App_Process(tick,localInputState); assert(canTxCount==2);
}
static void test_busoff_and_identifier(void) {
 reset(); canStarted=0; assert(CAN_App_Init(&hcan1,0)==HAL_OK); assert(started && notifications && canStarted && canInitStage==6);
 assert(configuredFilter.FilterIdLow==4 && configuredFilter.FilterMaskIdLow==6);
 assert(configuredFilter.FilterMaskIdHigh==0);
 uint32_t id1=CAN_DeriveIdentifier(), id2=CAN_DeriveIdentifier();
 assert(id1==id2 && id1<=0x1FFFFFFFU);
 fakeUid[2]^=1; assert(CAN_DeriveIdentifier()!=id1); fakeUid[2]^=1;
 CAN_App_Process(0,localInputState); registers.ESR=CAN_ESR_BOFF; mockError=0x10;
 HAL_CAN_ErrorCallback(&hcan1); assert(canErrorCount==1 && canLastError==0x14);
 for(tick=1;tick<100;++tick) CAN_App_Process(tick,localInputState);
 assert(canBusOff==1 && canBusOffCount==1 && abortCalls==1 && canTxCount==1);
 registers.ESR=0; complete(0); tick=100; CAN_App_Process(tick,localInputState);
 assert(canBusOff==0 && canTxCount==1);
 tick=199; CAN_App_Process(tick,localInputState); assert(canTxCount==1);
 tick=200; CAN_App_Process(tick,localInputState); assert(canTxCount==2);
}
static void test_completion_and_error_diagnostics(void) {
 /* Each mailbox uses a different bit group; only RQCP + TXOK is success. */
 for (uint32_t m=1; m<=4; m*=2) {
   reset(); selectedMailbox=m; CAN_App_Process(0,localInputState);
   tick=1; CAN_App_Process(tick,localInputState); assert(canTxCompleteCount==0);
   complete(CAN_TSR_TXOK0); tick=2; CAN_App_Process(tick,localInputState);
   assert(canTxCompleteCount==1 && lastTxCompleteTick==2 && txOutstanding==0);
   tick=3; CAN_App_Process(tick,localInputState); assert(canTxCompleteCount==1);
   reset(); selectedMailbox=m; CAN_App_Process(0,localInputState);
   complete(CAN_TSR_TERR0); tick=1; CAN_App_Process(tick,localInputState);
   assert(canTxCompleteCount==0 && canTxFailureCount==1 && txPending==1);
   uint32_t shift=(m==1) ? 0 : (m==2) ? 2 : 4;
   assert(canLastError==(HAL_CAN_ERROR_TX_TERR0 << shift));
   tick=100; CAN_App_Process(tick,localInputState); assert(canTxCount==1);
   localInputState=1; tick=101; CAN_App_Process(tick,localInputState);
   assert(canTxCount==2 && sentData==1);
   reset(); selectedMailbox=m; CAN_App_Process(0,localInputState);
   complete(CAN_TSR_ALST0); tick=1; CAN_App_Process(tick,localInputState);
   assert(canLastError==(HAL_CAN_ERROR_TX_ALST0 << shift) && canTxCompleteCount==0);
 }
 /* Success racing an abort must be counted, not mistaken for an abort. */
 reset(); CAN_App_Process(0,localInputState); tick=100; CAN_App_Process(tick,localInputState);
 assert(abortCalls==1); complete(CAN_TSR_TXOK0); tick=101; CAN_App_Process(tick,localInputState);
 assert(canTxCompleteCount==1 && canTxAbortedCount==0);
 /* Missing ACK is visible even without LEC interrupts or warning/passive. */
 reset(); CAN_App_Process(0,localInputState); registers.ESR=3U << CAN_ESR_LEC_Pos;
 tick=1; CAN_App_Process(tick,localInputState);
 assert(canLastError==HAL_CAN_ERROR_ACK && canLastEsr==0x30 && canErrorCount==1);
 tick=2; CAN_App_Process(tick,localInputState); assert(canErrorCount==1);
 registers.ESR=2U << CAN_ESR_LEC_Pos; tick=3; CAN_App_Process(tick,localInputState);
 assert(canLastError==(HAL_CAN_ERROR_ACK | HAL_CAN_ERROR_FOR) && canErrorCount==2);
 /* Clear live ESR on recovery; retain the error history. */
 registers.ESR=0; tick=4; CAN_App_Process(tick,localInputState);
 assert(canEsr==0 && canLastError==(HAL_CAN_ERROR_ACK | HAL_CAN_ERROR_FOR));
 reset(); receive(CAN_ID_EXT,0,localCanId,1,1);
 assert(canRxCallbackCount==1 && canIgnoredRxCount==1 && canRxCount==0);
 receive(CAN_ID_EXT,0,1,1,1);
 assert(canRxCallbackCount==2 && canRxCount==1 && lastReceivedState==1 && led==1);
}

static void test_public_api(void) {
 reset(); canStarted=0; canHandle=NULL;
 CAN_App_Process(0,1); assert(addCalls==0 && led==0);
 assert(CAN_App_Init(NULL,0)==HAL_ERROR);
 CAN_HandleTypeDef wrong={NULL};
 assert(CAN_App_Init(&wrong,0)==HAL_ERROR);
 reset(); canStarted=0; filterStatus=HAL_ERROR;
 assert(CAN_App_Init(&hcan1,0)==HAL_ERROR && canInitStage==3 && !canStarted);
 reset(); canStarted=0; notificationStatus=HAL_ERROR;
 assert(CAN_App_Init(&hcan1,0)==HAL_ERROR && canInitStage==4 && !canStarted);
 reset(); canStarted=0; startStatus=HAL_ERROR;
 assert(CAN_App_Init(&hcan1,0)==HAL_ERROR && canInitStage==5 && !canStarted);
 reset(); canStarted=0; tick=25;
 assert(CAN_App_Init(&hcan1,1)==HAL_OK);
 assert(canStarted && canInitStage==6 && canTxCount==1 && sentData==1);
 assert(localCanId<=0x1FFFFFFFU && sentHeader.ExtId==localCanId);
 assert(CAN_App_Init(&hcan1,0)==HAL_ERROR); /* Init is once per boot. */
 complete(CAN_TSR_TXOK0); tick=26; CAN_App_Process(tick,1);
 assert(canTxCompleteCount==1 && canTxCount==1);
 tick=27; CAN_App_Process(tick,0); /* Falling edge before heartbeat. */
 assert(canTxCount==2 && sentData==0);
 reset(); mockError=HAL_CAN_ERROR_ACK;
 CAN_App_RecordError(&hcan1); CAN_App_RecordError(NULL); CAN_App_RecordError(&wrong);
 assert(canLastError==HAL_CAN_ERROR_ACK && canErrorCount==1);
 CAN_HandleTypeDef other={CAN1};
 HAL_CAN_RxFifo0MsgPendingCallback(&other); HAL_CAN_ErrorCallback(&other);
 assert(canRxCallbackCount==0 && canErrorCount==1);
}

static void test_peer_capacity_and_timeouts(void) {
 reset(); tick=10;
 for (uint32_t i=0; i<CAN_APP_MAX_PEERS; ++i)
   receive(CAN_ID_EXT,0,0x100+i,1,(i==0) ? 1 : 0);
 assert(canPeerCount==CAN_APP_MAX_PEERS && canRxCount==CAN_APP_MAX_PEERS);
 assert(led==1 && canRxTimedOut==0);
 /* A full table never replaces a currently live sender. */
 receive(CAN_ID_EXT,0,0x900,1,0);
 assert(canPeerTableFullCount==1 && canIgnoredRxCount==1);
 assert(canPeers[0].canId==0x100 && canRxCount==CAN_APP_MAX_PEERS && led==1);
 tick=509; CAN_UpdatePeerOutput(); assert(led==1);
 tick=510; CAN_UpdatePeerOutput();
 assert(led==0 && canPeerCount==0 && canRxTimedOut==1);
 /* Expired slots can be reused; stale HIGH cannot keep the LED lit. */
 rxHead=rxTail=0; receive(CAN_ID_EXT,0,0x900,1,0);
 assert(canPeerCount==1 && canPeers[0].canId==0x900 && canPeers[0].rxCount==1);
 assert(canRxTimedOut==0 && led==0);
 receive(CAN_ID_EXT,0,0x900,1,1);
 assert(canPeers[0].rxCount==2 && led==1);
 receive(CAN_ID_EXT,0,0x900,1,0); assert(led==0);
 /* A timed-out HIGH must not revive when the tick returns to the same value
    after an entire 32-bit counter cycle without another received frame. */
 reset(); tick=100; receive(CAN_ID_EXT,0,0x101,1,1);
 tick=600; CAN_UpdatePeerOutput(); assert(led==0 && canPeerCount==0);
 tick=100; CAN_UpdatePeerOutput(); assert(led==0 && canPeerCount==0);

#if CAN_APP_NODE_COUNT == 4
 /* A third/fourth board's LOW must not overwrite another board's HIGH. */
 reset(); tick=0; receive(CAN_ID_EXT,0,0x101,1,1);
 tick=100; receive(CAN_ID_EXT,0,0x102,1,0);
 tick=200; receive(CAN_ID_EXT,0,0x103,1,0);
 assert(led==1 && canPeerCount==3 && lastReceivedState==0);
 /* A healthy LOW peer cannot mask the HIGH peer going offline. */
 tick=499; CAN_UpdatePeerOutput(); assert(led==1);
 tick=500; receive(CAN_ID_EXT,0,0x102,1,0);
 assert(led==0 && canPeers[0].active==0 && canPeerCount==2 && canRxTimedOut==0);
 tick=501; receive(CAN_ID_EXT,0,0x103,1,1); assert(led==1);
 tick=502; receive(CAN_ID_EXT,0,0x103,1,0); assert(led==0);
 /* Recover the old peer, preserving its slot and independent receive count. */
 tick=503; receive(CAN_ID_EXT,0,0x101,1,1);
 assert(canPeerCount==3 && canPeers[0].rxCount==2 && led==1);

 /* Per-peer expiry also tolerates the 32-bit HAL tick wrapping. */
 reset(); tick=UINT32_MAX-100;
 receive(CAN_ID_EXT,0,0x101,1,1);
 tick=20; receive(CAN_ID_EXT,0,0x102,1,0);
 tick=398; CAN_UpdatePeerOutput(); assert(led==1);
 tick=399; CAN_UpdatePeerOutput();
 assert(led==0 && canPeerCount==1 && canRxTimedOut==0);
#endif
}

int main(void) {
 test_debounce(); test_receive_and_timeout(); test_transmit(); test_busoff_and_identifier();
 test_completion_and_error_diagnostics(); test_public_api(); test_peer_capacity_and_timeouts();
 puts("PASS: debounce/edges/rollover, RX validation/timeout/output, TX scheduling/backoff, TXOK vs abort/error on all mailboxes, abort race, ACK error latching, bus-off recovery, UID/filter/start diagnostics, public API, peer capacity/timeout/recovery/OR policy");
}
'''
with tempfile.TemporaryDirectory(prefix='stm32-can-tests-') as td:
    p = Path(td)
    # The harness declares HAL mocks before including the real module/header.
    (p / 'stm32l4xx_hal.h').write_text('/* HAL mocks are declared by test.c. */\n')
    harness = (mock + '#include "CAN/can_app.c"\n' + region('PD') + region('PV')
               + region('PFP') + region('4') + tests)
    (p / 'test.c').write_text(harness)
    for nodes in (2, 4):
        executable = p / f'test_{nodes}_nodes'
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                        f'-DCAN_APP_NODE_COUNT={nodes}', '-I' + str(p), '-I' + str(root),
                        str(p / 'test.c'), '-o', str(executable)], check=True)
        print(f'{nodes}-node configuration:', flush=True)
        subprocess.run([str(executable)], check=True)
