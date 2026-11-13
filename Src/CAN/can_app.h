#ifndef CAN_APP_H
#define CAN_APP_H

#include "stm32l4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Network size includes this board. Change 2 to 4 and rebuild the SAME
   firmware for all four boards. PB5 follows the OR of active remote inputs;
   this board's own PA8 never directly controls its own PB5. */
#ifndef CAN_APP_NODE_COUNT
#define CAN_APP_NODE_COUNT       2U
#endif
#if CAN_APP_NODE_COUNT < 2U
#error "CAN_APP_NODE_COUNT must include this board and at least one peer"
#endif
#define CAN_APP_MAX_PEERS        (CAN_APP_NODE_COUNT - 1U)

/* Preserve the existing 125 kbps hardware configuration in main.c. */
#define CAN_APP_BITRATE_HZ       125000U
#define CAN_APP_PRESCALER        8U
#define CAN_APP_BIT_TIME_TQ      16U
#define CAN_APP_PERIOD_MS        100U
#define CAN_APP_RX_TIMEOUT_MS    500U

typedef struct
{
    uint32_t canId;
    uint32_t lastRxTick;
    uint32_t rxCount;
    uint8_t inputState;
    uint8_t occupied;             /* Slot has been assigned an ID. */
    uint8_t active;               /* Valid heartbeat within the timeout. */
} CAN_AppPeer;

/* Call once after MX_CAN1_Init(), with the initial sampled PA8 state.
   Configures filtering/notifications, starts CAN, and queues the initial state.
   HAL failures are returned to main's Error_Handler. */
HAL_StatusTypeDef CAN_App_Init(CAN_HandleTypeDef *hcan, uint8_t initialState);

/* Nonblocking main-loop service. inputState must already be debounced.
   Handles state changes/heartbeats, TX results/retries, and peer timeouts. */
void CAN_App_Process(uint32_t now, uint8_t inputState);

/* Preserve CAN error diagnostics when main enters Error_Handler. */
void CAN_App_RecordError(CAN_HandleTypeDef *hcan);

/* Debugger-visible diagnostics. Observe these; the module owns their values. */
extern volatile uint32_t localCanId;
extern volatile uint32_t uidWords[3];
extern volatile uint32_t canPclk1Hz;
extern volatile uint8_t canInitStage;
extern volatile uint8_t canStarted;
extern volatile uint32_t canTxCount;
extern volatile uint32_t canTxCompleteCount;
extern volatile uint32_t canTxFailureCount;
extern volatile uint32_t canTxAbortCount;
extern volatile uint32_t canTxAbortedCount;
extern volatile uint32_t lastTxTick;
extern volatile uint32_t lastTxCompleteTick;
extern volatile uint32_t canLastTxStatus;
extern volatile uint32_t canRxCount;
extern volatile uint32_t canRxCallbackCount;
extern volatile uint32_t canIgnoredRxCount;
extern volatile uint8_t lastReceivedState;
extern volatile uint32_t lastReceivedCanId;
extern volatile uint32_t lastRxTick;
extern volatile uint8_t canRxTimedOut; /* No active remote peers. */
extern volatile uint32_t canErrorCount;
extern volatile uint32_t canLastError;
extern volatile uint32_t canLastEsr;
extern volatile uint32_t canEsr;
extern volatile uint8_t canBusOff;
extern volatile uint32_t canBusOffCount;

/* Independent per-peer state/timeout diagnostics, in arrival order.
   Unknown IDs cannot evict a live peer when the table is full. A timed-out
   slot can be reused; first valid arrival wins, not an authenticated pairing. */
extern volatile CAN_AppPeer canPeers[CAN_APP_MAX_PEERS];
extern volatile uint32_t canPeerCount; /* Currently active peers. */
extern volatile uint32_t canPeerTableFullCount;

#ifdef __cplusplus
}
#endif

#endif /* CAN_APP_H */
