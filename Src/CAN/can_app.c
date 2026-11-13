/* CAN1 input-state protocol and diagnostics for STM32L476.
 * Hardware initialization stays in main.c / the CubeMX MSP and IRQ files.
 * Send: extended data frame, ID from MCU UID, DLC=1, payload 0 or 1.
 * Receive: track each peer independently; PB5 is HIGH if any live peer is HIGH.
 */
#include "can_app.h"
#include <stddef.h>

/* Private retry policy. All time comparisons tolerate HAL tick rollover. */
#define CAN_MAILBOX_RETRY_MS    10U
#define CAN_ERROR_RETRY_MS      100U
#define CAN_TX_MAX_AGE_MS       100U

/* Public, volatile symbols for CubeIDE Expressions / Live Expressions. */
volatile uint8_t lastReceivedState;
volatile uint8_t canRxTimedOut = 1U;
volatile uint8_t canBusOff;
volatile uint32_t localCanId;
volatile uint32_t uidWords[3];
volatile uint32_t canPclk1Hz;
volatile uint32_t canTxCount;          /* Frames accepted into a mailbox, not ACKs. */
volatile uint32_t canTxCompleteCount;  /* RQCP + TXOK: successful normal-mode TX. */
volatile uint32_t canTxAbortedCount;   /* Completed aborts without TXOK. */
volatile uint32_t lastTxCompleteTick;
volatile uint32_t canLastTxStatus;     /* TSR snapshot before clearing RQCP. */
volatile uint8_t canInitStage;         /* 1 timing, 2 init, 3 filter, 4 IRQ, 5 start, 6 running. */
volatile uint8_t canStarted;
volatile uint32_t canRxCallbackCount;  /* Includes callbacks with rejected frames. */
volatile uint32_t canRxCount;          /* Valid peer state frames only. */
volatile uint32_t canIgnoredRxCount;
volatile uint32_t canErrorCount;
volatile uint32_t canLastError;        /* Latched HAL + polled ESR error bits. */
volatile uint32_t canLastEsr;          /* ESR snapshot at the last error. */
volatile uint32_t canEsr;              /* Current ESR: TEC, REC, LEC, BOFF. */
volatile uint32_t canTxFailureCount;
volatile uint32_t canTxAbortCount;
volatile uint32_t canBusOffCount;
volatile uint32_t lastRxTick;
volatile uint32_t lastTxTick;
volatile uint32_t lastReceivedCanId;

volatile CAN_AppPeer canPeers[CAN_APP_MAX_PEERS];
volatile uint32_t canPeerCount;
volatile uint32_t canPeerTableFullCount;

/* Main-loop-owned transmit state; RX never queues another transmission. */
static CAN_HandleTypeDef *canHandle;
static uint8_t txInputState;
static uint8_t txPending = 1U;
static uint8_t txOutstanding;
static uint8_t txAbortRequested;
static uint32_t txMailbox;
static uint32_t txQueuedTick;
static uint32_t txAttemptTick;
static uint32_t txRetryDelay;
static uint32_t lastPolledLec;

static uint32_t CAN_DeriveIdentifier(void);
static void CAN_SendState(uint32_t now);
static void CAN_AbortOutstanding(void);
static void CAN_CheckTxCompletion(uint32_t now);
static void CAN_RecordError(CAN_HandleTypeDef *hcan, uint32_t error);
static volatile CAN_AppPeer *CAN_FindPeer(uint32_t id, uint32_t now);
static void CAN_UpdatePeerOutput(void);

/* ----------------------------- Public API -------------------------------- */

HAL_StatusTypeDef CAN_App_Init(CAN_HandleTypeDef *hcan, uint8_t initialState)
{
  if ((hcan == NULL) || (hcan->Instance != CAN1) || (canStarted != 0U))
  {
    return HAL_ERROR;
  }
  canHandle = hcan;
  localCanId = CAN_DeriveIdentifier();
  txInputState = (initialState != 0U) ? 1U : 0U;
  lastRxTick = HAL_GetTick();
  CAN_FilterTypeDef filter = {0};
  filter.FilterBank = 0U;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  /* 32-bit filter register: identifier[31:3], IDE[2], RTR[1].
     Match IDE=1 and RTR=0; all 29 identifier bits are don't-care. */
  filter.FilterIdHigh = 0U;
  filter.FilterIdLow = CAN_ID_EXT;
  filter.FilterMaskIdHigh = 0U;
  filter.FilterMaskIdLow = CAN_ID_EXT | CAN_RTR_REMOTE;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;

  canInitStage = 3U;
  if (HAL_CAN_ConfigFilter(canHandle, &filter) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* Arm reception before leaving initialization mode to avoid a startup gap.
     Warning/passive/bus-off errors are enabled; omit per-retry LEC interrupts
     so a missing ACK cannot create an interrupt storm. ESR remains readable. */
  /* TX completion is polled below. Keep TMEIE disabled: only the main loop
     owns RQCP/TXOK, so the HAL IRQ handler cannot clear them first. */
  canInitStage = 4U;
  if (HAL_CAN_ActivateNotification(canHandle,
        CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN |
        CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE |
        CAN_IT_BUSOFF | CAN_IT_ERROR) != HAL_OK)
  {
    return HAL_ERROR;
  }
  canInitStage = 5U;
  if (HAL_CAN_Start(canHandle) != HAL_OK)
  {
    return HAL_ERROR;
  }
  canStarted = 1U;
  canInitStage = 6U;
  CAN_SendState(HAL_GetTick());
  return HAL_OK;
}

void CAN_App_Process(uint32_t now, uint8_t inputState)
{
  if ((canHandle == NULL) || (canStarted == 0U))
  {
    return;
  }
  uint8_t state = (inputState != 0U) ? 1U : 0U;
  if (state != txInputState)
  {
    txInputState = state;
    txPending = 1U; /* Coalesce changes while busy; always send the newest. */
  }
  if ((uint32_t)(now - lastTxTick) >= CAN_APP_PERIOD_MS)
  {
    txPending = 1U;
  }
  CAN_SendState(now);
  CAN_UpdatePeerOutput();
}

void CAN_App_RecordError(CAN_HandleTypeDef *hcan)
{
  if ((hcan != NULL) && (hcan->Instance == CAN1))
  {
    CAN_RecordError(hcan, 0U);
  }
}

/* ------------------------ CAN protocol and transport ---------------------- */

static uint32_t CAN_DeriveIdentifier(void)
{
  uint32_t hash = 2166136261UL;
  uidWords[0] = HAL_GetUIDw0();
  uidWords[1] = HAL_GetUIDw1();
  uidWords[2] = HAL_GetUIDw2();

  /* FNV-1a over all 96 UID bits in a fixed byte order, then avalanche. */
  for (uint32_t word = 0U; word < 3U; ++word)
  {
    for (uint32_t byte = 0U; byte < 4U; ++byte)
    {
      hash ^= (uidWords[word] >> (8U * byte)) & 0xFFU;
      hash *= 16777619UL;
    }
  }
  hash ^= hash >> 16;
  hash *= 0x85EBCA6BUL;
  hash ^= hash >> 13;
  hash *= 0xC2B2AE35UL;
  hash ^= hash >> 16;
  /* A 96 -> 29 bit mapping can collide: compare localCanId on both boards. */
  return hash & 0x1FFFFFFFUL;
}

static void CAN_AbortOutstanding(void)
{
  if ((txOutstanding != 0U) && (txAbortRequested == 0U))
  {
    uint32_t shift = (txMailbox == CAN_TX_MAILBOX0) ? 0U :
                     (txMailbox == CAN_TX_MAILBOX1) ? 8U : 16U;
    /* Asynchronous abort, writing ABRQ only. The HAL's TSR read-modify-write
       can write back a newly observed RQCP=1 and erase the TXOK evidence.
       A transmission that wins the abort race is still counted as success. */
    txAbortRequested = 1U;
    WRITE_REG(canHandle->Instance->TSR, CAN_TSR_ABRQ0 << shift);
    ++canTxAbortCount;
  }
}

static void CAN_CheckTxCompletion(uint32_t now)
{
  if (txOutstanding == 0U)
  {
    return;
  }
  uint32_t shift = (txMailbox == CAN_TX_MAILBOX0) ? 0U :
                   (txMailbox == CAN_TX_MAILBOX1) ? 8U : 16U;
  uint32_t status = canHandle->Instance->TSR;
  uint32_t result = status >> shift;
  if ((result & CAN_TSR_RQCP0) == 0U)
  {
    return;
  }
  canLastTxStatus = status;
  /* RQCP is write-one-to-clear; clear only this mailbox after saving TXOK. */
  WRITE_REG(canHandle->Instance->TSR, CAN_TSR_RQCP0 << shift);
  if ((result & CAN_TSR_TXOK0) != 0U)
  {
    ++canTxCompleteCount;
    lastTxCompleteTick = now;
  }
  else
  {
    if (txAbortRequested != 0U)
    {
      ++canTxAbortedCount;
    }
    else
    {
      ++canTxFailureCount;
      uint32_t error = 0U;
      if ((result & CAN_TSR_TERR0) != 0U)
      {
        error |= HAL_CAN_ERROR_TX_TERR0 << (shift / 4U);
      }
      if ((result & CAN_TSR_ALST0) != 0U)
      {
        error |= HAL_CAN_ERROR_TX_ALST0 << (shift / 4U);
      }
      CAN_RecordError(canHandle, error);
    }
    /* A freed mailbox alone is not delivery. Retry the latest input. */
    txPending = 1U;
    txAttemptTick = now;
    txRetryDelay = CAN_ERROR_RETRY_MS;
  }
  txOutstanding = 0U;
  txAbortRequested = 0U;
}

static void CAN_SendState(uint32_t now)
{
  CAN_TxHeaderTypeDef header = {0};
  uint8_t data[8] = {0}; /* HAL reads both data registers even with DLC=1. */
  canEsr = canHandle->Instance->ESR;
  /* Capture ACK/bit errors without enabling an interrupt for every retry.
     Count observed LEC changes, not individual failed wire attempts. */
  uint32_t lec = (canEsr & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos;
  if ((lec != 0U) && (lec < 7U) && (lec != lastPolledLec))
  {
    CAN_RecordError(canHandle, 0U);
  }
  lastPolledLec = lec;
  CAN_CheckTxCompletion(now);

  if ((canEsr & CAN_ESR_BOFF) != 0U)
  {
    if (canBusOff == 0U)
    {
      canBusOff = 1U;
      ++canBusOffCount;
      CAN_AbortOutstanding();
    }
    txPending = 1U;
    txAttemptTick = now;
    txRetryDelay = CAN_ERROR_RETRY_MS;
    /* ABOM recovers in hardware after 128 occurrences of 11 recessive bits.
       Do not busy-wait or repeatedly stop/restart the CAN peripheral. */
    return;
  }
  if (canBusOff != 0U)
  {
    canBusOff = 0U;
    txPending = 1U;
    txAttemptTick = now;
    txRetryDelay = CAN_ERROR_RETRY_MS;
  }

  if (txOutstanding != 0U)
  {
    if (HAL_CAN_IsTxMessagePending(canHandle, txMailbox) != 0U)
    {
      if ((uint32_t)(now - txQueuedTick) >= CAN_TX_MAX_AGE_MS)
      {
        CAN_AbortOutstanding();
        txPending = 1U;
        txAttemptTick = now;
        txRetryDelay = CAN_ERROR_RETRY_MS;
      }
      return;
    }
    /* RQCP may have appeared since the earlier sample. Collect it on the
       next iteration before reusing the mailbox and clearing its status. */
    return;
  }

  if ((txPending == 0U) ||
      ((uint32_t)(now - txAttemptTick) < txRetryDelay))
  {
    return;
  }
  txAttemptTick = now;
  if (HAL_CAN_GetTxMailboxesFreeLevel(canHandle) == 0U)
  {
    txRetryDelay = CAN_MAILBOX_RETRY_MS;
    return;
  }
  header.ExtId = localCanId;
  header.IDE = CAN_ID_EXT;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 1U;
  header.TransmitGlobalTime = DISABLE;
  data[0] = txInputState;
  if (HAL_CAN_AddTxMessage(canHandle, &header, data, &txMailbox) == HAL_OK)
  {
    ++canTxCount;
    lastTxTick = now;
    txQueuedTick = now;
    txOutstanding = 1U;
    txPending = 0U;
    txRetryDelay = 0U;
  }
  else
  {
    ++canTxFailureCount;
    CAN_RecordError(canHandle, 0U);
    txRetryDelay = CAN_ERROR_RETRY_MS;
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
  if ((canHandle == NULL) || (hcan != canHandle))
  {
    return;
  }
  ++canRxCallbackCount;
  /* Bound each ISR to the FIFO depth. Further arrivals reassert RX0 IRQ. */
  for (uint32_t frames = 0U; frames < 3U; ++frames)
  {
    if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) == 0U)
    {
      break;
    }
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
    {
      HAL_CAN_ErrorCallback(hcan);
      break;
    }
    if ((header.IDE != CAN_ID_EXT) || (header.RTR != CAN_RTR_DATA) ||
        (header.ExtId == localCanId) || (header.DLC != 1U) || (data[0] > 1U))
    {
      ++canIgnoredRxCount;
      continue;
    }
    uint32_t now = HAL_GetTick();
    volatile CAN_AppPeer *peer = CAN_FindPeer(header.ExtId, now);
    if (peer == NULL)
    {
      ++canPeerTableFullCount;
      ++canIgnoredRxCount;
      continue;
    }
    peer->canId = header.ExtId;
    peer->inputState = data[0];
    peer->lastRxTick = now;
    peer->occupied = 1U;
    peer->active = 1U;
    ++peer->rxCount;

    /* These retain their original meaning: the latest accepted frame. The
       LED is calculated from ALL active peers, not only this latest frame. */
    lastReceivedState = data[0];
    lastReceivedCanId = header.ExtId;
    lastRxTick = now;
    ++canRxCount;
    CAN_UpdatePeerOutput();
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  if ((canHandle != NULL) && (hcan == canHandle))
  {
    CAN_RecordError(hcan, 0U);
  }
}

static void CAN_RecordError(CAN_HandleTypeDef *hcan, uint32_t error)
{
  static const uint32_t lecErrors[8] = {
    0U, HAL_CAN_ERROR_STF, HAL_CAN_ERROR_FOR, HAL_CAN_ERROR_ACK,
    HAL_CAN_ERROR_BR, HAL_CAN_ERROR_BD, HAL_CAN_ERROR_CRC, 0U
  };
  /* Called by both the main loop and ISR. Preserve cumulative error bits. */
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  uint32_t esr = hcan->Instance->ESR;
  error |= HAL_CAN_GetError(hcan);
  error |= lecErrors[(esr & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos];
  if ((esr & CAN_ESR_EWGF) != 0U) { error |= HAL_CAN_ERROR_EWG; }
  if ((esr & CAN_ESR_EPVF) != 0U) { error |= HAL_CAN_ERROR_EPV; }
  if ((esr & CAN_ESR_BOFF) != 0U) { error |= HAL_CAN_ERROR_BOF; }
  ++canErrorCount;
  canLastError |= error;
  canLastEsr = esr;
  __set_PRIMASK(primask);
}

/* ---------------------- Peer tracking and LED policy ---------------------- */

static volatile CAN_AppPeer *CAN_FindPeer(uint32_t id, uint32_t now)
{
  volatile CAN_AppPeer *replacement = NULL;
  for (uint32_t i = 0U; i < CAN_APP_MAX_PEERS; ++i)
  {
    volatile CAN_AppPeer *peer = &canPeers[i];
    if ((peer->occupied != 0U) && (peer->canId == id))
    {
      return peer;
    }
    if ((replacement == NULL) &&
        ((peer->occupied == 0U) || (peer->active == 0U) ||
         ((uint32_t)(now - peer->lastRxTick) >= CAN_APP_RX_TIMEOUT_MS)))
    {
      replacement = peer;
    }
  }
  if (replacement != NULL)
  {
    replacement->rxCount = 0U;
  }
  return replacement;
}

static void CAN_UpdatePeerOutput(void)
{
  /* Main loop and RX ISR both call this. Sample time inside the critical
     section so it cannot precede a newly received peer timestamp. */
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  uint32_t now = HAL_GetTick();
  uint32_t activePeers = 0U;
  uint8_t ledOn = 0U;

  for (uint32_t i = 0U; i < CAN_APP_MAX_PEERS; ++i)
  {
    volatile CAN_AppPeer *peer = &canPeers[i];
    if ((peer->active != 0U) &&
        ((uint32_t)(now - peer->lastRxTick) >= CAN_APP_RX_TIMEOUT_MS))
    {
      /* Only reception can reactivate a peer, even after a full tick wrap. */
      peer->active = 0U;
    }
    if (peer->active != 0U)
    {
      ++activePeers;
      /* Four-node policy: ANY active remote HIGH turns on this board's LED.
         To implement paired outputs later, select the peer->canId here. */
      ledOn |= peer->inputState;
    }
  }
  canPeerCount = activePeers;
  canRxTimedOut = (activePeers == 0U) ? 1U : 0U;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,
                   (ledOn != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  __set_PRIMASK(primask);
}
