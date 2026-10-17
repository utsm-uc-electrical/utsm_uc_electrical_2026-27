#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>  // Select version 7.1.2 in Arduino Library Manager.
#include "telemetry.h"

// MainSchemRX.SchDoc and MainSchem.SchDoc: ESP32-DEVKIT-V1 + Core1262-HF.
static const int PIN_SPI_SCK = 18;
static const int PIN_SPI_MISO = 19;
static const int PIN_SPI_MOSI = 23;
static const int PIN_LORA_CS = 5;
static const int PIN_LORA_RST = 14;
static const int PIN_LORA_DIO1 = 12;
static const int PIN_LORA_BUSY = 25;
// If an SD card shares SPI, set its CS pin here to keep it deselected.
static const int PIN_UNUSED_SD_CS = -1;

// Waveshare-linked ws_dev example uses TCXO_CTRL_1_7V and 5 ms startup.
// RadioLib 7.1.2's setTCXO default startup delay is also 5 ms.
static const float LORA_TCXO_VOLTAGE = 1.7f;
// DIO2 is explicitly unconnected in both user schematics.
static const bool LORA_DIO2_RF_SWITCH = false;
static const bool LORA_USE_LDO = false;
// Core1262 schematic + RTC6603SP truth table: RX = HIGH/LOW; TX = LOW/HIGH.
static const uint32_t PIN_LORA_RXEN = 13;
static const uint32_t PIN_LORA_TXEN = 15;

Module radioModule(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RST,
                   PIN_LORA_BUSY, SPI);
SX1262 radio(&radioModule);

// Must match transmit.ino; pin numbers do not have to match the transmitter.
static const long LORA_FREQ_HZ = 915000000L;
static const int LORA_SYNC_WORD = 0xA7;
static const int LORA_SF = 7;
static const long LORA_BW_HZ = 250000L;
static const int LORA_CR_DENOM = 5;

static bool radioOk = false;
static bool havePacket = false;
static uint32_t lastRxMs = 0;
static uint32_t lastRetryMs = 0;
static uint32_t lastStatusMs = 0;
static uint32_t received = 0;
static uint32_t rejected = 0;
static int16_t lastRadioError = RADIOLIB_ERR_NONE;
static const uint32_t LINK_TIMEOUT_MS = 2000;

bool startRadio() {
  // RadioLib takes MHz/kHz and an 8-bit sync word. It converts 0xA7 to the
  // SX1262 register representation (0xA474); do not pass 0xA474 to begin().
  lastRadioError = radio.begin(LORA_FREQ_HZ / 1000000.0f,
                              LORA_BW_HZ / 1000.0f, LORA_SF,
                              LORA_CR_DENOM, LORA_SYNC_WORD, 10, 8,
                              LORA_TCXO_VOLTAGE, LORA_USE_LDO);
  if (lastRadioError != RADIOLIB_ERR_NONE) return false;
  lastRadioError = radio.setDio2AsRfSwitch(LORA_DIO2_RF_SWITCH);
  if (lastRadioError != RADIOLIB_ERR_NONE) return false;
  if (PIN_LORA_RXEN != RADIOLIB_NC || PIN_LORA_TXEN != RADIOLIB_NC)
    radio.setRfSwitchPins(PIN_LORA_RXEN, PIN_LORA_TXEN);
  lastRadioError = radio.explicitHeader();
  if (lastRadioError != RADIOLIB_ERR_NONE) return false;
  lastRadioError = radio.setCRC(2);
  if (lastRadioError != RADIOLIB_ERR_NONE) return false;
  lastRadioError = radio.invertIQ(false);
  if (lastRadioError != RADIOLIB_ERR_NONE) return false;
  lastRadioError = radio.startReceive();
  return lastRadioError == RADIOLIB_ERR_NONE;
}

void printStatus(uint32_t now) {
  Serial.printf("{\"type\":\"status\",\"radio\":%s,\"link\":%s,"
                "\"received\":%lu,\"rejected\":%lu,\"radioError\":%d,\"lastRxAgeMs\":",
                radioOk ? "true" : "false",
                radioOk && havePacket && now - lastRxMs < LINK_TIMEOUT_MS ? "true" : "false",
                (unsigned long)received, (unsigned long)rejected, (int)lastRadioError);
  if (havePacket) Serial.print((unsigned long)(now - lastRxMs));
  else Serial.print("null");
  Serial.println("}");
}

void setup() {
  Serial.begin(115200);
  // Serial is UART0 on this ESP32, carried to USB by a CP210x bridge.
  const uint32_t started = millis();
  while (!Serial && millis() - started < 1500) delay(10);

  if (PIN_UNUSED_SD_CS >= 0) {
    pinMode(PIN_UNUSED_SD_CS, OUTPUT);
    digitalWrite(PIN_UNUSED_SD_CS, HIGH);
  }
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
  pinMode(PIN_LORA_DIO1, INPUT);
  radioOk = startRadio();
  lastRetryMs = millis();
  Serial.println("{\"type\":\"hello\",\"name\":\"UTSM-LoRa-Receiver\","
                 "\"chip\":\"SX1262\",\"protocol\":1,\"packetBytes\":80,\"baud\":115200}");
  printStatus(millis());
}

void loop() {
  const uint32_t now = millis();
  if (!radioOk && now - lastRetryMs >= 2000) {
    lastRetryMs = now;
    radioOk = startRadio();
  }

  // DIO1 is latched by the radio when a receive event completes. Poll the pin
  // without blocking; SPI access and USB output stay in the main loop.
  if (radioOk && digitalRead(PIN_LORA_DIO1) == HIGH) {
    lastRadioError = radio.standby();
    if (lastRadioError != RADIOLIB_ERR_NONE) {
      radioOk = false;
      lastRetryMs = millis();
      return;
    }
    const size_t size = radio.getPacketLength();
    uint8_t bytes[sizeof(LoraTelemetry)];
    // Never copy an unexpected-length packet into the 80-byte destination.
    const int16_t readState = size == sizeof(bytes)
        ? radio.readData(bytes, sizeof(bytes)) : RADIOLIB_ERR_PACKET_TOO_LONG;
    const int rssi = (int)lroundf(radio.getRSSI());
    const float snr = radio.getSNR();
    // Restart after valid, malformed and CRC-failed packets. This also clears
    // pending radio IRQs. Rearm before formatting/writing the USB record.
    const int16_t restartState = radio.startReceive();
    radioOk = restartState == RADIOLIB_ERR_NONE;
    lastRadioError = radioOk ? readState : restartState;
    if (!radioOk) lastRetryMs = millis();

    LoraTelemetry packet;
    if (readState != RADIOLIB_ERR_NONE ||
        !decodeTelemetry(bytes, size, packet)) {
      ++rejected;
    } else {
      char json[1536];
      if (telemetryJson(packet, now, rssi, snr, json, sizeof(json))) {
        ++received;
        havePacket = true;
        lastRxMs = now;
        Serial.println(json);
      } else {
        ++rejected;
      }
    }
  }

  if (now - lastStatusMs >= 2000) {
    lastStatusMs = now;
    printStatus(now);
  }
  delay(1);
}
