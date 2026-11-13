#include <CAN.h>
#include <Bounce2.h>
#include "SPI.h"
#include "TFT_eSPI.h"

//CAN PINS
#define TX_GPIO_NUM   32
#define RX_GPIO_NUM   33
#define CAN_ID        0x10
#define PT_CAN_ID     0x3F
#define H2_CAN_ID     0x06

#define HAZARDS_CAN_MASK      0x00
#define BR_CAN_MASK           0x01
#define BL_CAN_MASK           0x02
#define HEADLIGHTS_CAN_MASK   0x04
#define WIPER_CAN_MASK        0x08
#define HORN_CAN_MASK         0x10

//INPUT BUTTONS
#define NUM_BUTTONS       6

#define HAZARDS_PIN       26
#define BLINK_R_PIN       27
#define BLINK_L_PIN       12
#define HEADLIGHTS_PIN    25
#define WIPER_PIN         22
#define HORN_PIN          13

const uint8_t button_pins[NUM_BUTTONS] = {HAZARDS_PIN, BLINK_R_PIN, BLINK_L_PIN, HEADLIGHTS_PIN, WIPER_PIN, HORN_PIN};
const uint32_t CAN_masks[NUM_BUTTONS] = {HAZARDS_CAN_MASK, BR_CAN_MASK, BL_CAN_MASK, HEADLIGHTS_CAN_MASK, WIPER_CAN_MASK, HORN_CAN_MASK};
Bounce * buttons_debounced = new Bounce[NUM_BUTTONS];
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite BR_sprite = TFT_eSprite(&tft);
TFT_eSprite BL_sprite = TFT_eSprite(&tft);
TFT_eSprite HL_sprite = TFT_eSprite(&tft);
TFT_eSprite Wiper_sprite = TFT_eSprite(&tft);
TFT_eSprite Horn_sprite = TFT_eSprite(&tft);
TFT_eSprite blank = TFT_eSprite(&tft);

uint8_t devices_enable = 0x00;
uint8_t PT_data_buf = 0x00;
uint8_t H2_data_buf[6];
uint32_t RPM = 0x00;
bool button_changed = false;
bool hazards_on = false;
uint32_t gpioStates = 0x0;
bool state = true;
int num_changes = 0;

void drawRightArrow(void);
void drawLeftArrow(void);
void drawHL(void);
void drawWiper(void);
void drawHorn(void);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  for(int i = 0; i < NUM_BUTTONS; i++){
    buttons_debounced[i].attach(button_pins[i], INPUT_PULLUP);
    buttons_debounced[i].interval(25);
  }

  Serial.begin (115200);
  while (!Serial);
  delay (100);

  Serial.println ("CAN Sender");

  // Set the pins
  CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);

  // start the CAN bus at 500 kbps -- halved? 250kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
  tft.init();
  tft.fillScreen(TFT_BLACK);

  tft.setRotation(3);
  utsm();
  metrics();
  
  //sprites
  BR_sprite.createSprite(50, 50);
  BR_sprite.fillSprite(TFT_BLACK);
  BL_sprite.createSprite(50, 50);
  BL_sprite.fillSprite(TFT_BLACK);
  HL_sprite.createSprite(50, 50);
  HL_sprite.fillSprite(TFT_BLACK);
  Wiper_sprite.createSprite(50, 50);
  Wiper_sprite.fillSprite(TFT_BLACK);
  Horn_sprite.createSprite(50, 50);
  Horn_sprite.fillSprite(TFT_BLACK);

  drawRightArrow(); drawLeftArrow();
  drawHL(); drawWiper(); drawHorn();

  blank.createSprite(50, 50);
  blank.fillSprite(TFT_BLACK);

  drawIcons(devices_enable);
}

void loop() {

  //poll buttons
  for(int i = 0; i < NUM_BUTTONS; i++){
    buttons_debounced[i].update();
    if(buttons_debounced[i].fell()){
      //button pressed, set bit to 1
      button_changed = true;

      if(i == 0){
        //if hazards, toggle BR and BL
        if(!hazards_on){
          //hazards OFF, so turn ON hazards and set blinkers ON
          devices_enable |= CAN_masks[1];
          devices_enable |= CAN_masks[2];
        } else{
          //hazards ON, so turn OFF hazards and set blinkers OFF only if Blinkers are not ON.
          if(digitalRead(BLINK_R_PIN) == HIGH) devices_enable &= ~(CAN_masks[1]);
          if(digitalRead(BLINK_L_PIN) == HIGH) devices_enable &= ~(CAN_masks[2]);
        }
        hazards_on = !hazards_on;
      } 
      else if (i == 3) {
        //if headlights, toggle
        devices_enable ^= CAN_masks[3];
      }
      else {
        devices_enable |= CAN_masks[i];
      }
      if(i == 1 || i == 2){
        if(hazards_on) button_changed = false;
      }
    } else if (buttons_debounced[i].rose()){
      //button released, clear bits to 0
      if(i == 0 || i == 3){
        //if hazards, check BR and HL
        //if(digitalRead(BLINK_L_PIN) == HIGH) devices_enable &= ~(CAN_masks[2]);
        //if(digitalRead(BLINK_R_PIN) == HIGH) devices_enable &= ~(CAN_masks[1]);
        button_changed = false;
      }
      else if (i == 1 || i == 2){
        //only clear blinkers if HAZARDS are OFF
        if(!hazards_on) {
          devices_enable &= ~(CAN_masks[i]);
          button_changed = true;
        }
      } 
      else {
        devices_enable &= ~(CAN_masks[i]);
        button_changed = true;
      }
    }
  }

  if(button_changed){
    Serial.printf("Sending packet 0x%02X ... ", (unsigned char) devices_enable);

    CAN.beginPacket(CAN_ID);
    CAN.write(devices_enable);
    CAN.endPacket();

    drawIcons(devices_enable);

    Serial.println("done");
    button_changed = false;
  }

    // try to parse CAN packet
  int packetSize = CAN.parsePacket();
  if (packetSize) {
    RPM = 0;
    int bytes_read = 0;
    // received a packet
    Serial.print ("Received ");

    if (CAN.packetExtended()) {
      Serial.print ("extended ");
    }

    if (CAN.packetRtr()) {
      // Remote transmission request, packet contains no data
      Serial.print ("RTR ");
    }
    uint8_t received_ID = CAN.packetId();
    Serial.print ("packet with id 0x");
    Serial.print (received_ID, HEX);

    if (CAN.packetRtr()) {
      Serial.print (" and requested length ");
      Serial.println (CAN.packetDlc());
    } else {
      Serial.print (" and length ");
      Serial.println (packetSize);
    
      // only print packet data for non-RTR packets
      while (CAN.available()) {
        if(received_ID == PT_CAN_ID){
          // current_device_states = (uint8_t) CAN.read();
          // Serial.printf("0x%02X", current_device_states);
          PT_data_buf = (uint8_t) CAN.read();
          bytes_read++;
          Serial.printf("0x%02X  ", PT_data_buf);
          if(bytes_read <= 4){
            RPM += (PT_data_buf << (8 * (4-bytes_read)));
            Serial.printf("0x%08X\n", RPM);
            Serial.printf("%d", bytes_read);
          }
        } else if (received_ID == H2_CAN_ID) {
            //draw to screen
            H2_data_buf[bytes_read] = (uint8_t) CAN.read();
            bytes_read++;
        }
      Serial.println();
      }
      speed_num((int) RPM);
      h2_num(H2_data_buf[0], H2_data_buf[1]);
      temp_num(H2_data_buf[2], H2_data_buf[3]);
    }
    Serial.println();
  }
}

unsigned long utsm() {
  tft.fillScreen(TFT_BLACK);
  unsigned long start = micros();
  tft.setCursor(140, 100);
  tft.setTextColor(TFT_GREEN);  tft.setTextSize(4);
  tft.println("Hello UTSM!");
  return micros() - start;
}

unsigned long metrics() {
  unsigned long start = micros();
  
  tft.setCursor(140, 160);
  tft.setTextColor(TFT_WHITE);  tft.setTextSize(3);
  tft.println("Speed: ");

  tft.setCursor(140, 160+30);
  tft.setTextColor(TFT_WHITE);  tft.setTextSize(2);
  tft.println("H2 (ppm): ");

  tft.setCursor(140, 160+30+30);
  tft.setTextColor(TFT_WHITE);  tft.setTextSize(2);
  tft.println("Temp C: ");

  return micros() - start;
}

unsigned long speed_num(int sp) {
  tft.fillRect(250, 160, 60, 20, TFT_RED);
  unsigned long start = micros();
  char buf[10];
  sprintf(buf, "%d", sp);
  tft.setCursor(255, 165);
  tft.setTextColor(TFT_WHITE);  tft.setTextSize(2);
  tft.println(buf);
  return micros() - start;
}

void h2_num(uint8_t high, uint8_t low) {
  tft.fillRect(250+20, 160+30, 60, 20, TFT_RED);
  char buf[10];
  sprintf(buf, "%.2f", ((high * 256) + low) * 10.0);
  tft.setCursor(255+20, 165+30);
  tft.setTextColor(TFT_WHITE);  tft.setTextSize(2);
  tft.println(buf);
}

void temp_num(uint8_t high, uint8_t low){
  tft.fillRect(250, 160+30+30, 62, 22, TFT_RED);
  char buf[10];
  sprintf(buf, "%.2f", (((high << 8) | low)) / 100.0);
  tft.setCursor(255, 165+30+30);
  tft.setTextColor(TFT_WHITE);  tft.setTextSize(2);
  tft.println(buf);
}


void drawIcons(uint8_t state) {
  static uint8_t lastState = 0x00;
  if (state == lastState) return;

  int w = tft.width();
  int h = tft.height();

  int box = 50;

  // Turn Right (top-right)
  if ((state & BR_CAN_MASK) != (lastState & BR_CAN_MASK)) {
    if((state & BR_CAN_MASK) != 0x00){
      BR_sprite.pushSprite(w-box, 0);
    } else {
      blank.pushSprite(w-box, 0);
    }
  }

 // LEFT TURN (top-left)
  if ((state & BL_CAN_MASK) != (lastState & BL_CAN_MASK)) {
    if((state & BL_CAN_MASK) != 0x00){
      //tft.fillTriangle(w - 50, 20, w - 30, 10, w - 30, 30, TFT_GREEN);
      BL_sprite.pushSprite(0, 0);
    } else {
      //tft.fillRect(w - 2 * box, 0, box, box, TFT_BLACK);
      blank.pushSprite(0, 0);
    }
  }

  // Headlights (mid-left)
  if ((state & HEADLIGHTS_CAN_MASK) != (lastState & HEADLIGHTS_CAN_MASK)) {
    if((state & HEADLIGHTS_CAN_MASK) != 0x00){
      HL_sprite.pushSprite(0, h/2-box/2);
    } else {
      blank.pushSprite(0, h/2-box/2);
    }
  }

  // Wiper (mid-left below HL)
  if ((state & WIPER_CAN_MASK) != (lastState & WIPER_CAN_MASK)) {
    if((state & WIPER_CAN_MASK) != 0x00) {
      Wiper_sprite.pushSprite(0, h/2-box/2+box);
    } else {
      blank.pushSprite(0, h/2-box/2+box);
    }
  }

  // Horn (mid-left below Wiper)
  if ((state & HORN_CAN_MASK) != (lastState & HORN_CAN_MASK)) {
    if((state & HORN_CAN_MASK) != 0x00){
      Horn_sprite.pushSprite(0, h/2-box/2+2*box);
    } else {
      blank.pushSprite(0, h/2-box/2+2*box);
    }
  }

  lastState = state;
}

void drawRightArrow(void){
  // Draw a simple right-pointing arrow

  // Arrow shaft
  BR_sprite.fillRect(5, 16, 25, 19, TFT_GREEN);

  // Arrow head (triangle)
  BR_sprite.fillTriangle(30, 10, 30, 40, 45, 25, TFT_GREEN);
}

void drawLeftArrow(void){
  // Draw a simple right-pointing arrow

  // Arrow shaft
  BL_sprite.fillRect(50-5-25, 16, 25, 19, TFT_GREEN);

  // Arrow head (triangle)
  BL_sprite.fillTriangle(50-30, 10, 50-30, 40, 50-45, 25, TFT_GREEN);
}

void drawHL(void){
  HL_sprite.fillCircle(15, 15, 8, TFT_YELLOW);
  HL_sprite.drawLine(25, 10, 35, 5, TFT_YELLOW);
  HL_sprite.drawLine(25, 15, 35, 15, TFT_YELLOW);
  HL_sprite.drawLine(25, 20, 35, 25, TFT_YELLOW);
}

void drawWiper(void){
  int y = 50 - 20;
  Wiper_sprite.drawLine(5, y, 35, y - 10, TFT_WHITE);
  Wiper_sprite.drawLine(5, y + 5, 35, y - 5, TFT_WHITE);
}

void drawHorn(void){
  // Base position for bottom-right corner
  int cx = 50 - 30; // center X
  int cy = 50 - 15; // center Y

  // Right-pointing triangle (replaces circle)
  // Same "radius" as headlight circle (8px)
  Horn_sprite.fillTriangle(cx - 8, cy - 8, cx - 8, cy + 8, cx + 8, cy, TFT_RED);

  // Three lines (same offsets as headlight lines)
  Horn_sprite.drawLine(cx + 10, cy - 6, cx + 20, cy - 10, TFT_RED);
  Horn_sprite.drawLine(cx + 10, cy,     cx + 22, cy,      TFT_RED);
  Horn_sprite.drawLine(cx + 10, cy + 6, cx + 20, cy + 10, TFT_RED);
}
