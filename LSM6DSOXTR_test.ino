/*
 * UTSM Telemetry - Vehicle transmitter TEST CODE
 * ESP32-DevKit-V1 + Adafruit_LSM6DSOX breakout
 *
 * Copy ../packet.h next to this .ino before opening in Arduino IDE
 * (Arduino only compiles files inside the sketch folder).
 *
 * Libraries: Adafruit_LSM6DSOX (+ Adafruit_BusIO, Adafruit_Sensor)
 *
 */

#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>

// ---------------- pin map ----------------
#define PIN_I2C_SDA    21
#define PIN_I2C_SCL    22

// ---------------- I2C config ----------------
static const uint32_t I2C_CLOCK_HZ = 400000;  // 400kHz "Fast Mode" I2C

// ---------------- sensor object ----------------
Adafruit_LSM6DSOX sox;
static bool imu_ok = false;

// -------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // wait for serial monitor to open
  
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);
  
  Serial.println("Looking for LSM6DSOX...");

  imu_ok = sox.begin_I2C(0x6A, &Wire) || sox.begin_I2C(0x6B, &Wire);
  if (!imu_ok) {
    Serial.println("Could not find LSM6DSOX chip. Check wiring!");
    while (1) delay(10);  // stops here 
  }

  Serial.println("LSM6DSOX found!");
}

void loop() {
  sensors_event_t accel, gyro, temp;
  sox.getEvent(&accel, &gyro, &temp);

  Serial.print("Accel X: "); Serial.print(accel.acceleration.x);
  Serial.print(" Y: "); Serial.print(accel.acceleration.y);
  Serial.print(" Z: "); Serial.println(accel.acceleration.z);

  Serial.print("Gyro X: "); Serial.print(gyro.gyro.x);
  Serial.print(" Y: "); Serial.print(gyro.gyro.y);
  Serial.print(" Z: "); Serial.println(gyro.gyro.z);

  Serial.println();
  delay(500);
}