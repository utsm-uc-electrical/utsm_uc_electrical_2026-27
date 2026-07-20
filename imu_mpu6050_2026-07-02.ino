// NOTE: the below is the original code, following the tutorial (apart from updating the delay time)

// Basic demo for accelerometer readings from Adafruit MPU6050

// ESP32 Guide: https://RandomNerdTutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/
// ESP8266 Guide: https://RandomNerdTutorials.com/esp8266-nodemcu-mpu-6050-accelerometer-gyroscope-arduino/
// Arduino Guide: https://RandomNerdTutorials.com/arduino-mpu-6050-accelerometer-gyroscope/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(50);
}

// NOTE: the below code prints the Address of every module that the ESP32 sees (use to check whether the ESP32 senses the IMU)
// // // // #include <Wire.h>

// // // // void setup() {
// // // //   Wire.begin();
// // // //   Serial.begin(115200);
// // // //   while (!Serial);
// // // //   Serial.println("\nI2C Scanner");
// // // // }

// // // // void loop() {
// // // //   byte error, address;
// // // //   int nDevices = 0;

// // // //   Serial.println("Scanning...");

// // // //   for(address = 1; address < 127; address++ ) {
// // // //     Wire.beginTransmission(address);
// // // //     error = Wire.endTransmission();

// // // //     if (error == 0) {
// // // //       Serial.print("I2C device found at address 0x");
// // // //       if (address < 16) 
// // // //         Serial.print("0");
// // // //       Serial.print(address, HEX);
// // // //       Serial.println(" !");
// // // //       nDevices++;
// // // //     } else if (error == 4) {
// // // //       Serial.print("Unknown error at address 0x");
// // // //       if (address < 16) 
// // // //         Serial.print("0");
// // // //       Serial.println(address, HEX);
// // // //     }    
// // // //   }
  
// // // //   if (nDevices == 0)
// // // //     Serial.println("No I2C devices found\n");
// // // //   else
// // // //     Serial.println("done\n");

// // // //   delay(5000); 
// // // // }

// NOTE: the below code checks whether the Adafruit_MPU6050 correctly reads the correct memory address
// // #include <Wire.h>

// // void setup() {
// //   Serial.begin(115200);
// //   while (!Serial);
// //   Wire.begin();
  
// //   Wire.beginTransmission(0x68);
// //   Wire.write(0x75); // The WHO_AM_I register
// //   Wire.endTransmission(false);
// //   Wire.requestFrom(0x68, 1, true);
  
// //   byte whoAmI = Wire.read();
// //   Serial.print("The true Device ID (WHO_AM_I) is: 0x");
// //   Serial.println(whoAmI, HEX);
// // }

// // void loop() {}

// NOTE: the below uses a different library to filter the raw data
// // #include "Wire.h"
// // #include <MPU6050_light.h>

// // MPU6050 mpu(Wire);

// // void setup() {
// //   Serial.begin(115200);
// //   Wire.begin();
  
// //   byte status = mpu.begin();
// //   Serial.print("MPU6050 status: ");
// //   Serial.println(status);
  
// //   // If status is not 0, it failed to connect. 
// //   while(status != 0){ } 
  
// //   Serial.println("Calculating offsets, do not move the sensor...");
// //   delay(1000);
// //   mpu.calcOffsets(true, true); // Auto-calibrates gyro and accelerometer
// //   Serial.println("Done!\n");
// // }

// // void loop() {
// //   mpu.update(); // Fetches the new data
  
// //   Serial.print("Angle X: ");
// //   Serial.print(mpu.getAngleX());
// //   Serial.print("\tAngle Y: ");
// //   Serial.print(mpu.getAngleY());
// //   Serial.print("\tAngle Z: ");
// //   Serial.println(mpu.getAngleZ());
  
// //   delay(50);
// // }

// NOTE: the below code prints out the raw data from the MPU (no libraries used)
// #include <Wire.h>

// void setup() {
//   Serial.begin(115200);
//   Wire.begin();
  
//   // 1. Manually wake up the MPU-6050
//   Wire.beginTransmission(0x68);
//   Wire.write(0x6B);  // Target the Power Management Register
//   Wire.write(0x00);  // Write 0 to wake it up
//   Wire.endTransmission(true);
  
//   Serial.println("Sensor Woken Up. Reading Raw Gravity...");
//   delay(500);
// }

// void loop() {
//   // 2. Point to the starting Accel Register (0x3B)
//   Wire.beginTransmission(0x68);
//   Wire.write(0x3B);  
//   Wire.endTransmission(false);
  
//   // 3. Request 6 consecutive bytes (X, Y, Z raw data)
//   Wire.requestFrom(0x68, 6, true);
  
//   if(Wire.available() == 6) {
//     // Combine the high and low bytes for each axis
//     int16_t accelX = Wire.read()<<8 | Wire.read(); 
//     int16_t accelY = Wire.read()<<8 | Wire.read(); 
//     int16_t accelZ = Wire.read()<<8 | Wire.read(); 
    
//     Serial.print("Raw X: "); Serial.print(accelX);
//     Serial.print(" | Raw Y: "); Serial.print(accelY);
//     Serial.print(" | Raw Z: "); Serial.println(accelZ);
//   } else {
//     Serial.println("I2C connection dropped!");
//   }
  
//   delay(50);
// }