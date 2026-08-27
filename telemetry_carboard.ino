#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Adafruit_LSM6DSOX.h>
#include <SD.h>
#include <SPI.h>

TinyGPSPlus gps;
HardwareSerial SerialGPS(2);   // uart 2
Adafruit_LSM6DSOX imu;

#define GPS_RX 16   
#define GPS_TX 17   

// Standard ESP32 VSPI pin definitions for SD Card
#define SD_CS   5
#define SD_MOSI 23
#define SD_MISO 19
#define SD_CLK  18

void setup() {
  Serial.begin(115200);
  delay(200);

  // gps uart init
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX); // check baud
  Serial.println("GPS UART started");

  // imu --> change to functions
  if (!imu.begin_I2C()) { // do i need to define my i2c pins???
    Serial.println("IMU NOT FOUND");
    while (1);
  }
  Serial.println("IMU FOUND");

  // SD CARD INITIALIZATION ----------------------------------------
  SD.begin(SD_CS);
  SPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card Mount Failed!");
  } else {
    Serial.println("SD Card OK");
    
    // Changed FILE_WRITE to FILE_APPEND
    File dataFile = SD.open("/Elec_Test/datalog.csv", FILE_APPEND);
    if (dataFile && dataFile.size() == 0) {
      dataFile.println("AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ");
    }
    if (dataFile) {
      dataFile.close();
    }
  }
}

void loop() {
  // GPS
  while (SerialGPS.available()) {
    char c = SerialGPS.read();
    Serial.print(c);
    gps.encode(c);
  }

  if (gps.location.isUpdated()) {
    Serial.print("GPS: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(", ");
    Serial.println(gps.location.lng(), 6);
  }

  // IMU
  sensors_event_t accel, gyro, temp;
  imu.getEvent(&accel, &gyro, &temp);

  // currently printing raw values, need to convert
  Serial.print("ACCEL: ");
  Serial.print(accel.acceleration.x); Serial.print(", ");
  Serial.print(accel.acceleration.y); Serial.print(", ");
  Serial.println(accel.acceleration.z);

  Serial.print("GYRO: ");
  Serial.print(gyro.gyro.x); Serial.print(", ");
  Serial.print(gyro.gyro.y); Serial.print(", ");
  Serial.println(gyro.gyro.z);

  Serial.println("----------------------");

  // SD card CSV formating
  String dataString = String(accel.acceleration.x) + "," +
                      String(accel.acceleration.y) + "," +
                      String(accel.acceleration.z) + "," +
                      String(gyro.gyro.x) + "," +
                      String(gyro.gyro.y) + "," +
                      String(gyro.gyro.z);

  File dataFile = SD.open("/Elec_Test/datalog.csv", FILE_APPEND);
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    Serial.println("Data logged to SD.");
  } else {
    Serial.println("Error opening datalog.csv for writing.");
  }

  Serial.println("----------------------");

  delay(200);
}
