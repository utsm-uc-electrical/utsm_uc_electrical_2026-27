#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Adafruit_LSM6DSOX.h>
#include <SD.h>
#include <SPI.h>

Adafruit_LSM6DSOX imu;

#define GPS_RX 16   
#define GPS_TX 17   

// Standard ESP32 VSPI pin definitions for SD Card
#define SD_CS   5
#define SD_MOSI 23
#define SD_MISO 19
#define SD_CLK  18

// Timer variables for SD logging
unsigned long lastLogTime = 0;
const unsigned long logInterval = 1000; // Log every 1000 ms (1 second). 
char filename[32] = "/Elec_Test/data_00.csv";


//-------------------GPS STUFF------------------------
#define GPS_BAUD_RATE 9600
#define GPS_RX_PIN    16
#define GPS_TX_PIN    17
#define GPS_UART_NUM  2
#define GPS_PPS_PIN   4

HardwareSerial GPSSerial(GPS_UART_NUM);
TinyGPSPlus gps;

volatile uint32_t gps_ppsCount = 0;
static uint32_t gps_lastReport = 0;
static uint32_t gps_ppsCountAtLastReport = 0;

void IRAM_ATTR gps_onPPS() { 
    gps_ppsCount++; 
}

void initGPS() {
    GPSSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    
    // Clear out partial garbage bytes
    delay(120);
    while (GPSSerial.available()) {
        GPSSerial.read();
    }

    // Initialize hardware interrupt
    pinMode(GPS_PPS_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(GPS_PPS_PIN), gps_onPPS, RISING);

    gps_lastReport = millis();
    Serial.println("GPS module initialized.");
}

void updateGPS() {
    // Feed the parser
    while (GPSSerial.available()) {
        gps.encode(GPSSerial.read());
    }

    // 1Hz non-blocking telemetry print
    if (millis() - gps_lastReport >= 1000) {
        gps_lastReport = millis();

        uint32_t currentPps = gps_ppsCount;
        uint32_t ppsDelta = currentPps - gps_ppsCountAtLastReport;
        gps_ppsCountAtLastReport = currentPps;

        if (gps.location.isValid()) {
            Serial.printf("FIX: %.6f, %.6f | SATS: %d | ALT: %.1fm | SPEED: %.2f km/h | PPS: %lu Hz\n",
                          gps.location.lat(), 
                          gps.location.lng(),
                          gps.satellites.value(),
                          gps.altitude.isValid() ? gps.altitude.meters() : 0.0,
                          gps.speed.isValid() ? gps.speed.kmph() : 0.0,
                          (unsigned long)ppsDelta);
        }
    }
}



void setup() {
  Serial.begin(115200);
  delay(200);

  // gps uart init
  initGPS();

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
    
    // Find the next available file name (from 00 to 99)
    for (int i = 0; i < 100; i++) {
      sprintf(filename, "/Elec_Test/data_%02d.csv", i);
      if (!SD.exists(filename)) {
        break;
      }
    }
    
    Serial.print("Logging to new file: ");
    Serial.println(filename);


    File dataFile = SD.open(filename, FILE_APPEND);
    if (dataFile && dataFile.size() == 0) {
      dataFile.println("AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Latitude,Longitude,Altitude,Speed,Satellites");
    }
    if (dataFile) {
      dataFile.close();
    }
  }


}

void loop() {
  // GPS
  updateGPS();
  String gpsLat = gps.location.isValid() ? String(gps.location.lat(), 6) : "0000000";
  String gpsLng = gps.location.isValid() ? String(gps.location.lng(), 6) : "0.000000";
  String gpsAlt = gps.altitude.isValid() ? String(gps.altitude.meters(), 1) : "0.0";
  String gpsSpd = gps.speed.isValid() ? String(gps.speed.kmph(), 2) : "0.0";
  String gpsSats = String(gps.satellites.value());

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

  // SD card CSV formating
  if (millis() - lastLogTime >= logInterval) {
    lastLogTime = millis();
    String dataString = String(accel.acceleration.x) + "," +
                        String(accel.acceleration.y) + "," +
                        String(accel.acceleration.z) + "," +
                        String(gyro.gyro.x) + "," +
                        String(gyro.gyro.y) + "," +
                        String(gyro.gyro.z) + "," +
                        gpsLat + "," +
                        gpsLng + "," +
                        gpsAlt + "," +
                        gpsSpd + "," +
                        gpsSats;

    File dataFile = SD.open(filename, FILE_APPEND);
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
      Serial.println("Data logged to SD.");
    } else {
      Serial.println("Error opening datalog.csv for writing.");
    }
  }

  Serial.println("----------------------");

  delay(200);
}


