#include "ICM42688.h"
#include <Arduino.h>
#include <esp_task_wdt.h>
// TODO: Need to test this with the ICM42688

// an ICM42688 object with the ICM42688 sensor on SPI bus 0 and chip select pin 5
ICM42688_FIFO IMU(SPI, 5);
int           status;

// variables to hold FIFO data, these need to be large enough to hold the data
int ax[300], ay[300], az[300];
uint8_t h[300];
size_t fifoSize;

volatile bool imuDataReady = false;

void IRAM_ATTR onDRDYInterrupt() { imuDataReady = true; }

void processIMUData() {
  IMU.readFifo();

  size_t fifoSize;
  uint8_t h[300];
  int ax[300], ay[300], az[300];

  IMU.getFifoHeader(&fifoSize, h);
  IMU.getFifoAccelX_raw(&fifoSize, ax);
  IMU.getFifoAccelY_raw(&fifoSize, ay);
  IMU.getFifoAccelZ_raw(&fifoSize, az);

  Serial.print("The FIFO buffer is ");
  Serial.print(IMU.getFifoSize());
  Serial.println(" samples long.");
  for (size_t i = 0; i < fifoSize; i++) {
    Serial.print(h[i]);
    Serial.print("\t");
    Serial.print(ax[i]);
    Serial.print("\t");
    Serial.print(ay[i]);
    Serial.print("\t");
    Serial.println(az[i]);
  }
}

void setup() {
  Serial.begin(921600);
  attachInterrupt(32, onDRDYInterrupt, RISING);

  esp_task_wdt_init(5, true); // WDT for long loops

  status = IMU.begin();
  IMU.setAccelODR(ICM42688::odr4k);
  IMU.setFifoSize(100);
  IMU.setFilters(0, 0);
  IMU.resetInterrupts();
  IMU.enableFifoThresholdInterrupt();

  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
  }

  IMU.enableFifo(true, false, false);
  IMU.streamToFifo();
}

void loop() {
  if (imuDataReady) {
    imuDataReady = false;
    processIMUData();
  }

  // Optionally reset the Task WDT if you're using it for long-loop protection
  esp_task_wdt_reset();
}