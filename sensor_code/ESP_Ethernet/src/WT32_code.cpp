// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos_idf.html#tasks

// include the necessary headers for Arduino and FreeRTOS.
#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// for accel reading
#include "ICM42688.h"        
#include <SPI.h>

#include <ETH.h>
#include <esp_task_wdt.h>

#define BUFFER_SIZE 1460 
char sendBuffer[BUFFER_SIZE];
int bufferIndex = 0;
int lastSendTime = 0;

// for receiver setting info
#define sensType "ICM42688" 
#define defaultRate 1000 // default ODR 
float LPF_rate = defaultRate;
#define defaultRange 16 // default range = +-16g
int sensRange = defaultRange; // sens range setting ICM42688 = 2, 4, 8, 16g

// WT32 int pin
int int_pin = 32;

int status = WL_IDLE_STATUS;
//IPAddress server(192,168,0,11);
//IPAddress server(192,168,254,102);
//IPAddress server(192,168,50,145);
IPAddress server(192, 168, 50, 197);
//IPAddress server(192,168,254,101);
WiFiClient client;

// FOR ACCELEROMETER
ICM42688 IMU(SPI, 5, 24000000); // Initialize IMU - ICM42688(SPI_BUS, CS_PIN, SPI_CLK);

// initialize variables for command handling
enum RunMode { CONTINUOUS, TIMED, SETUP };
RunMode currentMode = SETUP;
String commandBuffer = ""; // Buffer for bits from incoming commands
String inputCommand = ""; // To store incoming commands
unsigned long runDuration = 0; // In microseconds
unsigned long modeStartTime = 0;

void startSendingData();
void stopSendingData();
void parseCommand(String);

TaskHandle_t setupHandle = NULL;
void setupTask(void *pvParameters) {
  
  //loop - read incoming
  while (1){
    if (client.connected() && client.available() > 0) {
        client.setNoDelay(true); // Disable Nagle's algorithm
        char incomingChar = client.read(); // Read a single character
        if (incomingChar == '\n') { // End of command
            parseCommand(commandBuffer); // Pass the complete command to the parser
            commandBuffer = ""; // Clear the buffer for the next command
        } else {
            commandBuffer += incomingChar; // Append the character to the buffer
        }
    }
    else{
         vTaskDelay(pdMS_TO_TICKS(10)); // 10 ms delay
    }
  }
}

void parseCommand(String command) {
  // debug print
  Serial.print("Received command: ");
  Serial.println(command);

  command.trim(); // Clean up the command string
  

  if (command.equalsIgnoreCase("START_CONTINUOUS")) {
      currentMode = CONTINUOUS;
      client.println("Started continuous mode.");
      startSendingData();
  } else if (command.startsWith("START_TIMED")) {
      int duration = command.substring(11).toInt();
      if (duration > 0) {
          currentMode = TIMED;
          runDuration = duration * 1000; // Convert ms to µs
          modeStartTime = micros();
          client.printf("Started timed mode for %d ms.\n", duration);
          startSendingData();
      } else {
          client.println("Error: Invalid duration.");
      }
  } else if (command.equalsIgnoreCase("STOP")) {
      stopSendingData();
      client.println("Stopped.");
      currentMode = SETUP;
  } else if (command.startsWith("SET_RANGE")) {
      int range = command.substring(9).toInt();
      if (range == 2 || range == 4 || range == 8 || range == 16){ // for ICM42688
        //adxl.setRangeSetting(range);
        switch(range) {
          case 2: sensRange=2; break; // for ICM42688
          case 4:  sensRange=4; break;
          case 8:  sensRange=8; break;
          case 16: sensRange=16; break;
          }
        client.printf("Range set to %d.\n", range);
        Serial.printf("Range set to %d.\n", range);
      }
      else{
        client.printf("%d is an invalid range for %s.\n", range, sensType);
        Serial.printf("%d is an invalid range for %s.\n", range, sensType);
      }
  } else if (command.startsWith("SET_FREQUENCY")) {
      // *needs validation*
      float frequency = command.substring(13).toFloat();
      
      if (frequency > 0) {
          //adxl.setRate(frequency);
          //LPF_rate = frequency/4;
          client.printf("Frequency set to %.2f Hz.\n", frequency/4); // 4 is 2 for ADXL345 BW to ODR
      } else {
          client.printf("Error: Invalid frequency %f.\n", frequency/4); // 4 is 2 for ADXL345 BW to ODR
      }
  } else {
      client.println("Error: Unknown command.");
  }
}

// FOR ACCELEROMETER
// OPTIMIZED DATA STRUCTURE - Pack to reduce memory bandwidth [ __attribute__((packed)) ]
struct accelData {
  uint16_t delta_t; //
  int16_t x, y, z;  //
};


// FOR QUEUE
TaskHandle_t consumerHandle = NULL;
bool sendingData = false;
QueueHandle_t dataQueue;

// IRAM ideal for interrupts (placing a function into IRAM may reduce delays caused by a cache miss and significantly improve that function's performance)
// IRAM vs ARDUINO_ISR_ATTR?? https://docs.espressif.com/projects/arduino-esp32/en/latest/api/gpio.html?highlight=pullup#interrupts

static volatile bool dataReady = false;
static volatile accelData isrData;
static unsigned long lastTime = 0;
unsigned long startTime;


void IRAM_ATTR onDRDYInterrupt() {
    //static accelData accelData;
    //accelData.delta_t = 0; // Delta time calculated in consumer task
    
    unsigned long currentTime = micros() - startTime;
    uint16_t deltaT = (lastTime == 0) ? 0 : (currentTime - lastTime);
    lastTime = currentTime;
  
    // For ICM42688

    IMU.getRawAGT();
    isrData.delta_t = deltaT;    // read sensor data into buffers
    isrData.x = (int16_t)IMU.rawAccX();
    isrData.y = (int16_t)IMU.rawAccY();
    isrData.z = (int16_t)IMU.rawAccZ(); // get the data from the buffers

    // Quick queue send - use higher priority version
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(dataQueue, (void *)&isrData, &xHigherPriorityTaskWoken);

    // Force context switch if needed
    if (xHigherPriorityTaskWoken) {
      portYIELD_FROM_ISR();
    }
}

void startSendingData() {
  Serial.println("Starting...");
  sendingData = true;
  startTime = micros();

  lastTime = 0;

  // Clear queue before starting
  accelData dummyData;
  while (xQueueReceive(dataQueue, &dummyData, 0) == pdTRUE) {
    // Empty the queue
  }

  vTaskResume(consumerHandle);
  // For ICM42688
  //IMU.setFilters(0,0);
  Serial.println(IMU.begin());
  IMU.setAccelODR(ICM42688::odr16k);
  switch (sensRange) {
  case 2:
    IMU.setAccelFS(ICM42688::gpm2);
    break; // for ICM42688
  case 4:
    IMU.setAccelFS(ICM42688::gpm4);
    break;
  case 8:
    IMU.setAccelFS(ICM42688::gpm8);
    break;
  case 16:
    IMU.setAccelFS(ICM42688::gpm16);
    break;
  }
  IMU.setFilters(0, 0);
  IMU.enableDataReadyInterrupt();

  // Re-enable the interrupt after sensor is ready
  //attachInterrupt(int_pin, onDRDYInterrupt, RISING || ONHIGH);
  attachInterrupt(int_pin, onDRDYInterrupt, CHANGE);

  vTaskSuspend(setupHandle);
}

void stopSendingData() {

  // First disable the interrupt to prevent new data
  detachInterrupt(int_pin);

  // For ICM42688
  IMU.disableDataReadyInterrupt();

  Serial.println("Sensor stopped.");
  sendingData = false;

  // Clear send buffer
  bufferIndex = 0;
  memset(sendBuffer, 0, BUFFER_SIZE);
  
  // Give time for the queue to clear
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Clear any remaining items in queue
  accelData dummyData;
  while (xQueueReceive(dataQueue, &dummyData, 0) == pdTRUE) {
    // Empty the queue
  
  client.flush(); //flush remaining data from client 
  }
  
  vTaskResume(setupHandle);
  vTaskSuspend(consumerHandle);
}


// consumer task
void consumerTask(void *pvParameters) {
  accelData receivedData;
  int msgCount = 0;
  const TickType_t xTicksToWait = pdMS_TO_TICKS(1); 

  while (1) {
    if (!sendingData) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Process multiple queue items per iteration
    int itemsProcessed = 0;
    while (itemsProcessed < 10 &&
           xQueueReceive(dataQueue, &receivedData,
                         (itemsProcessed == 0) ? xTicksToWait : 0) == pdTRUE) {
      // Check for buffer space
      if (bufferIndex + sizeof(accelData) > BUFFER_SIZE) {
        if (client.connected()) {
          size_t sent = client.write(sendBuffer, bufferIndex);
          if (sent != bufferIndex) {
            Serial.printf("Write failed: sent %d of %d\n", sent, bufferIndex);
          }
        }
        bufferIndex = 0;
        msgCount++;
        lastSendTime = micros();
      }

      // Copy data to buffer
      memcpy(sendBuffer + bufferIndex, &receivedData, sizeof(accelData));
      bufferIndex += sizeof(accelData);
      itemsProcessed++;

      // Send when buffer is nearly full
      if (bufferIndex >= BUFFER_SIZE - sizeof(accelData)) {
        if (client.connected()) {
          size_t sent = client.write(sendBuffer, bufferIndex);
          if (sent != bufferIndex) {
            Serial.printf("Write failed: sent %d of %d\n", sent, bufferIndex);
          }
        }
        bufferIndex = 0;
        msgCount++;
        lastSendTime = micros();
      }
    }

    if (bufferIndex > 0 &&
        (micros() - lastSendTime) > 5000) { // 
      if (client.connected()) {
        size_t sent = client.write(sendBuffer, bufferIndex);
        if (sent != bufferIndex) {
          Serial.printf("Write failed: sent %d of %d\n", sent, bufferIndex);
        }
      }
      bufferIndex = 0;
      msgCount++;
      lastSendTime = micros();
    }

    if (itemsProcessed == 0) {
      taskYIELD();
    }
  }
}

TimerHandle_t ethReconnectTimer;
void connectToEth() {
    if (ETH.begin()) {
        Serial.println("ETH begun");
        //startSendingData();
    } else {
        Serial.println("ETH Failed to begin. Restarting...");
        ESP.restart();
    }
}

void WiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_ETH_START:
            Serial.println("ETH Started");
            //set eth hostname here
            ETH.setHostname("esp32-ethernet");
            break;
        case ARDUINO_EVENT_ETH_CONNECTED:
            Serial.println("ETH Connected");
            break;
        case ARDUINO_EVENT_ETH_GOT_IP:
            Serial.print("ETH MAC: ");
            Serial.print(ETH.macAddress());
            Serial.print(", IPv4: ");
            Serial.print(ETH.localIP());
            if (ETH.fullDuplex()) {
                Serial.print(", FULL_DUPLEX");
            }
            Serial.print(", ");
            Serial.print(ETH.linkSpeed());
            Serial.println("Mbps");
            if (client.connect(server, 81)) {
              Serial.println("connected");
              client.printf("SENSOR_INFO:%s,%d,%g\n", sensType, sensRange, LPF_rate*4);
              Serial.printf("SENSOR_INFO:%s,%d,%g\n", sensType, sensRange,
                        LPF_rate * 4);
            }
            break;
        case ARDUINO_EVENT_ETH_DISCONNECTED:
            Serial.println("ETH Disconnected");
            xTimerStart(ethReconnectTimer, 0);
            break;
        case ARDUINO_EVENT_ETH_STOP:
            Serial.println("ETH Stopped");
            xTimerStart(ethReconnectTimer, 0);
            break;
        default:
            break;
    }
}


void setup() {

  Serial.begin(921600);
  delay(1000);

  WiFi.onEvent(WiFiEvent);
  connectToEth();
  ethReconnectTimer = xTimerCreate("ethTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToEth));

  // Create queue
  dataQueue = xQueueCreate(8192, sizeof(accelData));  // (max # of items, single item size)

  // Configure task watchdog timer (optional)
  esp_task_wdt_init(5, true); // 5 second timeout, panic on timeout
  
  // Create consumerTask pinned to core 1
  xTaskCreatePinnedToCore(consumerTask, "consumerTask",
                          12288 , // 8192, //10000
                          NULL,
                          5, &consumerHandle, 1);

  vTaskSuspend(consumerHandle);
  
  xTaskCreatePinnedToCore(
    setupTask,
    "setupTask",
    5000,
    NULL,
    3,
    &setupHandle,
    0
  );
}

void loop() {
   // The loop function is empty because our tasks are running independently
}
