#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <inttypes.h>
#include <stdio.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "DHT20.h"
#include <SparkFunLSM6DSO.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "HttpClient.h"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
DHT20 DHT;

uint8_t count = 0;

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

char ssid[50]; // your network SSID (name)
char pass[50]; // your network password (use for WPA, or use
               // as key for WEP)

// Name of the server we want to connect to
const char kHostname[] = "worldtimeapi.org";
// Path to download (this is the bit after the hostname in the URL
// that you want to download
const char kPath[] = "/api/timezone/Europe/London.txt";
// Number of milliseconds to wait without receiving any data before we give up
const int kNetworkTimeout = 30 * 1000;
// Number of milliseconds to wait if no data is available before trying again
const int kNetworkDelay = 1000;

LSM6DSO myIMU;
int stepCount = 0;
float threshold = 1.70;
int minInterval = 200;
float previousValue = 0.0;
unsigned long previousPeak = 0;
bool detected = false;

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength = 100; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
int lastCount = stepCount;

byte pulseLED = 32; //Must be on PWM pin
byte readLED = 33; //Blinks with each data read

void nvs_access() {
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Open
    Serial.printf("\n");
    Serial.printf("Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        Serial.printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        Serial.printf("Done\n");
        Serial.printf("Retrieving SSID/PASSWD\n");
        size_t ssid_len;
        size_t pass_len;
        err = nvs_get_str(my_handle, "ssid", ssid, &ssid_len);
        err |= nvs_get_str(my_handle, "pass", pass, &pass_len);
        Serial.printf("SSID = %s\n", ssid);
        Serial.printf("PASSWD = %s\n", pass);
        switch (err) {
            case ESP_OK:
                Serial.printf("Done\n");
                
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                Serial.printf("The value is not initialized yet!\n");
                break;
            default:
                Serial.printf("Error (%s) reading!\n", esp_err_to_name(err));
        }
    }
    // Close
    nvs_close(my_handle);
}

void max30101Task(void *pvParameters) {
    while (true) {
       //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
      for (byte i = 25; i < 100; i++)
      {
        redBuffer[i - 25] = redBuffer[i];
        irBuffer[i - 25] = irBuffer[i];
      }

      //take 25 sets of samples before calculating the heart rate.
      for (byte i = 75; i < 100; i++)
      {
        while (particleSensor.available() == false) //do we have new data?
          particleSensor.check(); //Check the sensor for new data

        digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample(); //We're finished with this sample so move to next sample
      }

      //After gathering 25 new samples recalculate HR and SP02
      maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    }
}

void max30101_data_init(){
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
    particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
  }
}

void sendDataTask(void *pvParameters) {
    while (true) {
        // 打包数据
        char query[100];
      
        snprintf(query, 100, "/?step=%lu&hr=%d&spo2=%d", stepCount, heartRate/2, spo2);

        // 发送数据到服务器
        WiFiClient c;
        HttpClient http(c);
        int err = http.get("3.12.165.91", 5000, query);
        if (err == 0) {
            Serial.println("Data sent successfully!");
        } else {
            Serial.println("Failed to send data");
        }
        http.stop();

        vTaskDelay(pdMS_TO_TICKS(1000)); // 每秒发送一次数据
    }
}


void CalstepCount(){
  float accelX = myIMU.readFloatAccelX();
  float accelY = myIMU.readFloatAccelY();
  float accelZ = myIMU.readFloatAccelZ();
  
  float accelValue = sqrt(pow(accelX, 2) + pow(accelY, 2) + pow(accelZ, 2));

  if (accelValue > threshold && accelValue > previousValue) {
    unsigned long currTime = millis();
    if(currTime-previousPeak > minInterval){
      stepCount++;
      previousPeak = currTime;
    }
  }
  previousValue = accelValue;
}

void stepCounterTask(void *pvParameters) {
    while (true) {
        CalstepCount(); // 计步逻辑

        vTaskDelay(pdMS_TO_TICKS(200)); // 调整计步检测频率
    }
}

void printTask(void *pvParameters) {
    while (true) {
        Serial.print("Step Count: ");
        Serial.println(stepCount);
        Serial.print("Heart Rate: ");
        Serial.println(heartRate/2);
        Serial.print("SPO2: ");
        Serial.println(spo2);
        vTaskDelay(pdMS_TO_TICKS(2000)); // 每秒打印一次
    }
}

void setup()
{
  Serial.begin(9600); // initialize serial communication at 115200 bits per second:

  Wire.begin(21, 22);
  DHT.begin();
  delay(2000);
  
  // Retrieve SSID/PASSWD from flash before anything else
  nvs_access();

  // We start by connecting to a WiFi network
  delay(1000);
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("MAC address: ");
  Serial.println(WiFi.macAddress());

  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  if (myIMU.begin() == 0) {
    Serial.println("Failed to initialize IMU.");
  } else {
    Serial.println("IMU initialized successfully!");
    if(myIMU.initialize(BASIC_SETTINGS)){
      Serial.println("loaded setting");
    }
  }

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

      //read the first 100 samples, and determine the signal range
    Serial.print("Initializing data in MAX30101");
    max30101_data_init();
    Serial.print("Finish init");

  xTaskCreate(max30101Task, "MAX30101 Task", 4096, NULL, 1, NULL);
  xTaskCreate(stepCounterTask, "Step Counter Task", 2048, NULL, 1, NULL);
  xTaskCreate(sendDataTask, "Send Data Task", 4096, NULL, 1, NULL);
  xTaskCreate(printTask, "Print Task", 2048, NULL, 2, NULL);
}

void loop()
{
  
}