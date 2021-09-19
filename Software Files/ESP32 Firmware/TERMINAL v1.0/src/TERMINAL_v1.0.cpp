#include <Arduino.h>
#include <Adafruit_Fingerprint.h>
#include <WiFi.h>
#include <PCF85063A.h>
#include <TimeLib.h>
#include <Wire.h>
#include "DFRobot_SHT20.h"
#include "Arduino_ST7789.h"
#include "Adafruit_GFX.h"
#include "Adafruit_I2CDevice.h"
#include <SPI.h>
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include "NTPClient.h"
#include <WiFiMulti.h>
#include <TimeLib.h>
#include <AsyncTCP.h>
#include <AsyncUDP.h>
#include <WiFiSTA.h>
WiFiMulti wifiMulti;
#define DEVICE "ESP32"
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include <InfluxData.h>
#include "SparkFun_LIS2DH12.h"

/* Functions */
void display(void *pvParameters);
void scanner(void *pvParameters);
void serialparser(void *pvParameters);
int8_t getFingerprintID(int8_t *fid, uint8_t *fconf);
#ifdef __cplusplus
extern "C"
{
#endif

  uint8_t temprature_sens_read();

#ifdef __cplusplus
}
#endif

uint8_t temprature_sens_read();
String timeStr = "";
long txn = 0;
float temp = 0;

/* InfluxDB Connection Parameters */
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#define INFLUXDB_URL "https://us-west-2-1.aws.cloud2.influxdata.com"
#define INFLUXDB_TOKEN "qXo1pHPocwaFZ1ZSSlDllihJ6owI5WyU7dgHJd9o9q5TnKgLjpAYIToSQPaH4OJxc87ZG5DV55_I_0LGOJMB6A=="
#define INFLUXDB_ORG "theprovidentinquisition@gmail.com"
#define INFLUXDB_BUCKET "MESH"
#define TZ_INFO "CET-1CEST,M3.5.0,M10.5.0/3"
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN);
/* Task Handles */
static TaskHandle_t disp;
static TaskHandle_t scan;
static TaskHandle_t serp;

static SemaphoreHandle_t scan2disp;
static SemaphoreHandle_t scan2serp;

static TickType_t scan_last_wake;

/* Strings to store incoming UART data */
String uart_recv_buffer;

/* Global variables for serial parser */
String uart_recv_pid;
String uart_recv_uid;
int pid;
int uid;
bool is_auth = false;
long int startup_time_rtc;

struct serp_data
{
  String timestamp[255];
  String data[255];
};

struct environment
{
  String time;
  uint16_t uid;
  int32_t temp;
  uint32_t humi;
  uint32_t press;
  uint32_t light;
};

struct motion
{
  String time;
  uint16_t uid;
  uint32_t count;
};

struct volatileorganiccompound
{
  String time;
  uint16_t uid;
  int32_t temp;
  uint32_t humi;
  uint32_t press;
  uint32_t light;
};

struct detector
{
  String time;
  uint16_t uid;
  uint32_t count;
};

struct carbondioxide
{
  String time;
  uint16_t uid;
  uint32_t co2;
};

struct gpsn
{
  struct environment env[255];
  struct motion mot[255];
  struct volatileorganiccompound voc[255];
};
#define SCR_WD 240
#define SCR_HT 240 // 320 - to allow access to full 240x320 frame buffer

#define SCAN_INTERVAL_MS 50
#define DISPLAY_UPDATE_INTERVAL_MS 200
/* Pin configuration */
#define UART2_TX 27
#define UART2_RX 25
#define SDA 22
#define SCL 23
#define TOUCH_SENSE 5
#define FP_TX 18
#define FP_RX 19
#define BUZZER 21
#define TFT_SCLK 14
#define TFT_MISO 4
#define TFT_MOSI 15
#define TFT_CS 12
#define TFT_DC 12
#define TFT_RST 13

/* Comms configuration */
#define UART_BAUDRATE 115200
#define I2C_CLK_RATE 100000

#define FPS_SAMPLE_RATE_MS 50
#define DISPLAY_UPDATE_INTERVAL_MS 200

/* Objects */
DFRobot_SHT20 sht20;
PCF85063A rtc;
WiFiServer server(80);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

Adafruit_Fingerprint fps = Adafruit_Fingerprint(&Serial1);
Arduino_ST7789 lcd = Arduino_ST7789(TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK);

void setup()
{

  Serial.begin(UART_BAUDRATE);
  Serial2.begin(UART_BAUDRATE, SERIAL_8N1, UART2_RX, UART2_TX); // initialize comms with nRF52
  Wire.begin(SDA, SCL, I2C_CLK_RATE);                           // initialize I2C for display (I2C0)

  lcd.init(240, 240); // initialize a ST7789 chip, 240x240 pixels

  fps.begin(57600);
  fps.getParameters();
  Serial.print(F("Status: 0x"));
  Serial.println(fps.status_reg, HEX);
  Serial.print(F("Sys ID: 0x"));
  Serial.println(fps.system_id, HEX);
  Serial.print(F("Capacity: "));
  Serial.println(fps.capacity);
  Serial.print(F("Security level: "));
  Serial.println(fps.security_level);
  Serial.print(F("Device address: "));
  Serial.println(fps.device_addr, HEX);
  Serial.print(F("Packet len: "));
  Serial.println(fps.packet_len);
  Serial.print(F("Baud rate: "));
  Serial.println(fps.baud_rate);
  ledcSetup(0, 10000, 8);
  ledcAttachPin(BUZZER, 0);
  pinMode(5, INPUT);

  Serial.println("[INIT] STARTUP INITIALIZATION OK");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("[INIT] ERROR CONNECTING TO WIFI");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    vTaskDelay(2000);
  }

  client.setHTTPOptions(HTTPOptions().httpReadTimeout(200));
  client.setHTTPOptions(HTTPOptions().connectionReuse(true));
  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");

  // Check server connection
  if (client.validateConnection())
  {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  }
  else
  {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }

  Serial.println("[INIT] WIFI CONNECTION OK");
  rtc.reset();
  Serial.println("[INIT] RTC OK");
  delay(10);
  tmElements_t tm;
  timeClient.begin();
  timeClient.setTimeOffset(28800);
  while (!timeClient.update())
  {
    timeClient.forceUpdate();
  }
  breakTime(timeClient.getEpochTime(), tm);
  rtc.time_set(&tm);
  delay(50);
  tmElements_t t;
  rtc.time_get(&t);
  startup_time_rtc = makeTime(t);
  Serial.println("[INIT] TIME SYNC OK");
  sht20.initSHT20(Wire);
  Serial.println("[INIT] TEMP SENSOR OK");

  xTaskCreatePinnedToCore( // Use xTaskCreate()
      display,             // Function to be called
      "Display",           // Name of task
      32768,               // Stack size
      NULL,                // Parameter to pass
      2,                   // Task priority
      &disp,               // Task handle
      0);                  // CPU Core

  Serial.println("[INIT] TASK IFDB CREATED");
  xTaskCreatePinnedToCore( // Use xTaskCreate()
      scanner,             // Function to be called
      "Scanner",           // Name of task
      16384,               // Stack size
      NULL,                // Parameter to pass
      2,                   // Task priority
      &scan,               // Task handle
      1);                  // CPU Core
  Serial.println("[INIT] TASK SERP CREATED");
  Serial.println("[INIT] TASK INIT TERMINATED");
  scan2disp = xSemaphoreCreateMutex();
  scan2serp = xSemaphoreCreateMutex();
  vTaskDelete(NULL);
}

void display(void *pvParameters)
{
  lcd.fillScreen(RED);
  lcd.setCursor(0, 0);
  lcd.setTextColor(BLACK);
  lcd.setTextSize(2);
  lcd.setTextWrap(true);
  //lcd.print("TERMINAL v1");
  String query = "from(bucket: \"MESH\") |> range(start: -1h) |> filter(fn: (r) => r._measurement == \"GATEWAY\") |> filter(fn: (r) => r._field == \"Total InfluxDB Transaction Count\") |> max()";

  while (1)
  {

    if (xSemaphoreTake(scan2disp, 1) == pdTRUE)
    {
      FluxQueryResult result = client.query(query);
      // code to print serial terminal data and additional stats
      while (result.next())
      {
        txn = result.getValueByName("_value").getLong();

        Serial.print(txn);
      }

      //lcd.fillRect(0,0,160,120,RED);
      lcd.fillRect(0, 30, 95, 30, RED);
      lcd.setTextColor(BLACK);
      lcd.setTextSize(4);
      lcd.setCursor(0, 0);
      lcd.print("TXNS");

      lcd.setTextColor(BLUE);
      lcd.setTextSize(4);
      lcd.setCursor(0, 30);
      lcd.print(txn);
      lcd.setTextColor(BLACK);
      lcd.setTextSize(4);
      lcd.setCursor(0, 60);
      lcd.print("CPU TEMP");
      lcd.fillRect(0, 90, 120, 30, RED);
      lcd.setTextColor(BLUE);
      lcd.setTextSize(4);
      lcd.setCursor(0, 90);
      lcd.print(temp);

      xSemaphoreGive(scan2disp);
    }
    else
    {
      // code to print basic stats
    }
    delay(1);
  }
}

void scanner(void *pvParameters)
{
  int8_t fid;
  uint8_t fconf;
  scan_last_wake = xTaskGetTickCount();
  while (1)
  {

    delay(200);
    temp = temprature_sens_read();
    temp -= 32;
    temp /= 1.8;
    Serial.println(digitalRead(TOUCH_SENSE));
    getFingerprintID(&fid, &fconf);
  }
  if (xSemaphoreTake(scan2disp, 10) != pdTRUE)
  {
    Serial.print("ERROR");
  }
  if (xSemaphoreTake(scan2serp, 10) != pdTRUE)
  {
    Serial.print("ERROR");
  }

  while (1)
  {
    xSemaphoreTake(scan2serp, portMAX_DELAY);
    if (getFingerprintID(&fid, &fconf) && is_auth == false) // auth
    {
      // code to authorize fingerprint
      is_auth = true;
      xSemaphoreGive(scan2disp);
      xSemaphoreGive(scan2serp);
      xTaskCreatePinnedToCore( // Use xTaskCreate()
          serialparser,        // Function to be called
          "Serial Parser",     // Name of task
          16384,               // Stack size
          NULL,                // Parameter to pass
          1,                   // Task priority
          &serp,               // Task handle
          1);                  // CPU Core
    }
    else if (is_auth == true)
    {
      xSemaphoreGive(scan2disp);
      xSemaphoreGive(scan2serp);
    }
    else if (getFingerprintID(&fid, &fconf) && is_auth == true) // deauth
    {
      // code to deauthorize fingerprint
      xSemaphoreTake(scan2disp, 10);
      xSemaphoreTake(scan2serp, portMAX_DELAY);
      vTaskDelete(&serp);
      vSemaphoreDelete(scan2serp);
    }
    vTaskDelayUntil(&scan_last_wake, 50);
  }
}

void serialparser(void *pvParameters)
{
  if (xSemaphoreTake(scan2serp, 1) == pdTRUE)
  {
    vTaskPrioritySet(serp, 3);

    // serial parser
    if (Serial2.available() > 0 && Serial2.read() == '>')
    {
      uart_recv_buffer = Serial2.readStringUntil('<');
      Serial.println("[SERP] UART2 DATA RECEIVED");
      Serial2.flush();
    }
    vTaskDelay(1);
    vTaskPrioritySet(serp, 1);
    xSemaphoreGive(scan2serp);
  }
}

void loop()
{
}

// returns -1 if failed, otherwise returns ID #
int8_t getFingerprintID(int8_t *fid, uint8_t *fconf)
{
  uint8_t p = fps.getImage();
  if (p == FINGERPRINT_NOFINGER)
    return -2;
  if (p != FINGERPRINT_OK)
    return -1;

  p = fps.image2Tz();
  if (p != FINGERPRINT_OK)
    return -1;

  p = fps.fingerFastSearch();
  if (p != FINGERPRINT_OK)
    return -1;

  // found a match!
  Serial.print("Found ID #");
  Serial.print(fps.fingerID);
  Serial.print(" with confidence of ");
  Serial.println(fps.confidence);
  *fid = fps.fingerID;
  *fconf = fps.confidence;
  return 1;
}