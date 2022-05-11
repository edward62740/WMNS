/*Includes */
#include <Arduino.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <PCF85063A.h>
#include <Wire.h>
#include "DFRobot_SHT20.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Adafruit_I2CDevice.h>
#include <WiFiMulti.h>
#include <TimeLib.h>
WiFiMulti wifiMulti;
#define DEVICE "ESP32"
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include <BMI160Gen.h>
#include <Adafruit_I2CDevice.h>
#include <esp_now.h>

uint8_t bc_addr[] = {0x4c, 0x11, 0xae, 0x78, 0xea, 0x10};
#ifdef __cplusplus
extern "C"
{
#endif

  uint8_t temprature_sens_read();

#ifdef __cplusplus
}
#endif

/* Functions */
void motiontriggered();
void scandevices();
void influxDB(void *pvParameters);
void serialparser(void *pvParameters);
void displayui(void *pvParameters);
String hostname = "";
/* InfluxDB Connection Parameters */
#define WIFI_SSID ""
#define WIFI_PASSWORD ""

#define INFLUXDB_URL ""
#define INFLUXDB_TOKEN ""
#define INFLUXDB_ORG ""
// InfluxDB v2 bucket name (Use: InfluxDB UI ->  Data -> Buckets)
#define INFLUXDB_BUCKET ""
#define TZ_INFO ""

/* InfluxDB Data Points */
Point Reset("Reset");
Point Status("Status");
Point Battery("Battery");
Point Environment("Environment");
Point Motion("Motion");
Point VOC("VOC");
Point Detector("Detector");
Point CO2("CO2");
Point ALS("ALS");
Point LTS("LTS");
Point GATEWAY("GATEWAY");
Point NC("NC");

/* Queue Handles */
static int queue_len = 10;
static QueueHandle_t rst_queue;
static QueueHandle_t stat_queue;
static QueueHandle_t batt_queue;
static QueueHandle_t env_queue;
static QueueHandle_t mot_queue;
static QueueHandle_t det_queue;
static QueueHandle_t co2_queue;
static QueueHandle_t als_queue;
static QueueHandle_t voc_queue;
static QueueHandle_t lts_queue;

/* Task Handles */
static TaskHandle_t ifdb;
static TaskHandle_t serp;
static TaskHandle_t disp;

/* Timer Handles */
TimerHandle_t displaySubTimer;
TimerHandle_t gatewayUpdateTimer;
TimerHandle_t scanDeviceIntTimer;
TimerHandle_t scanDeviceDurTimer;

#define DISPLAY_UPDATE_RATE_MS 2500
#define SELF_REPORT_INTERVAL_MS 5000
#define DEVICE_SCAN_INTERVAL 1800000
#define DEVICE_SCAN_DURATION 600000

/* Data Structures */
struct rst_struct
{
  String uid;
  uint32_t rst;
};
struct stat_struct
{
  String uid;
  float alive;
  float rssi;
  float retx;
};
struct batt_struct
{
  String uid;
  float volt;
};
struct env_struct
{
  String uid;
  float light;
  float temp;
  float press;
  float hum;
};
struct mot_struct
{
  String uid;
  float x;
  float y;
  float z;
  float shock_dur;
  float shock_count;
  float step_count;
};
struct det_struct
{
  String uid;
  float distance;
  float cal_point;
  float cal_tolerance;
  float range_mode;
  float count;
  float warning;
};
struct co2_struct
{
  String uid;
  float temp;
  float hum;
  float press;
  float co2;
  float stat;
};
struct als_struct
{
  String uid;
  float lux;
  float lux2;
  float w;
  float uvi;
};
struct voc_struct
{
  String uid;
  float light;
  float temp;
  float press;
  float hum;
  float voc;
};
struct lts_struct
{
  String uid;
  float temp;
  float hum;
};
/* WMNS Alive Packet Device-Specific TXN Rate/h */
float alive_rate_r = 0.0083;  // for routers
float alive_rate_lp = 0.0833; // for all other LP nodes

/* GATEWAY Status Information */
#define SELF_UID 0000
#define SELF_CH 50
struct self_struct
{
  String uid = (String)SELF_UID;
  String ch = (String)SELF_CH;
  float temp;
  float hum;
  int rssi;
  uint32_t COUNT_INTRUSIONTRIG = 0;
  uint32_t COUNT_QUEUEOVERFLOW = 0;     //QOV
  uint32_t COUNT_QUEUEWRITEFAIL = 0;    //QWF
  uint32_t COUNT_RXPIDOUTOFBOUNDS = 0;  //PXB
  uint32_t COUNT_RXUIDOUTOFBOUNDS = 0;  //UXB
  uint32_t COUNT_IFDBWRITEFAIL = 0;     //IFF
  uint32_t COUNT_TOTALTXNFROMSTART = 0; //TXN
  uint8_t COUNT_NUM_LP = 0;
  uint8_t COUNT_NUM_R = 0;
  uint8_t COUNT_NUM_G = 0;
  uint32_t espnow_fail = 0;
} self;

/* Strings to store incoming UART data */
String uart_recv_buffer;

/* Global variables for serial parser */
String uart_recv_pid;
String uart_recv_uid;
int pid;
int uid;

/* Global variables for display UI */
bool s_timer = true;
long int startup_time_rtc;

/* Global variables for device counter */
int uid_lp[255] = {};
int uid_r[255] = {};
int uid_g[255] = {};
int uid_lp_pos = 1;
int uid_r_pos = 1;
int uid_g_pos = 1;
int uid_temp[3] = {};
bool scan = false;

/* Global variables for intrusion protection */
bool motion = false;

/* Global variables for self update */
bool update_self = false;

/* Pin configuration */
#define UART2_TX 27
#define UART2_RX 25
#define LED1 2
#define LED2 4
#define LED3 23
#define LED4 26
#define SDA2 19
#define SCL2 18
#define SDA1 17
#define SCL1 16

/* Comms configuration */
#define UART_BAUDRATE 115200
#define I2C_CLK_RATE 100000

/* Registered PIDs in the mesh network */
typedef enum
{
  PID_RESET = 0,
  PID_STATUS,
  PID_BATTERY,
  PID_ENVIRONMENT = 176,
  PID_MOTION,
  PID_VOC,
  PID_DETECTOR,
  PID_CO2,
  PID_ALS,
  PID_LTS
} dev_pid_t;

/* Bitmaps for display */
const unsigned char symbol[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x3f, 0xe0, 0x00, 0x00, 0x7f, 0xfc, 0x00,
    0x00, 0xf6, 0xff, 0xc0, 0x01, 0xe3, 0x1f, 0xf0, 0x03, 0xc1, 0x83, 0xf0, 0x0f, 0x80, 0xc7, 0xf0,
    0x1f, 0x00, 0x7c, 0x38, 0x3c, 0x00, 0xe0, 0x38, 0x78, 0x01, 0xa0, 0x38, 0x70, 0x03, 0x20, 0x38,
    0x78, 0x06, 0x30, 0x1c, 0x7c, 0x0c, 0x10, 0x1c, 0x7e, 0x18, 0x10, 0x1c, 0x3a, 0x30, 0x10, 0x1c,
    0x39, 0xe0, 0x18, 0x1c, 0x38, 0xe0, 0x18, 0x0e, 0x38, 0xfc, 0x08, 0x0e, 0x1d, 0x87, 0x88, 0x1e,
    0x1d, 0x00, 0xff, 0xfe, 0x1d, 0x00, 0x1e, 0x3c, 0x1f, 0x00, 0x08, 0x78, 0x0e, 0x00, 0x08, 0xf0,
    0x0e, 0x00, 0x19, 0xe0, 0x0f, 0x80, 0x13, 0xc0, 0x0f, 0xf0, 0x17, 0x80, 0x03, 0xff, 0x3f, 0x00,
    0x00, 0x7f, 0xfe, 0x00, 0x00, 0x0f, 0xfc, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00};

const unsigned char wifi[] PROGMEM = {
    0x00, 0x00, 0x07, 0xe0, 0x1f, 0xf8, 0x38, 0x3c, 0xf3, 0x8f, 0xcf, 0xf3, 0x1e, 0x78, 0x38, 0x18,
    0x07, 0xc0, 0x07, 0xe0, 0x00, 0x00, 0x01, 0x80, 0x01, 0x80, 0x00, 0x00};

const unsigned char aux_linked[] PROGMEM = {
	0x00, 0x10, 0x00, 0x7c, 0x01, 0xe6, 0x01, 0xc6, 0x00, 0x02, 0x0f, 0xc6, 0x1f, 0xee, 0x38, 0x1c, 
	0x77, 0x78, 0xe3, 0xf0, 0xc0, 0x00, 0xe3, 0x80, 0xe7, 0x80, 0x3e, 0x00, 0x00, 0x00};

/* Objects */
DFRobot_SHT20 sht20;
PCF85063A rtc;
WiFiServer server(80);
Adafruit_SSD1306 display(128, 64, &Wire, 4);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if (status != 0)
  {
    self.espnow_fail++;
  }
}
void IRAM_ATTR MOTION_ISR(void)
{
  motion = true;
  Serial.println("[M_ISR] MOTION DETECTED");
}
void vTimerCallback1(TimerHandle_t displaySubTimer)
{
  s_timer = !s_timer;
}
void vTimerCallback2(TimerHandle_t gatewayUpdateTimer)
{
  update_self = true;
}

void vTimerCallback3(TimerHandle_t scanDeviceIntTimer)
{
  Serial.println("[VTIMERCALLBACK3] SCAN INITIATED");
  scan = true;
  uid_temp[0] = uid_lp_pos - 1;
  uid_temp[1] = uid_r_pos - 1;
  uid_temp[2] = uid_g_pos - 1;
  for (int i = 0; i < 255; i++)
  {
    uid_lp[i] = 0;
    uid_r[i] = 0;
    uid_g[i] = 0;
  }
  uid_lp_pos = 1;
  uid_r_pos = 1;
  uid_g_pos = 1;
}
void vTimerCallback4(TimerHandle_t scanDeviceDurTimer)
{
  Serial.println("[VTIMERCALLBACK4] SCAN TERMINATED");
  scan = false;
  uid_temp[0] = 0;
  uid_temp[1] = 0;
  uid_temp[2] = 0;
  xTimerStop(scanDeviceDurTimer, 10);
}

/*! setup() :: TASK
   @brief initializes peripherals, database connection and starts tasks influxDB() and serialparser()

   @note TASK no pin

   @param void
*/
void setup()
{

  Serial.begin(UART_BAUDRATE);
  Serial2.begin(UART_BAUDRATE, SERIAL_8N1, UART2_RX, UART2_TX); // initialize comms with nRF52
  Wire.begin(SDA1, SCL1, (uint32_t)I2C_CLK_RATE);                         // initialize I2C for display (I2C0)
  Wire1.begin(SDA2, SCL2, (uint32_t)I2C_CLK_RATE);                        // initialize I2C for RTC and temp/hum sensor

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(255);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(10, 30);
  display.print("initializing ...");
  display.display();

  Serial.println("[INIT] STARTUP INITIALIZATION OK");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(hostname.c_str()); //define hostname
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("[INIT] ERROR CONNECTING TO WIFI");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    vTaskDelay(2000);
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
  // Accurate time is necessary for certificate validation and writing in batches
  // For the fastest time sync find NTP servers in your area: https://www.pool.ntp.org/zone/
  // Syncing progress and the time will be printed to Serial.
  


   configTzTime("SGT-8", "pool.ntp.org", "time.nis.gov");
  client.setHTTPOptions(HTTPOptions().httpReadTimeout(200));
  client.setHTTPOptions(HTTPOptions().connectionReuse(true));
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
  Serial.println("[INIT] IFDB CONNECTION OK");

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("[INIT] Error initializing ESP-NOW");
    return;
  }
  /***************************************************************************************************************/
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, bc_addr, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("[INIT] Failed to add peer");
  }
  Serial.println("[INIT] Connected to auxiliary");
  display.drawBitmap(85, 50, aux_linked, 15, 15, SSD1306_WHITE);
  display.display();

  sht20.initSHT20(Wire1);
  Serial.println("[INIT] TEMP SENSOR OK");

  BMI160.begin(BMI160GenClass::I2C_MODE, 0x68, 5);
  BMI160.attachInterrupt(MOTION_ISR);
  BMI160.setIntMotionEnabled(true);
  Serial.println("[INIT] MOTION SENSOR OK");

  rst_queue = xQueueCreate(queue_len, sizeof(struct rst_struct));
  stat_queue = xQueueCreate(queue_len, sizeof(struct stat_struct));
  batt_queue = xQueueCreate(queue_len, sizeof(struct batt_struct));
  env_queue = xQueueCreate(queue_len, sizeof(struct env_struct));
  mot_queue = xQueueCreate(queue_len, sizeof(struct mot_struct));
  det_queue = xQueueCreate(queue_len, sizeof(struct det_struct));
  co2_queue = xQueueCreate(queue_len, sizeof(struct co2_struct));
  als_queue = xQueueCreate(queue_len, sizeof(struct als_struct));
  voc_queue = xQueueCreate(queue_len, sizeof(struct voc_struct));
  lts_queue = xQueueCreate(queue_len, sizeof(struct lts_struct));
  Serial.println("[INIT] IFDB QUEUES CREATED");

  gatewayUpdateTimer = xTimerCreate("Timer2", SELF_REPORT_INTERVAL_MS, pdTRUE, (void *)0, vTimerCallback2);
  xTimerStart(gatewayUpdateTimer, 0);

  Serial.println("[INIT] IFDB TIMERS STARTED");

  xTaskCreatePinnedToCore( // Use xTaskCreate()
      influxDB,            // Function to be called
      "Influx DB",         // Name of task
      32768,               // Stack size
      NULL,                // Parameter to pass
      1,                   // Task priority
      &ifdb,               // Task handle
      0);                  // CPU Core

  Serial.println("[INIT] TASK IFDB CREATED");
  xTaskCreatePinnedToCore( // Use xTaskCreate()
      serialparser,        // Function to be called
      "Serial Parser",     // Name of task
      16384,               // Stack size
      NULL,                // Parameter to pass
      1,                   // Task priority
      &serp,               // Task handle
      1);                  // CPU Core
  Serial.println("[INIT] TASK SERP CREATED");
  vTaskDelete(NULL);
  Serial.println("[INIT] TASK INIT TERMINATED");
}

/*! influxDB() :: TASK
   @brief receives data structs from queues and posts data to InfluxDB.

   @note HIGHEST PRIORITY TASK pinned to core 0

   @param pvParameters task structure
*/
void influxDB(void *pvParameters)
{

  xTaskCreatePinnedToCore( // Use xTaskCreate()
      displayui,           // Function to be called
      "Display UI",        // Name of task
      4096,                // Stack size
      NULL,                // Parameter to pass
      0,                   // Task priority
      &disp,               // Task handle
      0);                  // CPU Core*/

  Serial.println("[IFDB] STARTED TASK");
  while (true)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("[IFDB] WIFI RECONNECTION ATTEMPTED");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      vTaskDelay(2000);
    }

    if (uxQueueMessagesWaiting(rst_queue) > 0)
    {
      struct rst_struct RST_OUT;
      if (xQueueReceive(rst_queue, (void *)&RST_OUT, 1) == pdPASS)
      {
        Serial.print(RST_OUT.rst);
        Reset.clearFields();
        Reset.clearTags();
        Reset.addTag("UID", (RST_OUT.uid));
        if (RST_OUT.rst == 0)
        {
          Reset.addField("Reset", "PWRRESET");
        }
        else if (RST_OUT.rst == 1)
        {
          Reset.addField("Reset", "SWRESET");
        }
        else if (RST_OUT.rst == 2)
        {
          Reset.addField("Reset", "WATCHDOG");
        }
        else if (RST_OUT.rst == 4)
        {
          Reset.addField("Reset", "LOCKUP");
        }
        else if (RST_OUT.rst == 65536)
        {
          Reset.addField("Reset", "GPIOWAKE");
        }
        else if (RST_OUT.rst == 131072)
        {
          Reset.addField("Reset", "COMPWAKE");
        }
        else if (RST_OUT.rst == 262144)
        {
          Reset.addField("Reset", "DEBUGWAKE");
        }
        else if (RST_OUT.rst == 524288)
        {
          Reset.addField("Reset", "NFCWAKE");
        }
        if (!client.writePoint(Reset))
        {
          self.COUNT_IFDBWRITEFAIL++;
        }
        self.COUNT_TOTALTXNFROMSTART++;
        Serial.print("[IFDB] TXN ");
        Serial.print(self.COUNT_TOTALTXNFROMSTART);
        Serial.println(" RST COMPLETED");
      }
    }

    if (uxQueueMessagesWaiting(stat_queue) > 0)
    {
      struct stat_struct STAT_OUT;
      if (xQueueReceive(stat_queue, (void *)&STAT_OUT, 1) == pdPASS)
      {
        Status.clearFields();
        Status.clearTags();
        Status.addTag("UID", (STAT_OUT.uid));
        float mul;
        if (((STAT_OUT.uid.toInt()) >= 16 && (STAT_OUT.uid.toInt()) <= 255))
        {
          mul = alive_rate_r;
        }
        else
        {
          mul = alive_rate_lp;
        }
        Status.addField("Alive", (STAT_OUT.alive) * mul);
        Status.addField("RSSI", (STAT_OUT.rssi));
        Status.addField("Retransmissions", (STAT_OUT.retx));
        if (!client.writePoint(Status))
        {
          self.COUNT_IFDBWRITEFAIL++;
        }
        self.COUNT_TOTALTXNFROMSTART++;
        Serial.print("[IFDB] TXN ");
        Serial.print(self.COUNT_TOTALTXNFROMSTART);
        Serial.println(" STAT COMPLETED");
      }
    }

    if (uxQueueMessagesWaiting(batt_queue) > 0)
    {
      struct batt_struct BATT_OUT;
      if (xQueueReceive(batt_queue, (void *)&BATT_OUT, 1) == pdPASS)
      {
        Battery.clearFields();
        Battery.clearTags();
        Battery.addTag("UID", (BATT_OUT.uid));
        Battery.addField("Voltage", (BATT_OUT.volt));
        if (!client.writePoint(Battery))
        {
          self.COUNT_IFDBWRITEFAIL++;
        }
        self.COUNT_TOTALTXNFROMSTART++;
        Serial.print("[IFDB] TXN ");
        Serial.print(self.COUNT_TOTALTXNFROMSTART);
        Serial.println(" BATT COMPLETED");
      }
    }

    if (uxQueueMessagesWaiting(env_queue) > 0)
    {
      struct env_struct ENV_OUT;
      if (xQueueReceive(env_queue, (void *)&ENV_OUT, 1) == pdPASS)
      {
        Environment.clearFields();
        Environment.clearTags();
        Environment.addTag("UID", (ENV_OUT.uid));
        Environment.addField("Temperature", (ENV_OUT.temp));
        Environment.addField("Humidity", (ENV_OUT.hum));
        Environment.addField("Pressure", (ENV_OUT.press));
        Environment.addField("Light", (ENV_OUT.light));
        if (!client.writePoint(Environment))
        {
          self.COUNT_IFDBWRITEFAIL++;
        }
        self.COUNT_TOTALTXNFROMSTART++;
        Serial.print("[IFDB] TXN ");
        Serial.print(self.COUNT_TOTALTXNFROMSTART);
        Serial.println(" ENV COMPLETED");
      }
    }

    if (uxQueueMessagesWaiting(mot_queue) > 0)
    {
      struct mot_struct MOT_OUT;
      if (xQueueReceive(mot_queue, (void *)&MOT_OUT, 1) == pdPASS)
      {
        Motion.clearFields();
        Motion.clearTags();
        Motion.addTag("UID", (MOT_OUT.uid));
        Motion.addField("X-axis", (MOT_OUT.x));
        Motion.addField("Y-axis", (MOT_OUT.y));
        Motion.addField("Z-axis", (MOT_OUT.z));
        Motion.addField("Impact counter", (MOT_OUT.shock_count));
        Motion.addField("Duration of impacts", (MOT_OUT.shock_dur));
        Motion.addField("Step counter", (MOT_OUT.step_count));
        if (!client.writePoint(Motion))
        {
          self.COUNT_IFDBWRITEFAIL++;
        }
        self.COUNT_TOTALTXNFROMSTART++;
        Serial.print("[IFDB] TXN ");
        Serial.print(self.COUNT_TOTALTXNFROMSTART);
        Serial.println(" MOT COMPLETED");
      }
    }

    if (uxQueueMessagesWaiting(det_queue) > 0)
    {
      struct det_struct DET_OUT;
      if (xQueueReceive(det_queue, (void *)&DET_OUT, 1) == pdPASS)
      {
        Serial.print(DET_OUT.cal_point);
        Serial.print(DET_OUT.cal_tolerance);
        Detector.clearFields();
        Detector.clearTags();
        Detector.addTag("UID", (DET_OUT.uid));
        Detector.addField("Distance", (DET_OUT.distance));
        Detector.addField("Max. Tolerance", (DET_OUT.cal_point) + (DET_OUT.cal_tolerance));
        Detector.addField("Min. Tolerance", (DET_OUT.cal_point) - (DET_OUT.cal_tolerance));
        Detector.addField("Mode", (DET_OUT.range_mode));
        Detector.addField("Counter", (DET_OUT.count));
        if (!client.writePoint(Detector))
        {
          self.COUNT_IFDBWRITEFAIL++;
        }
        self.COUNT_TOTALTXNFROMSTART++;
        Serial.print("[IFDB] TXN ");
        Serial.print(self.COUNT_TOTALTXNFROMSTART);
        Serial.println(" DET COMPLETED");
      }
    }

    if (uxQueueMessagesWaiting(co2_queue) > 0)
    {
      struct co2_struct CO2_OUT;
      if (xQueueReceive(co2_queue, (void *)&CO2_OUT, 1) == pdPASS)
      {
        CO2.clearFields();
        CO2.clearTags();
        CO2.addTag("UID", (CO2_OUT.uid));
        CO2.addField("CO2", (CO2_OUT.co2));
        CO2.addField("Temperature", (CO2_OUT.temp));
        CO2.addField("Humidity", (CO2_OUT.hum));
        if (!client.writePoint(CO2))
        {
          self.COUNT_IFDBWRITEFAIL++;
        }
        self.COUNT_TOTALTXNFROMSTART++;
        Serial.print("[IFDB] TXN ");
        Serial.print(self.COUNT_TOTALTXNFROMSTART);
        Serial.println(" CO2 COMPLETED");
      }
    }

    if (uxQueueMessagesWaiting(als_queue) > 0)
    {
      struct als_struct ALS_OUT;
      if (xQueueReceive(als_queue, (void *)&ALS_OUT, 1) == pdPASS)
      {
        ALS.clearFields();
        ALS.clearTags();
        ALS.addTag("UID", (ALS_OUT.uid));
        ALS.addField("ALS1", (ALS_OUT.lux));
        ALS.addField("ALS2", (ALS_OUT.lux2));
        ALS.addField("W", (ALS_OUT.w));
        ALS.addField("UVI", (ALS_OUT.uvi));
        if (!client.writePoint(ALS))
        {
          self.COUNT_IFDBWRITEFAIL++;
        }
        self.COUNT_TOTALTXNFROMSTART++;
        Serial.print("[IFDB] TXN ");
        Serial.print(self.COUNT_TOTALTXNFROMSTART);
        Serial.println(" ALS COMPLETED");
      }
    }

    if (uxQueueMessagesWaiting(voc_queue) > 0)
    {
      struct voc_struct VOC_OUT;
      if (xQueueReceive(voc_queue, (void *)&VOC_OUT, 1) == pdPASS)
      {
        VOC.clearFields();
        VOC.clearTags();
        VOC.addTag("UID", (VOC_OUT.uid));
        VOC.addField("Temperature", (VOC_OUT.temp));
        VOC.addField("Humidity", (VOC_OUT.hum));
        VOC.addField("Pressure", (VOC_OUT.press));
        VOC.addField("Light", (VOC_OUT.light));
        VOC.addField("VOC", (VOC_OUT.voc));
        if (!client.writePoint(VOC))
        {
          self.COUNT_IFDBWRITEFAIL++;
        }
        self.COUNT_TOTALTXNFROMSTART++;
        Serial.print("[IFDB] TXN ");
        Serial.print(self.COUNT_TOTALTXNFROMSTART);
        Serial.println(" VOC COMPLETED");
      }
    }

    if (uxQueueMessagesWaiting(lts_queue) > 0)
    {
      struct lts_struct LTS_OUT;
      if (xQueueReceive(lts_queue, (void *)&LTS_OUT, 1) == pdPASS)
      {
        LTS.clearFields();
        LTS.clearTags();
        LTS.addTag("UID", (LTS_OUT.uid));
        LTS.addField("Temperature", (LTS_OUT.temp));
        LTS.addField("Humidity", (LTS_OUT.hum));
        if (!client.writePoint(LTS))
        {
          self.COUNT_IFDBWRITEFAIL++;
        }
        self.COUNT_TOTALTXNFROMSTART++;
        Serial.print("[IFDB] TXN ");
        Serial.print(self.COUNT_TOTALTXNFROMSTART);
        Serial.println(" LTS COMPLETED");
      }
    }

    if (update_self == true)
    {
      int dta = 201;
      esp_err_t result = esp_now_send(bc_addr, (uint8_t *)&dta, sizeof(dta));
      Serial.print("[IFDB] AUX UPDATED");
      update_self = false;
      GATEWAY.clearFields();
      GATEWAY.clearTags();
      GATEWAY.addTag("UID", (self.uid));
      GATEWAY.addField("RF Channel", (self.ch).toFloat());
      GATEWAY.addField("WiFi RSSI", (self.rssi));
      GATEWAY.addField("Intrusion Count", (self.COUNT_INTRUSIONTRIG));
      GATEWAY.addField("Queue Overflow Count", (self.COUNT_QUEUEOVERFLOW));
      GATEWAY.addField("Queue Write Fail Count", (self.COUNT_QUEUEWRITEFAIL));
      GATEWAY.addField("Received PID Out-of-Range Count", (self.COUNT_RXPIDOUTOFBOUNDS));
      GATEWAY.addField("Received UID Out-of-Range Count", (self.COUNT_RXUIDOUTOFBOUNDS));
      GATEWAY.addField("InfluxDB Write Fail Count", (self.COUNT_IFDBWRITEFAIL));
      GATEWAY.addField("Total InfluxDB Transaction Count", (self.COUNT_TOTALTXNFROMSTART));
      GATEWAY.addField("Board Temperature", (self.temp));
      GATEWAY.addField("Board Humidity", (self.hum));
      GATEWAY.addField("LP Device Count", (self.COUNT_NUM_LP));
      GATEWAY.addField("R Device Count", (self.COUNT_NUM_R));
      GATEWAY.addField("G Device Count", (self.COUNT_NUM_G + 1));
      GATEWAY.addField("ESPNOW Fail Count", (self.espnow_fail));
      if (!client.writePoint(GATEWAY))
      {
        self.COUNT_IFDBWRITEFAIL++;
      }
      self.COUNT_TOTALTXNFROMSTART++;
      Serial.print("[IFDB] TXN ");
      Serial.print(self.COUNT_TOTALTXNFROMSTART);
      Serial.println(" GATEWAY COMPLETED");
      Serial.println();
    }

    vTaskDelay(10);
    digitalWrite(LED1, !digitalRead(LED1));
  }
}

/*! serialparser() :: TASK
   @brief captures serial data from nRF52832, parses serial data into their respective data structs and send to queue.

   @note HIGHEST PRIORITY TASK pinned to core 1

   @param pvParameters task structure
*/

void serialparser(void *pvParameters)
{

  scanDeviceIntTimer = xTimerCreate("Timer3", DEVICE_SCAN_INTERVAL, pdTRUE, (void *)0, vTimerCallback3);
  scanDeviceDurTimer = xTimerCreate("Timer4", DEVICE_SCAN_DURATION, pdTRUE, (void *)0, vTimerCallback4);
  xTimerStart(scanDeviceIntTimer, 0);

  while (true)
  {
    if (Serial2.available() > 0 && Serial2.read() == '>')
    {
      uart_recv_buffer = Serial2.readStringUntil('<');
      Serial.println("[SERP] UART2 DATA RECEIVED");
      Serial2.flush();
      Serial.println(uart_recv_buffer);
      Serial.println("[SERP] DEVLIST UPDATED");
      uart_recv_pid = uart_recv_buffer.substring(uart_recv_buffer.indexOf("pid:") + 4, uart_recv_buffer.indexOf(";ctrl:"));
      uart_recv_uid = uart_recv_buffer.substring(uart_recv_buffer.indexOf("src:") + 4, uart_recv_buffer.indexOf(";", uart_recv_buffer.indexOf("src:")));
      pid = uart_recv_pid.toInt();
      uid = uart_recv_uid.toInt();
      if (scan == true)
      {
        scandevices();
        xTimerStart(scanDeviceDurTimer, 0);
      }
      String recv_buffer = uart_recv_buffer;
      if (uid > 0 && uid < 65535)
      {
        digitalWrite(LED4, !digitalRead(LED4));

        switch (pid)
        {
        case PID_RESET:
        {
          struct rst_struct RST_IN;
          RST_IN.uid = (String)uid;
          RST_IN.rst = (recv_buffer.substring(recv_buffer.indexOf("reset:") + 6)).toInt();
          if (uxQueueSpacesAvailable(rst_queue) == 0)
          {
            xQueueReset(rst_queue);
            self.COUNT_QUEUEOVERFLOW++;
          }
          if (xQueueSend(rst_queue, (void *)&RST_IN, 1) != pdPASS)
          {
            self.COUNT_QUEUEWRITEFAIL++;
          }
          break;
        }

        case PID_STATUS:
        {
          struct stat_struct STAT_IN;
          STAT_IN.uid = (String)uid;
          STAT_IN.alive = (recv_buffer.substring(recv_buffer.indexOf("alive:") + 6, recv_buffer.indexOf(";nb:"))).toFloat();
          STAT_IN.rssi = (recv_buffer.substring(recv_buffer.indexOf(",") + 1, recv_buffer.indexOf(",") + 4)).toFloat();
          STAT_IN.retx = (recv_buffer.substring(recv_buffer.indexOf("nb:") + 3, recv_buffer.indexOf(";rx1:"))).toFloat();
          if (uxQueueSpacesAvailable(stat_queue) == 0)
          {
            xQueueReset(stat_queue);
            self.COUNT_QUEUEOVERFLOW++;
          }
          if (xQueueSend(stat_queue, (void *)&STAT_IN, 1) != pdPASS)
          {
            self.COUNT_QUEUEWRITEFAIL++;
          }
          break;
        }
        case PID_BATTERY:
        {
          struct batt_struct BATT_IN;
          BATT_IN.uid = (String)uid;
          BATT_IN.volt = (recv_buffer.substring(recv_buffer.indexOf("voltage:") + 8, recv_buffer.indexOf("voltage:") + 12)).toFloat();
          if (uxQueueSpacesAvailable(batt_queue) == 0)
          {
            xQueueReset(batt_queue);
            self.COUNT_QUEUEOVERFLOW++;
          }
          if (xQueueSend(batt_queue, (void *)&BATT_IN, 1) != pdPASS)
          {
            self.COUNT_QUEUEWRITEFAIL++;
          }
          break;
        }
        case PID_ENVIRONMENT:
        {
          struct env_struct ENV_IN;
          ENV_IN.uid = (String)uid;
          ENV_IN.temp = (recv_buffer.substring(recv_buffer.indexOf("temp:") + 5, recv_buffer.indexOf(";hum:"))).toFloat();
          ENV_IN.hum = (recv_buffer.substring(recv_buffer.indexOf("hum:") + 4, recv_buffer.indexOf(";press:"))).toFloat();
          ENV_IN.press = (recv_buffer.substring(recv_buffer.indexOf("press:") + 6, recv_buffer.indexOf(";light:"))).toFloat();
          ENV_IN.light = (recv_buffer.substring(recv_buffer.indexOf("light:") + 6, recv_buffer.length())).toFloat();
          if (uxQueueSpacesAvailable(env_queue) == 0)
          {
            xQueueReset(env_queue);
            self.COUNT_QUEUEOVERFLOW++;
          }
          if (xQueueSend(env_queue, (void *)&ENV_IN, 1) != pdPASS)
          {
            self.COUNT_QUEUEWRITEFAIL++;
          }
          break;
        }

        case PID_MOTION:
        {
          struct mot_struct MOT_IN;
          MOT_IN.uid = (String)uid;
          MOT_IN.x = (recv_buffer.substring(recv_buffer.indexOf("x:") + 2, recv_buffer.indexOf(";y:"))).toFloat();
          MOT_IN.y = (recv_buffer.substring(recv_buffer.indexOf("y:") + 2, recv_buffer.indexOf(";z:"))).toFloat();
          MOT_IN.z = (recv_buffer.substring(recv_buffer.indexOf("z:") + 2, recv_buffer.indexOf(";shockdur:"))).toFloat();
          MOT_IN.shock_dur = (recv_buffer.substring(recv_buffer.indexOf("shockdur:") + 9, recv_buffer.indexOf(";shockcount:"))).toFloat();
          MOT_IN.shock_count = (recv_buffer.substring(recv_buffer.indexOf("shockcount:") + 11, recv_buffer.indexOf(";stepcount:"))).toFloat();
          MOT_IN.step_count = (recv_buffer.substring(recv_buffer.indexOf("stepcount:") + 10, recv_buffer.indexOf(";sensetemp:"))).toFloat();
          if (uxQueueSpacesAvailable(mot_queue) == 0)
          {
            xQueueReset(mot_queue);
            self.COUNT_QUEUEOVERFLOW++;
          }
          if (xQueueSend(mot_queue, (void *)&MOT_IN, 1) != pdPASS)
          {
            self.COUNT_QUEUEWRITEFAIL++;
          }
          break;
        }
        case PID_DETECTOR:
        {
          struct det_struct DET_IN;
          DET_IN.uid = (String)uid;
          DET_IN.distance = (recv_buffer.substring(recv_buffer.indexOf("distance:") + 9, recv_buffer.indexOf(";cal:"))).toFloat();
          DET_IN.cal_point = (recv_buffer.substring(recv_buffer.indexOf("cal:") + 4, recv_buffer.indexOf(";tol:"))).toFloat();
          DET_IN.cal_tolerance = (recv_buffer.substring(recv_buffer.indexOf("tol:") + 4, recv_buffer.indexOf(";mode:"))).toFloat();
          DET_IN.range_mode = (recv_buffer.substring(recv_buffer.indexOf("mode:") + 5, recv_buffer.indexOf(";count:"))).toFloat();
          DET_IN.count = (recv_buffer.substring(recv_buffer.indexOf("count:") + 6, recv_buffer.indexOf(";warn:"))).toFloat();
          DET_IN.warning = (recv_buffer.substring(recv_buffer.indexOf("warn:") + 5, recv_buffer.length())).toFloat();
          if (uxQueueSpacesAvailable(det_queue) == 0)
          {
            xQueueReset(det_queue);
            self.COUNT_QUEUEOVERFLOW++;
          }
          if (xQueueSend(det_queue, (void *)&DET_IN, 1) != pdPASS)
          {
            self.COUNT_QUEUEWRITEFAIL++;
          }
          break;
        }

        case PID_CO2:
        {
          struct co2_struct CO2_IN;
          CO2_IN.uid = (String)uid;
          CO2_IN.temp = (recv_buffer.substring(recv_buffer.indexOf("temp:") + 5, recv_buffer.indexOf(";hum:"))).toFloat();
          CO2_IN.hum = (recv_buffer.substring(recv_buffer.indexOf("hum:") + 4, recv_buffer.indexOf(";press:"))).toFloat();
          CO2_IN.press = (recv_buffer.substring(recv_buffer.indexOf("press:") + 6, recv_buffer.indexOf(";co2:"))).toFloat();
          CO2_IN.co2 = (recv_buffer.substring(recv_buffer.indexOf("co2:") + 4, recv_buffer.indexOf(";status:"))).toFloat();
          CO2_IN.stat = (recv_buffer.substring(recv_buffer.indexOf("status:") + 7, recv_buffer.length())).toFloat();

          if (uxQueueSpacesAvailable(co2_queue) == 0)
          {
            xQueueReset(co2_queue);
            self.COUNT_QUEUEOVERFLOW++;
          }
          if (xQueueSend(co2_queue, (void *)&CO2_IN, 1) != pdPASS)
          {
            self.COUNT_QUEUEWRITEFAIL++;
          }
          break;
        }

        case PID_ALS:
        {
          struct als_struct ALS_IN;
          ALS_IN.uid = (String)uid;
          ALS_IN.lux = (recv_buffer.substring(recv_buffer.indexOf("als1:") + 5, recv_buffer.indexOf(";als2:"))).toFloat();
          ALS_IN.lux2 = (recv_buffer.substring(recv_buffer.indexOf("als2:") + 5, recv_buffer.indexOf(";w:"))).toFloat();
          ALS_IN.w = (recv_buffer.substring(recv_buffer.indexOf("w:") + 2, recv_buffer.indexOf(";uvi:"))).toFloat();
          ALS_IN.uvi = (recv_buffer.substring(recv_buffer.indexOf("uvi:") + 4, recv_buffer.length())).toFloat();
          if (uxQueueSpacesAvailable(als_queue) == 0)
          {
            xQueueReset(als_queue);
            self.COUNT_QUEUEOVERFLOW++;
          }
          if (xQueueSend(als_queue, (void *)&ALS_IN, 1) != pdPASS)
          {
            self.COUNT_QUEUEWRITEFAIL++;
          }
          break;
        }

        case PID_VOC:
        {
          struct voc_struct VOC_IN;
          VOC_IN.uid = (String)uid;
          VOC_IN.temp = (recv_buffer.substring(recv_buffer.indexOf("temp:") + 5, recv_buffer.indexOf(";hum:"))).toFloat();
          VOC_IN.hum = (recv_buffer.substring(recv_buffer.indexOf("hum:") + 4, recv_buffer.indexOf(";press:"))).toFloat();
          VOC_IN.press = (recv_buffer.substring(recv_buffer.indexOf("press:") + 6, recv_buffer.indexOf(";light:"))).toFloat();
          VOC_IN.light = (recv_buffer.substring(recv_buffer.indexOf("light:") + 6, recv_buffer.indexOf(";voc:"))).toFloat();
          VOC_IN.voc = (recv_buffer.substring(recv_buffer.indexOf("voc:") + 4, recv_buffer.length())).toFloat();
          if (uxQueueSpacesAvailable(voc_queue) == 0)
          {
            xQueueReset(voc_queue);
            self.COUNT_QUEUEOVERFLOW++;
          }

          if (xQueueSend(voc_queue, (void *)&VOC_IN, 1) != pdPASS)
          {
            self.COUNT_QUEUEWRITEFAIL++;
          }
          break;
        }

        case PID_LTS:
        {
          struct lts_struct LTS_IN;
          LTS_IN.uid = (String)uid;
          LTS_IN.temp = (recv_buffer.substring(recv_buffer.indexOf("temp:") + 5, recv_buffer.indexOf(";hum:"))).toFloat();
          LTS_IN.hum = (recv_buffer.substring(recv_buffer.indexOf("hum:") + 4, recv_buffer.indexOf(";press:"))).toFloat();
          if (uxQueueSpacesAvailable(lts_queue) == 0)
          {
            xQueueReset(lts_queue);
            self.COUNT_QUEUEOVERFLOW++;
          }

          if (xQueueSend(lts_queue, (void *)&LTS_IN, 1) != pdPASS)
          {
            self.COUNT_QUEUEWRITEFAIL++;
          }
          break;
        }

        default:
          self.COUNT_RXPIDOUTOFBOUNDS++;
          break;
        }
      }
      else
      {
        self.COUNT_RXUIDOUTOFBOUNDS++;
      }
      Serial.print("[SERP] PARSING COMPLETED");
    }
  }
}

/*! influxDB() :: TASK
   @brief updates display with device, error and network information

   @note LOW PRIORITY TASK pinned to core 0

   @param pvParameters task structure
*/
void displayui(void *pvParameters)
{
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(35);
  display.clearDisplay();
  display.setTextColor(BLACK, WHITE);
  display.setCursor(0, 0);
  display.print("       TERMINAL      ");
  display.display();
  display.setTextColor(WHITE);
  display.setCursor(0, 15);
  display.print("[DISP] system ok");
  display.display();
  vTaskDelay(200);
  display.clearDisplay();
  displaySubTimer = xTimerCreate("Timer1", DISPLAY_UPDATE_RATE_MS, pdTRUE, (void *)0, vTimerCallback1);
  xTimerStart(displaySubTimer, 0);
  Serial.println("[DISP] STARTED TASK");
  while (true)
  {
    if (!motion)
    {
      digitalWrite(LED3, !digitalRead(LED3));
      display.clearDisplay();
      display.drawBitmap(0, 16, symbol, 32, 32, SSD1306_WHITE);
      display.drawBitmap(112, 50, wifi, 16, 14, SSD1306_WHITE);
      display.drawBitmap(85, 50, aux_linked, 15, 15, SSD1306_WHITE);
      if (scan)
      {
        display.setTextSize(1);
        display.setCursor(34, 19);
        display.print("LP");
        display.print(uid_temp[0]);
        display.setCursor(34, 27);
        display.print("R ");
        display.print(uid_temp[1]);
        display.setCursor(34, 35);
        display.print("G ");
        display.print(uid_temp[2]);

        self.COUNT_NUM_LP = uid_temp[0];
        self.COUNT_NUM_R = uid_temp[1];
        self.COUNT_NUM_G = uid_temp[2];
      }
      else
      {
        display.setTextSize(1);
        display.setCursor(34, 19);
        display.print("LP");
        display.print(uid_lp_pos - 1);
        display.setCursor(34, 27);
        display.print("R ");
        display.print(uid_r_pos - 1);
        display.setCursor(34, 35);
        display.print("G ");
        display.print(uid_g_pos - 1);

        self.COUNT_NUM_LP = uid_lp_pos - 1;
        self.COUNT_NUM_R = uid_r_pos - 1;
        self.COUNT_NUM_G = uid_g_pos - 1;
      }
      //display.fillRect(0, 0, 57, 64, BLACK);
      //display.fillRect(0, 42, 128, 23, BLACK);
      display.setTextColor(BLACK, WHITE);
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.print("GATEWAY");
      display.setTextColor(WHITE);
      display.setCursor(0, 8);
      display.print("FW 2.6");

      tmElements_t t;
      rtc.time_get(&t);
      vTaskDelay(10);
      long int time_now_rtc = makeTime(t);
      long int secs = time_now_rtc - startup_time_rtc;
      long int mins = (secs / 60);
      long int hrs = (mins / 60);
      long int days = (hrs / 24);
      display.setCursor(0, 48);
      display.print("UT:");
      if (secs > 0)
      {
        display.print(days);
        display.print("d");
        display.print(hrs % 24);
        display.print("h");
        display.print(mins % 60);
        display.print("m");
        display.print(secs % 60);
        display.print("s");
        display.setCursor(0, 56);
      }
      display.print("TXNs:");
      display.print(self.COUNT_TOTALTXNFROMSTART);

      if (s_timer)
      {
        //display.fillRect(58, 0, 75, 41, BLACK);
        display.setTextSize(1);
        display.drawFastHLine(58, 8, 75, WHITE);
        display.setCursor(60, 0);
        display.print("DEVICE INFO");
        display.setTextColor(WHITE);
        display.setCursor(60, 10);
        display.print("UID:");
        display.print(SELF_UID);
        display.setCursor(60, 18);
        display.print("CH:");
        display.print(SELF_CH);
        display.setCursor(60, 26);
        display.print("TEMP:");
        self.temp = (float)((temprature_sens_read() - 32) / 1.8);
        self.hum = (float)sht20.readHumidity();
        display.print(self.temp);
        display.setCursor(60, 34);
        display.print("RSSI:");
        self.rssi = WiFi.RSSI();
        display.print(self.rssi);
        display.print("dBm");
        display.drawFastVLine(58, 0, 42, WHITE);
        display.drawFastHLine(58, 42, 75, WHITE);
        Serial.println("[DISP] INFO DISPLAY UPDATED");
      }
      else
      {
        //display.fillRect(58, 0, 75, 41, BLACK);
        display.setTextSize(1);
        display.drawFastHLine(58, 8, 75, WHITE);
        display.setCursor(60, 0);
        display.print(" ERROR INFO");
        display.setTextColor(WHITE);
        display.setCursor(60, 10);
        display.print("QOV:");
        display.print(self.COUNT_QUEUEOVERFLOW);
        display.setCursor(60, 18);
        display.print("PXB:");
        display.print(self.COUNT_RXPIDOUTOFBOUNDS);
        display.setCursor(60, 26);
        display.print("UXB:");
        display.print(self.COUNT_RXUIDOUTOFBOUNDS);
        display.setCursor(60, 34);
        display.print("IFF:");
        display.print(self.COUNT_IFDBWRITEFAIL);
        display.drawFastVLine(58, 0, 42, WHITE);
        display.drawFastHLine(58, 42, 75, WHITE);
        Serial.println("[DISP] ERROR DISPLAY UPDATED");
      }
      display.display();
    }
    vTaskDelay(500);
    if (motion)
    {
      motiontriggered();
    }
  }
}

/*! scandevices() :: FUNCTION
   @brief determines the number of devices connected to the network

   @note executes after serial data received in task serialparser()

   @param void
*/
void scandevices()
{
  if (uid >= 0 && uid <= 15)
  {
    for (int i = 1; i++; i < 255)
    {

      if (uid_g[i] == uid)
      {
        break;
      }
      if (i == 254)
      {
        uid_g_pos++;
        uid_g[uid_g_pos] = uid;
      }
    }
  }
  if (uid >= 16 && uid <= 255)
  {
    for (int i = 1; i++; i < 255)
    {

      if (uid_r[i] == uid)
      {
        break;
      }
      if (i == 254)
      {
        uid_r_pos++;
        uid_r[uid_r_pos] = uid;
      }
    }
  }
  if (uid >= 4096 && uid <= 40000)
  {
    for (int i = 1; i++; i < 255)
    {
      if (uid_lp[i] == uid)
      {
        break;
      }
      if (i == 254)
      {
        uid_lp_pos++;
        uid_lp[uid_lp_pos] = uid;
      }
    }
  }
}

/*! motiontriggered() :: FUNCTION
   @brief writes to display and increments intrusiontrig

   @note executes on motion detected

   @param void
*/
void motiontriggered()
{
  self.COUNT_INTRUSIONTRIG++;
  motion = false;
  display.setTextSize(1);
  display.clearDisplay();
  display.setTextColor(BLACK, WHITE);
  display.setCursor(0, 0);
  display.print("       TERMINAL      ");
  display.display();
  display.setTextColor(WHITE);
  display.setCursor(0, 15);
  display.print("[MASKED] trig'd");
  display.display();
  vTaskDelay(1000);
  while (motion)
  {
    motion = false;
    vTaskDelay(1000);
  }
  display.setCursor(0, 25);
  display.print("[MASKED] gen secrep");
  display.display();
  vTaskDelay(20);
  display.setCursor(0, 35);
  display.print("[MASKED] post to db");
  display.display();
  vTaskDelay(20);
  display.display();
  display.setCursor(0, 45);
  display.print("[MASKED] response ok");
  display.display();
  vTaskDelay(50);
  display.setCursor(0, 55);
  display.print("[MASKED] exit");
  display.display();
}

void loop() {}
