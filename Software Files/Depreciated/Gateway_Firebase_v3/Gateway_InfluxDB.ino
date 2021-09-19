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
#if defined(ESP32)
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#define DEVICE "ESP32"
#elif defined(ESP8266)
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;
#define DEVICE "ESP8266"
#endif
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>

// WiFi AP SSID
#define WIFI_SSID "MAIN"
// WiFi password
#define WIFI_PASSWORD "2e9fc16a"
// InfluxDB v2 server url, e.g. https://eu-central-1-1.aws.cloud2.influxdata.com (Use: InfluxDB UI -> Load Data -> Client Libraries)
#define INFLUXDB_URL "https://us-west-2-1.aws.cloud2.influxdata.com"
// InfluxDB v2 server or cloud API authentication token (Use: InfluxDB UI -> Data -> Tokens -> <select token>)
#define INFLUXDB_TOKEN "qXo1pHPocwaFZ1ZSSlDllihJ6owI5WyU7dgHJd9o9q5TnKgLjpAYIToSQPaH4OJxc87ZG5DV55_I_0LGOJMB6A=="
// InfluxDB v2 organization id (Use: InfluxDB UI -> User -> About -> Common Ids )
#define INFLUXDB_ORG "theprovidentinquisition@gmail.com"
// InfluxDB v2 bucket name (Use: InfluxDB UI ->  Data -> Buckets)
#define INFLUXDB_BUCKET "MESH"


// Set timezone string according to https://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html
// Examples:
//  Pacific Time: "PST8PDT"
//  Eastern: "EST5EDT"
//  Japanesse: "JST-9"
//  Central Europe: "CET-1CEST,M3.5.0,M10.5.0/3"
#define TZ_INFO "CET-1CEST,M3.5.0,M10.5.0/3"

InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

Point Reset("Reset");
Point Status("Status");
Point Battery("Battery");
Point Environment("Environment");
Point Motion("Motion");
Point Airquality("Airquality");
Point Detector("Detector");
Point Ranger("Ranger");
Point NC("NC");

DFRobot_SHT20 sht20;
PCF85063A rtc;
WiFiServer server(80);
Adafruit_SSD1306 display(128, 64, &Wire, 4);

/* Strings to store incoming UART data */
String uart_recv_buffer;


/* Volatile global variables*/
String uart_recv_pid;
String uart_recv_uid;
int pid;
int uid;



int queue_rst = 0;
int queue_stat = 0;
int queue_batt = 0;
int queue_env = 0;
int queue_motion = 0;
int queue_aqi = 0;
int queue_detector = 0;
int queue_ranger = 0;


/* Arrrays to keep track of WMNS device count */
int uid_env[255] = {};
int uid_motion[255] = {};
int uid_aqi[255] = {};
int uid_detector[255] = {};
int uid_ranger[255] = {};
int uid_r32pa[255] = {};
int uid_r40[255] = {};
/* Variables to indicate position to append arrays ^ */
int uid_env_pos = 0;
int uid_motion_pos = 0;
int uid_aqi_pos = 0;
int uid_detector_pos = 0;
int uid_ranger_pos = 0;


/* Flags to indicate datatypes present */
typedef struct flags {
  bool rst;
  bool stat;
  bool batt;
  bool env;
  bool motion;
  bool aqi;
  bool detector;
  bool ranger;
} flags;

/* Strings to store parsed data */
typedef struct recv_data {
  String rst[16];     //rst

  String light[16];   //env
  String temp[16];    //
  String press[16];   //
  String hum[16];     //

  String airquality[16];

  String x[16];  //motion
  String y[16];
  String z[16];
  String shock_dur[16];
  String shock_count[16];
  String step_count[16];

  String rdistance[16]; //ranger
  String rcal_point[16];
  String rcal_tolerance[16];
  String rrange_mode[16];

  String ddistance[16]; //detector
  String dcal_point[16];
  String dcal_tolerance[16];
  String drange_mode[16];

  String batt[16];    //batt

  String rssi[16];    //stat
  String alive[16];   //
  String retx[16];    //
  String path[16];    //


  String rstUID[16];
  String statUID[16];
  String battUID[16];
  String envUID[16];
  String motionUID[16];
  String rangerUID[16];
  String detectorUID[16];

} recv_data;

/* Pin configuration (on nRF52 Router - WiFi) */
#define nRF52_TX    27
#define nRF52_RX    25
#define Core0_LED   21
#define Core1_LED   22
#define Wifi_LED    26
#define Comms_LED   23
#define SDA         19
#define SCL         18

/* Registered PIDs in the mesh network*/
#define pid_rst     0
#define pid_stat    1
#define pid_batt    2
#define pid_env     176
#define pid_motion  177
#define pid_iaq     178
#define pid_range   179

/* Wi-Fi and Firebase parameters */

/* Tasks */
TaskHandle_t serial_monitor;
TaskHandle_t firebase;

/* Structs */
recv_data rdata;
flags flag;

/* Display index pointer */
int displayindex = 0;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

long prev_sync = 0;
long const sync_interval = 30000;
long prev_timeout = 0;
long const timeout_interval = 2500;
long prev_firebase_timeout = 0;
long const firebase_timeout_interval = 2500;

static const unsigned char PROGMEM logo_bmp[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x03, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x07, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x0f, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x1f, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x3f, 0x3f, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x7e, 0x18, 0x7f, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xfc, 0x0c, 0x0f, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x01, 0xf8, 0x04, 0x01, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x03, 0xf0, 0x02, 0x00, 0x3f, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x07, 0xe0, 0x01, 0x00, 0x03, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x0f, 0xc0, 0x00, 0x80, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x3f, 0x80, 0x00, 0x40, 0x01, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x7f, 0x00, 0x00, 0x20, 0x1e, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xfe, 0x00, 0x00, 0x19, 0xe0, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0xf8, 0x00, 0x00, 0x1e, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x03, 0xf0, 0x00, 0x00, 0x38, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x07, 0xe0, 0x00, 0x00, 0x68, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x0f, 0xc0, 0x00, 0x00, 0xc8, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1f, 0x80, 0x00, 0x01, 0x84, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1f, 0x00, 0x00, 0x03, 0x04, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1f, 0x00, 0x00, 0x04, 0x04, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1f, 0x00, 0x00, 0x08, 0x04, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1f, 0x80, 0x00, 0x10, 0x02, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1f, 0xc0, 0x00, 0x20, 0x02, 0x00, 0x03, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1f, 0x60, 0x00, 0x40, 0x02, 0x00, 0x03, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x0f, 0x30, 0x00, 0x80, 0x02, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x0f, 0x18, 0x01, 0x00, 0x00, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x0f, 0x08, 0x02, 0x00, 0x01, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x0f, 0x84, 0x04, 0x00, 0x01, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x07, 0x82, 0x18, 0x00, 0x01, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x07, 0x81, 0x30, 0x00, 0x01, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x07, 0x80, 0xe0, 0x00, 0x00, 0x80, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x07, 0xc0, 0xf0, 0x00, 0x00, 0x80, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x07, 0xc0, 0xce, 0x00, 0x00, 0x80, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x03, 0xc0, 0x81, 0xe0, 0x00, 0x80, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x03, 0xc0, 0x80, 0x3c, 0x00, 0x40, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x03, 0xe1, 0x00, 0x03, 0x80, 0x40, 0x03, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x03, 0xe1, 0x00, 0x00, 0x70, 0x40, 0x3f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0xe1, 0x00, 0x00, 0x0f, 0xe3, 0xc3, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0xe3, 0x00, 0x00, 0x01, 0xfc, 0x07, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0xe2, 0x00, 0x00, 0x00, 0x60, 0x0f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0xf2, 0x00, 0x00, 0x00, 0x40, 0x1f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0xf6, 0x00, 0x00, 0x00, 0x40, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xf4, 0x00, 0x00, 0x00, 0x80, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xfc, 0x00, 0x00, 0x00, 0x80, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xfc, 0x00, 0x00, 0x00, 0x81, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xfc, 0x00, 0x00, 0x00, 0x83, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x7c, 0x00, 0x00, 0x01, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x7f, 0x80, 0x00, 0x01, 0x0f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x7f, 0xf8, 0x00, 0x01, 0x1f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x3f, 0xff, 0x00, 0x02, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x07, 0xff, 0xe0, 0x02, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xff, 0xfc, 0x03, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x1f, 0xff, 0xc7, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x03, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x7f, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x07, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


void setup() {

  Serial.begin(921600); // initialize USB comms
  Wire.begin(17, 16, 400000); // initialize I2C for display (I2C0)
  Serial2.begin(115200, SERIAL_8N1, nRF52_RX, nRF52_TX); // initialize comms with nRF52
  Serial.println("[BOOT] booted into setup successfully");
  Serial.println("[BOOT] cpu frequency " + (String)getCpuFrequencyMhz());
  Serial.println("[UART] usb serial initialized");
  Serial.println("[UART] mesh comms initialized");
  Wire1.begin(19, 18); // initialize I2C for RTC and temp/hum sensor
  Serial.println("[I2C] i2c interface initialized");

  /* Initialize GPIOs for indicator LEDs*/
  pinMode(Core0_LED, OUTPUT);
  pinMode(Core1_LED, OUTPUT);
  pinMode(Wifi_LED, OUTPUT);
  pinMode(Comms_LED, OUTPUT);
  Serial.println("[GPIO] gpio initialized");

  /* Initialize display */
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.setFont();
  display.print("ACTIVE");

  display.setCursor(0, 15);
  display.print("GATEWAY v2");
  display.setCursor(0, 23);
  display.print("UID 0x0000");
  display.drawBitmap(64, 0, logo_bmp, 128, 64, SSD1306_WHITE);

  display.display();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("[RADIO] wifi initialized");
  while (WiFi.status() != WL_CONNECTED)
  {
    unsigned long current_timeout = millis();
    digitalWrite(Wifi_LED, !digitalRead(Wifi_LED));
    delay(50);
    if (current_timeout - prev_timeout >= timeout_interval) {
      prev_timeout = current_timeout;
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  }
  Serial.println("[RADIO] wifi connection established");
  delay(100);
  Serial.println("[RADIO] wifi rssi: " + WiFi.RSSI());
  delay(50);
  digitalWrite(Wifi_LED, HIGH);
  Serial.println("[RADIO] firebase connection initialized");

  Serial.println("[RADIO] firebase connected");
  /* Sync RTC to NTP Server */
  Serial.println("[RTC] rtc initialized");
  rtc.reset();
  delay(10);

  tmElements_t tm;
  timeClient.begin();
  timeClient.setTimeOffset(28800);
  Serial.println("[RADIO] connecting to ntp server");
  while (!timeClient.update()) {
    timeClient.forceUpdate();
  }
  breakTime(timeClient.getEpochTime(), tm);
  Serial.println("[RADIO] received ntp time");
  rtc.time_set(&tm);
  delay(50);
  Serial.println("[RTC] rtc synced to ntp time");

  // Accurate time is necessary for certificate validation and writing in batches
  // For the fastest time sync find NTP servers in your area: https://www.pool.ntp.org/zone/
  // Syncing progress and the time will be printed to Serial.
  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");

  client.setHTTPOptions(HTTPOptions().httpReadTimeout(500));

  client.setHTTPOptions(HTTPOptions().connectionReuse(true));
  // Check server connection
  if (client.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }


  /* Initialize onboard temp/humi sensor */
  sht20.initSHT20();
  Serial.println("[TEMP] temp sensor initialized");
  delay(100);
  sht20.checkSHT20();
  Serial.println("[TEMP] temp sensor active");

  /* Initialize dual-core operations */

  Serial.println("[NVIC] starting task0 serial_monitor on cpu0");

  xTaskCreatePinnedToCore(
    firebase_task,                  /* pvTaskCode */
    "firebase",            /* pcName */
    10000,                   /* usStackDepth */
    NULL,                   /* pvParameters */
    0,                      /* uxPriority */
    &firebase,                 /* pxCreatedTask */
    0);
  delay(100);
  Serial.println("[NVIC] starting task1 serial_monitor on cpu1");
  xTaskCreatePinnedToCore(
    serial_monitor_task,                  /* pvTaskCode */
    "serial_monitor",            /* pcName */
    10000,                   /* usStackDepth */
    NULL,                   /* pvParameters */
    1,                      /* uxPriority */
    &serial_monitor,                 /* pxCreatedTask */
    1);



}

/*! firebase_task() :: THREAD
   @brief This thread monitors UART2 for data, and runs uart_parser() when data is detected.

   @note This thread runs on core 0 (RF core)

   @param pvParameters task structure
*/
void firebase_task( void * pvParameters ) {
  Serial.println("[TASK0] task0 firebase_task started on cpu0");
  delay(10);
  for (int i = 0; i < 9; i++) {
    digitalWrite(Core0_LED, !digitalRead(Core0_LED));
    delay(40);
  }
  digitalWrite(Core0_LED, LOW);
  digitalWrite(Comms_LED, HIGH);
  delay(10);

  while (true) {

    if (flag.rst == true) {
      Serial.print("[TASK0] radio transmit rst data ");
      Serial.print(rdata.rst[queue_rst - 1]);
      Serial.print(" at qd");
      Serial.println(queue_rst);
      Reset.clearFields();
      Reset.clearTags();
      Reset.addTag("UID", rdata.rstUID[queue_rst - 1]);
      Reset.addField("Reset", rdata.rst[queue_rst - 1].toFloat());
      if (!client.writePoint(Reset)) {
        Serial.print("InfluxDB write failed: ");
        Serial.println(client.getLastErrorMessage());
      }
      queue_rst--;
      if (queue_rst == 0) {
        flag.rst = false;
      }
    }

    if (flag.env == true) {

      if (rdata.hum[queue_env - 1].toFloat() > 0 && rdata.hum[queue_env - 1].toFloat() < 100 && rdata.temp[queue_env - 1].toFloat() != 0) {
        Serial.print("[TASK0] radio transmit env data ");
        Serial.print(rdata.temp[queue_env - 1]);
        Serial.print(" at qd");
        Serial.println(queue_env);
        Environment.clearFields();
        Environment.clearTags();
        Environment.addTag("UID", rdata.envUID[queue_env - 1]);
        Environment.addField("Temperature", rdata.temp[queue_env - 1].toFloat());
        Environment.addField("Humidity", rdata.hum[queue_env - 1].toFloat());
        Environment.addField("Pressure", rdata.press[queue_env - 1].toFloat());
        Environment.addField("Light", rdata.light[queue_env - 1].toFloat());
        if (!client.writePoint(Environment)) {
          Serial.print("InfluxDB write failed: ");
          Serial.println(client.getLastErrorMessage());
        }
      }

      queue_env--;
      if (queue_env == 0) {
        flag.env = false;
      }
    }
    if (flag.stat == true) {
      if (rdata.retx[queue_stat - 1].toFloat() > 0 && rdata.retx[queue_stat - 1].toFloat() < 7) {
        Serial.print("[TASK0] radio transmit stat data ");
        Serial.print(rdata.rssi[queue_stat - 1]);
        Serial.print(" at qd");
        Serial.println(queue_stat);
        Status.clearFields();
        Status.clearTags();
        Status.addTag("UID", rdata.statUID[queue_stat - 1]);
        Status.addField("RSSI", rdata.rssi[queue_stat - 1].toFloat());
        Status.addField("Alive", rdata.alive[queue_stat - 1].toFloat());
        Status.addField("Retransmissions", rdata.retx[queue_stat - 1].toFloat());

        if (!client.writePoint(Status)) {
          Serial.print("InfluxDB write failed: ");
          Serial.println(client.getLastErrorMessage());
        }
      }

      queue_stat--;
      if (queue_stat == 0) {
        flag.stat = false;
      }
    }

    if (flag.batt == true) {
      if (rdata.batt[queue_batt - 1].toFloat() > 0.8 && rdata.batt[queue_batt - 1].toFloat() < 6) {
        Serial.print("[TASK0] radio transmit batt data ");
        Serial.print(rdata.batt[queue_batt - 1]);
        Serial.print(" at qd");
        Serial.println(queue_batt);
        Battery.clearFields();
        Battery.clearTags();
        Battery.addTag("UID", rdata.battUID[queue_batt - 1]);
        Battery.addField("Battery", rdata.batt[queue_batt - 1].toFloat());
        if (!client.writePoint(Battery)) {
          Serial.print("InfluxDB write failed: ");
          Serial.println(client.getLastErrorMessage());
        }
      }

      queue_batt--;
      if (queue_batt == 0) {
        flag.batt = false;
      }
    }


    if (flag.motion == true) {
      if (rdata.shock_dur[queue_motion - 1].toFloat() > 0 && rdata.shock_dur[queue_motion - 1].toFloat() < 10) {
        Serial.print("[TASK0] radio transmit motion data ");
        Serial.print(rdata.shock_count[queue_motion - 1]);
        Serial.print(" at qd");
        Serial.println(queue_motion);
        Motion.clearFields();
        Motion.clearTags();
        Motion.addTag("UID", rdata.motionUID[queue_motion - 1]);
        Motion.addField("X-axis", rdata.x[queue_motion - 1].toFloat());
        Motion.addField("Y-axis", rdata.y[queue_motion - 1].toFloat());
        Motion.addField("Z-axis", rdata.z[queue_motion - 1].toFloat());
        Motion.addField("Impact counter", rdata.shock_count[queue_motion - 1].toFloat());
        Motion.addField("Duration of impacts", rdata.shock_dur[queue_motion - 1].toFloat());
        Motion.addField("Step counter", rdata.step_count[queue_motion - 1].toFloat());
        if (!client.writePoint(Motion)) {
          Serial.print("InfluxDB write failed: ");
          Serial.println(client.getLastErrorMessage());
        }
      }

      queue_motion--;
      if (queue_motion == 0) {
        flag.motion = false;
      }
    }

    if (flag.ranger == true) {
      Serial.print("[TASK0] radio transmit ranger data ");
      Serial.print(rdata.rdistance[queue_ranger - 1]);
      Serial.print(" at qd");
      Serial.println(queue_batt);
      Ranger.clearFields();
      Ranger.clearTags();
      Ranger.addTag("UID", rdata.rangerUID[queue_ranger - 1]);
      Ranger.addField("Distance", rdata.rdistance[queue_ranger - 1].toFloat());
      if (!client.writePoint(Ranger)) {
        Serial.print("InfluxDB write failed: ");
        Serial.println(client.getLastErrorMessage());
      }
      queue_ranger--;
      if (queue_ranger == 0) {
        flag.ranger = false;
      }
    }

    if (flag.detector == true) {
      Serial.print("[TASK0] radio transmit detector data ");
      Serial.print(rdata.ddistance[queue_detector - 1]);
      Serial.print(" at qd");
      Serial.println(queue_detector);
      Detector.clearFields();
      Detector.clearTags();
      Detector.addTag("UID", rdata.detectorUID[queue_detector - 1]);
      Detector.addField("Distance", rdata.ddistance[queue_detector - 1].toFloat());
      Detector.addField("Max. Tolerance", rdata.dcal_point[queue_detector - 1].toFloat() + rdata.dcal_tolerance[queue_detector - 1].toFloat());
      Detector.addField("Min. Tolerance", rdata.dcal_point[queue_detector - 1].toFloat() - rdata.dcal_tolerance[queue_detector - 1].toFloat());
      Detector.addField("Mode", rdata.drange_mode[queue_detector - 1].toFloat());
      if (!client.writePoint(Detector)) {
        Serial.print("InfluxDB write failed: ");
        Serial.println(client.getLastErrorMessage());
      }
      queue_detector--;
      if (queue_detector == 0) {
        flag.detector = false;
      }
    }


    delay(10);
    digitalWrite(Core0_LED, !digitalRead(Core0_LED));
    digitalWrite(Comms_LED, !digitalRead(Comms_LED));
  }

}
/*! serial_monitor_task() :: THREAD
   @brief This thread monitors UART2 for data, and runs uart_parser() when data is detected.

   @note This thread runs on core 1 (Application core)

   @param pvParameters task structure
*/
void serial_monitor_task( void * pvParameters ) {
  Serial.println("[TASK1] task1 serial_monitor started on cpu1");
  delay(10);
  for (int i = 0; i < 9; i++) {
    digitalWrite(Core1_LED, !digitalRead(Core1_LED));
    delay(40);
  }
  digitalWrite(Core1_LED, LOW);
  delay(10);
  int ledstat = 1;
  bool incr = true;
  while (true) {
    if (Serial2.available() > 0) {
      digitalWrite(Core1_LED, HIGH);
      Serial2.setTimeout(5);
      uart_recv_buffer = Serial2.readString();
      Serial2.flush();
      uart_recv_pid = uart_recv_buffer.substring(uart_recv_buffer.indexOf("pid:") + 4, uart_recv_buffer.indexOf(";ctrl:"));
      uart_recv_uid = uart_recv_buffer.substring(uart_recv_buffer.indexOf("src:") + 4, uart_recv_buffer.indexOf(";", uart_recv_buffer.indexOf("src:")));
      uart_parser(uart_recv_pid, uart_recv_uid, uart_recv_buffer);
    }

    /* Non-blocking code for comms LED */


  }
}

/*! uart_parser() :: FUNCTION
   @brief This function takes in raw uart data, parses, triggers flags and categorizes received data.

   @note

   @param uart_pid raw pid parsed from uart_buffer
   @param uart_uid raw uid parsed from uart_buffer
   @param uart_buffer raw Serial2 data from Serial2.read()
*/

void uart_parser(String uart_pid, String uart_uid, String uart_buffer) {
  pid = uart_pid.toInt();
  uid = uart_uid.toInt();
  if (uid >= 0 && uid <= 65535) {
    switch (pid) {
      case pid_rst:
        if (uid == 0) {
          break;
        }
        rdata.rstUID[queue_rst] = (String)uid;
        if (queue_stat > 14) {
          queue_rst = 0;
          flag.rst = false;
          Serial.print("BUFFER OVERFLOW");
        }
        queue_rst++;
        flag.rst = true;
        Serial.println("[TASK1] recv rst data from device " + (String)uid);
        break;

      case pid_stat:
        rdata.alive[queue_stat] = uart_buffer.substring(uart_buffer.indexOf("alive:") + 6, uart_buffer.indexOf(";nb:"));
        rdata.retx[queue_stat] = uart_buffer.substring(uart_buffer.indexOf("nb:") + 3, uart_buffer.indexOf(";rx1:"));
        rdata.rssi[queue_stat] = uart_buffer.substring(uart_buffer.indexOf(",") + 1, uart_buffer.indexOf(",") + 4);
        rdata.statUID[queue_stat] = (String)uid;
        if (queue_stat > 14) {
          queue_stat = 0;
          flag.stat = false;
          Serial.print("BUFFER OVERFLOW");
        }
        queue_stat++;
        flag.stat = true;
        Serial.println("[TASK1] recv stat data from device " + (String)uid);
        break;

      case pid_env:
        if (uid < 4096 || uid > 8191) {
          break;
        }
        rdata.temp[queue_env] = uart_buffer.substring(uart_buffer.indexOf("temp:") + 5, uart_buffer.indexOf(";hum:"));
        rdata.hum[queue_env] = uart_buffer.substring(uart_buffer.indexOf("hum:") + 4, uart_buffer.indexOf(";press:"));
        rdata.press[queue_env] = uart_buffer.substring(uart_buffer.indexOf("press:") + 6, uart_buffer.indexOf("press:") + 13);
        rdata.light[queue_env] = uart_buffer.substring(uart_buffer.indexOf("light:") + 6, uart_buffer.indexOf("light:") + 13);
        rdata.envUID[queue_env] = (String)uid;
        if (queue_env > 14) {
          queue_env = 0;
          flag.env = false;
          Serial.print("BUFFER OVERFLOW");
        }
        queue_env++;
        flag.env = true;
        Serial.println("[TASK1] recv env data from device " + (String)uid);
        break;

      case pid_motion:
        if (uid < 12288 || uid > 16383) {
          break;
        }
        rdata.x[queue_motion] = uart_buffer.substring(uart_buffer.indexOf("x:") + 2, uart_buffer.indexOf(";y:"));
        rdata.y[queue_motion] = uart_buffer.substring(uart_buffer.indexOf("y:") + 2, uart_buffer.indexOf(";z:"));
        rdata.z[queue_motion] = uart_buffer.substring(uart_buffer.indexOf("z:") + 2, uart_buffer.indexOf(",shockdur:"));
        rdata.shock_dur[queue_motion] = uart_buffer.substring(uart_buffer.indexOf("shockdur:") + 9, uart_buffer.indexOf(",shockcount:"));
        rdata.shock_count[queue_motion] = uart_buffer.substring(uart_buffer.indexOf("shockcount:") + 11, uart_buffer.indexOf(",stepcount:"));
        rdata.step_count[queue_motion] = uart_buffer.substring(uart_buffer.indexOf("stepcount:") + 10, uart_buffer.indexOf(",sensetemp:"));
        rdata.motionUID[queue_motion] = (String)uid;
        if (queue_motion > 14) {
          queue_motion = 0;
          flag.motion = false;
          Serial.print("BUFFER OVERFLOW");
        }
        queue_motion++;
        flag.motion = true;
        break;

      case pid_range:

        if (uid >= 20480 && uid <= 20991) {
          Serial.println("[TASK1] recv ranger data from device " + (String)uid);
          rdata.rdistance[queue_ranger] = uart_buffer.substring(uart_buffer.indexOf("distance:") + 9, uart_buffer.indexOf(";cal:"));
          rdata.rcal_point[queue_ranger] = uart_buffer.substring(uart_buffer.indexOf("cal:") + 4, uart_buffer.indexOf(";tol:"));
          rdata.rcal_tolerance[queue_ranger] = uart_buffer.substring(uart_buffer.indexOf("tol:") + 4, uart_buffer.indexOf(";mode:"));
          rdata.rrange_mode[queue_ranger] = uart_buffer.substring(uart_buffer.indexOf("mode:") + 5, uart_buffer.indexOf("mode:") + 6);
          rdata.rangerUID[queue_ranger] = (String)uid;
          if (queue_ranger > 14) {
            queue_ranger = 0;
            flag.ranger = false;
            Serial.print("BUFFER OVERFLOW");
          }
          queue_ranger++;
          flag.ranger = true;
        }

        if (uid >= 20992 && uid <= 21503) {
          Serial.println("[TASK1] recv detector data from device " + (String)uid);
          rdata.ddistance[queue_detector] = uart_buffer.substring(uart_buffer.indexOf("distance:") + 9, uart_buffer.indexOf(";cal:"));
          rdata.dcal_point[queue_detector] = uart_buffer.substring(uart_buffer.indexOf("cal:") + 4, uart_buffer.indexOf(";tol:"));
          rdata.dcal_tolerance[queue_detector] = uart_buffer.substring(uart_buffer.indexOf("tol:") + 4, uart_buffer.indexOf(";mode:"));
          rdata.drange_mode[queue_detector] = uart_buffer.substring(uart_buffer.indexOf("mode:") + 5, uart_buffer.indexOf("mode:") + 6);
          rdata.detectorUID[queue_detector] = (String)uid;
          if (queue_detector > 14) {
            queue_detector = 0;
            flag.detector = false;
            Serial.print("BUFFER OVERFLOW");
          }
          queue_detector++;
          flag.detector = true;
        }
        break;

      case pid_batt:
        if (uid < 255) {
          break;
        }
        rdata.batt[queue_batt] = uart_buffer.substring(uart_buffer.indexOf("voltage:") + 8, uart_buffer.indexOf("voltage:") + 12);
        rdata.battUID[queue_batt] = (String)uid;
        if (queue_batt > 14) {
          queue_batt = 0;
          flag.batt = false;
          Serial.print("BUFFER OVERFLOW");
        }
        queue_batt++;
        flag.batt = true;
        Serial.println("[TASK1] recv batt data from device " + (String)uid);
        break;

      default:
        break;

    }
    digitalWrite(Core1_LED, LOW);
  }
  else {
  }
  digitalWrite(Core1_LED, LOW);

}

void loop()
{

}
