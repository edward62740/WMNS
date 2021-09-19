#include <WiFi.h>
#include <stdlib.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <PCF85063A.h>
#include <Wire.h>
#include "DFRobot_SHT20.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeSans9pt7b.h>

DFRobot_SHT20 sht20;
PCF85063A rtc;
WiFiServer server(80);
Adafruit_SSD1306 display(128, 64, &Wire, 4);


/* Strings to store incoming UART data */
String uart_recv_buffer;

/* Volatile global variables*/
String uart_recv_pid;
String uart_recv_uid;
uint8_t pid;
uint16_t uid;



// Variable to store the HTTP request
String header;
// Assign output variables to GPIO pins

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0;
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;


/* Strings to store parsed data */
typedef struct {
  uint16_t device[255] = {};
  String rst[255];     //rst
  String info[255];    // null
  String light[255];   //env
  String temp[255];    //
  String press[255];   //
  String hum[255];     //
  String batt[255];    //batt
  String rssi[255];    //stat
  String alive[255];   //
  String retx[255];    //
  String path[255];    //
} data;
int data_struct_top = 0;

/* Pin configuration (on nRF52 Router - WiFi) */
#define nRF52_TX    27
#define nRF52_RX    25
#define Core0_LED   2
#define Core1_LED   4
#define Wifi_LED    23
#define Comms_LED   26
#define SDA         19
#define SCL         18

/* Registered PIDs in the mesh network*/
#define pid_rst     0
#define pid_stat    1
#define pid_batt    2
#define pid_env     176
#define pid_motion  177
#define pid_iaq     162

/* Wi-Fi and Firebase parameters */
#define WIFI_SSID "MAIN"
#define WIFI_PASSWORD "2e9fc16a"

/* Tasks */
TaskHandle_t serial_monitor;
TaskHandle_t firebase;

/* Structs */
data data_struct;

int x = 0; //set refresh counter to 0


WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);


long wifi_timeout_prev = 0;
long const wifi_interval = 2500;

long display_prev = 0;
long const display_interval = 1000;


/*! setup() :: SETUP
   @brief This function runs once to initialize all functions of the gateway.

   @note This thread runs on core 1 (Application core)

   @param void
*/
void setup() {

  /* Initialize UARTs */
  Serial.begin(115200); // initialize USB comms (UART0)
  Serial2.begin(115200, SERIAL_8N1, nRF52_RX, nRF52_TX); // initialize comms with nRF52 (UART1)
  Serial.println("[SYSTEM] booted successfully");
  Serial.println("[SYSTEM] cpu frequency " + (String)getCpuFrequencyMhz());
  Serial.println("[UART0] usb serial initialized");
  Serial.println("[UART1] mesh comms initialized");

  /* Initialize I2Cs */
  Wire.begin(17, 16, 400000); // initialize I2C for display (I2C0)
  Wire1.begin(19, 18, 400000); // initialize I2C for RTC and temp/hum sensor (I2C1)
  Serial.println("[I2C0] i2c interface initialized");
  Serial.println("[I2C1] i2c interface initialized");

  /* Initialize display */
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.setFont();

  display.println("firmware v1.3 booting");
  display.display();
  delay(200);
  display.clearDisplay();
  display.display();

  /* Initialize GPIOs for indicator LEDs*/
  pinMode(Core0_LED, OUTPUT);
  pinMode(Core1_LED, OUTPUT);
  pinMode(Wifi_LED, OUTPUT);
  pinMode(Comms_LED, OUTPUT);
  Serial.println("[GPIO] gpio initialized");


  /******DISPLAY PRINT PREVIOUS INITIALIZATIONS******/
  display.setCursor(0, 0);
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.print("BOOT");
  display.setTextColor(WHITE); // 'inverted' text
  display.println(" boot ok");
  display.display();
  delay(10);
  display.setCursor(0, 10);
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.print("BOOT");
  display.setTextColor(WHITE); // 'inverted' text
  display.println(" cpu freq" + (String)getCpuFrequencyMhz() + "M");
  display.display();
  delay(10);
  display.setCursor(0, 20);
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.print("UART0");
  display.setTextColor(WHITE); // 'inverted' text
  display.println(" uart0 115200 ok");
  display.display();
  delay(10);
  display.setCursor(0, 30);
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.print("UART1");
  display.setTextColor(WHITE); // 'inverted' text
  display.println(" uart1 115200 ok");
  display.display();
  delay(10);
  display.setCursor(0, 40);
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.print("GPIO");
  display.setTextColor(WHITE); // 'inverted' text
  display.println(" gpio init ok");
  display.display();
  delay(10);
  display.setCursor(0, 50);
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.print("I2C0");
  display.setTextColor(WHITE); // 'inverted' text
  display.println(" i2c0 100000 ok");
  display.display();
  delay(10);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.print("I2C1");
  display.setTextColor(WHITE); // 'inverted' text
  display.println(" i2c1 100000 ok");
  display.display();

  /******DISPLAY PRINT PREVIOUS INITIALIZATIONS******/


  /* Start Wi-Fi service */
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("[RADIO] waiting for connection");
  display.setCursor(0, 10);
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.print("RADIO");
  display.setTextColor(WHITE); // 'inverted' text
  display.print(" connecting..");
  display.display();
  while (WiFi.status() != WL_CONNECTED)
  {
    unsigned long wifi_timeout = millis();
    digitalWrite(Wifi_LED, !digitalRead(Wifi_LED));
    delay(50);
    if (wifi_timeout - wifi_timeout_prev >= wifi_interval) {
      wifi_timeout_prev = wifi_timeout;
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  }
  server.begin();
  Serial.println("[RADIO] connection established");
  Serial.println("[RADIO] rssi: " + WiFi.RSSI());
  digitalWrite(Wifi_LED, HIGH);
  display.println("ok");


  /* Sync RTC to NTP Server */
  Serial.println("[RTC] rtc initialized");
  rtc.reset();
  delay(100);
  display.setCursor(0, 20);
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.print("RTC");
  display.setTextColor(WHITE); // 'inverted' text
  display.println(" rtc ok");
  display.display();
  tmElements_t tm;
  timeClient.begin();
  timeClient.setTimeOffset(28800);
  Serial.println("[RADIO] connecting to ntp server");
  while (!timeClient.update()) {
    timeClient.forceUpdate();
    delay(500);
    display.print(".");
    display.display();
  }
  breakTime(timeClient.getEpochTime(), tm);
  Serial.println("[RADIO] received ntp time");
  rtc.time_set(&tm);
  delay(50);
  Serial.println("[RTC] rtc synced to ntp time");
  display.setCursor(0, 30);
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.print("RADIO");
  display.setTextColor(WHITE); // 'inverted' text
  display.println(" ntp sync ok");
  display.display();
  Serial.println(WiFi.localIP());


  /* Initialize onboard temp/humi sensor */
  sht20.initSHT20(Wire1);
  Serial.println("[TEMP] temp sensor initialized");
  sht20.checkSHT20();
  Serial.println("[TEMP] temp sensor active");
  display.setCursor(0, 40);
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.print("TEMP");
  display.setTextColor(WHITE); // 'inverted' text
  display.println(" temp sense ok");
  display.display();



  /* Initialize dual-core operations */
  Serial.println("[SYSTEM] starting task0 serial_monitor on cpu0");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.print("SYSTEM");
  display.setTextColor(WHITE); // 'inverted' text
  display.println(" task0 init ok");
  display.display();

  xTaskCreatePinnedToCore(
    firebase_task,                  /* pvTaskCode */
    "firebase",            /* pcName */
    10000,                   /* usStackDepth */
    NULL,                   /* pvParameters */
    0,                      /* uxPriority */
    &firebase,                 /* pxCreatedTask */
    0);
  delay(10);

  Serial.println("[SYSTEM] starting task1 serial_monitor on cpu1");
  display.setCursor(0, 10);
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.print("SYSTEM");
  display.setTextColor(WHITE); // 'inverted' text
  display.println(" task1 init ok");
  display.display();
  xTaskCreatePinnedToCore(
    serial_monitor_task,                  /* pvTaskCode */
    "serial_monitor",            /* pcName */
    10000,                   /* usStackDepth */
    NULL,                   /* pvParameters */
    1,                      /* uxPriority */
    &serial_monitor,                 /* pxCreatedTask */
    1);

  display.setCursor(0, 20);
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.print("SYSTEM");
  display.setTextColor(WHITE); // 'inverted' text
  display.println(" booted");
  display.display();
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
    delay(10);
  }
  digitalWrite(Core0_LED, LOW);
  digitalWrite(Wifi_LED, HIGH);

  while (true) {

    WiFiClient client = server.available();   // Listen for incoming clients
    if (client) {                             // If a new client connects,
      digitalWrite(Core0_LED, HIGH);
      digitalWrite(Wifi_LED, LOW);
      Serial.println("New Client.");          // print a message out in the serial port
      String currentLine = "";                // make a String to hold incoming data from the client
      currentTime = millis();
      previousTime = currentTime;
      while (client.connected() && currentTime - previousTime <= timeoutTime) { // loop while the client's connected
        currentTime = millis();
        if (client.available()) {             // if there's bytes to read from the client,
          char c = client.read();             // read a byte, then
          Serial.write(c);                    // print it out the serial monitor
          header += c;
          if (c == '\n') {                    // if the byte is a newline character
            // if the current line is blank, you got two newline characters in a row.
            // that's the end of the client HTTP request, so send a response:
            if (currentLine.length() == 0) {
              // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
              // and a content-type so the client knows what's coming, then a blank line:
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/html");
              client.println("Connection: close");
              client.println();
              bool checked = false;
              // Display the HTML web page
              x = x + 1; //page upload counte
              client.println("<!DOCTYPE html><html>");
              client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
              client.print("<meta http-equiv=\"refresh\" content=\"15\">");
              client.println("<link rel=\"icon\" href=\"data:,\">");
              // CSS to style the on/off buttons
              // Feel free to change the background-color and font-size attributes to fit your preferences
              client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: left;}");
              client.println(".button { background-color: #0d01f9; border: none; color: white; padding: 16px 40px;}</style></head>");
              client.println("<body><h1>Mesh Sensor Network Web Interface v1.1</h1><br>");
              client.println("<body><h2>DEBUG MODE</h2><br>");
              tmElements_t t;
              rtc.time_get(&t);
              client.println("<body><h3>-------------------------------------------------------------GATEWAY STAT-------------------------------------------------------------</h3>");
              String router_stat = "<body><h4>t0 = " + (String)sht20.readTemperature() + ", h0 = " + (String)sht20.readHumidity() + ", tclk = " + (String)makeTime(t) + " rssiwifi = " + WiFi.RSSI() + "</h4>";
              client.println("<body><h4>AUTO REFRESH = ON</h4>");

              client.println(router_stat);
              client.println("<body><h3>---------------------------------------------------------HARDWARE SPECS---------------------------------------------------------</h3>");
              client.println("<body><h4>TYPE soc<br>NAME esp32d0wdq6<br>CPUCLK 240000000<br>CPU0 xtensalx6 CPU1 xtensalx6<br>MAC 30:ae:a4:55:da:5c<br>FLASH 4M<br>RAM 520k <br>   INTERFACES uart1-921600  uart2-115200  i2c1-100000  i2c1-400000 <br>RADIO default</h4>");
              client.println("<body><h4>TYPE soc<br>NAME nrf52832qfaa<br>CPUCLK 64000000<br>CPU0 cortex-m4fpu                 <br>MAC null             <br>FLASH 512k<br>RAM 64k<br>INTERFACES uart1-112500  i2c1-100000  <br>RADIO 1mbps_ble</h4><br><hr><br><br>");
              client.println("<body><h3>Environment Sensors</h3><br>");

              for (int i = 0; i < 255; i++) {
                if (data_struct.device[i] > 4095 && data_struct.device[i] < 12288) {
                  String device_print = "<h4>Device: " + (String)data_struct.device[i] + "</h4>";
                  client.print(device_print);
                  String temp_print = "<b4>Temperature: " + (String)data_struct.temp[i] + " deg</b4><br>";
                  client.print(temp_print);
                  String humi_print = "<b4>Humidity: " + (String)data_struct.hum[i] + " %</b4><br>";
                  client.print(humi_print);
                  String press_print = "<b4>Pressure: " + (String)data_struct.press[i] + " hPa</b4><br>";
                  client.print(press_print);
                  String light_print = "<b4>Light: " + (String)data_struct.light[i] + " lux</b4><br>";
                  client.print(light_print);
                  String reset_print = "<b4>Reset: " + (String)data_struct.rst[i] + "</b4><br>";
                  client.print(reset_print);
                  String battery_print = "<b4>Voltage: " + (String)data_struct.batt[i] + " V</b4><br>";
                  client.print(battery_print);
                  String alive_print = "<b4>Alive: " + (String)data_struct.alive[i] + "</b4><br>";
                  client.print(alive_print);
                  String retransmission_print = "<b4>reTX: " + (String)data_struct.retx[i] + "</b4><br>";
                  client.print(retransmission_print);
                  String rssi_print = "<b4>RSSI: " + (String)data_struct.rssi[i] + " dBm</b4><br>";
                  client.print(rssi_print);
                  client.print("<hr>");
                }

              }
              client.println("<br>");
              /*
                client.println("<body><h3>Air Quality Sensors</h3><br>");

                for (int i = 0; i < 255; i++) {
                if (device[i] == 162) {

                  String device_print = "<h4>Device: " + (String)i + "</h4>";
                  client.print(device_print);
                  String temp_print = "<b4>Temperature: " + (String)rdata.temp[i] + " deg</b4><br>";
                  client.print(temp_print);
                  String humi_print = "<b4>Humidity: " + (String)rdata.hum[i] + " %</b4><br>";
                  client.print(humi_print);
                  String press_print = "<b4>Pressure: " + (String)rdata.press[i] + " hPa</b4><br>";
                  client.print(press_print);
                  String light_print = "<b4>Light: " + (String)rdata.light[i] + " lux</b4><br>";
                  client.print(light_print);
                  String iaq_print = "<b4>Relative Air Quality: " + (String)rdata.iaq[i] + " %</b4><br>";
                  client.print(iaq_print);
                  String reset_print = "<b4>Reset: " + (String)rdata.rst[i] + "</b4><br>";
                  client.print(reset_print);
                  String battery_print = "<b4>Voltage: " + (String)rdata.batt[i] + " V</b4><br>";
                  client.print(battery_print);
                  String alive_print = "<b4>Alive: " + (String)rdata.alive[i] + "</b4><br>";
                  client.print(alive_print);
                  String retransmission_print = "<b4>reTX: " + (String)rdata.retx[i] + "</b4><br>";
                  client.print(retransmission_print);
                  String rssi_print = "<b4>RSSI: " + (String)rdata.rssi[i] + " dBm</b4><br>";
                  client.print(rssi_print);
                  client.print("<hr>");
                  digitalWrite(Core0_LED, LOW);
                }
                }*/
              // The HTTP response ends with another blank line
              client.println();
              // Break out of the while loop
              break;
            } else { // if you got a newline, then clear currentLine
              currentLine = "";
            }
          } else if (c != '\r') {  // if you got anything else but a carriage return character,
            currentLine += c;      // add it to the end of the currentLine
          }
        }
      }
      // Clear the header variable
      header = "";
      // Close the connection
      client.stop();
      digitalWrite(Wifi_LED, HIGH);
      Serial.println("Client disconnected.");
      Serial.println("");
    }
    digitalWrite(Core0_LED, !digitalRead(Core0_LED));
    delay(5);
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
    delay(10);
    digitalWrite(Comms_LED, HIGH);
  }
  digitalWrite(Core1_LED, LOW);
  delay(10);
  display.clearDisplay();
  display.display();

  display.setCursor(0, 0);
  uint8_t color = WHITE;
  for (int16_t i = min(128, 64) / 2; i > 0; i -= 5) {
    display.drawTriangle(128 / 2, 64 / 2 - i,
                         128 / 2 - i, 64 / 2 + i,
                         128 / 2 + i, 64 / 2 + i, WHITE);
    if (color == WHITE) color = BLACK;
    else color = WHITE;
    display.display();
  }
  display.setCursor(5, 35);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setFont(&FreeSans9pt7b);
  display.print("GATEWAY V2");
  display.setFont();
  display.setCursor(0, 4);
  display.setTextSize(1);
  display.print("FW VER");
  display.setCursor(0, 12);
  display.setTextSize(1);
  display.print("v1.3");
  display.display();

  for (int i = 0; i < 101; i=i+7) {
    display.fillRoundRect(50, 45, 25, 12, 2, BLACK);
    display.setCursor(52, 47);
    display.print(i);
    display.print("%");
    display.display();
  }
  display.fillRoundRect(50, 45, 25, 12, 2, BLACK);
  display.setCursor(52, 47);
  display.print("100%");
  display.display();

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 8);
  display.setFont(&FreeSans9pt7b);
  display.print("Gateway v2");
  display.setTextSize(1);
  display.setFont();
  display.setCursor(0, 18);
  display.print("SSID:" + (String)WIFI_SSID + "  RSSI:" + WiFi.RSSI());
  display.setCursor(0, 26);
  display.print("IP:" + WiFi.localIP().toString());


  delay(10);
  int ledstat = 1;
  bool incr = true;
  while (true) {
    unsigned long display_time = millis();
    if (display_time - display_prev >= display_interval) {
      display_prev = display_time;
      tmElements_t t;
      rtc.time_get(&t);
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.fillRect(0, 34, 128, 24, BLACK);
      display.setTextColor(WHITE);
      display.setCursor(0, 34);
      display.print("TEMP:" + (String)sht20.readTemperature() + "  HUM:" + (String)sht20.readHumidity());
      display.setCursor(0, 42);
      display.print("SYSTIME:" + (String)t.Day + "/" + (String)t.Month + "/" + (String)(t.Year + 1970));
      display.setCursor(0, 50);

      display.print("        " + (String)t.Hour + ":" + (String)t.Minute + + ":" + (String)t.Second);
      display.display();
      delay(10);


    }

    if (Serial2.available() > 0) {
      digitalWrite(Core1_LED, HIGH);
      digitalWrite(Comms_LED, LOW);
      Serial2.setTimeout(5);
      uart_recv_buffer = Serial2.readString();
      Serial2.flush();
      uart_recv_pid = uart_recv_buffer.substring(uart_recv_buffer.indexOf("pid:") + 4, uart_recv_buffer.indexOf(";ctrl:"));

      uart_recv_uid = uart_recv_buffer.substring(uart_recv_buffer.indexOf("src:") + 4, uart_recv_buffer.indexOf(";", uart_recv_buffer.indexOf("src:")));
      uart_parser(uart_recv_pid, uart_recv_uid, uart_recv_buffer);
      digitalWrite(Comms_LED, HIGH);
      Serial.print("data in");
    }
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
  uid = strtol(uart_uid.c_str(), NULL, 0);
  if (uid > 0 && uid < 65535) {

    switch (pid) {
      /*case pid_rst:
        {
          for (int i = 0; i < 255; i++) {
            if (data_struct.device[i] == uid && uart_buffer.substring(uart_buffer.indexOf("reset:") + 6, uart_buffer.indexOf("reset:") + 7) == "1") {
              String temp = data_struct.rst[uid];
              int temp_int = temp.toInt();
              temp_int++;
              temp = String(temp_int);
              data_struct.rst[uid] = temp;
              Serial.println("[TASK1] recv rst data from device " + (String)uid);
              break;
            }
          }
          break;
        }
        case pid_stat:
        for (int i = 0; i < 255; i++) {
          if (data_struct.device[i] == uid) {
            data_struct.alive[uid] = uart_buffer.substring(uart_buffer.indexOf("alive:") + 6, uart_buffer.indexOf(";nb:"));
            data_struct.retx[uid] = uart_buffer.substring(uart_buffer.indexOf("nb:") + 3, uart_buffer.indexOf(";rx1:"));
            data_struct.rssi[uid] = uart_buffer.substring(uart_buffer.indexOf("rx1:") + 6, uart_buffer.indexOf(",") + 4);
            Serial.println("[TASK1] recv stat data from device " + (String)uid);
            break;
          }
        }
        break;*/
      case pid_env:
        for (int i = 0; i < 255; i++) {
          if (data_struct.device[i] == uid) {
            data_struct.temp[i] = uart_buffer.substring(uart_buffer.indexOf("temp:") + 5, uart_buffer.indexOf(";hum:"));
            data_struct.hum[i] = uart_buffer.substring(uart_buffer.indexOf("hum:") + 4, uart_buffer.indexOf(";press:"));
            data_struct.press[i] = uart_buffer.substring(uart_buffer.indexOf("press:") + 6, uart_buffer.indexOf("press:") + 13);
            data_struct.light[i] = uart_buffer.substring(uart_buffer.indexOf("light:") + 6, uart_buffer.indexOf("light:") + 13);
            Serial.print(i);
            Serial.print(data_struct.light[i]);
            break;
          }
          else if (i == 254) {
            int data_struct_current = data_struct_top + 1;
            data_struct.device[data_struct_current] = uid;
            data_struct.temp[data_struct_current] = uart_buffer.substring(uart_buffer.indexOf("temp:") + 5, uart_buffer.indexOf(";hum:"));
            data_struct.hum[data_struct_current] = uart_buffer.substring(uart_buffer.indexOf("hum:") + 4, uart_buffer.indexOf(";press:"));
            data_struct.press[data_struct_current] = uart_buffer.substring(uart_buffer.indexOf("press:") + 6, uart_buffer.indexOf("press:") + 13);
            data_struct.light[data_struct_current] = uart_buffer.substring(uart_buffer.indexOf("light:") + 6, uart_buffer.indexOf("light:") + 13);
            data_struct_top = data_struct_current;
            Serial.println("NEW");
          }
        }
        Serial.println("[TASK1] recv env data from device " + (String)uid);
        break;
      /*
        case pid_iaq:
        if (!device[uid]) {
        device[uid] = 162;
        }
        rdata.temp[uid] = uart_buffer.substring(uart_buffer.indexOf("temp:") + 5, uart_buffer.indexOf(";hum:"));
        rdata.hum[uid] = uart_buffer.substring(uart_buffer.indexOf("hum:") + 4, uart_buffer.indexOf(";press:"));
        rdata.press[uid] = uart_buffer.substring(uart_buffer.indexOf("press:") + 6, uart_buffer.indexOf("press:") + 13);
        rdata.light[uid] = uart_buffer.substring(uart_buffer.indexOf("light:") + 6, uart_buffer.indexOf("light:") + 13);
        rdata.iaq[uid] = uart_buffer.substring(uart_buffer.indexOf("iaq:") + 4, uart_buffer.indexOf("iaq:") + 9);
        Serial.println("[TASK1] recv iaq data from device " + (String)uid);
        break;
        case pid_motion:
        break;
        case pid_batt:
        rdata.batt[uid] = uart_buffer.substring(uart_buffer.indexOf("voltage:") + 8, uart_buffer.indexOf("voltage:") + 13);
        //flag.batt = true;
        break;*/
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
