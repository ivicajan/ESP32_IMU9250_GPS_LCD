/***************************************************************************
LCD DISPLAY 
IMU 9250 Accel, pitch, roll, magnet
MCP 9808 Temp
GPS
***************************************************************************/
#include <Wire.h>
// IMU9250
#include <MPU9250_WE.h>
#define MPU9250_ADDR 0x68
#define SDA_PIN 23
#define SCL_PIN 22
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

// MCP9808 Temp sensor
#include "Adafruit_MCP9808.h"
#define MCP9808_ADDR 0x18
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();  // Create the MCP9808 temperature sensor object

// LCD part, I2C address is 0x3C, connected to 19 and 18 pin
#define OLED_SDA 19
#define OLED_SCL 18
#include <U8g2lib.h>
U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ U8X8_PIN_NONE);

// GPS
#include <TinyGPS++.h>
// GPS setup GPIO 16 -> GPS RX , GPIO 17 -> GPS TX
static const int RXPin = 17, TXPin = 16;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial ss(1);

// WEB SERVER
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
const char* ssid = "ESP32";   // Wifi ssid and password
const char* password = "tt";
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
// Create an Event Source on /events
AsyncEventSource events("/events");

#include <Time.h>
#define AWST_TIME_OFFSET (8 * 3600)  // Local Time (AWST)
#define DECLINATION 1.f // Magnetic declination for Perth

byte last_second, Second, Minute, Hour, Day, Month;
int Year;
double last_lng = 0.0;
double last_lat = 0.0;
float total_distance;
String tTemp, tSpeed;
String csvOutStr = "";
String tTime, tDate, tDateTime;
String tmonth, tday, thour, tminute, tsecond;
String tLocation, tAge, tDist, tTDist, tSat;
String dataMessage;
bool sdOK, gpsDateOK, gpsLocationOK;
int readingID = 0;

typedef struct {
  xyzFloat gValue;
  xyzFloat gyr;
  float heading;
  float MPU_heading;
  float pitch;
  float roll;
  float MPU_temp;
  float resultantG;
} IMU_Readback;

static IMU_Readback imu_readback;

static float temperature; // MCP temp sensor

// functions
void u8g2_ascii_2() {
  char s[2] = " ";
  u8g2.drawStr(0, 0, "ASCII page 2");
  for(uint8_t y = 0; y < 6; y++) {
    for(uint8_t x = 0; x < 16; x++) {
      s[0] = y * 16 + x + 160;
      u8g2.drawStr(x * 7, y * 10 + 10, s);
    }
  }
}

void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

String processor(const String& var){
//  getSensorReadings();
  //Serial.println(var);
  if(var == "TEMPERATURE"){
    return String(temperature);
  }
  else if(var == "HEADING"){
    return String(imu_readback.heading);
  }
  else if(var == "DATETIME"){
    return String(tDateTime);
  }
  else if(var == "LON"){
    return String(last_lng);
  }
  else if(var == "LAT"){
    return String(last_lat);
  }
  else if(var == "ROLL"){
    return String(imu_readback.roll);
  }  
  else if(var == "PITCH"){
    return String(imu_readback.pitch);
  }
  return String();
}

// web page
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
  <head>
    <title>ESP Web Server</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      html {font-family: Arial; display: inline-block; text-align: center;}
      p { font-size: 1.2rem;}
      body {  margin: 0;}
      .topnav { overflow: hidden; background-color: #50B8B4; color: white; font-size: 1rem; }
      .topnav2 { overflow: hidden; background-color: #50B8B4; color: red; font-size: 1.5rem; }
      .content { padding: 20px; }
      .card { background-color: white; box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5); }
      .cards { max-width: 800px; margin: 0 auto; display: grid; grid-gap: 2rem; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); }
      .reading { font-size: 1.8rem; }
    </style>
  </head>
  <body>
    <div class="topnav">
    <h1> GPS <span id="lon"> %LON% </span> &deg E,  <span id="lat"> %LAT% </span> &deg S</h1>
    <h1> <span id="datetime"> %DATETIME% </span> AWST </h1>
    </div>
    <div class="topnav2">
    <h1>Heading <span id="heading"> %HEADING% </span> &deg</span></h1>
    </div>
    <div class="content">
      <div class="cards">
      <div class="card">
        <p><i class="fas fa-angle-double-down" style="color:#e1e437;"></i> PITCH  </p><p><span class="reading"><span id="pitch">%PITCH%</span> &deg </span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-angle-double-down" style="color:#e1e437;"></i> ROLL   </p><p><span class="reading"><span id="roll">%ROLL%</span> &deg </span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-thermometer-half"  style="color:#059e8a;"></i> TEMPERATURE</p><p><span class="reading"><span id="temperature">%TEMPERATURE%</span> &deg C </span></p>
      </div>
      </div>
    </div>
    <script>
      if (!!window.EventSource) {
        var source = new EventSource('/events');
        
        source.addEventListener('open', function(e) {
          console.log("Events Connected");
        }, false);
        source.addEventListener('error', function(e) {
          if (e.target.readyState != EventSource.OPEN) {
            console.log("Events Disconnected");
          }
        }, false);
        source.addEventListener('message', function(e) {
          console.log("message", e.data);
        }, false);
        source.addEventListener('temperature', function(e) {
          console.log("temperature", e.data);
          document.getElementById("temp").innerHTML = e.data;
        }, false);        
        source.addEventListener('heading', function(e) {
          console.log("heading", e.data);
          document.getElementById("heading").innerHTML = e.data;
        }, false);
        source.addEventListener('last_lng', function(e) {
          console.log("lon", e.data);
          document.getElementById("lon").innerHTML = e.data;
        }, false);
        source.addEventListener('last_lat', function(e) {
          console.log("lat", e.data);
          document.getElementById("lat").innerHTML = e.data;
         }, false);
        source.addEventListener('pitch', function(e) {
          console.log("pitch", e.data);
          document.getElementById("pitch").innerHTML = e.data;
        }, false);
        source.addEventListener('roll', function(e) {
          console.log("roll", e.data);
          document.getElementById("roll").innerHTML = e.data;
         }, false);
         source.addEventListener('datetime', function(e) {
          console.log("datetime", e.data);
          document.getElementById("datetime").innerHTML = e.data;
         }, false);
        }
    </script>
  </body>
</html>)rawliteral";

void init_wifi() {
  
  WiFi.disconnect(true);             // that no old information is stored
  WiFi.mode(WIFI_OFF);               // switch WiFi off
  delay(1000);                       // short wait to ensure WIFI_OFF
  WiFi.persistent(false);            // avoid that WiFi-parameters will be stored in persistent
  
  WiFi.softAP(ssid);
  delay(2000); // add delay to be safe
  Serial.println(WiFi.softAPIP());    // Print ESP32 Local IP Address
}

void init_lcd() {
  u8g2.begin();     // Start display library
  u8g2.clearBuffer();
  u8g2_prepare();
  u8g2.drawBox(5, 10, 20, 10);
  u8g2.sendBuffer();
  delay(50);
}

void init_mpu() {
  //MPU 9250
  Wire.begin(SDA_PIN, SCL_PIN);
  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }
  if(!myMPU9250.initMagnetometer()){
    Serial.println("Magnetometer does not respond");
  }
  else{
    Serial.println("Magnetometer is connected");
  }
  delay(50);
}

void calibrate_mpu() {
  // Calibrate MPU
  Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  delay(2000);
  myMPU9250.autoOffsets();
  Serial.println("Offset done!");
  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6);
  myMPU9250.setSampleRateDivider(5);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  delay(50);
}

void init_temp_sensor() {
  // MCP 9808 init
  int cnt = 0;
  while (!tempsensor.begin(MCP9808_ADDR)) {
    Serial.println("Couldn't find MCP9808!");
    cnt++;
    if (cnt > 10) {
      return;
    }
    delay(50);
  }
  tempsensor.setResolution(3);  // 0.0625°C    250 ms
  Serial.println("MCP9808 temp: " + String(tempsensor.readTempC()));
  Serial.println(" ");
  delay(50);
}

void init_web_server_events() {
  // Handle Web Server Events
  // NOTE: This could be a source of the disconnection issue?
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
}

void setup() {
  Serial.begin(115200);

  init_wifi();
  init_lcd();

  init_mpu();
  calibrate_mpu();

  init_temp_sensor();

  // GPS init
  ss.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

  // WEB server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  init_web_server_events();

  server.addHandler(&events);
  server.begin();

  delay(200);
}

void imu_get_data() {
  imu_readback.gValue = myMPU9250.getGValues();
  imu_readback.gyr = myMPU9250.getGyrValues();
  const xyzFloat magValue = myMPU9250.getMagValues();
  const xyzFloat angleVal = myMPU9250.getAngles();
  imu_readback.MPU_temp = myMPU9250.getTemperature();
  imu_readback.resultantG = myMPU9250.getResultantG(imu_readback.gValue);
  imu_readback.pitch = myMPU9250.getPitch();
  imu_readback.roll  = myMPU9250.getRoll();
  
  const float Xh = (magValue.x * cos(imu_readback.pitch / 180 * PI) + magValue.y * sin(imu_readback.roll / 180 * PI) * sin(imu_readback.pitch / 180 * PI) - \
         magValue.z * cos(imu_readback.roll / 180 * PI) * sin(imu_readback.pitch / 180 * PI)) * 0.9 + magValue.x * 0.1;
  const float Yh = (magValue.y * cos(imu_readback.roll / 180 * PI) + magValue.z * sin(imu_readback.roll / 180 * PI)) * 0.9 + magValue.y * 0.1;

  Serial.println("Acceleration in g (x,y,z):");
  Serial.print(imu_readback.gValue.x);
  Serial.print("   ");
  Serial.print(imu_readback.gValue.y);
  Serial.print("   ");
  Serial.println(imu_readback.gValue.z);
  Serial.print("Resultant g: ");
  Serial.println(imu_readback.resultantG);

  Serial.println("Gyroscope data in degrees/s: ");
  Serial.print(imu_readback.gyr.x);
  Serial.print("   ");
  Serial.print(imu_readback.gyr.y);
  Serial.print("   ");
  Serial.println(imu_readback.gyr.z);

  Serial.print("Pitch   = "); 
  Serial.print(imu_readback.pitch);
  Serial.print("  |  Roll    = ");
  Serial.println(imu_readback.roll);

/*
  Serial.println("Magnetometer Data in µTesla: ");
  Serial.print(magValue.x);
  Serial.print("   ");
  Serial.print(magValue.y);
  Serial.print("   ");
  Serial.println(magValue.z);
 */
  
  Serial.print("Heading: ");
  imu_readback.MPU_heading = atan2(magValue.x, magValue.y) * 180 / PI + DECLINATION;
  Serial.print(imu_readback.MPU_heading);
  Serial.print("    |    Heading Corr: ");
  imu_readback.heading = atan2(Xh, Yh) * 180 / PI;
  imu_readback.heading += DECLINATION;
  
  if (imu_readback.heading > 360.f) imu_readback.heading -= 360.f;
  if (imu_readback.heading < 0.f)   imu_readback.heading += 360.f;
  // ----- Allow for under/overflow
  if (imu_readback.heading < 0.f) imu_readback.heading += 360.f;
  if (imu_readback.heading >= 360.f) imu_readback.heading -= 360.f;

  Serial.println(imu_readback.heading);

/*
  Serial.print("Orientation of the module: ");
  Serial.println(myMPU9250.getOrientationAsString());
*/
  // Send Events to the Web Server with the Sensor Readings
  events.send(String(imu_readback.heading).c_str(), "heading", millis());
  events.send(String(imu_readback.pitch).c_str(), "pitch", millis());
  events.send(String(imu_readback.roll).c_str(), "roll", millis());
  delay(50);
}

void temp_sensor_get_data() {
  temperature = tempsensor.readTempC();
  // Send Events to the Web Server with the Sensor Readings
  events.send(String(temperature).c_str(), "temperature", millis());
  delay(50);
}

void draw_heading() {
  u8g2.setCursor(0, 0);
  u8g2.print("HEADING ");
  u8g2.setCursor(50, 0);
  u8g2.print(String(imu_readback.heading, 1));
  u8g2.setCursor(90, 0);
  u8g2.print(String(imu_readback.MPU_heading, 1));
}

void draw_pitch() {
  u8g2.setCursor(0, 10);
  u8g2.print("PITCH ");
  u8g2.setCursor(35, 10);
  u8g2.print(String(imu_readback.pitch, 0));
}

void draw_roll() {
  u8g2.setCursor(70, 10);
  u8g2.print("ROLL ");
  u8g2.setCursor(100, 10);
  u8g2.print(String(imu_readback.roll, 0));
}

void draw_acc() {
  u8g2.setCursor(0, 20);
  u8g2.print("Acc");
  u8g2.setCursor(35, 20);
  u8g2.print(String(imu_readback.gValue.x, 1));
  u8g2.setCursor(65, 20);
  u8g2.print(String(imu_readback.gValue.y, 1));
  u8g2.setCursor(100, 20);
  u8g2.print(String(imu_readback.gValue.z, 1));
}

void draw_gyro() {
  u8g2.setCursor(0, 30);
  u8g2.print("Gyro");
  u8g2.setCursor(35, 30);
  u8g2.print(String(imu_readback.gyr.x, 0));
  u8g2.setCursor(65, 30);
  u8g2.print(String(imu_readback.gyr.y, 0));
  u8g2.setCursor(100, 30);
  u8g2.print(String(imu_readback.gyr.z, 0));
}

void draw_temp() {
  u8g2.setCursor(0, 40);
  u8g2.print("Temp (C): ");
  u8g2.setCursor(70, 40);
  u8g2.print(String(temperature, 1));   // MCP 9808 temp
}

void draw_total_g() {
  u8g2.setCursor(0, 50);
  u8g2.print("Total  G:");
  u8g2.setCursor(70, 50);
  u8g2.print(String(imu_readback.resultantG, 1));
}

void update_display() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  //u8g2.setFont(u8g2_font_helvB10_tr);

  draw_heading();
  draw_pitch();
  draw_roll();
  draw_acc();
  draw_gyro();
  draw_temp();
  draw_total_g();

  u8g2.sendBuffer();
  delay(50);
}

void loop() {
  imu_get_data();
  temp_sensor_get_data();

  Serial.print(" MCP9808 temp: " + String(temperature));
  Serial.println("   |    MPU9250 temp: " + String(imu_readback.MPU_temp));
  // GET GPS DATA
  SerialGPSDecode();
  
  // DISPLAY DATA ON LCD
  Serial.println("********************************************");
  update_display();
  // UPDATE WEB SERVER
  // Send Ping to the Web Server
  events.send("ping", NULL, millis());
  delay(50);
}

void SerialGPSDecode() {
    unsigned long start = millis();
    gpsDateOK = false;
    gpsLocationOK = false;
    do {
      while (ss.available() > 0)
      gps.encode(ss.read());
    } while (millis() - start < 800);
       if (gps.date.isValid())
        { 
          gpsDateOK = true;
          tSat = String(gps.satellites.value());
          Minute = gps.time.minute();
          Second = gps.time.second();
          Hour   = gps.time.hour();
          Day   = gps.date.day();
          Month = gps.date.month();
          Year  = gps.date.year();
          // set current UTC time
          setTime(Hour, Minute, Second, Day, Month, Year); // This update Time structure (small letters)
          // add the offset to get local time
          adjustTime(AWST_TIME_OFFSET); // This adds offset to Time structure
          if (month() < 10) {
            tmonth = "0" + String(month());
          } else {
            tmonth = String(month());
          }
          if (day() < 10) {
            tday = "0" + String(day());
          } else {
            tday = String(day());
          }
          if (hour() < 10) {
            thour = "0" + String(hour());
          } else {
            thour = String(hour());
          }
          if (minute() < 10) {
            tminute = "0" + String(minute());
          } else {
            tminute = String(minute());
          }
          if (second()< 10) {
            tsecond = "0" + String(second());
          } else {
            tsecond = String(second());
          }
          tDate = String(year()) + "/" + tmonth + "/" + tday;
          tTime = thour + ":" + tminute + ":" + tsecond;
          tDateTime = tDate + " " + tTime;
          last_second = gps.time.second();
        }

        if (gps.location.isValid() && gps.location.age() < 1000) {
          gpsLocationOK = true;
          tAge = String(gps.location.age());
          tLocation = String(gps.location.lng(),6) + "," + String(gps.location.lat(),6);
          int speed = gps.speed.kmph();
          tSpeed = String(speed);
          if (last_lat) { 
            const float distance = gps.distanceBetween(gps.location.lat(),gps.location.lng(),last_lat,last_lng);  // in meters
            tDist = String(distance);
            if (distance > 2 && speed > 2)
            {
              total_distance += distance/1000.0;
            }
          }
          last_lng = gps.location.lng();
          last_lat = gps.location.lat();
        }
          if (!gpsDateOK) {
            tTime = "--:--:--";
            tAge = "--";
          }
          if (!gpsLocationOK) {
            tSpeed = "---";
            tAge = "--";
            tDist = "--";
          }
          tTDist = String(total_distance, 1);
          csvOutStr = tDateTime + "," + tLocation + "," + tTemp + "," + tSpeed + "," + tDist + "," + tTDist + "\n";
          Serial.print(csvOutStr);
          events.send(String(last_lng).c_str(),"lon",millis()); 
          events.send(String(last_lat).c_str(),"lat",millis());
          events.send(String(tDateTime).c_str(),"datetime",millis());              
}
