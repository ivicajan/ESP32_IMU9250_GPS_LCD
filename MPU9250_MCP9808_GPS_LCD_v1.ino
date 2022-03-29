/***************************************************************************
LCD DISPLAY 
IMU 9250 Accel, pitch, roll, magnet
MCP 9808 Temp
GPS
***************************************************************************/
#include <Wire.h>
#define USE_MPU
#define USE_MCP
#define USE_GPS
#define USE_LCD
#define USE_WEB

#ifdef USE_MPU
// IMU9250
#include <MPU9250_WE.h>
#define MPU9250_ADDR 0x68
#define SDA_PIN 23
#define SCL_PIN 22
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
#endif

#ifdef USE_MCP
// MCP9808 Temp sensor
#include "Adafruit_MCP9808.h"
#define MCP9808_ADDR 0x18
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();  // Create the MCP9808 temperature sensor object
#endif

#ifdef USE_LCD
// LCD part, I2C address is 0x3C, connected to 19 and 18 pin
#define OLED_SDA 19
#define OLED_SCL 18
#include <U8g2lib.h>
U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ U8X8_PIN_NONE);
#endif

#ifdef USE_GPS
// GPS
#include <TinyGPS++.h>
// GPS setup GPIO 16 -> GPS RX , GPIO 17 -> GPS TX
static const int RXPin = 17, TXPin = 16;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial ss(1);
#endif

#ifdef USE_WEB
// WEB SERVER
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
const char* ssid = "ESP32";   // Wifi ssid and password
// const char* password = "tt";
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
// Create an Event Source on /events
AsyncEventSource events("/events");
#endif

#include <Time.h>
const int time_offset = 8*3600;  // Local Time (AWST)
byte last_second, Second, Minute, Hour, Day, Month;
int Year;
double last_lng = NULL;
double last_lat = NULL;
float total_distance;
String tTemp, tSpeed;
String csvOutStr = "";
String tTime, tDate, tDateTime;
String tmonth, tday, thour, tminute, tsecond;
String tLocation, tAge, tDist, tTDist, tSat;
String dataMessage;
bool sdOK, gpsDateOK, gpsLocationOK;
int readingID = 0;

const float Pi = 3.14159;
const float Declination = 1.0; // Magnetic declination for Perth
float heading, MPU_heading, pitch, roll, temperature, MPU_temp, resultantG ;

String processor(const String& var){
//  getSensorReadings();
  //Serial.println(var);
  if(var == "TEMPERATURE"){
    return String(temperature);
  }
  else if(var == "HEADING"){
    return String(heading);
  }
  else if(var == "ROLL"){
    return String(roll);
  }
  else if(var == "PITCH"){
    return String(pitch);
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
    .content { padding: 20px; }
    .card { background-color: white; box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5); }
    .cards { max-width: 800px; margin: 0 auto; display: grid; grid-gap: 2rem; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); }
    .reading { font-size: 1.4rem; }
  </style>
</head>
<body>
  <div class="topnav">
    <h1>IMU MCP GPS </h1>
  </div>
  <div class="content">
    <div class="cards">
      <div class="card">
        <p><i class="fas fa-tint" style="color:#00add6;"></i> HEADING</p><p><span class="reading"><span id="hum">%HEADING%</span> &deg N;</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-angle-double-down" style="color:#e1e437;"></i> PITCH</p><p><span class="reading"><span id="pitch">%PITCH%</span> &deg; </span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-angle-double-down" style="color:#e1e437;"></i> ROLL</p><p><span class="reading"><span id="roll">%ROLL%</span> &deg; </span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-thermometer-half" style="color:#059e8a;"></i> TEMPERATURE</p><p><span class="reading"><span id="temperature">%TEMPERATURE%</span> &deg;C</span></p>
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
 
 source.addEventListener('pitch', function(e) {
  console.log("pitch", e.data);
  document.getElementById("pitch").innerHTML = e.data;
 }, false);

 source.addEventListener('roll', function(e) {
  console.log("roll", e.data);
  document.getElementById("roll").innerHTML = e.data;
 }, false);
}

</script>
</body>
</html>)rawliteral";
 

void setup() {
  int cnt = 0;
  Serial.begin(115200);

  init_WIFI();
  delay(100);
  
  init_LCD();
  delay(50);

  init_MPU();
  delay(50);  

  init_MCP();
  delay(50);  
  
  init_GPS();
  delay(50);
  
  init_WEB();
  delay(200);  
}

void loop() {    

  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr = myMPU9250.getGyrValues();
  xyzFloat magValue = myMPU9250.getMagValues();
  xyzFloat angleVal = myMPU9250.getAngles();
  MPU_temp = myMPU9250.getTemperature();
  resultantG = myMPU9250.getResultantG(gValue);
  pitch = myMPU9250.getPitch();
  roll  = myMPU9250.getRoll();
  
  float Xh = magValue.x*cos(pitch/180*Pi) + magValue.y*sin(roll/180*Pi)*sin(pitch/180*Pi) - \
  			 magValue.z*cos(roll/180*Pi)*sin(pitch/180*Pi);
  float Yh = magValue.y*cos(roll/180*Pi) + magValue.z*sin(roll/180*Pi);
  Xh = Xh * 0.9 + magValue.x * 0.1;
  Yh = Yh * 0.9 + magValue.y * 0.1;
  Serial.println("Acceleration in g (x,y,z):");
  Serial.print(gValue.x);
  Serial.print("   ");
  Serial.print(gValue.y);
  Serial.print("   ");
  Serial.println(gValue.z);
  Serial.print("Resultant g: ");
  Serial.println(resultantG);

  Serial.println("Gyroscope data in degrees/s: ");
  Serial.print(gyr.x);
  Serial.print("   ");
  Serial.print(gyr.y);
  Serial.print("   ");
  Serial.println(gyr.z);

  Serial.print("Pitch   = "); 
  Serial.print(pitch); 
  Serial.print("  |  Roll    = "); 
  Serial.println(roll); 
/*
  Serial.println("Magnetometer Data in µTesla: ");
  Serial.print(magValue.x);
  Serial.print("   ");
  Serial.print(magValue.y);
  Serial.print("   ");
  Serial.println(magValue.z);
 */  
  Serial.print("HEADING  MPU:");
  MPU_heading = atan2(magValue.x, magValue.y) * 180 / Pi;
  Serial.print(MPU_heading); 
  Serial.print("    |    COR: ");
  heading = atan2(Xh, Yh) * 180 / Pi;
  heading += Declination ; 
  if (heading > 360.0) heading -= 360.0;
  if (heading < 0.0)   heading += 360.0;
  // ----- Allow for under/overflow
  if (heading < 0) heading += 360 ;
  if (heading >= 360) heading -= 360 ;
  Serial.println(heading); 
  
  // MCP TEMPERATURE
  float temperature = tempsensor.readTempC();
  Serial.print("TEMPERATURE MCP: " + String(temperature));
  Serial.println("   |    MPU: " + String(MPU_temp));
/*
  Serial.print("Orientation of the module: ");
  Serial.println(myMPU9250.getOrientationAsString());
*/

  // GPS DATA
  SerialGPSDecode(ss, gps);
  
  // DISPLAY DATA ON LCD 
  Serial.println("********************************************");
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setCursor(0, 0);
  u8g2.print("HEADING ");
  u8g2.setCursor(50, 0);
  u8g2.print(String(heading,1));
  u8g2.setCursor(90, 0);
  u8g2.print(String(MPU_heading,1));
  u8g2.setCursor(0, 10);
  u8g2.print("PITCH ");
  u8g2.setCursor(35, 10);
  u8g2.print(String(pitch,0));
  u8g2.setCursor(70, 10);
  u8g2.print("ROLL ");
  u8g2.setCursor(100, 10);
  u8g2.print(String(roll,0));
  u8g2.setCursor(0, 20);
  u8g2.print("Acc");
  u8g2.setCursor(35, 20);
  u8g2.print(String(gValue.x,1));
  u8g2.setCursor(65, 20);
  u8g2.print(String(gValue.y,1));
  u8g2.setCursor(100, 20);
  u8g2.print(String(gValue.z,1));
  u8g2.setCursor(0, 30);
  u8g2.print("Gyro");
  u8g2.setCursor(35, 30);
  u8g2.print(String(gyr.x,0));
  u8g2.setCursor(65, 30);
  u8g2.print(String(gyr.y,0));
  u8g2.setCursor(100, 30);
  u8g2.print(String(gyr.z,0));
  u8g2.setCursor(0, 40);
  u8g2.print("Temp (C): ");
  u8g2.setCursor(70, 40);
  u8g2.print(String(temperature,1));   // MCP 9808 temp
  u8g2.setCursor(0, 50);
  u8g2.print("Total  G:");
  u8g2.setCursor(70, 50);
  u8g2.print(String(resultantG,1));
  u8g2.sendBuffer();  
  delay(50);

  // UPDATE WEB SERVER
  // Send Events to the Web Server with the Sensor Readings
  events.send("ping",NULL,millis());
  events.send(String(temperature).c_str(),"temperature",millis());
  events.send(String(heading).c_str(),"heading",millis());
  events.send(String(roll).c_str(),"roll",millis());
  events.send(String(pitch).c_str(),"pitch",millis()); 
  delay(50);
}

// DEFINITION OF USED FUNCTIONS 

void init_WIFI(void) {
  // WIFI server in AP mode
  WiFi.softAP(ssid);
  Serial.println(WiFi.softAPIP());
}    

void init_LCD(void) {
  // LCD INIT
  u8g2.begin();     // Start display library
  u8g2.clearBuffer();
  u8g2_prepare(); 
  u8g2.drawBox(5,10,20,10);
  u8g2.sendBuffer();
}

void ini_MPU(void) {
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
}    

void init_MCP(void) {
  // MCP 9808 init
  while (!tempsensor.begin(MCP9808_ADDR)) {
    Serial.println("Couldn't find MCP9808!");
    cnt++;
    if (cnt > 10) {
      return;
    }
    delay(50);
  }
  // tempsensor.shutdown_wake(0);
  // delay(250);
  tempsensor.setResolution(3);  // 0.0625°C    250 ms
  float c = tempsensor.readTempC();
  Serial.println(" MCP9808 temp: " + String(c));
  Serial.println(" ");
}    

void init_GPS(void) {
 // GPS init
  ss.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin); 
}
    
init_WEB(void) {

  // WEB server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);
  server.begin();
}  

void u8g2_ascii_2(void) {
  char s[2] = " ";
  uint8_t x, y;
  u8g2.drawStr( 0, 0, "ASCII page 2");
  for( y = 0; y < 6; y++ ) {
    for( x = 0; x < 16; x++ ) {
      s[0] = y*16 + x + 160;
      u8g2.drawStr(x*7, y*10+10, s);
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

void SerialGPSDecode(Stream &mySerial, TinyGPSPlus &myGPS) {
    unsigned long start = millis();
    gpsDateOK = false;
    gpsLocationOK = false;
    do
    {
     while (ss.available() > 0)
     gps.encode(ss.read());
    } while (millis() - start < 800);
//       if ((gps.date.isValid()) && (gps.location.age()<1000)) 
       if ((gps.date.isValid()) ) 
        { gpsDateOK = true;
          tSat = String(gps.satellites.value());
          Minute = gps.time.minute();
          Second = gps.time.second();
          Hour   = gps.time.hour();
          Day   = gps.date.day();
          Month = gps.date.month();
          Year  = gps.date.year();
          // set current UTC time
          setTime(Hour, Minute, Second, Day, Month, Year);
          // add the offset to get local time
          adjustTime(time_offset);
          if (month()<10) {
            tmonth = "0" + String(month());
          } else {
            tmonth = String(month());
          }
          if (day()<10) {
            tday = "0" + String(day());
          } else {
            tday = String(day());
          }
          if (hour()<10) {
            thour = "0" + String(hour());
          } else {
            thour = String(hour());
          }
          if (minute()<10) {
            tminute = "0" + String(minute());
          } else {
            tminute = String(minute());
          }
          if (second()<10) {
            tsecond = "0" + String(second());
          } else {
            tsecond = String(second());
          }
          tDate = String(year()) + "/" + tmonth + "/" + tday;
          tTime = thour + ":" + tminute + ":" + tsecond;
          tDateTime = tDate + " " + tTime;
          last_second = gps.time.second();
          //Serial.println(tDateTime);
        }

        if ((gps.location.isValid()) && (gps.location.age()<1000)) 
        { gpsLocationOK = true;
          tAge = String(gps.location.age());
          tLocation = String(gps.location.lng(),6) + "," + String(gps.location.lat(),6);
          int speed = gps.speed.kmph();
          tSpeed = String(speed);
        /*  
          if (speed < 10) 
          {
          tSpeed = "  " + String(speed);
          }
          else if ((speed > 10) && (speed <100))
          {
          tSpeed = " " + String(speed);
          } else {
          tSpeed = String(speed);
          }  
        */
          if ((last_lat != NULL))
          { float distance = gps.distanceBetween(gps.location.lat(),gps.location.lng(),last_lat,last_lng);  // in meters
//            Serial.println("distance =" + String(distance));
            tDist = String(distance);
            if ((distance > 2) && (speed > 2)) 
           // if ((distance > 2)) 
            {
            total_distance += distance/1000.0;
            }  
          }
          last_lng = gps.location.lng();
          last_lat = gps.location.lat();
        }  
          
          if (gpsDateOK != true) 
          {
            tTime = "--:--:--";
            tAge = "--";
           }
          if (gpsLocationOK != true) 
          {
            tSpeed = "---";
            tAge = "--";
            tDist = "--";
          }
          tTDist= String(total_distance,1);
/*          Serial.println("total distance =" + String(total_distance));
          Serial.println("date flag:" + String(gpsDateOK));
          Serial.println("location flag:" + String(gpsLocationOK));
          csvOutStr = tDateTime + "," + tLocation + "," + tTemp + "," + tSpeed + "\n";
*/          
          csvOutStr = tDateTime + "," + tLocation + "," + tTemp + "," + tSpeed + "," + tDist + "," + tTDist + "\n";
          Serial.print(csvOutStr);          
}
