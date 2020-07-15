/*
Board Name: "WeMos" WiFi & Bluetooth Battery
References: https://www.instructables.com/id/ESP32-WiFi-SOIL-MOISTURE-SENSOR/
github: https://github.com/JJSlabbert/Higrow-ESP32-WiFi-Soil-Moisture-Sensor

Library used:
- ArduinoOTA: from .arduino15/packages/esp32/hardware/esp32/1.0.4/libraries/ArduinoOTA
  copy to ./platformio/lib/ArduinoOTA_ESP32
- Ticker-esp32: for Timer based Periodic function  (install from PlatformIO Libraries)
- FirebaseESP32
    - require: HTTPClientESP32Ex

  Progress:
  - ESP32 OTA ok
  - Use mDNS to register esp32 hostname, so device can address by hostname.local, 
    device no longer need to use static IP
  - Add Ticker library to remove delay() in loop()  
  - Remove ArduinoOTA onStart, onProgess, onEnd -> speed up detection and upload progress 
  - Add OTA password
  - Add OTA custom port
*/
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <FirebaseESP32.h>
// #include <FirebaseJson.h>
#include <Ticker.h>

#define WIFI_SSID "Steff-IoT"
#define WIFI_PASSWORD "steffiot123"
#define HOSTNAME "esp32-steff"

#define FIREBASE_HOST "https://smarthydro-a448b.firebaseio.com/"
#define FIREBASE_AUTH "idXcyYOkq7aAluuzanJpQ2NtAGnCTb8L53JcOrcK"

#define FIREBASE_SENSOR_TEMP "SoilMoisture/Sensor/Temperature"
#define FIREBASE_SENSOR_HUMIDITY "SoilMoisture/Sensor/Humidity"
#define FIREBASE_SENSOR_SOIL_MOISTURE "SoilMoisture/Sensor/SoilMoisture"
#define FIREBASE_CONTROL "SoilMoisture/Control"

#define PIN_SOIL_MOISTURE_SENSOR 32
#define PIN_DHT 22
#define DHTTYPE DHT11

float asoilmoist=analogRead(PIN_SOIL_MOISTURE_SENSOR);
// int nSensorUpdateInterval = 3;

DHT dht(PIN_DHT, DHTTYPE);
FirebaseData firebaseData;
FirebaseData firebaseDataCallback;

void onTimer1Sec();
void onBlinkOff();
void onReadSensor();
void onFirebaseStream(StreamData data);
void printResult(StreamData &data);
void updateSoilMostureSensor();

Ticker timer1Sec(onTimer1Sec, 1000, 0, MILLIS);
Ticker timerReadSensor(onReadSensor, 10*60*1000, 0, MILLIS);
Ticker timerBlinkOff(onBlinkOff, 100, 1, MILLIS);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  delay(500);
  dht.begin();

  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // register Hostname to mDNS service
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.setPort(7878);
  ArduinoOTA.setPassword("steff!@#");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.begin();
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  //tiny, small, medium, large and unlimited.
  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(firebaseData, "tiny");
  Firebase.beginStream(firebaseDataCallback, FIREBASE_CONTROL);
  Firebase.setStreamCallback(firebaseDataCallback, onFirebaseStream);
  Serial.println("System ready, connected with IP Address");
  Serial.println(WiFi.localIP());
  onReadSensor();
  timerReadSensor.start();
  timer1Sec.start();
}

void loop() {
  // static int nCount=0;
  ArduinoOTA.handle();
  timer1Sec.update();
  // timerBlinkOff.update();
  timerReadSensor.update();
}

void onBlinkOff()
{
  digitalWrite(LED_BUILTIN, HIGH);
}

void onReadSensor()
{
  digitalWrite(LED_BUILTIN, LOW);
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
  }
  else
  {
    // float hic = dht.computeHeatIndex(t, h, false);
    updateSoilMostureSensor();
    Serial.printf("Temp: %.2f, Hum: %.2f, Moisture: %.2f\n", t, h, asoilmoist);
    Firebase.pushFloat(firebaseData, FIREBASE_SENSOR_TEMP, t);
    Firebase.pushFloat(firebaseData, FIREBASE_SENSOR_HUMIDITY, h);
    Firebase.pushFloat(firebaseData, FIREBASE_SENSOR_SOIL_MOISTURE, asoilmoist);
  }    
  digitalWrite(LED_BUILTIN, HIGH);
}

void updateSoilMostureSensor()
{
  asoilmoist=0.095*asoilmoist+0.05*analogRead(PIN_SOIL_MOISTURE_SENSOR);//exponential smoothing of soil moisture
}

void onTimer1Sec()
{
  // digitalWrite(LED_BUILTIN, LOW);
  updateSoilMostureSensor();
  // timerBlinkOff.start();
}

/*
STREAM PATH: /SoilMoisture/Control
EVENT PATH: /SensorUpdateInterval
DATA TYPE: int
EVENT TYPE: put
VALUE: 3
*/
void onFirebaseStream(StreamData data)
{
  /*
  Serial.println("Stream Data1 available...");
  Serial.println("STREAM PATH: " + data.streamPath());
  Serial.println("EVENT PATH: " + data.dataPath());
  Serial.println("DATA TYPE: " + data.dataType());
  Serial.println("EVENT TYPE: " + data.eventType());
  Serial.print("VALUE: ");
  printResult(data);
  Serial.println();
*/
  if (data.dataPath()=="/SensorUpdateInterval")
  {
    int nSensorUpdateInterval = data.intData()*1000;
    Serial.printf("Set SensorUpdateInterval=%d secs\n", nSensorUpdateInterval);
    timerReadSensor.stop();
    timerReadSensor.interval(nSensorUpdateInterval);
    timerReadSensor.start();
  }
}

void printResult(StreamData &data)
{

  if (data.dataType() == "int")
    Serial.println(data.intData());
  else if (data.dataType() == "float")
    Serial.println(data.floatData(), 5);
  else if (data.dataType() == "double")
    printf("%.9lf\n", data.doubleData());
  else if (data.dataType() == "boolean")
    Serial.println(data.boolData() == 1 ? "true" : "false");
  else if (data.dataType() == "string" || data.dataType() == "null")
    Serial.println(data.stringData());
  else if (data.dataType() == "json")
  {
    Serial.println();
    FirebaseJson *json = data.jsonObject();
    //Print all object data
    Serial.println("Pretty printed JSON data:");
    String jsonStr;
    json->toString(jsonStr, true);
    Serial.println(jsonStr);
    Serial.println();
    Serial.println("Iterate JSON data:");
    Serial.println();
    size_t len = json->iteratorBegin();
    String key, value = "";
    /*
    int type = 0;
    for (size_t i = 0; i < len; i++)
    {
      json->iteratorGet(i, type, key, value);
      Serial.print(i);
      Serial.print(", ");
      Serial.print("Type: ");
      Serial.print(type == FirebaseJson:: JSON_OBJECT ? "object" : "array");
      if (type == FirebaseJson::JSON_OBJECT)
      {
        Serial.print(", Key: ");
        Serial.print(key);
      }
      Serial.print(", Value: ");
      Serial.println(value);
    }
    json->iteratorEnd();
  }
  else if (data.dataType() == "array")
  {
    Serial.println();
    //get array data from FirebaseData using FirebaseJsonArray object
    FirebaseJsonArray *arr = data.jsonArrayPtr();
    //Print all array values
    Serial.println("Pretty printed Array:");
    String arrStr;
    arr->toString(arrStr, true);
    Serial.println(arrStr);
    Serial.println();
    Serial.println("Iterate array values:");
    Serial.println();

    for (size_t i = 0; i < arr->size(); i++)
    {
      Serial.print(i);
      Serial.print(", Value: ");

      FirebaseJsonData *jsonData = data.jsonDataPtr();
      //Get the result data from FirebaseJsonArray object
      arr->get(*jsonData, i);
      if (jsonData->typeNum == FirebaseJson::JSON_BOOL)
        Serial.println(jsonData->boolValue ? "true" : "false");
      else if (jsonData->typeNum == FirebaseJson::JSON_INT)
        Serial.println(jsonData->intValue);
      else if (jsonData->typeNum == FirebaseJson::JSON_DOUBLE)
        printf("%.9lf\n", jsonData->doubleValue);
      else if (jsonData->typeNum == FirebaseJson::JSON_STRING ||
               jsonData->typeNum == FirebaseJson::JSON_NULL ||
               jsonData->typeNum == FirebaseJson::JSON_OBJECT ||
               jsonData->typeNum == FirebaseJson::JSON_ARRAY)
        Serial.println(jsonData->stringValue);
    }
    */
  }
}