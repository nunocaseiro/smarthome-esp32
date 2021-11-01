  #include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>
#include <base64.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "esp_camera.h"

#define MAX_SENSORS 20

// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

int pictureNumber = 0;
camera_fb_t * fb = NULL;

class Sensor {
  private:
    int id;
    String name;
    int gpio;
    double value;
    String type;
    int state;
    double lastValue;
  public:
    Sensor(int id, String name, int gpio, double value, String type) {
      this->id = id;
      this->name = name;
      this->gpio = gpio;
      this->value = value;
      this->type = type;
      this->state = LOW;
      this->lastValue = value;
      init();
    }

    Sensor() {
    }

    void init() {
      if (type.equals("led")) {
        pinMode(gpio, OUTPUT);
      }

      if (type.equals("plug")) {
        pinMode(gpio, OUTPUT);
      }

      if (type.equals("motion")) {
        Serial.println("Init.");
        pinMode(gpio, INPUT);
      }
    }

    void on() {
      digitalWrite(gpio, HIGH);
    }

    void off() {
      digitalWrite(gpio, LOW);
    }

    void read() {

    }

    int getGpio() {
      return this->gpio;
    }

    String getType() {
      return this->type;
    }

    String getName() {
      return this->name;
    }

    String getId() {
      return String(this->id);
    }

    void setValue(double val) {
      this->value = val;
    }

    double getValue() {
      return this->value;
    }

    void setLastValue(double val) {
      this->lastValue = val;
    }

    double getLastValue() {
      return this->lastValue;
    }

    void setState(int val) {
      this->state = val;
    }

    int getState() {
      return this->state;
    }

    double getCurrentTemperature() {
      OneWire oneWire(this->gpio);
      DallasTemperature dallas(&oneWire);
      dallas.begin();
      
      dallas.requestTemperatures();
      double temperature = dallas.getTempCByIndex(0);

      return temperature;
    }
};

//class Room ...
#define USE_SERIAL Serial
const char* ssid = "Visita Quinta Marques";
const char* password = "****";

const char* mqtt_server = "161.35.8.148";
int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

String authUsername = "room10";
String authPassword = "****";
String auth = base64::encode(authUsername + ":" + authPassword);

int roomID = 10;
Sensor sensors[MAX_SENSORS];

String getCountStr = "http://161.35.8.148/api/countSensorsByRoom/?room=";
String sensorsStr = "http://161.35.8.148/api/sensors/";
String sensorsOfRoom = "http://161.35.8.148/api/sensorsofroom/?room=";

String countReadings;
String sensorReadings;
int countSensors;

const unsigned long eventInterval = 1000;
const unsigned long eventIntervalToTakePicture = 500;
unsigned long previousTime = 0;
unsigned long previousSensorTime = 0;
size_t capacity = 0;


String httpGETRequest(String serverName) {
  HTTPClient http;

  // Your IP address with path or Domain name with URL path
  http.begin(serverName);
  http.addHeader("Authorization", "Basic " + auth);
  
  // Send HTTP POST request
  int httpResponseCode = http.GET();

  String payload = "{}";

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  
  // Free resources
  http.end();

  return payload;
}

void setup() {
  Serial.begin(115200);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_CIF; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  WiFi.begin(ssid, password);
  client.setBufferSize(1024);
  Serial.println(auth);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to the WiFi network");

    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    Serial.print("\nconnecting...");
    while (!client.connect("esp32J", "smarthome", "smarthome")) {
      Serial.print(".");
      delay(1000);
    }

    client.subscribe("all");

/*
    // Init Camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x", err);
      return;
    }
*/
    getCountStr.concat(roomID);
    sensorsOfRoom.concat(roomID);

    getAllSensors();
  }
}

void getAllSensors() {
  
  countReadings = httpGETRequest(getCountStr);
  Serial.println(countReadings);

  JSONVar myObject = JSON.parse(countReadings);

  // JSON.typeof(jsonVar) can be used to get the type of the var
  if (JSON.typeof(myObject) == "undefined") {
    Serial.println("Parsing input failed!");
    return;
  }

  Serial.print("JSON object = ");
  Serial.println(myObject);
  countSensors = myObject;
  Serial.println(countSensors);

  //malloc para os sensores
  //countSensors++;
  if (countSensors > 0) {
    //sensors = (Sensor*)realloc(sensors ,10 * sizeof(Sensor));

    sensorReadings = httpGETRequest(sensorsOfRoom);

    JSONVar sensorsObject = JSON.parse(sensorReadings);

    if (JSON.typeof(sensorsObject) == "undefined") {
      Serial.println("Parsing input failed!");
      return;
    }

    Serial.print("JSON object = ");
    Serial.println(sensorsObject);

    int idR;
    String nameR;
    int gpioR;
    double valueR;
    String sensortypeR;
    
    for (int i = 0; i < countSensors; i++) {
      idR = sensorsObject[i]["id"];
      nameR = sensorsObject[i]["name"];
      gpioR = sensorsObject[i]["gpio"];
      valueR = sensorsObject[i]["value"];
      sensortypeR = sensorsObject[i]["sensortype"];
      sensors[i] = Sensor(idR, nameR, gpioR, valueR, sensortypeR);

      String channel = "/" + String(idR);
      client.subscribe(channel.c_str());
      Serial.println("/" + String(idR));
    }

    Serial.print("Connected");

  } else {
    Serial.println("WiFi Disconnected");
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    // Create a random client ID
    String clientId = "ESP32Joao";
    clientId += String(random(0xffff), HEX);
    
    // Attempt to connect
    if (client.connect(clientId.c_str(), "smarthome", "smarthome")) {
      Serial.println("connected");

      for (int i = 0; i < countSensors; i++) {
        String channel = "/" + String(sensors[i].getId());
        client.subscribe(channel.c_str());
      }
     
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(1000);
    }
  }
}

void loop() {
  if(WiFi.status() == WL_CONNECTED) {
    unsigned long currentTime = millis();
    
    while (!client.connected()) {
      reconnect();
    }
    client.loop();

    if (currentTime - previousTime >= eventInterval) {      
      handleSensors();
      previousTime = currentTime;
    }
  }
  else {
    Serial.println("WiFi Disconnected");
  }
}


void handleSensors() {
  // unsigned long currentSensorTime = millis();

  for (int i = 0; i < countSensors; i++) {

    if (sensors[i].getType().equals("motion")) {
      sensors[i].setValue(digitalRead(sensors[i].getGpio()));
      Serial.println(digitalRead(sensors[i].getGpio()));

      if (sensors[i].getValue() == HIGH) {
        if (sensors[i].getState() == LOW) {
          Serial.println("Motion detected!");
          sensors[i].setState(HIGH);
        }
      } else {
        if (sensors[i].getState() == HIGH) {
          Serial.println("Motion ended!");
          sensors[i].setState(LOW);
        }
      }
    }

    if (sensors[i].getType().equals("temperature")) {
      double temperatureC = sensors[i].getCurrentTemperature();
      Serial.println(temperatureC);
      sensors[i].setValue(temperatureC);
    }

    if (sensors[i].getType().equals("luminosity")) {
      double ldrPercentage = (analogRead(sensors[i].getGpio()) * 100) / 4095;
      Serial.println(ldrPercentage);
      sensors[i].setValue(ldrPercentage);
      // sensors[i].setValue(analogRead(sensors[i].getGpio()));
    }
  
    if (sensors[i].getValue() != sensors[i].getLastValue()) {
      sensors[i].setLastValue(sensors[i].getValue());
      String channel = "/" + sensors[i].getId();
      client.publish(channel.c_str(), createMessage("server", String(roomID), "sval", String(sensors[i].getValue())));
    }
  }
}

const char* createMessage(String dest, String from, String action, String value) {
  JSONVar myObject;
  myObject["to"] = dest;
  myObject["from"] = from;
  myObject["action"] = action;
  myObject["value"] = value;

  String jsonString = JSON.stringify(myObject);
  return jsonString.c_str();
}


String createMessagePhotoAux(String aux) {
  JSONVar myObject;
  myObject["photo"] = aux;

  String jsonString = JSON.stringify(myObject);
  return jsonString;
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.println("] ");
  String top = String(topic);
  String sensorId = top.substring(1);

  String pay;

  for (int i = 0; i < length; i++) {
    pay += (char)payload[i];
  }

  JSONVar msgJson = JSON.parse(pay);

  String dest;
  String action;
  String value;
  dest = msgJson["to"];
  action = msgJson["action"];
  value = msgJson["value"];

  if (dest.equals(String(roomID))) {
    for (int i = 0; i < countSensors; i++) {
      if (sensors[i].getId().equals(sensorId)) {

        if (action.equals("turn")) {

          if (sensors[i].getType().equals("led") ||  sensors[i].getType().equals("plug")) {
            if (value.equals("on")) {
              
              sensors[i].setLastValue(0.0);
              sensors[i].setValue(1.0);
              sensors[i].on();
            }

            if (value.equals("off")) {
              sensors[i].setLastValue(1.0);
              sensors[i].setValue(0.0);
              sensors[i].off();
            }
          }
        }

        if (action.equals("photo") && value.equals("take") ) {
          takePicture(top);
        }

        if (action.equals("photo") && value.equals("on") ) {
          sensors[i].setLastValue(0.0);
          sensors[i].setValue(1.0);
        }

        if (action.equals("photo") && value.equals("off") ) {
          sensors[i].setLastValue(1.0);
          sensors[i].setValue(0.0);
        }

        if (sensors[i].getType().equals("motion")) {
          //client.publish("esp32/humidity", humString);
          Serial.println("motion");
        }
      }
    }
  }

  if ((dest.equals("all") && action.equals("updateSensors")) || action.equals("removeSensor")) {
    getAllSensors();
  }
  
}

void takePicture(String id) {
  // Take Picture with Camera
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    
    return;
  } else {
    String aux = base64::encode(fb->buf, fb->len);
    esp_camera_fb_return(fb);
    String jsonString = createMessagePhotoAux(aux);
    postImage(jsonString);
    client.publish(id.c_str(), createMessage("server", String(roomID), "photo", "sent"));
  }
}

void postImage(String rawdata) {
  HTTPClient http;

  http.begin("http://161.35.8.148/api/insertPhoto/");
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", "Basic " + auth);

  int httpResponseCode = http.POST(rawdata);
  Serial.print("HTTP Response code: ");

  // Free resources
  http.end();
}
