/*============================= LIBRARY =============================*/
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <DHT_U.h>
#include <ArduinoJson.h>
/*=============================== END ===============================*/

/*============================== Macro ==============================*/
// Set up for wifi, that means you can modify this fileds:
//   + ssid = wifi name
//   + password = wifi password

#define ssid       "D"
#define password   "12345678"

// Set up for broker parameters:
#define mqtt_server "2759ffdd48794684beb3616485e2afb9.s1.eu.hivemq.cloud"

/* Account accessing to broker  */
#define mqtt_username "kit2512"
#define mqtt_password "Ohio@2022"
#define mqtt_port     8883

/* MQTT works with method topic and sub & pub */
#define command1_topic       "mqtt"
#define commandcontrol_topic "control_house"

/* Size of message receive and send */
#define MSG_BUFFER_SIZE 4000

/* Range for gas sensor. if value of sensor is greater than GAS_MIN, the alarm will turn on */
#define GAS_MIN  40
#define TEMP_MIN 35

/* DHT11 ==> Humi and Temperature  */
#define DHTPIN  15      // Digital pin connected to the DHT sensor ( Define DHT sensor - humi and temp sensor works in pin 15)
#define DHTTYPE DHT11  // Type of DHT sensor

/* Gas Sensor Pin */
#define AO_PIN   35
/* BUZZ - Alarm Pin */
#define BUZZ_PIN 32
/* Sensor detect moving */
#define HUMAN_SENSOR 2
/*  Button to switch mode (AUTO or Manual) */
#define MODE_BUTTON 18

/* Pin for all device */
#define Device_01_PIN 12 
#define Device_02_PIN 14 // LED1
#define Device_03_PIN 27 // LED2
#define Device_04_PIN 26 // LED3
#define Device_05_PIN 25 // FAN

// define previous value
int prevHumidity = -1;
int prevTemperature = -1;
int prevGas = -1;
int prevMode = -1;
int humanDetect = -1;
int prevDeviceStates[6] = { -1, -1, -1, -1, -1, -1 };

const char* Home_UID = "b475bdc765bf4e36a2ffaab1f730c891";
const char* Pump_UID = "P3Y4A";
const char* Led1_UID = "RPTzK";
const char* Led2_UID = "YSLYa";
const char* Led3_UID = "5BAWs";
const char* Fan_UID = "YtPUW";

/* PIN for devicde  */
const int devicePin[] = { 12, 14, 27, 26, 25 };
const int numdevice = sizeof(devicePin) / sizeof(devicePin[0]);



DHT dht(DHTPIN, DHTTYPE);

//===========================
static const char* root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";


//==========================================
// DUAL CORE SETUP
TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;

int lastButtonState = LOW;
int mode = 0;
int auto_server = -1;

// Declare Variable support feature wireless
WiFiClientSecure espClient;
PubSubClient client(espClient);
char msg[MSG_BUFFER_SIZE];
unsigned long count = 0;

/*=======================================================================================*/

void callback(char* topic, byte* message, unsigned int length);
void publishMessage(const char* topic, String payload, boolean retained);
void reconnect();

/* Funtion setup for step "Set up" */
void setup_wifi();
void setup_devide();
void setupcore();
/*=======================================================================================*/



void setup() {
  Serial.begin(115200);
  setup_wifi();
  espClient.setCACert(root_ca);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.subscribe(commandcontrol_topic);
  setupcore();
  setup_devide();
  dht.begin();

  pinMode(BUZZ_PIN, OUTPUT_OPEN_DRAIN);
  pinMode(HUMAN_SENSOR, INPUT);
  pinMode(MODE_BUTTON, INPUT);

  digitalWrite(BUZZ_PIN, 1);
}


void loop() {
}

/*=================================== TASK CORE ======================================================*/
//Task1code:
void Task1code(void* pvParameters) {
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    if (!client.connected()) reconnect();
    client.loop();
    vTaskDelay(500);
  }
}
//Task3code:
void Task3code(void* pvParameters) {
  Serial.print("Task3 running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    int pirState = digitalRead(HUMAN_SENSOR);
    if (auto_server == 1 || mode == 1)
      if (pirState == HIGH) {
        digitalWrite(Device_02_PIN, 0);
        digitalWrite(Device_03_PIN, 0);
        digitalWrite(Device_04_PIN, 0);
      } else {
        digitalWrite(Device_02_PIN, 1);
        digitalWrite(Device_03_PIN, 1);
        digitalWrite(Device_04_PIN, 1);
      }
    vTaskDelay(200);
  }
}
//Task4code:
void Task4code(void* pvParameters) {
  Serial.print("Task4 running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    int buttonState = digitalRead(MODE_BUTTON);

    // Detect transition from LOW to HIGH
    if (lastButtonState == LOW && buttonState == HIGH) {
      if (mode == 0) {
        mode = 1;
        digitalWrite(Device_02_PIN, 1);
        digitalWrite(Device_03_PIN, 1);
        digitalWrite(Device_04_PIN, 1);
      } else {
        mode = 0;
        digitalWrite(Device_02_PIN, 1);
        digitalWrite(Device_03_PIN, 1);
        digitalWrite(Device_04_PIN, 1);
      }
      auto_server = 0;
      Serial.print("Detect : ");

      Serial.print("mode : ");
      Serial.println(mode);
    }

    // Save the current button state as the last state for next iteration
    lastButtonState = buttonState;

    vTaskDelay(200);  // Task delay in milliseconds
  }
}

//Task2code:
void Task2code(void* pvParameters) {
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {

    int humidity = readHumidity();
    int temperature = readTemperature();
    int gas = convertToPercentage();

    if (isnan(humidity) || isnan(temperature)) {
      Serial.println(F("Failed to read from DHT sensor!"));
    }

    // Create a JSON document
    StaticJsonDocument<1000> doc;
    if (humidity != prevHumidity)
      doc["h"] = humidity;
    if (temperature != prevTemperature)
      doc["t"] = temperature;
    if (gas != prevGas)
      doc["g"] = gas;
    // home_id
    doc["i"] = Home_UID;

    int pump_value = digitalRead(Device_01_PIN);
    if ((pump_value != prevDeviceStates[0])) {
      JsonObject pump_data = doc.createNestedObject(Pump_UID);
      pump_data["v"] = pump_value;
      pump_data["a"] = mode;
    }


    int led1_value = digitalRead(Device_02_PIN);
    if ((led1_value != prevDeviceStates[1]) || (mode != prevDeviceStates[5])) {
      JsonObject led1_data = doc.createNestedObject(Led1_UID);
      led1_data["v"] = led1_value;
      led1_data["a"] = mode;
    }

    int led2_value = digitalRead(Device_03_PIN);
    if ((led2_value != prevDeviceStates[2])) {
      JsonObject led2_data = doc.createNestedObject(Led2_UID);
      led2_data["v"] = led2_value;
      led2_data["a"] = mode;
    }

    int led3_value = digitalRead(Device_04_PIN);
    if ((led3_value != prevDeviceStates[3])) {
      JsonObject led3_data = doc.createNestedObject(Led3_UID);
      led3_data["v"] = led3_value;
      led3_data["a"] = mode;
    }

    int fan_value = digitalRead(Device_05_PIN);
    if ((fan_value != prevDeviceStates[4])) {
      JsonObject fan_data = doc.createNestedObject(Fan_UID);
      fan_data["v"] = fan_value;
      fan_data["a"] = mode;
    }


    int deviceStates[6];
    deviceStates[0] = pump_value;
    deviceStates[1] = led1_value;
    deviceStates[2] = led2_value;
    deviceStates[3] = led3_value;
    deviceStates[4] = fan_value;
    deviceStates[5] = mode;

    // Check if values have changed
    bool hasChanged = (humidity != prevHumidity) || (temperature != prevTemperature) || (gas != prevGas) || (mode != prevMode);
    for (int i = 0; i < 6; i++) {
      if (deviceStates[i] != prevDeviceStates[i]) {
        hasChanged = true;
        break;
      }
    }

    if (hasChanged) {
      prevHumidity = humidity;
      prevTemperature = temperature;
      prevGas = gas;
      prevMode = mode;
      for (int i = 0; i < 6; i++) {
        prevDeviceStates[i] = deviceStates[i];
      }
      // Serialize JSON to string and print it

      serializeJson(doc, msg);
      Serial.println("SEND DATA");
      Serial.println(msg);
      publishMessage(command1_topic, msg, true);
    }
    vTaskDelay(1000);
  }
}
/*=========================================================================================*/

void setupcore() {
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    Task1code, /* Task function. */
    "Task1",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task1,    /* Task handle to keep track of created task */
    0);        /* pin task to core 0 */
  vTaskDelay(500);

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
    Task2code, /* Task function. */
    "Task2",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task2,    /* Task handle to keep track of created task */
    1);        /* pin task to core 1 */
  vTaskDelay(500);
  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
    Task3code, /* Task function. */
    "Task3",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task3,    /* Task handle to keep track of created task */
    1);        /* pin task to core 1 */
  vTaskDelay(500);
  xTaskCreatePinnedToCore(
    Task4code, /* Task function. */
    "Task4",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task4,    /* Task handle to keep track of created task */
    0);        /* pin task to core 1 */
  vTaskDelay(500);
}


/*=========================================================================================*/
void setup_wifi() {
  delay(10);
  Serial.print("\nConnecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("\nWiFi connected\nIP address: ");
  Serial.println(WiFi.localIP());
}

/*=========================================================================================*/
void setup_devide() {
  for (int i = 0; i < numdevice; i++) {
    pinMode(devicePin[i], OUTPUT);
    digitalWrite(devicePin[i], HIGH);
  }
}
/*=================================== CALLBACK for MQTT =====================================================*/
void callback(char* topic, byte* message, unsigned int length) {
  if (strcmp(topic, commandcontrol_topic) == 0) {
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    String messageTemp;

    for (int i = 0; i < length; i++) {
      // Serial.print((char)message[i]);
      messageTemp += (char)message[i];
    }

    Serial.println(messageTemp);
    DynamicJsonDocument doc(2000);
    DeserializationError error = deserializeJson(doc, messageTemp);
    // Test if parsing succeeds
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
    // Extract values and print them
    // Iterate through the JSON object
    for (JsonPair kv : doc.as<JsonObject>()) {
      const char* key = kv.key().c_str();
      int v = kv.value()["v"];
      int a = kv.value()["a"];
      if (strcmp(key, Led1_UID) == 0) {
        Serial.print("LED 1 ");
        Serial.print("v: ");
        Serial.print(v);
        Serial.print(", a: ");
        Serial.println(a);
        if (a == 1) {
          auto_server = 1;
          digitalWrite(Device_02_PIN, v);
          Serial.println("AUTO DETECTED ON ");
        } else {
          auto_server = 0;
          digitalWrite(Device_02_PIN, v);
          Serial.println("AUTO DETECTED OFF");
        }
        digitalWrite(Device_02_PIN, v);
      }
      if (strcmp(key, Pump_UID) == 0) {
        Serial.print("Pump 1 ");
        Serial.print("v: ");
        Serial.print(v);
        Serial.print(", a: ");
        Serial.println(a);
        if (a == 0) {
          // Serial.println("AUTO DETECTED ");
        }
        digitalWrite(Device_01_PIN, v);
      }
      if (strcmp(key, Led2_UID) == 0) {
        Serial.print("LED 2 ");
        Serial.print("v: ");
        Serial.print(v);
        Serial.print(", a: ");
        Serial.println(a);
        if (a == 0) {
          // Serial.println("AUTO DETECTED ");
        }
        digitalWrite(Device_03_PIN, v);
      }
      if (strcmp(key, Led3_UID) == 0) {
        Serial.print("LED 3");
        Serial.print(" v: ");
        Serial.print(v);
        Serial.print(", a: ");
        Serial.println(a);
        if (a == 0) {
          // Serial.println("AUTO DETECTED ");
        }
        digitalWrite(Device_04_PIN, v);
      }
      if (strcmp(key, Fan_UID) == 0) {
        Serial.print("FAN ");
        Serial.print("v: ");
        Serial.print(v);
        Serial.print(", a: ");
        Serial.println(a);
        if (a == 0) {
          // Serial.println("AUTO DETECTED ");
        }
        digitalWrite(Device_05_PIN, v);
      }
    }
  }
}

/*=========================================================================================*/
void publishMessage(const char* topic, String payload, boolean retained) {
  client.publish(topic, payload.c_str(), true);

  // Serial.println("Message publised [" + String(topic) + "]: " + payload);
}
/*=========================================================================================*/
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32-";  // Create a random client ID
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");
      client.subscribe(command1_topic);        // subscribe the topics here
      client.subscribe(commandcontrol_topic);  // subscribe the topics here

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");  // Wait 5 seconds before retrying
      vTaskDelay(5000);
    }
  }
}




// Function to read humidity
float readHumidity() {
  float h = dht.readHumidity();
  if (isnan(h)) {
    Serial.println(F("Failed to read humidity from DHT sensor!"));
    return NAN;
  }

  return h;
}

// Function to read temperature in Celsius
int readTemperature() {
  float t = dht.readTemperature();
  if (isnan(t)) {
    Serial.println(F("Failed to read temperature from DHT sensor!"));
    return NAN;
  }
  if(mode == 1)
  {
    if(t > TEMP_MIN)
    {
      digitalWrite(Device_05_PIN,0);
    }
    else
    digitalWrite(Device_05_PIN,1);
    
  }
  return (int)t;
}
/* Func tion to read Gas sensor */
float convertToPercentage() {

  int gasValue = analogRead(AO_PIN);
  float voltage = gasValue * (5.0 / 4095.0);  // Convert ADC reading to voltage (assuming 5V range)
  int percentage;

  // Example conversion (adjust as per your sensor datasheet)

  percentage = map(gasValue, 0, 4095, 0, 100);  // Map ADC reading to 0-100%
  if (percentage > GAS_MIN) {
    digitalWrite(BUZZ_PIN, 0);
  } else {
    digitalWrite(BUZZ_PIN, 1);
  }
  return percentage;
}