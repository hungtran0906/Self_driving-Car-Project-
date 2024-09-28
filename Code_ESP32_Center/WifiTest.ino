//esp32 library version 2.11
#include <WiFi.h>
#include <PubSubClient.h>
#include <CAN.h>
#include <ArduinoJson.h>
// WiFi setup and server initialization
// Replace with your network credentials
const char* ssid = "DESKTOP-3EIJM35 8834";
const char* password = "@068v0J7";
const char* mqtt_server = "192.168.137.1";

WiFiClient espClient;
PubSubClient client(espClient);

TaskHandle_t Task1;
TaskHandle_t Task2;

const int freq = 30000;
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int resolution = 8;

int motor1Pin1 = 18;
int motor1Pin2 = 19;
int enable1Pin = 21;
int motor2Pin1 = 16;
int motor2Pin2 = 17;
int enable2Pin = 4;

uint8_t realtimeSignal = 0;
uint16_t maxVelocity = 0;
char mqttBuffer[10000];
JsonDocument doc;
uint8_t sendMQTTCount = 0;

void setup() {
  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

  // configure PWM functionalities
  ledcSetup(pwmChannel1, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel1);
  ledcAttachPin(enable2Pin, pwmChannel2);

  Serial.begin(115200);

  //create a task that will be executed in the MQTTHandle() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    MQTTHandle, /* Task function. */
    "Task1",    /* name of task. */
    10000,      /* Stack size of task */
    NULL,       /* parameter of the task */
    1,          /* priority of the task */
    &Task1,     /* Task handle to keep track of created task */
    0);         /* pin task to core 0 */
  delay(500);

  //create a task that will be executed in the CANHandle() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
    CANHandle, /* Task function. */
    "Task2",   /* name of task. */
    50000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task2,    /* Task handle to keep track of created task */
    1);        /* pin task to core 1 */
  delay(500);
}

void callback(char* topic, byte* message, unsigned int length) {
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  int signal = messageTemp.toInt();
  if (String(topic) == "esp32_OJT7546/pwm") {
    ledcWrite(pwmChannel1, signal);
    ledcWrite(pwmChannel2, signal);
  } else if (String(topic) == "esp32_OJT7546/engine") {
    if (signal) {
      moveForward();
    } else {
      stopMotors();
    }
    realtimeSignal = signal;
  } else if (String(topic) == "esp32_OJT7546/realtime") {
    realtimeSignal = signal;
    doc.clear();
    sendMQTTCount = 0;
  } else if (String(topic) == "esp32_OJT7546/maxVelocity") {
    client.publish("data/maxVelocity", String(maxVelocity).c_str());
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client2354")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32_OJT7546/+");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {
}

void moveForward() {
  motor1(1);
  motor2(1);
}

void moveBackward() {
  motor1(-1);
  motor2(-1);
}

void turnLeft() {
  motor1(-1);
  motor2(1);
}

void turnRight() {
  motor1(1);
  motor2(-1);
}

void stopMotors() {
  motor1(0);
  motor2(0);
}

void motor1(int x) {
  if (x == 1) {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
  } else if (x == -1) {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
  } else {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
  }
}

void motor2(int x) {
  if (x == 1) {
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  } else if (x == -1) {
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  } else {
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
  }
}

void speedcontrol(int motor1, int motor2) {
  ledcWrite(pwmChannel1, motor1);
  ledcWrite(pwmChannel2, motor2);
}

//MQTTHandle: communication data MQTT
void MQTTHandle(void* pvParameters) {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  client.setBufferSize(10000);
  for (;;) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    vTaskDelay(1);
  }
}

//CANHandle: communication data CAN
void CANHandle(void* pvParameters) {
  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }

  uint8_t buffer[8] = { 0 };
  uint16_t currentSpeed = 0;
  uint8_t count = 0;
  for (;;) {
    // try to parse packet
    int packetSize = CAN.parsePacket();
    if (packetSize) {
      if (!CAN.packetRtr()) {
        // only print packet data for non-RTR packets
        count = 0;
        while (CAN.available()) {
          buffer[count] = (uint8_t)CAN.read();
          count++;
        }
        if ((uint16_t)(buffer[5] << 8 | buffer[6]) > currentSpeed) {
          maxVelocity = buffer[5] << 8 | buffer[6];
        }
        currentSpeed = buffer[5] << 8 | buffer[6];
        if(buffer[0] && currentSpeed){
          stopMotors();
        }
        if (realtimeSignal) {
          // Serial.println();
          doc["crash"][sendMQTTCount] = (int8_t)buffer[0];
          doc["mass"][sendMQTTCount] = (int8_t)buffer[1];
          // doc["x"][sendMQTTCount] = (int8_t)buffer[2];
          doc["y"][sendMQTTCount] = (int8_t)buffer[3];
          // doc["z"][sendMQTTCount] = (int8_t)buffer[4];
          doc["v"][sendMQTTCount] = currentSpeed;
          // doc["emp1"][sendMQTTCount] = buffer[6];
          // doc["emp2"][sendMQTTCount] = buffer[7];
          sendMQTTCount++;
          if (sendMQTTCount == 100) {
            sendMQTTCount = 0;
            memset(mqttBuffer, 0, sizeof(mqttBuffer));
            serializeJson(doc, mqttBuffer);
            doc.clear();
            client.publish("data/realtime", mqttBuffer);
          }
        }
      }
    }
    vTaskDelay(1);
  }
}