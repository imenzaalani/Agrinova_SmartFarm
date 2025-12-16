#include <Wire.h>
#include <BH1750.h>             // Capteur de luminosité
#include <DHT.h>                 // Capteur température/humidité
#include <DallasTemperature.h>   // Capteur DS18B20
#include <OneWire.h>
#include <ArduinoJson.h>         // JSON pour MQTT
#include <WiFi.h>                // Connexion WiFi
#include <PubSubClient.h>        // MQTT

// ------------------- WIFI -------------------
const char* ssid     = "moham";
const char* password = "moham1234567";

// ------------------- MQTT -------------------
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_topic = "sensor";
const char* mqtt_client_id = "esp32_client";

// ------------------- PINS -------------------
#define DHTPIN        15        // Pin DHT11
#define DHTTYPE       DHT22
#define DS18B20_PIN    4        // Pin pour DS18B20
#define waterSensorPin 34       // Capteur niveau d'eau
#define soilSensorPin  32       // Capteur humidité sol
#define mq135Pin       33       // Capteur gaz
#define motorPin       25       // Relais moteur
#define buzzerPin      26       // Buzzer
#define ledWaterPin    14       // LED niveau d'eau bas
#define ledMotorPin    27       // LED moteur ON
#define ledLumPin      13       // LED faible luminosité

// ------------------- OBJETS -------------------
BH1750 lightSensor;               // Capteur luminosité
DHT dht(DHTPIN, DHTTYPE);         // Capteur DHT11
OneWire oneWire(DS18B20_PIN);     
DallasTemperature dsSensor(&oneWire); // DS18B20
WiFiClient espClient;
PubSubClient client(espClient);

// ------------------- VARIABLES -------------------
unsigned long lastMsg = 0;
const long interval = 30000;      // Intervalle en ms pour envoi des données
int dryValue = 3000;              // Humidité sol: valeur sol sec
int wetValue = 1500;              // Humidité sol: valeur sol humide
#define GAS_THRESHOLD 30         // Seuil gaz pour activer moteur + buzzer

// ------------------- FONCTIONS WIFI/MQTT -------------------
void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(300);
  Serial.println("WiFi connecté");
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect(mqtt_client_id)) {
      Serial.println("MQTT connecté");
    } else delay(1000);
  }
}

// ------------------- FONCTIONS CAPTEURS -------------------
float lux() {
  float value = lightSensor.readLightLevel();
  return isnan(value) ? -1 : value;
}

int waterLevel() {
  int raw = analogRead(waterSensorPin);
  return constrain(map(raw, 0, 4095, 0, 100), 0, 100);
}

int soilMoisture() {
  int raw = analogRead(soilSensorPin);
  return constrain(map(raw, dryValue, wetValue, 0, 100), 0, 100);
}

int readMQ135() {
  int raw = analogRead(mq135Pin);
  return constrain(map(raw, 0, 4095, 0, 100), 0, 100);
}

void readDHT(float &h, float &t) {
  h = dht.readHumidity();
  t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println("Erreur lecture DHT22");
    h = t = -1;
  }
}

float readDS18B20() {
  dsSensor.requestTemperatures();
  float temp = dsSensor.getTempCByIndex(0);
  if (temp == DEVICE_DISCONNECTED_C) {
    Serial.println("Erreur DS18B20");
    return -1;
  }
  return temp;
}

// ------------------- ENVOI DONNEES -------------------
void sendData() {
  float h, t;
  readDHT(h, t);
  float dsTemp = readDS18B20();

  int water = waterLevel();
  int soil  = soilMoisture();
  int gas   = readMQ135();
  float l   = lux();

  // ------------------- ACTIONNEURS -------------------
  // Relais moteur et buzzer si gaz élevé
  if (gas > GAS_THRESHOLD) {
    digitalWrite(motorPin, HIGH);
    digitalWrite(buzzerPin, HIGH);
    digitalWrite(ledMotorPin, HIGH);
    Serial.println("⚠️ Gaz élevé → moteur + buzzer ON");
  } else {
    digitalWrite(motorPin, LOW);
    digitalWrite(buzzerPin, LOW);
    digitalWrite(ledMotorPin, LOW);
  }

  // LED niveau d'eau bas
  if (water < 20) digitalWrite(ledWaterPin, HIGH);
  else digitalWrite(ledWaterPin, LOW);

  // LED faible luminosité
  if (l >= 0 && l < 20) digitalWrite(ledLumPin, HIGH);
  else digitalWrite(ledLumPin, LOW);

  // ------------------- MQTT -------------------
  StaticJsonDocument<256> doc;
  doc["lum"]   = (l < 0) ? "N/A" : String(l,1);
  doc["rhum"]  = (h < 0) ? "N/A" : String(h,1);
  doc["rtmp"]  = (t < 0) ? "N/A" : String(t,1);
  doc["dsTemp"]= (dsTemp < 0) ? "N/A" : String(dsTemp,1);
  doc["water"] = water;
  doc["soil"]  = soil;
  doc["gas"]   = gas;

  String jsonString;
  serializeJson(doc, jsonString);
  Serial.println("Données envoyées: " + jsonString);

  client.publish(mqtt_topic, jsonString.c_str());
}

// ------------------- SETUP -------------------
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  lightSensor.begin();
  dht.begin();
  dsSensor.begin();

  pinMode(waterSensorPin, INPUT);
  pinMode(soilSensorPin, INPUT);
  pinMode(mq135Pin, INPUT);

  pinMode(motorPin, OUTPUT);
  digitalWrite(motorPin, LOW);

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  pinMode(ledWaterPin, OUTPUT);
  digitalWrite(ledWaterPin, LOW);

  pinMode(ledMotorPin, OUTPUT);
  digitalWrite(ledMotorPin, LOW);

  pinMode(ledLumPin, OUTPUT);
  digitalWrite(ledLumPin, LOW);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);

  Serial.println("Setup terminé");
}

// ------------------- LOOP -------------------
void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  if (millis() - lastMsg > interval) {
    lastMsg = millis();
    sendData();
  }
}