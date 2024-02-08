/*#define BLYNK_TEMPLATE_ID "TMPL4K70OT8Yv"
#define BLYNK_TEMPLATE_NAME "Moisture Alert"
#define BLYNK_AUTH_TOKEN "UjvYeuFs1cpAqgBCIaLQ3uQBNodP9s-0"
#define BLYNK_PRINT Serial
#define DHTPIN 5
#define DHTTYPE DHT11

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <SoftwareSerial.h>
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>

DHT dht(DHTPIN, DHTTYPE);
char ssid[] = "NTIG Guest";
char pass[] = "TeknikPassion";

SoftwareSerial Serial1(6, 7);

BlynkTimer timer;

ESP8266 wifi(&Serial1);
int light_sensor = A0;

BLYNK_CONNECTED()
{
  Serial.println("Blynk Connected");
}

void setup()
{
  Serial1.begin(9600);
  dht.begin();
  delay(10);
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, wifi, ssid, pass);
}

void loop()
{
  delay(2000);

  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print("%  Temperature: ");
    Serial.print(temperature);
    Serial.println("°C");

    Blynk.virtualWrite(V2, humidity);
    Blynk.virtualWrite(V3, temperature);

     int analogValue = analogRead(A0);

    Serial.print("Analog reading: ");
    Serial.print(analogValue);   // the raw analog reading

 
    delay(500);

    Blynk.virtualWrite(V4, light); 

    Blynk.run();
    timer.run(); 
  }
}*/
/*
#define BLYNK_TEMPLATE_ID "TMPL4K70OT8Yv"
#define BLYNK_TEMPLATE_NAME "Moisture Alert"
#define BLYNK_AUTH_TOKEN "UjvYeuFs1cpAqgBCIaLQ3uQBNodP9s-0"
#define BLYNK_PRINT Serial
#define DHTPIN 5
#define DHTTYPE DHT11

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <SoftwareSerial.h>
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>

DHT dht(DHTPIN, DHTTYPE);
char ssid[] = "NTIG Guest";
char pass[] = "TeknikPassion";

SoftwareSerial Serial1(6, 7);

BlynkTimer timer;

ESP8266 wifi(&Serial1);
int light_sensor = A0;
int sensorPin = 2;
volatile long pulse;
unsigned long lastTime;
float volume;

BLYNK_CONNECTED() {
  Serial.println("Blynk Connected");
}

void setup() {
  Serial1.begin(9600);
  dht.begin();
  delay(10);
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, wifi, ssid, pass);
  pinMode(sensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(sensorPin), increase, RISING);
}

void loop() {
  static unsigned long lastMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastMillis >= 2000) {
    lastMillis = currentMillis;

    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Failed to read from DHT sensor!");
    } else {
      Serial.print("Humidity: ");
      Serial.print(humidity);
      Serial.print("%  Temperature: ");
      Serial.print(temperature);
      Serial.println("°C");

      Blynk.virtualWrite(V2, humidity);
      Blynk.virtualWrite(V3, temperature);

      int analogValue = analogRead(A0);
      float mappedValue = map(analogValue, 0, 1023, 0, 100);

      Serial.print("Analog reading: ");
      Serial.print(mappedValue);
      Blynk.virtualWrite(V4, mappedValue);

 
      volume = 2.663 * pulse / 1000 * 30;
      if (millis() - lastTime > 2000) {
        pulse = 0;
        lastTime = millis();
        Serial.print(volume);
        Serial.println(" L/m");
        Blynk.virtualWrite(V5, volume); 

        delay(100);

        Blynk.run();
        timer.run();
      }
    }
  }
}

void increase() {
  pulse++;
}

*/
#define BLYNK_TEMPLATE_ID "TMPL4K70OT8Yv"
#define BLYNK_TEMPLATE_NAME "Moisture Alert"
#define BLYNK_AUTH_TOKEN "UjvYeuFs1cpAqgBCIaLQ3uQBNodP9s-0"
#define BLYNK_PRINT Serial
#define DHTPIN 5
#define DHTTYPE DHT11

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <SoftwareSerial.h>
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>

DHT dht(DHTPIN, DHTTYPE);
char ssid[] = "NTIG Guest";
char pass[] = "TeknikPassion";

SoftwareSerial Serial1(6, 7);

BlynkTimer timer;

ESP8266 wifi(&Serial1);
int light_sensor = A0;
int sensorPin = 2; // This is now your flow sensor pin
volatile int pulseCount;
float flowRate;

BLYNK_CONNECTED() {
  Serial.println("Blynk Connected");
}

void setup() {
  Serial1.begin(9600);
  dht.begin();
  delay(10);
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, wifi, ssid, pass);
  pinMode(sensorPin, INPUT);
  digitalWrite(sensorPin, HIGH);
  attachInterrupt(digitalPinToInterrupt(sensorPin), pulseCounter, FALLING);
}

void loop() {
  static unsigned long lastMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastMillis >= 2000) {
    lastMillis = currentMillis;

    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Failed to read from DHT sensor!");
    } else {
      Serial.print("Humidity: ");
      Serial.print(humidity);
      Serial.print("%  Temperature: ");
      Serial.print(temperature);
      Serial.println("°C");

      Blynk.virtualWrite(V2, humidity);
      Blynk.virtualWrite(V3, temperature);

      int analogValue = analogRead(light_sensor);
      float mappedValue = map(analogValue, 0, 1023, 0, 100);

      Serial.print("Analog reading: ");
      Serial.print(mappedValue);
      Blynk.virtualWrite(V4, mappedValue);

      // Calculate flow rate in liters per minute
      flowRate = pulseCount / 7.5; // 7.5 is the sensor's K-factor

      // Display the flow rate
      Serial.print("Flow Rate: ");
      Serial.print(flowRate);
      Serial.println(" LPM");

      // Reset pulse count for the next measurement
      pulseCount = 0;

      Blynk.virtualWrite(V5, flowRate);

      delay(100);

      Blynk.run();
      timer.run();
    }
  }
}

// Interrupt Service Routine for pulse counting
void pulseCounter() {
  pulseCount++;
}
 
