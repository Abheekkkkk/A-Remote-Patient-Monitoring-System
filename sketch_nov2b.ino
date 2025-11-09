#define BLYNK_TEMPLATE_ID "TMPL3dCCB-N7g"
#define BLYNK_TEMPLATE_NAME "Heart n SpO2 Monitor"
#define BLYNK_AUTH_TOKEN "h6jUsntBljL74M3PtAz38AgaJiIr8Uxq"

#include <HX711.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// WiFi Credentials
char ssid[] = "Airtel_Abheek";
char pass[] = "9538869662";

// Pins
#define DT 21
#define SCK 22
#define BUZZER 15
#define LED 2

HX711 scale;

// Variables
float calibration_factor = -7050; // You will calibrate this
float lastWeight = 0;
unsigned long lastTime = 0;
float dripRate = 0;
float timeRemainingMin = 0;

// Blynk Virtual Pins
#define VPIN_WEIGHT V3
#define VPIN_TIME_LEFT V3
#define VPIN_ALERT V5

void setup() {
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);

  scale.begin(DT, SCK);
  scale.set_scale(calibration_factor);
  scale.tare(); // Zero initial weight

  lastTime = millis();
  Serial.println("IV Drip Monitoring Started");
}

void loop() {
  Blynk.run();

  float currentWeight = scale.get_units(5);
  unsigned long currentTime = millis();

  float timePassedMin = (currentTime - lastTime) / 60000.0;
  float weightDrop = lastWeight - currentWeight;

  if (timePassedMin > 0.1 && weightDrop > 0) {
    dripRate = weightDrop / timePassedMin; // g/min
    timeRemainingMin = currentWeight / dripRate;
    lastWeight = currentWeight;
    lastTime = currentTime;
  }

  // Blynk send data
  Blynk.virtualWrite(VPIN_WEIGHT, currentWeight);
  Blynk.virtualWrite(VPIN_TIME_LEFT, timeRemainingMin);

  // Alert if bottle nearly empty
  if (currentWeight < 0.05) { // less than 10g left
    digitalWrite(BUZZER, HIGH);
    digitalWrite(LED, HIGH);
    Blynk.virtualWrite(VPIN_ALERT, "ALERT: IV Bottle Almost Empty!");
  } else {
    digitalWrite(BUZZER, LOW);
    digitalWrite(LED, LOW);
    Blynk.virtualWrite(VPIN_ALERT, "IV Level Normal");
  }

  Serial.print("Weight: "); Serial.print(currentWeight);
  Serial.print(" g | Time Left: "); Serial.print(timeRemainingMin);
  Serial.println(" min");

  delay(500);
}
