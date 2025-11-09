#define BLYNK_TEMPLATE_ID "***"
#define BLYNK_TEMPLATE_NAME "***"
#define BLYNK_AUTH_TOKEN "*****"
/* 
  ESP32 + MAX30102 -> OLED + Blynk
  - Displays Heart Rate (BPM) and SpO2 on OLED
  - Sends values to Blynk virtual pins V1 (HR) and V2 (SpO2)
  NOTE: This is demo/prototype code. Not medical-grade.
*/

#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

/* 
  ESP32 + MAX30102 -> OLED + Blynk
  - Displays Heart Rate (BPM) and SpO2 on OLED
  - Sends values to Blynk virtual pins V1 (HR) and V2 (SpO2)
  NOTE: This is demo/prototype code. Not medical-grade.
*/

#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Choose your MAX3010x library header if using SparkFun or Adafruit
// For SparkFun MAX3010x:
#include "MAX30105.h"
#include "heartRate.h"

// If using Adafruit MAX30105 library, change includes accordingly.

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

MAX30105 particleSensor;

// ----- USER CONFIG -----
const char WIFI_SSID[] = "***";
const char WIFI_PASS[] = "***";
const char BLYNK_AUTH[] = "***";

// Blynk virtual pins
#define VPIN_HR   V1
#define VPIN_SPO2 V2

// I2C pins for ESP32
#define SDA_PIN 21
#define SCL_PIN 22


// sampling & buffers
#define BUFFER_SIZE 100  // number of samples to keep for SPO2 calc
uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFilled = false;

// timing
unsigned long lastSampleMillis = 0;
const unsigned long sampleIntervalMs = 25; // ~40 Hz (25ms) -> good for pulse sensor

// outputs
volatile int heartRateBPM = 0;
volatile int spo2Value = 0;

void setup() {
  Serial.begin(115200);
  delay(100);

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    while (1) delay(10);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // MAX30102 init
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 not found. Check wiring!");
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Sensor not found!");
    display.display();
    while (1) delay(10);
  }

  // Configure sensor: sample average, LED pulse, sample rate, etc.
  // You may tune these parameters for your breakout or desired power/accuracy
  particleSensor.setup(); // default: 411kHz? library chooses defaults
  particleSensor.setPulseAmplitudeRed(0x1F); // 0x0 - 0xFF
  particleSensor.setPulseAmplitudeIR(0x1F);

  // Blank the buffers
  for (int i=0;i<BUFFER_SIZE;i++){ irBuffer[i]=0; redBuffer[i]=0; }

  // WiFi + Blynk
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  unsigned long startWi = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startWi < 15000) {
    Serial.print('.');
    delay(500);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected.");
  } else {
    Serial.println("\nWiFi connect failed - continuing without network.");
  }

  Blynk.begin(BLYNK_AUTH, WIFI_SSID, WIFI_PASS);

  displayStartup();
}

void displayStartup() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("MAX30102 HR+SpO2");
  display.println("Initializing...");
  display.display();
}

// Main loop
void loop() {
  Blynk.run();

  // Sample at ~40Hz (25ms)
  if (millis() - lastSampleMillis >= sampleIntervalMs) {
    lastSampleMillis = millis();
    sampleAndStore();
  }

  // Process when buffer enough samples
  if (bufferFilled) {
    computeHeartRate();   // updates heartRateBPM (uses checkForBeat)
    computeSpO2();        // updates spo2Value
    publishResults();
    // after publishing, zero bufferFilled so calculation doesn't continuously re-run
    bufferFilled = false;
  }

  // Update OLED periodically even if no new computed values
  static unsigned long lastOLED = 0;
  if (millis() - lastOLED > 800) {
    lastOLED = millis();
    updateOLED(heartRateBPM, spo2Value);
  }
}

// Take one sample from sensor and put in circular buffer
void sampleAndStore() {
  long ir = particleSensor.getIR();
  long red = particleSensor.getRed();

  // store safely into buffer (clamp)
  if (ir < 0) ir = 0;
  if (red < 0) red = 0;

  irBuffer[bufferIndex] = (uint32_t)ir;
  redBuffer[bufferIndex] = (uint32_t)red;

  bufferIndex++;
  if (bufferIndex >= BUFFER_SIZE) {
    bufferIndex = 0;
    bufferFilled = true;
  }
}

// Compute heart rate using the simple checkForBeat() library helper
void computeHeartRate() {
  // We scan buffer for beats using checkForBeat on IR samples
  // Each time checkForBeat returns true we compute time delta between beats.
  static unsigned long lastBeatTime = 0;
  int beatsFound = 0;
  unsigned long firstBeatTime = 0;
  unsigned long lastFoundBeatTime = 0;

  // We'll iterate through the buffer in time order
  int start = bufferIndex; // newest index
  for (int i = 0; i < BUFFER_SIZE; i++) {
    int idx = (start + i) % BUFFER_SIZE;
    long sample = (long)irBuffer[idx];

    if (checkForBeat(sample)) {
      unsigned long t = millis() - (BUFFER_SIZE - 1 - i) * sampleIntervalMs;
      if (beatsFound == 0) firstBeatTime = t;
      lastFoundBeatTime = t;
      beatsFound++;
      //avoid double counting: tiny delay
      delay(1);
    }
  }

  if (beatsFound >= 2) {
    unsigned long interval = (lastFoundBeatTime - firstBeatTime) / (beatsFound - 1);
    if (interval > 0) {
      heartRateBPM = (int)(60000.0 / interval + 0.5);
      Serial.print("Computed BPM: "); Serial.println(heartRateBPM);
      return;
    }
  }
  // If not enough beats found, keep previous value or set 0
  // heartRateBPM = 0; // optional: keep last known value
}

// Compute SpO2 using simple AC/DC ratio method
void computeSpO2() {
  // Find DC (mean) and AC (peak-to-peak) for red and IR in buffer window
  uint32_t minIR = UINT32_MAX, maxIR = 0, minRed = UINT32_MAX, maxRed = 0;
  double sumIR = 0, sumRed = 0;
  for (int i=0;i<BUFFER_SIZE;i++){
    uint32_t v = irBuffer[i];
    uint32_t r = redBuffer[i];
    if (v < minIR) minIR = v;
    if (v > maxIR) maxIR = v;
    if (r < minRed) minRed = r;
    if (r > maxRed) maxRed = r;
    sumIR += v;
    sumRed += r;
  }
  double meanIR = sumIR / BUFFER_SIZE;
  double meanRed = sumRed / BUFFER_SIZE;

  double acIR = (double)maxIR - (double)minIR;
  double acRed = (double)maxRed - (double)minRed;
  double dcIR = meanIR;
  double dcRed = meanRed;

  // Avoid division by zero
  if (dcIR <= 0 || dcRed <= 0 || acIR <= 0 || acRed <= 0) {
    // invalid data; keep previous spo2
    Serial.println("Invalid data for SpO2 calc");
    return;
  }

  // Ratio R = (AC_red/DC_red) / (AC_ir/DC_ir)
  double ratio = (acRed / dcRed) / (acIR / dcIR);

  // Empirical linear relation (approximate)
  // Many implementations use: SpO2 = -45 * R + 110  (empirical)
  double spo2 = -45.0 * ratio + 110.0;

  // Boundaries and sanity check
  if (spo2 < 50) spo2 = 50;
  if (spo2 > 100) spo2 = 100;

  spo2Value = (int)(spo2 + 0.5);
  Serial.print("SpO2 ratio: "); Serial.print(ratio, 3);
  Serial.print("  SpO2: "); Serial.println(spo2Value);
}

// Send values to Blynk and serial
void publishResults() {
  Blynk.virtualWrite(VPIN_HR, heartRateBPM);
  Blynk.virtualWrite(VPIN_SPO2, spo2Value);
  Serial.print("Publish -> HR: "); Serial.print(heartRateBPM);
  Serial.print(" bpm  SpO2: "); Serial.println(spo2Value);
}

// Update the OLED display
void updateOLED(int bpm, int spo2) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("Heart Rate & SpO2");
  display.drawFastHLine(0, 12, 128, SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(0,18);
  if (bpm > 0) display.print(bpm); else display.print("--");
  display.setTextSize(1);
  display.setCursor(70, 28);
  display.println("BPM");

  display.setTextSize(2);
  display.setCursor(0,42);
  if (spo2 > 0) display.print(spo2); else display.print("--");
  display.setTextSize(1);
  display.setCursor(70, 50);
  display.println("SpO2%");

  display.display();
}
