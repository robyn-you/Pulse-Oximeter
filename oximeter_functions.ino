#include <MAX30105.h>
#include <heartRate.h>
#include <spo2_algorithm.h>

#include <Adafruit_GFX.h>
#include <Adafruit_GrayOLED.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>

#include <Adafruit_SSD1306.h>
#include <splash.h>

//Raspberry Pi Pico + MAX30102 + 128x32 OLED
// Required libraries: SparkFun MAX3010x, Adafruit GFX, Adafruit SSD1306.
// SparkFun library supports MAX30102

#include <Wire.h>
#include <MAX30105.h>             
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

const int SDA_PIN = 8;           // Pico GPIO8 -> SDA
const int SCL_PIN = 9;           // Pico GPIO9 -> SCL
const int SCREEN_WIDTH = 128;
const int SCREEN_HEIGHT = 32;

MAX30105 sensor;
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Designed for nominal 100 samples/second (must be verified experimentally).
const float HP_ALPHA = 0.9695f;  // ~0.5 Hz high-pass
const float LP_ALPHA = 0.2222f;  // ~4 Hz low-pass
const float DC_ALPHA = 0.005f;  // slow running DC baseline
const float AC_ALPHA = 0.01f;   // smooth absolute AC amplitude

uint32_t redRaw = 0, irRaw = 0;
float redDC = 0, irDC = 0;
float redHP = 0, irHP = 0;
float redLP = 0, irLP = 0;
float lastRedRaw = 0, lastIrRaw = 0;
float redACsize = 0, irACsize = 0;
bool firstSample = true;
bool fingerPresent = false;
unsigned long fingerSince = 0;

float previousWave = 0, olderWave = 0, waveSize = 0;
unsigned long lastBeat = 0;
int beatIntervals[4] = {0, 0, 0, 0};
int intervalCount = 0, intervalPosition = 0;
float bpm = 0, spo2Estimate = 0;
bool bpmReady = false, spo2Ready = false;

const int WAVE_POINTS = 76;
float waveHistory[WAVE_POINTS] = {0};
int waveIndex = 0;
unsigned long lastDisplay = 0;
int serialCounter = 0;

// FUNCTION 1: Connect to OLED and MAX30102.
void setupDevices() {
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(100000); // I2C bus clock: 100 kHz, not the sensor sample rate.

  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not detected - check wiring");
    while (true) delay(1000);
  }
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println("Starting sensor...");
  oled.display();

  if (!sensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 not detected - check wiring");
    oled.println("MAX30102 ERROR");
    oled.display();
    while (true) delay(1000);
  }
  sensor.setup(0x24, 1, 2, 100, 411, 4096); // LED power, avg, mode, Hz, pulse, range
  Serial.println("DATA,ms,red_raw,ir_raw,ir_filtered,bpm,spo2_est");
}

// FUNCTION 2: Clear old readings if finger is removed.
void resetMeasurements() {
  firstSample = true;
  redACsize = irACsize = waveSize = 0;
  bpmReady = spo2Ready = false;
  bpm = spo2Estimate = 0;
  lastBeat = 0;
  intervalCount = intervalPosition = 0;
  olderWave = previousWave = 0;
  fingerSince = 0;
  waveIndex = 0;
  for (int i = 0; i < WAVE_POINTS; i++) waveHistory[i] = 0;
}

// FUNCTION 3: Extract the slowly varying DC level and filter RED/IR signals.
void filterSignals() {
  if (firstSample) {
    lastRedRaw = redDC = (float)redRaw;
    lastIrRaw = irDC = (float)irRaw;
    redHP = irHP = redLP = irLP = 0;
    firstSample = false;
  }
  redDC += DC_ALPHA * ((float)redRaw - redDC);
  irDC  += DC_ALPHA * ((float)irRaw - irDC);

  // One high-pass stage to suppress drift.
  redHP = HP_ALPHA * (redHP + (float)redRaw - lastRedRaw);
  irHP  = HP_ALPHA * (irHP + (float)irRaw - lastIrRaw);
  lastRedRaw = (float)redRaw;
  lastIrRaw = (float)irRaw;

  // One low-pass stage to smooth high-frequency noise.
  redLP += LP_ALPHA * (redHP - redLP);
  irLP  += LP_ALPHA * (irHP - irLP);

  // Simple AC amplitude measurements for the RED/IR ratio.
  redACsize += AC_ALPHA * (fabsf(redLP) - redACsize);
  irACsize  += AC_ALPHA * (fabsf(irLP) - irACsize);
}

// FUNCTION 4: Detect pulse peaks and calculate an average BPM.
void calculateBPM(unsigned long now) {
  float wave = -irLP; // flip IR waveform so peaks point upward
  waveSize += 0.01f * (fabsf(wave) - waveSize);
  float threshold = max(35.0f, 0.65f * waveSize);
  bool isPeak = previousWave > olderWave && previousWave >= wave;

  if (isPeak && previousWave > threshold &&
      (lastBeat == 0 || now - lastBeat >= 350)) {
    if (lastBeat != 0) {
      int interval = (int)(now - lastBeat);
      if (interval >= 375 && interval <= 1500) { // 40-160 BPM
        int sum = 0;
        for (int i = 0; i < intervalCount; i++) sum += beatIntervals[i];
        float average = intervalCount ? (float)sum / intervalCount : interval;
        if (intervalCount == 0 ||
            (interval > 0.7f * average && interval < 1.3f * average)) {
          beatIntervals[intervalPosition] = interval;
          intervalPosition = (intervalPosition + 1) % 4;
          if (intervalCount < 4) intervalCount++;
          sum = 0;
          for (int i = 0; i < intervalCount; i++) sum += beatIntervals[i];
          bpm = 60000.0f * intervalCount / sum;
          bpmReady = intervalCount >= 2;
        }
      }
    }
    lastBeat = now;
  }
  olderWave = previousWave;
  previousWave = wave;

  if (lastBeat != 0 && now - lastBeat > 3000) {
    bpmReady = false;
    lastBeat = 0;
    intervalCount = intervalPosition = 0;
  }
}

// FUNCTION 5: Rough, uncalibrated SpO2 estimate using RED/IR AC/DC ratio.
void calculateSpO2(unsigned long now) {
  spo2Ready = false;
  if (now - fingerSince < 5000) return; // wait ~5 seconds for signal settling
  if (redDC <= 10000 || irDC <= 10000 || redACsize <= 10 || irACsize <= 10) return;

  float ratio = (redACsize / redDC) / (irACsize / irDC);
  float estimate = 110.0f - 25.0f * ratio;
  if (estimate >= 70 && estimate <= 100) {
    spo2Estimate = estimate;
    spo2Ready = true;
  }
}

// FUNCTION 6: Remember the latest IR waveform samples for the OLED graph.
void saveWaveform() {
  waveHistory[waveIndex] = -irLP;
  waveIndex = (waveIndex + 1) % WAVE_POINTS;
}

// FUNCTION 7: Draw numbers and scrolling waveform on the OLED.
void drawOLED() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  // ===== BPM =====
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("BPM");

  oled.setTextSize(2);
  oled.setCursor(0, 8);

  if (fingerPresent && bpmReady) {
    oled.print((int)(bpm + 0.5f));
  } else {
    oled.print("--");
  }

  // ===== SpO2 =====
  oled.setTextSize(1);
  oled.setCursor(0, 17);
  oled.print("O2%");

  oled.setTextSize(2);
  oled.setCursor(0, 25); // See note below

  if (fingerPresent && spo2Ready) {
    oled.print((int)(spo2Estimate + 0.5f));
  } else {
    oled.print("--");
  }

  // ===== Waveform =====
  oled.setTextSize(1);
  oled.drawFastVLine(49, 0, 32, SSD1306_WHITE);

  if (fingerPresent) {
    float low = waveHistory[0];
    float high = waveHistory[0];

    for (int i = 1; i < WAVE_POINTS; i++) {
      if (waveHistory[i] < low) low = waveHistory[i];
      if (waveHistory[i] > high) high = waveHistory[i];
    }

    float span = max(1.0f, high - low);

    for (int x = 1; x < WAVE_POINTS; x++) {
      float a = waveHistory[(waveIndex + x - 1) % WAVE_POINTS];
      float b = waveHistory[(waveIndex + x) % WAVE_POINTS];

      int ya = 30 - (int)((a - low) * 29.0f / span);
      int yb = 30 - (int)((b - low) * 29.0f / span);

      oled.drawLine(51 + x - 1, ya,
                    51 + x, yb, SSD1306_WHITE);
    }
  }

  oled.display();
}

// FUNCTION 8: Send values to a computer over USB serial.
void printSerial(unsigned long now) {
  serialCounter++;
  if (serialCounter < 4) return; // print only every 4th sample
  serialCounter = 0;
  Serial.print("DATA,");
  Serial.print(now); Serial.print(',');
  Serial.print(redRaw); Serial.print(',');
  Serial.print(irRaw); Serial.print(',');
  Serial.print(irLP, 1); Serial.print(',');
  if (fingerPresent && bpmReady) Serial.print(bpm, 1);
  Serial.print(',');
  if (fingerPresent && spo2Ready) Serial.print(spo2Estimate, 1);
  Serial.println();
}

// Arduino runs setup() once.
void setup() {
  Serial.begin(115200);
  setupDevices();
}

// Arduino repeats loop() continuously.
void loop() {
  sensor.check();                  // collect any waiting sensor samples
  while (sensor.available()) {    // process all samples currently available
    unsigned long now = millis();
    redRaw = sensor.getFIFORed();
    irRaw = sensor.getFIFOIR();
    sensor.nextSample();

    fingerPresent = (irRaw > 10000); // simple finger-presence estimate
    if (!fingerPresent) {
      resetMeasurements();
    } else {
      if (fingerSince == 0) fingerSince = now;
      filterSignals();
      calculateBPM(now);
      calculateSpO2(now);
      saveWaveform();
    }
    printSerial(now);
  }

  if (millis() - lastDisplay >= 200) { // update OLED ~5 times/sec
    lastDisplay = millis();
    drawOLED();
  }
}
