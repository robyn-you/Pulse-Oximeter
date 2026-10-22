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
//Hardware
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

//measurments/ data aquision 
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

//OLED waveform 56 samples fit in the right-hand panes 
const int WAVE_POINTS = 56; //56 sample can be viewed
float waveHistory[WAVE_POINTS] = {0};
int waveIndex = 0;
int waveCount = 0;
int waveSkip = 0;
unsigned long lastDisplay = 0;
int serialSkip = 0;


// FUNCTION 1: Connect to OLED and MAX30102.
void setupDevices() {
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(100000); // I2C bus clock: 100 kHz, not the sensor sample rate.

 
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found");
    while (true) delay(1000);
  }
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 10);
  oled.println("Starting sensor...");
  oled.display();

  if (!sensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 not found");
    oled.clearDisplay();
    oled.setCursor(0, 10);
    oled.print("MAX30102 ERROR");
    oled.display();
    while (true) delay(1000);
  }
  // LED power, averaging, LED mode (red+IR), sample rate, pulse width, ADC range
  sensor.setup(0x24, 1, 2, 100, 411, 4096);
  Serial.println("DATA,ms,red_raw,ir_raw,ir_filtered,bpm,spo2_est");
}


// FUNCTION 2:  Clear measurements after removing a finger.
void resetMeasurements() {
  firstSample = true;
  redAC = irAC = 0;
  bpm = spo2Estimate = 0;
  bpmReady = spo2Ready = false;
  lastBeat = 0;
  beatCount = beatPosition = 0;
  olderWave = prevWave = waveLevel = 0;
  fingerStart = 0;
  waveIndex = waveCount = waveSkip = 0;
  for (int i = 0; i < WAVE_POINTS; i++) waveHistory[i] = 0;
}

// FUNCTION 3: Estimate the baseline and filter RED + IR signals.
void filterPPG() {
  if (firstSample) {
    previousRed = redDC = (float)redRaw;
    previousIR = irDC = (float)irRaw;
    redHP = irHP = redLP = irLP = 0;
    firstSample = false;
  }

  // Slowly changing light level (DC)
  redDC += DC_ALPHA * ((float)redRaw - redDC);
  irDC += DC_ALPHA * ((float)irRaw - irDC);

  // High-pass: remove slow baseline drift
  redHP = HP_ALPHA * (redHP + (float)redRaw - previousRed);
  irHP = HP_ALPHA * (irHP + (float)irRaw - previousIR);
  previousRed = (float)redRaw;
  previousIR = (float)irRaw;

  // Low-pass: remove faster noise
  redLP += LP_ALPHA * (redHP - redLP);
  irLP += LP_ALPHA * (irHP - irLP);

  // Simple size of the changing RED and IR signals
  redAC += AC_ALPHA * (fabsf(redLP) - redAC);
  irAC += AC_ALPHA * (fabsf(irLP) - irAC);
}



// FUNCTION 4: Detect IR peaks and average valid beat intervals.
void calculateBPM(unsigned long now) {
  float wave = -irLP; // make the pulse peaks point upwards
  waveLevel += 0.01f * (fabsf(wave) - waveLevel);
  float threshold = max(35.0f, 0.65f * waveLevel);
  bool peak = (prevWave > olderWave && prevWave >= wave && prevWave > threshold);

  if (peak && (lastBeat == 0 || now - lastBeat >= 350)) {
    if (lastBeat != 0) {
      int interval = (int)(now - lastBeat);
      if (interval >= 375 && interval <= 1500) { // about 40-160 BPM
        int sum = 0;
        for (int i = 0; i < beatCount; i++) sum += beatTimes[i];
        float average = beatCount ? (float)sum / beatCount : interval;

        // Ignore beats very different from recent accepted intervals.
        if (beatCount == 0 || (interval > 0.7f * average && interval < 1.3f * average)) {
          beatTimes[beatPosition] = interval;
          beatPosition = (beatPosition + 1) % 4;
          if (beatCount < 4) beatCount++;
          sum = 0;
          for (int i = 0; i < beatCount; i++) sum += beatTimes[i];
          bpm = 60000.0f * beatCount / sum;
          bpmReady = (beatCount >= 2);
        }
      }
    }
    lastBeat = now;
  }

  olderWave = prevWave;
  prevWave = wave;
  if (lastBeat != 0 && now - lastBeat > 3000) {
    bpmReady = false;
    beatCount = beatPosition = 0;
    lastBeat = 0;
  }
}

// FUNCTION 5: Rough SpO2 estimate from RED/IR AC and DC.
void calculateSpO2(unsigned long now) {
  spo2Ready = false;
  if (now - fingerStart < 5000) return; // settle for five seconds
  if (redDC < 10000 || irDC < 10000 || redAC < 10 || irAC < 10) return;

  float ratio = (redAC / redDC) / (irAC / irDC);
  float estimate = 110.0f - 25.0f * ratio;
  if (estimate >= 70 && estimate <= 100) {
    spo2Estimate = estimate;
    spo2Ready = true;
  }
}

// FUNCTION 6: Save points for the right-hand PPG graph.
void saveWaveform() {
  waveSkip++;
  if (waveSkip < 3) return; // save every third sample for a longer trace
  waveSkip = 0;
  waveHistory[waveIndex] = -irLP;
  waveIndex = (waveIndex + 1) % WAVE_POINTS;
  if (waveCount < WAVE_POINTS) waveCount++;
}


// FUNCTION 7:Draw the PPG waveform on the right-hand side.
void drawWaveform() {
  if (waveCount < 2) return;
  int start = (waveIndex + WAVE_POINTS - waveCount) % WAVE_POINTS;
  float low = waveHistory[start], high = waveHistory[start];
  for (int i = 1; i < waveCount; i++) {
    float v = waveHistory[(start + i) % WAVE_POINTS];
    if (v < low) low = v;
    if (v > high) high = v;
  }
  float span = max(80.0f, high - low); // don't greatly magnify tiny noise
  for (int i = 1; i < waveCount; i++) {
    float a = waveHistory[(start + i - 1) % WAVE_POINTS];
    float b = waveHistory[(start + i) % WAVE_POINTS];
    int x1 = 127 - (waveCount - i) * 55 / (WAVE_POINTS - 1);
    int x2 = 127 - (waveCount - 1 - i) * 55 / (WAVE_POINTS - 1);
    int y1 = constrain(30 - (int)((a - low) * 29.0f / span), 1, 30);
    int y2 = constrain(30 - (int)((b - low) * 29.0f / span), 1, 30);
    oled.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
  }
}


//FUNCTION 8: Display a simple status, or large BPM + SpO2 + waveform.
void drawOLED() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);

  if (!fingerPresent) {
    oled.setCursor(24, 12);
    oled.print("Place finger");
  }
  else {
    // Left panel: separate rows, with large numbers.
    oled.drawFastVLine(69, 0, 32, SSD1306_WHITE);
    oled.drawFastHLine(0, 15, 69, SSD1306_WHITE);

    oled.setTextSize(1);
    oled.setCursor(0, 4);
    oled.print("BPM:");
    oled.setCursor(0, 20);
    oled.print("SpO2:");

    oled.setTextSize(2);
    oled.setCursor(32, 0);
    if (bpmReady) oled.print((int)(bpm + 0.5f));
    else oled.print("--");
    oled.setCursor(32, 16);
    if (spo2Ready) oled.print((int)(spo2Estimate + 0.5f));
    else oled.print("--");

    // Right panel: finding pulse first, then display waveform.
    oled.setTextSize(1);
    if (!bpmReady) {
      oled.setCursor(76, 8);
      oled.print("Finding");
      oled.setCursor(78, 18);
      oled.print("pulse...");
    } else {
      drawWaveform();
    }
  }
  oled.display();
}

// FUNCTION 9: Output basic measurements to Arduino Serial Monitor.
void printSerial(unsigned long now) {
  serialSkip++;
  if (serialSkip < 4) return;
  serialSkip = 0;
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

void setup() {
  Serial.begin(115200);
  setupDevices();
}

void loop() {
  sensor.check();
  while (sensor.available()) {
    unsigned long now = millis();
    redRaw = sensor.getFIFORed();
    irRaw = sensor.getFIFOIR();
    sensor.nextSample();

    bool detected = (irRaw > 10000); // simple finger detection threshold
    if (!detected) {
      if (fingerPresent) resetMeasurements();
      fingerPresent = false;
    } else {
      if (!fingerPresent) { resetMeasurements(); fingerStart = now; }
      fingerPresent = true;
      filterPPG();
      calculateBPM(now);
      calculateSpO2(now);
      saveWaveform();
    }
    printSerial(now);
  }

  if (millis() - lastDisplay >= 200) { // OLED updates five times per second
    lastDisplay = millis();
    drawOLED();
  }
}

