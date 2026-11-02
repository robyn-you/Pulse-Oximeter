#include <Arduino.h>
#include <Wire.h>
#include <MAX30105.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

//Raspberry Pi Pico + MAX30102 + 128x32 OLED
// Required libraries: SparkFun MAX3010x, Adafruit GFX, Adafruit SSD1306.
// SparkFun library supports MAX30102

//Hardware setup (pico)
const int SDA_PIN = 8;           // Pico GPIO8 -> SDA
const int SCL_PIN = 9;           // Pico GPIO9 -> SCL
const int SCREEN_WIDTH = 128;
const int SCREEN_HEIGHT = 32;

MAX30105 sensor;
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Designed for nominal 100 samples/second (must be verified experimentally).
const float HP_ALPHA = 0.9695f;  // ~0.5 Hz high-pass 
const float LP_ALPHA = 0.2222f;  // ~4 Hz low-pass
const float AC_ALPHA = 0.01f;   // smooth absolute AC amplitude
const float DC_ALPHA = 0.005f;  // slow running DC baseline -> tracking the changing DC -> able to calculate the SpO_2; R = (AC_Red / DC_Red) / (AC_IR / DC_IR)

// Sensor and measurement assumptions time line
const uint32_t SAMPLE_PERIOD_MS = 10;   // configured 100 Hz, FIFO averaging 1; im samples are processed correctly, 
// MAX30102 hardware FIFO holds 32 samples (about 320 ms at 100 Hz, if read promptly).
const uint32_t CLOCK_GAP_MS = 250;     // Actual host processing stall diagnostic; not accumulated clock drift.
const int32_t CLOCK_TRACK_TOLERANCE_MS = 30; // Soft-align sample index clock when normal oscillator drift accumulates.
const uint32_t CLOCK_DEBUG_INTERVAL_MS = 10000; // Limit diagnostic messages.
//data setting time 
const uint32_t FINGER_SETTLE_MS = 2000; // provisional, verify experimentally,2sc. adating time of big light changing situation (when the finger detected) ->//신호안정화 기다림 시간...2초.....
const uint32_t SPO2_SETTLE_MS = 5000; // minimum first data set collection wating (when the finger placed properly), wait calculating the SpO_2 until get the set of satbilized data collection (AC/ and DC ratio)
const uint32_t BEAT_TIMEOUT_MS = 2500; //2.5sc. as the min bpm =40, so the next pulse should be detected in max 1.5 sec
// Accepted 334–1500 ms beat intervals correspond to approximately 40–180 BPM.
// This is an algorithmic tracking range, NOT a clinical or pulse-presence criterion.
const uint32_t MIN_BEAT_MS = 334; // Minimum interval: approximately 180 BPM (60000/180)
const uint32_t MAX_BEAT_MS = 1500;       // 40 BPM  min: 40-50 bom 
// --- Finger detection (CHANGE 4: raised, with hysteresis) --------------------------------
// Measured IR DC with this module: finger ~135000-142000, object blocking the sensor
// ~103000-107000, cloth ~70000. The old single threshold of 10000 accepted all of them.
// Hysteresis stops the state chattering while the finger settles.
// WARNING: DC depends on skin tone, finger temperature and pressure. Check every
// team member's finger; if "Place finger" stays on screen, lower both values
// (the original behaviour was ON = OFF = 10000).
const uint32_t FINGER_ON_THRESHOLD  = 120000;  // IR must exceed this to start a measurement
const uint32_t FINGER_OFF_THRESHOLD = 100000;  // IR must fall below this to stop it

// --- Beat detection (CHANGES 2 and 3) -----------------------------------------------------
const int MIN_BEATS_FOR_BPM = 4;   // accepted beat intervals needed before BPM is shown (was 2)
                                   // trade-off: BPM appears about 2 beats later than before
// Sample timing state
unsigned long sampleTimeMs = 0;
bool sampleClockStarted = false;
unsigned long previousHostSampleMs = 0;
unsigned long lastTimingDebugMs = 0;
uint32_t clockCorrections = 0;
uint32_t timingStallEvents = 0;



//measurments/ data aquision 
//measurments/ data aquision 
uint32_t redRaw = 0, irRaw = 0; // Raw optical measure
float redDC = 0, irDC = 0; // DC baseline estimate
float redHP = 0, irHP = 0; // HPF and LPF
float redLP = 0, irLP = 0;
float previousRed = 0, previousIR = 0;
float redAC = 0, irAC = 0;
bool firstSample = true;       // Initialise filters with first sample
bool fingerPresent = false;    // Whether a finger is detected
unsigned long fingerStart = 0; // Time finger detection began, in ms

//BPM peak/interval tracking 
float previousWave = 0, olderWave = 0, waveSize = 0;
unsigned long lastBeat = 0;
int beatTimes[4] = {0, 0, 0, 0};
int beatCount = 0, beatPosition = 0;
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


// FUNCTION 1: Connect to OLED and MAX30102.=>I^2C communication set up, initializing system
void setupDevices() {
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(400000); // 400 kHz I2C bus clock, independent of 100 Hz sensor sampling. 


 //component connectivity check
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { //0x3C
    Serial.println("OLED not found");
    while (true) delay(1000); // why delay 1000? -> pauses execution for 1 second on each repetition.
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
    oled.print("Sensor ERROR");
    oled.display();
    while (true) delay(1000);
  }
  // LED power, averaging, LED mode (red+IR), sample rate, pulse width, ADC range
  sensor.setup(0x24, 1, 2, 100, 411, 4096); // 100 mean 100Hz
  sensor.clearFIFO();
  Serial.println(
    "TYPE,Time_ms,RED_raw,IR_raw,PPG_filtered,"
    "BPM,SpO2_est,RED_DC,IR_DC,RED_AC,IR_AC,Ratio_R,FingerPresent,BPMReady,SpO2Ready"
  );
}


// FUNCTION 2:  Clear measurements after removing a finger.
void resetMeasurements() { // Clear optical filters and estimates on real loss of contact / stall 
  firstSample = true;
  redDC = irDC = redHP = irHP = redLP = irLP = 0;
  previousRed = previousIR = redAC = irAC = 0;
  bpm = spo2Estimate = 0;
  bpmReady = spo2Ready = false;
  lastBeat = 0;
  beatCount = beatPosition = 0;
  olderWave = previousWave = waveSize = 0;
  fingerStart = 0;
  waveIndex = waveCount = waveSkip = 0;
  for (int i = 0; i < WAVE_POINTS; ++i) waveHistory[i] = 0;
}
void resetBeatTracking() { // used when the BPM filed or abnormal beat interval 
  lastBeat = 0;
  beatCount = beatPosition = 0;
  bpm = 0;
  bpmReady = false;
  olderWave = previousWave = 0;
  waveSize = 0;
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
  float wave = -irLP;
  waveSize += 0.01f * (fabsf(wave) - waveSize);
  float threshold = max(35.0f, 0.65f * waveSize);
  bool peak = (
    previousWave > olderWave && 
    previousWave >= wave && 
    previousWave > threshold
  ); // 3 point peak detation => but according to the PPG signal characteristic, it could be differ 

  if (peak) {
    Serial.print("CANDIDATE,"); //candidate peak found 
    Serial.print(now);
    Serial.print(",");
    Serial.println(previousWave, 1); 

    if (lastBeat == 0) {
      lastBeat = now; // anchor first candidate; no BPM calculation yet
    } else {
      unsigned long interval = now - lastBeat;

      // CHANGE 2: tolerate ONE missed beat, but only in the case that used to reset tracking.
      // If a rhythm is established and an interval is longer than MAX_BEAT_MS yet about twice
      // the running mean, a beat was skipped: use half of the interval instead of discarding
      // the whole rhythm (old behaviour: at 62 BPM one missed beat gave ~1.9 s > MAX_BEAT_MS,
      // which reset tracking and blanked the BPM). Fast rhythms (mean < 750 ms) are
      // unaffected, so noise locked on the 334 ms refractory limit gets no extra help.
      if (interval > MAX_BEAT_MS && beatCount >= 2) {
        int missedSum = 0;
        for (int i = 0; i < beatCount; ++i) missedSum += beatTimes[i];
        float runningMean = (float)missedSum / beatCount;
        if ((float)interval >= 1.7f * runningMean && (float)interval <= 2.3f * runningMean) {
          interval = interval / 2;
        }
      }

      if (interval > MAX_BEAT_MS) {
        // Reacquire rhythm after a long gap or an incorrectly anchored peak.
        resetBeatTracking();
        lastBeat = now;
      } else if (interval >= MIN_BEAT_MS) {
        int total = 0;
        for (int i = 0; i < beatCount; ++i) total += beatTimes[i];
        float meanInterval = beatCount ? (float)total / beatCount : (float)interval;
        bool plausible = (beatCount == 0) ||
                         ((float)interval >= 0.70f * meanInterval &&
                          (float)interval <= 1.30f * meanInterval);
        if (plausible) {
          Serial.print("ACCEPTED,"); // 간격검증 => peak 발견 후 간격검증까지 통과하면 BPM 계산에
          Serial.print(now);
          Serial.print(",");
          Serial.println(interval);  

          beatTimes[beatPosition] = (int)interval;
          beatPosition = (beatPosition + 1) % 4;
          if (beatCount < 4) ++beatCount;
          total = 0;
          for (int i = 0; i < beatCount; ++i) total += beatTimes[i];
          if (total > 0) {
            bpm = 60000.0f * beatCount / total;
            bpmReady = beatCount >= MIN_BEATS_FOR_BPM;   // CHANGE 3 (was: beatCount >= 2)
          }
          lastBeat = now;
        }
      }
    }
  }

  olderWave = previousWave;
  previousWave = wave;

  if (lastBeat != 0 && now - lastBeat > BEAT_TIMEOUT_MS) {
    resetBeatTracking();
  }
}

// FUNCTION 5: Rough SpO2 estimate from RED/IR AC and DC.
// FUNCTION 5: Provisional SpO2 estimation
void calculateSpO2(unsigned long elapsedSinceFinger) {
  spo2Ready = false;

  // Allow optical signals to settle
  if (elapsedSinceFinger < SPO2_SETTLE_MS) return;

  // Minimum signal requirements
  if (redDC < 10000 || irDC < 10000 ||
      redAC < 10 || irAC < 10) return;

  // Ratio of ratios
  float ratio = (redAC / redDC) / (irAC / irDC);

  // Uncalibrated SpO2 estimation
  float estimate = 110.0f - 25.0f * ratio;

  // Experimental display acceptance range: 70–104% (NOT a validated accuracy bound).
  if (estimate < 70.0f || estimate > 104.0f) {
    return;  // spo2Ready remains false; OLED shows "--"
  }

  // Display saturation cannot exceed 100%
  spo2Estimate = min(estimate, 100.0f);
  spo2Ready = true;
}

// FUNCTION 6: Save points for the right-hand PPG graph.// waveform history
void saveWaveform() {
  ++waveSkip;
  if (waveSkip < 3) return;
  waveSkip = 0;
  waveHistory[waveIndex] = -irLP;
  waveIndex = (waveIndex + 1) % WAVE_POINTS;
  if (waveCount < WAVE_POINTS) ++waveCount;
}

// FUNCTION 7:Draw the PPG waveform (on the right-hand side).
void drawWaveform() {
  if (waveCount < 2) return;
  int start = (waveIndex + WAVE_POINTS - waveCount) % WAVE_POINTS;
  float low = waveHistory[start], high = waveHistory[start];
  for (int i = 1; i < waveCount; ++i) {
    float v = waveHistory[(start + i) % WAVE_POINTS];
    if (v < low) low = v;
    if (v > high) high = v;
  }
  float span = max(80.0f, high - low);
  for (int i = 1; i < waveCount; ++i) {
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

//BPM display
    oled.setTextSize(1);
    oled.setCursor(0, 4);
    oled.print("BPM");
    oled.setTextSize(2);
    oled.setCursor(32, 0);
    if (bpmReady) oled.print((int)(bpm + 0.5f));
    else oled.print("--");

// SPO2 display
    oled.setTextSize(1);
    oled.setCursor(0, 20);
    oled.print("SpO2");
    oled.setTextSize(2);
    oled.setCursor(31, 16);
    if (spo2Ready) oled.print((int)(spo2Estimate + 0.5f));
    else oled.print("--");

// Right panel: finding pulse first, then display waveform.
    oled.setTextSize(1);
    if (!bpmReady) {
      oled.setCursor(78, 8);
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
//CSV, retaining the original output format and every-4th-sample rate

void printSerial(unsigned long now) {
  ++serialSkip;
  if (serialSkip < 4) return;
  serialSkip = 0;
  Serial.print("DATA,");
  Serial.print(now); Serial.print(',');
  Serial.print(redRaw); Serial.print(',');
  Serial.print(irRaw); Serial.print(',');
  Serial.print(irLP, 1); Serial.print(',');
  
  if (fingerPresent && bpmReady) 
    Serial.print(bpm, 1);
  Serial.print(',');
  
  if (fingerPresent && spo2Ready)
    Serial.print(spo2Estimate, 1);
  Serial.print(',');


  if (fingerPresent) {
    Serial.print(redDC, 2);
    Serial.print(',');
    Serial.print(irDC, 2);
    Serial.print(',');
    Serial.print(redAC, 2);
    Serial.print(',');
    Serial.print(irAC, 2);
    Serial.print(',');

    if (redDC > 0 && irDC > 0 && irAC > 0) {
      float ratio = (redAC / redDC) / (irAC / irDC);
      Serial.print(ratio, 4);
    }
  } else {
    // Five empty diagnostic fields
    Serial.print(",,,,");
  }

  // Diagnostic-only flags: preserve 15-column DATA format expected by Python.
  Serial.print(','); Serial.print(fingerPresent ? 1 : 0);
  Serial.print(','); Serial.print(bpmReady ? 1 : 0);
  Serial.print(','); Serial.print(spo2Ready ? 1 : 0);
  Serial.println();
}


void setup() {
  Serial.begin(115200);
  setupDevices();
}

void loop() {
  sensor.check(); // transfer any new sensor FIFO samples into library buffer

  while (sensor.available()) {
    unsigned long hostNow = millis();

    // Use sample-index time for FIFO burst processing. Correct SMALL accumulated
    // sensor/Pico oscillator mismatch instead of turning clock drift into a full
    // measurement reset every ~45-50 seconds.
    // A real >250 ms gap between *host processing times* remains a reset event.
    if (!sampleClockStarted) {
      sampleTimeMs = hostNow;
      previousHostSampleMs = hostNow;
      sampleClockStarted = true;
    } else {
      uint32_t hostGap = hostNow - previousHostSampleMs;
      previousHostSampleMs = hostNow;
      sampleTimeMs += SAMPLE_PERIOD_MS;

      if (hostGap > CLOCK_GAP_MS) {
        ++timingStallEvents;
        Serial.print("EVENT,CLOCK_STALL,host_ms="); Serial.print(hostNow);
        Serial.print(",gap_ms="); Serial.println(hostGap);
        sampleTimeMs = hostNow;
        resetMeasurements();
        fingerPresent = false;
      } else {
        // Track host time with a bounded +/-1ms correction per sample.
        // This avoids artificial 250ms gaps caused by long-term clock drift.
        int32_t phaseError = (int32_t)(hostNow - sampleTimeMs);
        if (phaseError > CLOCK_TRACK_TOLERANCE_MS) {
          ++sampleTimeMs;
          ++clockCorrections;
        } else if (phaseError < -CLOCK_TRACK_TOLERANCE_MS) {
          --sampleTimeMs;
          ++clockCorrections;
        }
      }
    }
    if (hostNow - lastTimingDebugMs >= CLOCK_DEBUG_INTERVAL_MS) {
      lastTimingDebugMs = hostNow;
      Serial.print("EVENT,TIMING_STATS,host_ms="); Serial.print(hostNow);
      Serial.print(",phase_ms="); Serial.print((int32_t)(hostNow - sampleTimeMs));
      Serial.print(",corrections="); Serial.print(clockCorrections);
      Serial.print(",stalls="); Serial.println(timingStallEvents);
    }

    redRaw = sensor.getFIFORed();
    irRaw = sensor.getFIFOIR();
    sensor.nextSample();



 //finger detection with hysteresis (ON threshold to start, lower OFF threshold to stop)
    bool detected = fingerPresent ? (irRaw > FINGER_OFF_THRESHOLD)
                                  : (irRaw > FINGER_ON_THRESHOLD);
    if (!detected) {
      if (fingerPresent) resetMeasurements();
      fingerPresent = false;
    } else {
      if (!fingerPresent) {
        resetMeasurements();
        fingerStart = sampleTimeMs;
      }
      fingerPresent = true;
      filterPPG();
      unsigned long sinceFinger = sampleTimeMs - fingerStart;
      if (sinceFinger >= FINGER_SETTLE_MS) {
        calculateBPM(sampleTimeMs);
      } else {
        // Warm the filters, but do not train the beat detector on the step transient.
        resetBeatTracking();
      }
      calculateSpO2(sinceFinger);
      saveWaveform();
    }
    printSerial(sampleTimeMs);
  }

  // While acquisition is stalled, hide stale values from the OLED.
  // Do not reset optical filters here; a true stall is logged on the next sample.
  if (fingerPresent && sampleClockStarted &&
      (uint32_t)(millis() - previousHostSampleMs) > CLOCK_GAP_MS) {
    bpmReady = false;
    spo2Ready = false;
  }

  if (millis() - lastDisplay >= 200) { // OLED updates five times per second
    lastDisplay = millis();
    drawOLED();
  }
}


