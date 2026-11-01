/*
  ESP32_I2S_Tester.ino
  Target Board: Keyes 2.8" ESP32-32E / ESP32 Dev Module (ESP32-W0WD-V3)
  Environment: Arduino IDE 2.x + Arduino-ESP32 Core 3.x
  Function: I2S Digital Receiver & Analyzer (384 kHz, 32-bit Stereo)
*/

#include <ESP_I2S.h>
#include <math.h>

// Pin mapping avoiding LCD, Touch, RGB LED, and SD card
const int I2S_BCLK = 18;  // Connected to T113-S3 PG13
const int I2S_LRCK = 19;  // Connected to T113-S3 PG12 (WS)
const int I2S_DATA = 27;  // Connected to T113-S3 PG15 (DOUT)

I2SClass i2s;

// Buffer and statistics variables
const size_t BUFFER_FRAMES = 1024; // Stereo frames (Left + Right per frame)
int32_t audioBuffer[BUFFER_FRAMES * 2];

unsigned long lastReportTime = 0;
uint32_t totalBytesReceived = 0;
uint32_t totalFramesReceived = 0;

// Latest calculated metrics for reporting
volatile int32_t peakLeft = 0;
volatile int32_t peakRight = 0;
volatile float rmsLeft = 0.0;
volatile float rmsRight = 0.0;
volatile float phaseCorrelation = 0.0;
volatile bool dataActive = false;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait for Serial Monitor

  Serial.println("\n========================================");
  Serial.println("       ESP32 I2S Digital Tester         ");
  Serial.println("       Mode: Slave RX (384 kHz)         ");
  Serial.println("========================================");

  // Configure I2S pins: BCLK, WS (LRCK), Dout (Data), MCLK (-1 = unused)
  i2s.setPins(I2S_BCLK, I2S_LRCK, I2S_DATA, -1);

  // Initialize I2S in Slave Receiver mode: 384000 Hz, 32-bit width, Stereo
  if (!i2s.begin(I2S_MODE_SLAVE, 384000, I2S_DATA_BIT_WIDTH_32BIT)) {
    Serial.println("[ERROR] Failed to initialize I2S bus in Slave mode!");
    while (1) { delay(1000); }
  }

  Serial.println("[INFO] I2S Slave RX initialized successfully.");
  Serial.println("[INFO] Waiting for T113-S3 clock and PCM data...\n");
}

void loop() {
  // Read raw 32-bit stereo audio data via DMA buffer
  size_t bytesRead = i2s.readBytes((char*)audioBuffer, sizeof(audioBuffer));

  if (bytesRead > 0) {
    dataActive = true;
    totalBytesReceived += bytesRead;
    size_t frames = bytesRead / 8; // 4 bytes Left + 4 bytes Right per stereo frame
    totalFramesReceived += frames;

    analyzeAudioData(frames);
  } else {
    // If no bytes read for a while, flag inactive
    if (millis() - lastReportTime > 2000) {
      dataActive = false;
    }
  }

  // Print test report once every 1 second
  if (millis() - lastReportTime >= 1000) {
    lastReportTime = millis();
    printTestReport();
  }
}

void analyzeAudioData(size_t frames) {
  int32_t maxL = 0, maxR = 0;
  int64_t sumSqL = 0, sumSqR = 0;
  int64_t dotProd = 0;

  for (size_t i = 0; i < frames; i++) {
    // Interleaved format: [Left0, Right0, Left1, Right1, ...]
    int32_t rawLeft = audioBuffer[i * 2];
    int32_t rawRight = audioBuffer[i * 2 + 1];

    // T113 sends 32-bit slot with upper 24-bit valid PCM. Extract 24-bit:
    int32_t pcmL = rawLeft >> 8;
    int32_t pcmR = rawRight >> 8;

    // Track Peaks
    if (abs(pcmL) > maxL) maxL = abs(pcmL);
    if (abs(pcmR) > maxR) maxR = abs(pcmR);

    // Sum of squares for RMS
    sumSqL += (int64_t)pcmL * pcmL;
    sumSqR += (int64_t)pcmR * pcmR;

    // Dot product for phase correlation
    dotProd += (int64_t)pcmL * pcmR;
  }

  peakLeft = maxL;
  peakRight = maxR;
  rmsLeft = sqrt((double)sumSqL / frames);
  rmsRight = sqrt((double)sumSqR / frames);

  double denom = sqrt((double)sumSqL * sumSqR);
  phaseCorrelation = (denom == 0) ? 0.0 : (float)(dotProd / denom);
}

void printTestReport() {
  Serial.println("----------------------------------------");
  Serial.printf("RX Status:   %s\n", dataActive ? "RUNNING (Active)" : "WAITING FOR CLOCK...");
  Serial.printf("Bytes:       %u bytes\n", totalBytesReceived);
  Serial.printf("Frames:      %u frames\n", totalFramesReceived);
  
  if (dataActive) {
    Serial.printf("Left Peak:   %d (24-bit) | RMS: %.1f\n", peakLeft, rmsLeft);
    Serial.printf("Right Peak:  %d (24-bit) | RMS: %.1f\n", peakRight, rmsRight);
    Serial.printf("Correlation: %.3f (Target ~ -1.0 for inverted sine)\n", phaseCorrelation);
    
    // Verification check for 1 kHz inverted sine test signal
    if (phaseCorrelation < -0.8f) {
      Serial.println("[VERIFY OK] Channels show strong anti-phase relationship.");
    }
  } else {
    Serial.println("[WARN] No clock/data detected. Check wiring:");
    Serial.println("       - BCLK -> IO18");
    Serial.println("       - LRCK -> IO19");
    Serial.println("       - DATA -> IO27");
    Serial.println("       - GND  -> GND");
  }
}