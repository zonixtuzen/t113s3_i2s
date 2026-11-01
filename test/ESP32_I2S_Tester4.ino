/*
 * ESP32_I2S_Tester4.ino
 *
 * Board:
 *   Keyes 2.8 inch ESP32-32E
 *   ESP32-WROOM-32E / W0WD-V3
 *
 * Arduino-ESP32: 3.3.5
 * ESP-IDF: 5.5
 *
 * LCD: ILI9341 240x320
 * Touch: XPT2046
 *
 * I2S:
 *   Slave RX
 *   Philips I2S
 *   Stereo, 32-bit slots
 *   Target: 384 kHz / 24.576 MHz BCLK
 *
 * T113 connections:
 *   PG13 BCLK -> GPIO18
 *   PG12 LRCK -> GPIO19
 *   PG15 DOUT -> GPIO27
 *   GND       -> GND
 *
 * Touch commands:
 *   START / STOP / RESET
 *
 * Serial:
 *   s = Start
 *   x = Stop
 *   p = Print stats
 *   z = Reset stats
 *   h = Help
 */

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <math.h>

#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_err.h"

// ============================================================
// I2S configuration
// ============================================================

static const uint32_t SAMPLE_RATE = 384000;

static const gpio_num_t PIN_BCLK = GPIO_NUM_18;
static const gpio_num_t PIN_LRCK = GPIO_NUM_19;
static const gpio_num_t PIN_DIN  = GPIO_NUM_27;

static const uint32_t DMA_DESC_NUM  = 8;
static const uint32_t DMA_FRAME_NUM = 256;

static const size_t RX_WORDS = 512;
static const size_t RX_BYTES = RX_WORDS * sizeof(uint32_t);

// ============================================================
// LCD configuration
// ============================================================

TFT_eSPI tft = TFT_eSPI();

static const int SCREEN_W = 320;
static const int SCREEN_H = 240;

static const uint16_t COLOR_BG     = TFT_BLACK;
static const uint16_t COLOR_PANEL  = 0x18E3;
static const uint16_t COLOR_TEXT   = TFT_WHITE;
static const uint16_t COLOR_GREEN  = TFT_GREEN;
static const uint16_t COLOR_RED    = TFT_RED;
static const uint16_t COLOR_YELLOW = TFT_YELLOW;
static const uint16_t COLOR_CYAN   = TFT_CYAN;

// Backlight
static const int PIN_LCD_BL = 21;

// ============================================================
// XPT2046 touch configuration
// ============================================================

static const int PIN_TP_SCK  = 25;
static const int PIN_TP_MOSI = 32;
static const int PIN_TP_MISO = 39;
static const int PIN_TP_CS   = 33;
static const int PIN_TP_IRQ  = 36;

// Touch calibration values.
// These are initial estimates, not factory calibration.
// Adjust after observing raw touch readings.
static const int TP_X_MIN = 250;
static const int TP_X_MAX = 3850;
static const int TP_Y_MIN = 250;
static const int TP_Y_MAX = 3850;

// If touch directions are reversed, change these.
static const bool TP_SWAP_XY = false;
static const bool TP_INVERT_X = false;
static const bool TP_INVERT_Y = true;

// ============================================================
// I2S state
// ============================================================

static i2s_chan_handle_t rxHandle = NULL;

static bool rxRunning = false;

static uint32_t rxBuffer[RX_WORDS];

static volatile uint32_t rxQueueOverflows = 0;

// ============================================================
// Statistics
// ============================================================

static uint64_t totalBytes = 0;
static uint64_t totalWords = 0;
static uint64_t totalFrames = 0;

static uint64_t totalReadErrors = 0;

static uint64_t windowFrames = 0;
static uint64_t lowByteErrors = 0;

static int32_t peakLeft = 0;
static int32_t peakRight = 0;

static double sumSqLeft = 0;
static double sumSqRight = 0;

static double sumLR = 0;
static double sumLeft = 0;
static double sumRight = 0;

static uint64_t zeroCrossings = 0;

static bool previousPositive = false;
static bool previousValid = false;

static uint32_t reportStart = 0;

static double measuredRate = 0;
static double measuredFrequency = 0;
static double rmsLeft = 0;
static double rmsRight = 0;
static double correlation = 0;

// ============================================================
// Waveform storage
// ============================================================

static const int WAVE_POINTS = 150;

static int16_t waveLeft[WAVE_POINTS];
static int16_t waveRight[WAVE_POINTS];

static int waveIndex = 0;
static uint32_t waveDivider = 0;

// ============================================================
// UI state
// ============================================================

static uint32_t lastScreenUpdate = 0;
static uint32_t lastTouchCheck = 0;

static bool screenInitialized = false;

// ============================================================
// Error helper
// ============================================================

static bool checkEsp(
    esp_err_t err,
    const char *operation
) {
    if (err == ESP_OK) {
        return true;
    }

    Serial.printf(
        "[I2S ERROR] %s: %s (%d)\n",
        operation,
        esp_err_to_name(err),
        (int)err
    );

    return false;
}

// ============================================================
// I2S overflow callback
// ============================================================

static bool IRAM_ATTR onRxQueueOverflow(
    i2s_chan_handle_t handle,
    i2s_event_data_t *event,
    void *userContext
) {
    rxQueueOverflows++;
    return false;
}

// ============================================================
// Touch bit-bang SPI
// ============================================================

static uint16_t touchTransfer(uint8_t command) {

    uint16_t result = 0;

    digitalWrite(PIN_TP_CS, LOW);

    // Send command

    for (int i = 7; i >= 0; i--) {

        digitalWrite(
            PIN_TP_MOSI,
            (command >> i) & 1
        );

        digitalWrite(PIN_TP_SCK, HIGH);
        delayMicroseconds(1);

        digitalWrite(PIN_TP_SCK, LOW);
        delayMicroseconds(1);
    }

    // Read 16 bits

    for (int i = 15; i >= 0; i--) {

        digitalWrite(PIN_TP_SCK, HIGH);
        delayMicroseconds(1);

        result <<= 1;

        if (digitalRead(PIN_TP_MISO)) {
            result |= 1;
        }

        digitalWrite(PIN_TP_SCK, LOW);
        delayMicroseconds(1);
    }

    digitalWrite(PIN_TP_CS, HIGH);

    return (result >> 3) & 0x0FFF;
}

static bool readTouchRaw(
    uint16_t &rawX,
    uint16_t &rawY
) {

    if (digitalRead(PIN_TP_IRQ) == LOW) {
        return false;
    }

    uint32_t x = 0;
    uint32_t y = 0;

    // Average several readings

    for (int i = 0; i < 4; i++) {

        x += touchTransfer(0xD0);
        y += touchTransfer(0x90);
    }

    rawX = x / 4;
    rawY = y / 4;

    return true;
}

static bool readTouchPoint(
    int &x,
    int &y
) {

    uint16_t rawX;
    uint16_t rawY;

    if (!readTouchRaw(rawX, rawY)) {
        return false;
    }

    int tx = rawX;
    int ty = rawY;

    if (TP_SWAP_XY) {
        int temp = tx;
        tx = ty;
        ty = temp;
    }

    x = map(
        tx,
        TP_X_MIN,
        TP_X_MAX,
        0,
        SCREEN_W - 1
    );

    y = map(
        ty,
        TP_Y_MIN,
        TP_Y_MAX,
        0,
        SCREEN_H - 1
    );

    x = constrain(x, 0, SCREEN_W - 1);
    y = constrain(y, 0, SCREEN_H - 1);

    if (TP_INVERT_X) {
        x = SCREEN_W - 1 - x;
    }

    if (TP_INVERT_Y) {
        y = SCREEN_H - 1 - y;
    }

    return true;
}

// ============================================================
// Statistics reset
// ============================================================

static void resetStatistics() {

    totalBytes = 0;
    totalWords = 0;
    totalFrames = 0;

    totalReadErrors = 0;
    rxQueueOverflows = 0;

    windowFrames = 0;
    lowByteErrors = 0;

    peakLeft = 0;
    peakRight = 0;

    sumSqLeft = 0;
    sumSqRight = 0;

    sumLR = 0;
    sumLeft = 0;
    sumRight = 0;

    zeroCrossings = 0;

    previousPositive = false;
    previousValid = false;

    measuredRate = 0;
    measuredFrequency = 0;

    rmsLeft = 0;
    rmsRight = 0;

    correlation = 0;

    for (int i = 0; i < WAVE_POINTS; i++) {
        waveLeft[i] = 0;
        waveRight[i] = 0;
    }

    waveIndex = 0;
    waveDivider = 0;

    reportStart = millis();
}

// ============================================================
// I2S start
// ============================================================

static bool startI2S() {

    if (rxRunning) {
        Serial.println("I2S already running.");
        return true;
    }

    Serial.println("Starting I2S Slave RX...");

    i2s_chan_config_t chanCfg =
        I2S_CHANNEL_DEFAULT_CONFIG(
            I2S_NUM_0,
            I2S_ROLE_SLAVE
        );

    chanCfg.dma_desc_num = DMA_DESC_NUM;
    chanCfg.dma_frame_num = DMA_FRAME_NUM;

    esp_err_t err = i2s_new_channel(
        &chanCfg,
        NULL,
        &rxHandle
    );

    if (!checkEsp(err, "i2s_new_channel")) {
        rxHandle = NULL;
        return false;
    }

    i2s_std_config_t stdCfg = {

        .clk_cfg =
            I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),

        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                I2S_DATA_BIT_WIDTH_32BIT,
                I2S_SLOT_MODE_STEREO
            ),

        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,

            .bclk = PIN_BCLK,
            .ws   = PIN_LRCK,

            .dout = I2S_GPIO_UNUSED,
            .din  = PIN_DIN,

            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };

    err = i2s_channel_init_std_mode(
        rxHandle,
        &stdCfg
    );

    if (!checkEsp(err, "i2s_channel_init_std_mode")) {

        i2s_del_channel(rxHandle);
        rxHandle = NULL;

        return false;
    }

    i2s_event_callbacks_t callbacks = {

        .on_recv = NULL,
        .on_recv_q_ovf = onRxQueueOverflow,
        .on_sent = NULL,
        .on_send_q_ovf = NULL
    };

    err = i2s_channel_register_event_callback(
        rxHandle,
        &callbacks,
        NULL
    );

    if (!checkEsp(err, "register callback")) {

        i2s_del_channel(rxHandle);
        rxHandle = NULL;

        return false;
    }

    err = i2s_channel_enable(rxHandle);

    if (!checkEsp(err, "i2s_channel_enable")) {

        i2s_del_channel(rxHandle);
        rxHandle = NULL;

        return false;
    }

    resetStatistics();

    rxRunning = true;

    Serial.println("I2S SLAVE RX started.");

    return true;
}

// ============================================================
// I2S stop
// ============================================================

static void stopI2S() {

    if (!rxRunning || rxHandle == NULL) {
        return;
    }

    rxRunning = false;

    i2s_channel_disable(rxHandle);

    i2s_del_channel(rxHandle);

    rxHandle = NULL;

    Serial.println("I2S RX stopped.");
}

// ============================================================
// 24-bit unpack
// ============================================================

static inline int32_t unpack24(uint32_t raw) {

    return ((int32_t)raw) >> 8;
}

// ============================================================
// Process received data
// ============================================================

static void processSamples(
    const uint32_t *data,
    size_t words
) {

    size_t frames = words / 2;

    for (size_t i = 0; i < frames; i++) {

        uint32_t rawL = data[i * 2];
        uint32_t rawR = data[i * 2 + 1];

        if ((rawL & 0xFF) != 0) {
            lowByteErrors++;
        }

        if ((rawR & 0xFF) != 0) {
            lowByteErrors++;
        }

        int32_t left = unpack24(rawL);
        int32_t right = unpack24(rawR);

        int32_t absL = abs(left);
        int32_t absR = abs(right);

        if (absL > peakLeft) {
            peakLeft = absL;
        }

        if (absR > peakRight) {
            peakRight = absR;
        }

        sumSqLeft += (double)left * left;
        sumSqRight += (double)right * right;

        sumLR += (double)left * right;

        sumLeft += left;
        sumRight += right;

        bool positive = (left >= 0);

        if (previousValid &&
            !previousPositive &&
            positive) {

            zeroCrossings++;
        }

        previousPositive = positive;
        previousValid = true;

        // Store waveform points.
        // Decimate input to reduce processing overhead.

        waveDivider++;

        if (waveDivider >= 64) {

            waveDivider = 0;

            waveLeft[waveIndex] =
                (int16_t)(left >> 8);

            waveRight[waveIndex] =
                (int16_t)(right >> 8);

            waveIndex++;

            if (waveIndex >= WAVE_POINTS) {
                waveIndex = 0;
            }
        }
    }

    windowFrames += frames;

    totalFrames += frames;
    totalWords += words;
    totalBytes += words * sizeof(uint32_t);
}

// ============================================================
// Calculate statistics
// ============================================================

static void calculateStatistics() {

    uint32_t now = millis();

    uint32_t elapsed = now - reportStart;

    if (elapsed == 0 || windowFrames == 0) {
        return;
    }

    double seconds = elapsed / 1000.0;

    measuredRate =
        (double)windowFrames / seconds;

    rmsLeft = sqrt(
        sumSqLeft / (double)windowFrames
    );

    rmsRight = sqrt(
        sumSqRight / (double)windowFrames
    );

    double meanL =
        sumLeft / (double)windowFrames;

    double meanR =
        sumRight / (double)windowFrames;

    double covariance =
        sumLR / (double)windowFrames -
        meanL * meanR;

    double varianceL =
        sumSqLeft / (double)windowFrames -
        meanL * meanL;

    double varianceR =
        sumSqRight / (double)windowFrames -
        meanR * meanR;

    if (varianceL > 0 && varianceR > 0) {

        correlation =
            covariance /
            sqrt(varianceL * varianceR);
    }
    else {
        correlation = 0;
    }

    measuredFrequency =
        zeroCrossings / (2.0 * seconds);

    // Reset window accumulators

    windowFrames = 0;

    lowByteErrors = 0;

    peakLeft = 0;
    peakRight = 0;

    sumSqLeft = 0;
    sumSqRight = 0;

    sumLR = 0;
    sumLeft = 0;
    sumRight = 0;

    zeroCrossings = 0;

    reportStart = now;
}

// ============================================================
// LCD static layout
// ============================================================

static void drawStaticScreen() {

    tft.fillScreen(COLOR_BG);

    // Header

    tft.fillRect(
        0, 0,
        SCREEN_W, 28,
        TFT_DARKCYAN
    );

    tft.setTextColor(TFT_WHITE, TFT_DARKCYAN);
    tft.drawString(
        "ESP32 I2S DIGITAL TESTER",
        7, 6, 2
    );

    // Status area

    tft.fillRoundRect(
        5, 34,
        310, 42,
        5,
        COLOR_PANEL
    );

    tft.setTextColor(COLOR_CYAN, COLOR_PANEL);
    tft.drawString("MODE", 12, 39, 1);

    tft.drawString("RATE", 110, 39, 1);

    tft.drawString("BCLK", 215, 39, 1);

    // Waveform area

    tft.setTextColor(COLOR_GREEN, COLOR_BG);
    tft.drawString("LEFT", 8, 82, 2);

    tft.setTextColor(COLOR_CYAN, COLOR_BG);
    tft.drawString("RIGHT", 8, 143, 2);

    tft.drawRect(
        55, 80,
        258, 57,
        TFT_DARKGREY
    );

    tft.drawRect(
        55, 141,
        258, 57,
        TFT_DARKGREY
    );

    // Bottom data area

    tft.fillRoundRect(
        5, 202,
        310, 32,
        4,
        COLOR_PANEL
    );

    tft.setTextColor(COLOR_TEXT, COLOR_PANEL);

    tft.drawString("L RMS", 10, 208, 1);
    tft.drawString("R RMS", 85, 208, 1);
    tft.drawString("FREQ", 160, 208, 1);
    tft.drawString("CORR", 245, 208, 1);

    screenInitialized = true;
}

// ============================================================
// Draw buttons
// ============================================================

static void drawButtons() {

    // START
    tft.fillRoundRect(
        7, 180,
        68, 20,
        4,
        COLOR_GREEN
    );

    tft.setTextColor(TFT_BLACK, COLOR_GREEN);
    tft.drawCentreString(
        "START",
        41, 185, 1
    );

    // STOP
    tft.fillRoundRect(
        82, 180,
        68, 20,
        4,
        COLOR_RED
    );

    tft.setTextColor(TFT_WHITE, COLOR_RED);
    tft.drawCentreString(
        "STOP",
        116, 185, 1
    );

    // RESET
    tft.fillRoundRect(
        157, 180,
        68, 20,
        4,
        COLOR_YELLOW
    );

    tft.setTextColor(TFT_BLACK, COLOR_YELLOW);
    tft.drawCentreString(
        "RESET",
        191, 185, 1
    );
}

// ============================================================
// Draw waveform
// ============================================================

static void drawWaveform(
    int16_t *wave,
    int yTop,
    uint16_t color
) {

    const int xStart = 57;
    const int width = 254;
    const int centerY = yTop + 28;

    // Clear waveform area

    tft.fillRect(
        xStart + 1,
        yTop + 1,
        width - 2,
        55,
        COLOR_BG
    );

    tft.drawFastHLine(
        xStart + 1,
        centerY,
        width - 2,
        TFT_DARKGREY
    );

    int previousX = xStart + 1;
    int previousY = centerY;

    for (int i = 0; i < WAVE_POINTS; i++) {

        int index =
            (waveIndex + i) % WAVE_POINTS;

        int value = wave[index];

        int y = centerY - (value * 25 / 32768);

        int x =
            xStart + 1 +
            (i * (width - 3) / WAVE_POINTS);

        if (i > 0) {

            tft.drawLine(
                previousX,
                previousY,
                x,
                y,
                color
            );
        }

        previousX = x;
        previousY = y;
    }
}

// ============================================================
// Draw dynamic information
// ============================================================

static void drawDynamicScreen() {

    // Header status

    tft.fillRect(
        10, 51,
        95, 20,
        COLOR_PANEL
    );

    tft.setTextColor(
        rxRunning ? COLOR_GREEN : COLOR_RED,
        COLOR_PANEL
    );

    tft.drawString(
        rxRunning ? "SLAVE RX ON" : "STOPPED",
        12, 53, 2
    );

    // Sample rate

    tft.fillRect(
        110, 51,
        100, 20,
        COLOR_PANEL
    );

    tft.setTextColor(COLOR_YELLOW, COLOR_PANEL);

    tft.drawString(
        "384 kHz",
        112, 53, 2
    );

    // BCLK target

    tft.fillRect(
        215, 51,
        95, 20,
        COLOR_PANEL
    );

    tft.setTextColor(COLOR_YELLOW, COLOR_PANEL);

    tft.drawString(
        "24.576M",
        217, 53, 2
    );

    // Waveforms

    drawWaveform(
        waveLeft,
        80,
        COLOR_GREEN
    );

    drawWaveform(
        waveRight,
        141,
        COLOR_CYAN
    );

    // RMS

    tft.fillRect(
        10, 219,
        70, 13,
        COLOR_PANEL
    );

    tft.setTextColor(COLOR_GREEN, COLOR_PANEL);

    tft.drawString(
        String((uint32_t)rmsLeft),
        10, 219, 1
    );

    tft.fillRect(
        85, 219,
        70, 13,
        COLOR_PANEL
    );

    tft.setTextColor(COLOR_CYAN, COLOR_PANEL);

    tft.drawString(
        String((uint32_t)rmsRight),
        85, 219, 1
    );

    // Frequency

    tft.fillRect(
        160, 219,
        75, 13,
        COLOR_PANEL
    );

    tft.setTextColor(COLOR_YELLOW, COLOR_PANEL);

    tft.drawString(
        String(measuredFrequency, 0) + "Hz",
        160, 219, 1
    );

    // Correlation

    tft.fillRect(
        245, 219,
        65, 13,
        COLOR_PANEL
    );

    tft.setTextColor(COLOR_TEXT, COLOR_PANEL);

    tft.drawString(
        String(correlation, 2),
        245, 219, 1
    );

    // Buttons

    drawButtons();
}

// ============================================================
// Touch handling
// ============================================================

static void handleTouch() {

    if (millis() - lastTouchCheck < 180) {
        return;
    }

    lastTouchCheck = millis();

    int x;
    int y;

    if (!readTouchPoint(x, y)) {
        return;
    }

    uint16_t rawX;
    uint16_t rawY;

    readTouchRaw(rawX, rawY);

    Serial.printf(
        "Touch: x=%d y=%d rawX=%u rawY=%u\n",
        x, y,
        rawX, rawY
    );

    // START
    if (x >= 7 && x <= 75 &&
        y >= 180 && y <= 200) {

        startI2S();
        return;
    }

    // STOP
    if (x >= 82 && x <= 150 &&
        y >= 180 && y <= 200) {

        stopI2S();
        return;
    }

    // RESET
    if (x >= 157 && x <= 225 &&
        y >= 180 && y <= 200) {

        resetStatistics();
        Serial.println("Statistics reset.");
        return;
    }
}

// ============================================================
// Serial commands
// ============================================================

static void printStatistics() {

    Serial.println();
    Serial.println("========== I2S STATS ==========");

    Serial.printf(
        "Running       : %s\n",
        rxRunning ? "YES" : "NO"
    );

    Serial.printf(
        "Total bytes   : %llu\n",
        (unsigned long long)totalBytes
    );

    Serial.printf(
        "Total words   : %llu\n",
        (unsigned long long)totalWords
    );

    Serial.printf(
        "Total frames  : %llu\n",
        (unsigned long long)totalFrames
    );

    Serial.printf(
        "Measured rate : %.2f frames/s\n",
        measuredRate
    );

    Serial.printf(
        "Read errors   : %llu\n",
        (unsigned long long)totalReadErrors
    );

    Serial.printf(
        "Queue overflows: %lu\n",
        (unsigned long)rxQueueOverflows
    );

    Serial.printf(
        "RMS L         : %.2f\n",
        rmsLeft
    );

    Serial.printf(
        "RMS R         : %.2f\n",
        rmsRight
    );

    Serial.printf(
        "Frequency     : %.2f Hz\n",
        measuredFrequency
    );

    Serial.printf(
        "Correlation   : %.6f\n",
        correlation
    );

    Serial.println("===============================");
}

static void handleSerial() {

    while (Serial.available()) {

        char cmd = Serial.read();

        switch (cmd) {

            case 's':
            case 'S':
                startI2S();
                break;

            case 'x':
            case 'X':
                stopI2S();
                break;

            case 'p':
            case 'P':
                printStatistics();
                break;

            case 'z':
            case 'Z':
                resetStatistics();
                Serial.println("Statistics reset.");
                break;

            case 'h':
            case 'H':
                Serial.println(
                    "s=start x=stop p=stats z=reset h=help"
                );
                break;

            default:
                break;
        }
    }
}

// ============================================================
// Arduino setup
// ============================================================

void setup() {

    Serial.begin(115200);

    delay(500);

    Serial.println();
    Serial.println("ESP32 I2S Tester 4");
    Serial.println("Arduino-ESP32 3.3.5");
    Serial.println("ESP-IDF native I2S Slave RX");

    // LCD backlight

    pinMode(PIN_LCD_BL, OUTPUT);
    digitalWrite(PIN_LCD_BL, HIGH);

    // Touch pins

    pinMode(PIN_TP_CS, OUTPUT);
    pinMode(PIN_TP_SCK, OUTPUT);
    pinMode(PIN_TP_MOSI, OUTPUT);

    pinMode(PIN_TP_MISO, INPUT);
    pinMode(PIN_TP_IRQ, INPUT);

    digitalWrite(PIN_TP_CS, HIGH);
    digitalWrite(PIN_TP_SCK, LOW);

    // LCD

    tft.init();

    tft.setRotation(1);

    tft.fillScreen(COLOR_BG);

    drawStaticScreen();
    drawButtons();

    resetStatistics();

    drawDynamicScreen();

    Serial.println("LCD initialized.");
    Serial.println("Touch controller: XPT2046");
    Serial.println("Ready.");
    Serial.println("Touch START or send serial command s.");
}

// ============================================================
// Arduino loop
// ============================================================

void loop() {

    handleSerial();

    handleTouch();

    // --------------------------------------------------------
    // Receive I2S
    // --------------------------------------------------------

    if (rxRunning && rxHandle != NULL) {

        size_t bytesRead = 0;

        esp_err_t err = i2s_channel_read(
            rxHandle,
            rxBuffer,
            RX_BYTES,
            &bytesRead,
            10
        );

        if (err == ESP_OK && bytesRead >= 8) {

            size_t words =
                bytesRead / sizeof(uint32_t);

            processSamples(
                rxBuffer,
                words
            );
        }
        else if (err != ESP_ERR_TIMEOUT) {

            totalReadErrors++;

            Serial.printf(
                "I2S read error: %s\n",
                esp_err_to_name(err)
            );
        }
    }

    // --------------------------------------------------------
    // Update statistics
    // --------------------------------------------------------

    if (rxRunning &&
        millis() - reportStart >= 1000) {

        calculateStatistics();
    }

    // --------------------------------------------------------
    // Update LCD
    // --------------------------------------------------------

    if (millis() - lastScreenUpdate >= 300) {

        lastScreenUpdate = millis();

        drawDynamicScreen();
    }
}