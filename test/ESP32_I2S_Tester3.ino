/*
 * ESP32_I2S_Tester.ino
 *
 * Hardware:
 *   ESP32-W0WD-V3 / ESP32-D0WD-V3
 *
 * Function:
 *   I2S Standard Philips Slave Receiver
 *   384 kHz, Stereo, 32-bit slots
 *   T113-S3 -> ESP32
 *
 * Wiring:
 *   T113 PG13 BCLK -> ESP32 GPIO18
 *   T113 PG12 LRCK -> ESP32 GPIO19
 *   T113 PG15 DOUT -> ESP32 GPIO27
 *   T113 GND       -> ESP32 GND
 *
 * Arduino IDE:
 *   Arduino-ESP32 Core 3.x
 *   Board: ESP32 Dev Module
 *   Serial: 115200 baud
 *
 * Commands:
 *   s = Start I2S RX
 *   x = Stop I2S RX
 *   p = Print status
 *   z = Clear statistics
 *   h = Help
 *
 * IMPORTANT:
 *   Original ESP32 24.576 MHz slave BCLK capability
 *   must be verified experimentally.
 */

#include <Arduino.h>
#include <ESP_I2S.h>
#include <math.h>

// =====================================================
// 1. Pin configuration
// =====================================================

static constexpr int PIN_BCLK = 18;
static constexpr int PIN_LRCK = 19;
static constexpr int PIN_DIN  = 27;

// =====================================================
// 2. I2S configuration
// =====================================================

static constexpr uint32_t SAMPLE_RATE = 384000;
static constexpr uint32_t EXPECTED_BCLK = 24576000;

static constexpr size_t RX_FRAMES = 512;
static constexpr size_t RX_WORDS = RX_FRAMES * 2;
static constexpr size_t RX_BYTES = RX_WORDS * sizeof(uint32_t);

static constexpr uint32_t REPORT_INTERVAL_MS = 1000;

// =====================================================
// 3. I2S object and receive buffer
// =====================================================

I2SClass I2S;

static uint32_t rxBuffer[RX_WORDS];

static bool i2sRunning = false;

// =====================================================
// 4. Global statistics
// =====================================================

static uint64_t totalBytes = 0;
static uint64_t totalFrames = 0;
static uint64_t shortReads = 0;

static uint64_t badLowByte = 0;
static uint64_t checkedWords = 0;

static uint32_t lastReadBytes = 0;
static uint32_t lastReadTime = 0;

static uint32_t reportStart = 0;
static uint32_t reportNumber = 0;

// =====================================================
// 5. Window statistics
// =====================================================

static uint64_t windowFrames = 0;
static uint64_t leftZeroCrossings = 0;

static double sumL = 0;
static double sumR = 0;

static double sumSqL = 0;
static double sumSqR = 0;

static double sumLR = 0;

static int32_t peakL = 0;
static int32_t peakR = 0;

static int32_t previousL = 0;
static bool havePreviousL = false;

static bool printRawOnce = false;

// =====================================================
// 6. PCM conversion
// =====================================================

/*
 * T113 test program format:
 *
 *   signed 24-bit PCM
 *   left aligned in 32-bit slot
 *
 *   raw = sample24 * 256
 *
 * Therefore:
 *
 *   raw[31:8] = PCM24
 *   raw[7:0]  = 0
 */

static inline int32_t unpackPCM24(uint32_t raw)
{
    return ((int32_t)raw) >> 8;
}

static inline int32_t absSafe(int32_t v)
{
    if (v == INT32_MIN)
        return INT32_MAX;

    return (v < 0) ? -v : v;
}

// =====================================================
// 7. Clear statistics
// =====================================================

static void clearWindowStats()
{
    windowFrames = 0;
    leftZeroCrossings = 0;

    sumL = 0;
    sumR = 0;

    sumSqL = 0;
    sumSqR = 0;

    sumLR = 0;

    peakL = 0;
    peakR = 0;
}

static void clearAllStats()
{
    totalBytes = 0;
    totalFrames = 0;
    shortReads = 0;

    badLowByte = 0;
    checkedWords = 0;

    lastReadBytes = 0;
    lastReadTime = 0;

    reportNumber = 0;

    previousL = 0;
    havePreviousL = false;

    clearWindowStats();

    Serial.println("[STATS] Cleared.");
}

// =====================================================
// 8. Help
// =====================================================

static void printHelp()
{
    Serial.println();
    Serial.println("========== COMMANDS ==========");
    Serial.println("s : Start I2S RX");
    Serial.println("x : Stop I2S RX");
    Serial.println("p : Print status");
    Serial.println("z : Clear statistics");
    Serial.println("h : Help");
    Serial.println("==============================");
}

// =====================================================
// 9. Start I2S receiver
// =====================================================

static bool startI2S()
{
    if (i2sRunning)
    {
        I2S.end();
        i2sRunning = false;
        delay(50);
    }

    Serial.println();
    Serial.println("[I2S] Starting receiver...");

    /*
     * setPins argument order:
     *
     * BCLK, WS, DOUT, DIN, MCLK
     *
     * ESP32 is receiver only.
     * DOUT and MCLK are unused.
     */

    I2S.setPins(
        PIN_BCLK,
        PIN_LRCK,
        -1,
        PIN_DIN
    );

    /*
     * Standard Philips I2S
     * Slave RX
     * Stereo
     * 32-bit data width
     */

    bool ok = I2S.begin(
        I2S_MODE_STD,
        SAMPLE_RATE,
        I2S_DATA_BIT_WIDTH_32BIT,
        I2S_SLOT_MODE_STEREO,
        -1,
        I2S_ROLE_SLAVE
    );

    if (!ok)
    {
        Serial.printf(
            "[I2S] Initialization failed. Error=%d\n",
            I2S.lastError()
        );

        return false;
    }

    i2sRunning = true;

    reportStart = millis();

    printRawOnce = true;
    havePreviousL = false;

    clearWindowStats();

    Serial.println("[I2S] Initialization successful.");

    Serial.println("Role       : SLAVE RX");
    Serial.println("Format     : Standard Philips I2S");
    Serial.println("Sample rate: 384000 Hz");
    Serial.println("BCLK       : 24.576 MHz");
    Serial.println("Slot       : 32-bit");
    Serial.println("Channels   : Stereo");

    Serial.println();
    Serial.println(
        "WARNING: Driver initialization does not prove "
        "24.576 MHz reception."
    );

    Serial.println(
        "Now start T113: i2s1_pcm_test start"
    );

    return true;
}

// =====================================================
// 10. Stop I2S
// =====================================================

static void stopI2S()
{
    if (!i2sRunning)
    {
        Serial.println("[I2S] Not running.");
        return;
    }

    I2S.end();

    i2sRunning = false;

    Serial.println("[I2S] Receiver stopped.");
}

// =====================================================
// 11. Print status
// =====================================================

static void printStatus()
{
    Serial.println();
    Serial.println("========== I2S STATUS ==========");

    Serial.printf(
        "Running          : %s\n",
        i2sRunning ? "YES" : "NO"
    );

    Serial.printf(
        "BCLK pin         : GPIO%d\n",
        PIN_BCLK
    );

    Serial.printf(
        "LRCK pin         : GPIO%d\n",
        PIN_LRCK
    );

    Serial.printf(
        "DIN pin          : GPIO%d\n",
        PIN_DIN
    );

    Serial.printf(
        "Sample rate      : %lu Hz\n",
        (unsigned long)SAMPLE_RATE
    );

    Serial.printf(
        "Expected BCLK    : %lu Hz\n",
        (unsigned long)EXPECTED_BCLK
    );

    Serial.printf(
        "Total bytes      : %llu\n",
        (unsigned long long)totalBytes
    );

    Serial.printf(
        "Total frames     : %llu\n",
        (unsigned long long)totalFrames
    );

    Serial.printf(
        "Short reads      : %llu\n",
        (unsigned long long)shortReads
    );

    Serial.printf(
        "Last read bytes  : %lu\n",
        (unsigned long)lastReadBytes
    );

    Serial.printf(
        "Last read time   : %lu ms\n",
        (unsigned long)lastReadTime
    );

    Serial.printf(
        "Bad low bytes    : %llu / %llu\n",
        (unsigned long long)badLowByte,
        (unsigned long long)checkedWords
    );

    Serial.println("================================");
}

// =====================================================
// 12. PCM data processing
// =====================================================

static void processPCM(size_t words)
{
    // Only process complete stereo frames.

    words &= ~((size_t)1);

    if (words < 2)
        return;

    // Print first received raw words.

    if (printRawOnce)
    {
        Serial.println();
        Serial.println("[RAW] First received words:");

        size_t count = min(words, (size_t)16);

        for (size_t i = 0; i < count; i++)
        {
            Serial.printf(
                "word[%02u] = 0x%08lX  %s\n",
                (unsigned)i,
                (unsigned long)rxBuffer[i],
                ((i & 1) == 0) ? "LEFT" : "RIGHT"
            );
        }

        Serial.println(
            "[RAW] Expected low 8 bits = 0 for T113 PCM test."
        );

        printRawOnce = false;
    }

    size_t frames = words / 2;

    windowFrames += frames;
    totalFrames += frames;

    for (size_t i = 0; i < words; i += 2)
    {
        uint32_t rawL = rxBuffer[i];
        uint32_t rawR = rxBuffer[i + 1];

        // Check left-aligned 24-bit data.

        checkedWords++;

        if ((rawL & 0xFF) != 0)
            badLowByte++;

        checkedWords++;

        if ((rawR & 0xFF) != 0)
            badLowByte++;

        // Convert to signed PCM24.

        int32_t L = unpackPCM24(rawL);
        int32_t R = unpackPCM24(rawR);

        int32_t aL = absSafe(L);
        int32_t aR = absSafe(R);

        if (aL > peakL)
            peakL = aL;

        if (aR > peakR)
            peakR = aR;

        double dL = (double)L;
        double dR = (double)R;

        sumL += dL;
        sumR += dR;

        sumSqL += dL * dL;
        sumSqR += dR * dR;

        sumLR += dL * dR;

        // Rising zero crossing.

        if (
            havePreviousL &&
            previousL <= 0 &&
            L > 0
        )
        {
            leftZeroCrossings++;
        }

        previousL = L;
        havePreviousL = true;
    }
}

// =====================================================
// 13. Print PCM analysis
// =====================================================

static void printReport()
{
    if (windowFrames == 0)
    {
        Serial.println(
            "[RX] No complete stereo frames received."
        );

        return;
    }

    double n = (double)windowFrames;

    double meanL = sumL / n;
    double meanR = sumR / n;

    double varL = sumSqL / n - meanL * meanL;
    double varR = sumSqR / n - meanR * meanR;

    double cov = sumLR / n - meanL * meanR;

    if (varL < 0)
        varL = 0;

    if (varR < 0)
        varR = 0;

    double rmsL = sqrt(varL);
    double rmsR = sqrt(varR);

    double denom = sqrt(varL * varR);

    double correlation = 0;

    if (denom > 0)
        correlation = cov / denom;

    uint32_t now = millis();

    uint32_t elapsed = now - reportStart;

    double estimatedFreq = 0;

    if (elapsed > 0)
    {
        estimatedFreq =
            (double)leftZeroCrossings *
            1000.0 /
            (double)elapsed;
    }

    reportNumber++;

    Serial.println();
    Serial.printf(
        "========== RX REPORT #%lu ==========\n",
        (unsigned long)reportNumber
    );

    Serial.printf(
        "Window frames       : %llu\n",
        (unsigned long long)windowFrames
    );

    Serial.printf(
        "Total frames        : %llu\n",
        (unsigned long long)totalFrames
    );

    Serial.printf(
        "Total bytes         : %llu\n",
        (unsigned long long)totalBytes
    );

    Serial.printf(
        "Left frequency      : %.1f Hz (target ~1000)\n",
        estimatedFreq
    );

    Serial.printf(
        "Left peak           : %ld\n",
        (long)peakL
    );

    Serial.printf(
        "Right peak          : %ld\n",
        (long)peakR
    );

    Serial.printf(
        "Left RMS            : %.1f\n",
        rmsL
    );

    Serial.printf(
        "Right RMS           : %.1f\n",
        rmsR
    );

    Serial.printf(
        "L/R correlation     : %.5f (target near -1)\n",
        correlation
    );

    Serial.printf(
        "Low byte nonzero    : %llu / %llu\n",
        (unsigned long long)badLowByte,
        (unsigned long long)checkedWords
    );

    if (rmsL < 1000 || rmsR < 1000)
    {
        Serial.println(
            "RESULT: Signal level too small. Check raw data."
        );
    }
    else if (correlation < -0.90)
    {
        Serial.println(
            "RESULT: Left/right opposite polarity confirmed."
        );
    }
    else
    {
        Serial.println(
            "RESULT: Data received, PCM relationship not confirmed."
        );
    }

    Serial.println("====================================");

    clearWindowStats();

    reportStart = now;
}

// =====================================================
// 14. Serial command handling
// =====================================================

static void handleSerial()
{
    while (Serial.available())
    {
        char c = Serial.read();

        switch (c)
        {
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
                printStatus();
                break;

            case 'z':
            case 'Z':
                clearAllStats();
                break;

            case 'h':
            case 'H':
            case '?':
                printHelp();
                break;

            default:
                break;
        }
    }
}

// =====================================================
// 15. Setup
// =====================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("====================================");
    Serial.println("      ESP32 I2S DIGITAL TESTER");
    Serial.println("====================================");

    Serial.println("Chip       : Classic ESP32");
    Serial.println("Mode       : I2S Slave RX");
    Serial.println("Format     : Standard Philips");
    Serial.println("Sample rate: 384 kHz");
    Serial.println("BCLK       : 24.576 MHz");
    Serial.println("Slot       : Stereo 32-bit");

    Serial.println();

    Serial.printf("BCLK GPIO  : %d\n", PIN_BCLK);
    Serial.printf("LRCK GPIO  : %d\n", PIN_LRCK);
    Serial.printf("DIN GPIO   : %d\n", PIN_DIN);

    Serial.println();

    Serial.println(
        "Type 's' to start I2S receiver."
    );

    printHelp();
}

// =====================================================
// 16. Main loop
// =====================================================

void loop()
{
    handleSerial();

    if (!i2sRunning)
    {
        delay(2);
        return;
    }

    uint32_t t0 = millis();

    size_t bytesRead = I2S.readBytes(
        (char *)rxBuffer,
        RX_BYTES
    );

    uint32_t t1 = millis();

    lastReadBytes = (uint32_t)bytesRead;
    lastReadTime = t1 - t0;

    if (bytesRead == 0)
    {
        delay(1);
    }
    else
    {
        totalBytes += bytesRead;

        if (
            (bytesRead % (sizeof(uint32_t) * 2)) != 0
        )
        {
            shortReads++;
        }

        size_t words = bytesRead / sizeof(uint32_t);

        processPCM(words);
    }

    uint32_t now = millis();

    if (
        (uint32_t)(now - reportStart) >= REPORT_INTERVAL_MS
    )
    {
        printReport();
    }
}