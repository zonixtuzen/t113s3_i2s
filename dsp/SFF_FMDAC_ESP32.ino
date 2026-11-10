#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <I2S.h>               // Arduino-ESP32 Core 3.x 现代 I2S 库
#include "SFFAudioDSP.h"

// --- Keyes 2.8寸 ESP32-32E 模组硬件引脚定义 ---
#define SD_CS_PIN    5         // SD卡片选信号
#define SD_SCK_PIN   18        // SPI 时钟
#define SD_MISO_PIN  19        // SPI MISO
#define SD_MOSI_PIN  23        // SPI MOSI
#define AMP_EN_PIN   4         // 音频功放使能：低电平有效

// Core 3.x I2S 音频总线引脚定义
#define I2S_BCLK_PIN 26        // BCLK / 音频 DAC 兼容输出引脚
#define I2S_WS_PIN   25        // WS / 声道切换时钟
#define I2S_DOUT_PIN 22        // 数据输出引脚

File wavFile;
bool isPlaying = false;
SFFAudioDSP* audioDSP = nullptr;

double sampleRate = 44100.0;
uint16_t bitsPerSample = 16;
uint16_t channels = 2;
uint32_t dataChunkSize = 0;
uint32_t bytesReadTotal = 0;

// 解析标准 WAV 文件头
bool parseWavHeader(File &file) {
    char buffer[44];
    if (file.read((uint8_t*)buffer, 44) < 44) return false;

    if (strncmp(buffer, "RIFF", 4) != 0 || strncmp(buffer + 8, "WAVE", 4) != 0) {
        Serial.println("Error: Not a valid WAV file format!");
        return false;
    }

    channels = *((uint16_t*)&buffer[22]);
    sampleRate = *((uint32_t*)&buffer[24]);
    bitsPerSample = *((uint16_t*)&buffer[34]);
    dataChunkSize = *((uint32_t*)&buffer[40]);

    Serial.printf("WAV Info -> SampleRate: %.0f Hz, Channels: %d, Bits: %d\n", sampleRate, channels, bitsPerSample);
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n--- SFF-FMDAC ESP32 (Core 3.3.5) Startup ---");

    // 1. 初始化板载功放使能引脚 (输出 LOW 使能功放发声)
    pinMode(AMP_EN_PIN, OUTPUT);
    digitalWrite(AMP_EN_PIN, LOW);
    Serial.println("Audio Amplifier Enabled (IO4 LOW)");

    // 2. 初始化 SD 卡 SPI 总线
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    if (!SD.begin(SD_CS_PIN, SPI, 40000000)) {
        Serial.println("SD Card Mount Failed! Check wiring or card format.");
        return;
    }
    Serial.println("SD Card Mounted Successfully.");

    // 3. 打开 SD 卡根目录下的测试音频文件 (请确保存入 /test.wav)
    wavFile = SD.open("/test.wav", FILE_READ);
    if (!wavFile) {
        Serial.println("Failed to open /test.wav on SD card!");
        return;
    }

    // 4. 解析 WAV 头信息
    if (!parseWavHeader(wavFile)) {
        wavFile.close();
        return;
    }

    // 5. 初始化 DSP 引擎 (传入原始采样率)
    audioDSP = new SFFAudioDSP(sampleRate);
    audioDSP->setMode(ApodizingPreFIR::NATURAL_AIR); // 可选 NATURAL_AIR 或 BRILLIANCE 档位

    // 6. 初始化 Arduino-ESP32 Core 3.x 现代 I2S 驱动
    // 插值 4× 后的目标输出采样率
    int outputSR = (int)(sampleRate * 4.0);
    
    I2S.setPins(I2S_BCLK_PIN, I2S_WS_PIN, I2S_DOUT_PIN);
    if (!I2S.begin(I2S_MODE_STD, outputSR, 16)) {
        Serial.println("Failed to initialize Core 3.x I2S bus!");
        return;
    }
    Serial.printf("Core 3.x I2S initialized successfully at %d Hz\n", outputSR);

    bytesReadTotal = 0;
    isPlaying = true;
}

void loop() {
    if (!isPlaying || !wavFile) return;

    uint8_t sampleBuffer[6];
    size_t bytesNeeded = (bitsPerSample == 24) ? (3 * channels) : (2 * channels);
    
    // 如果播完则循环重置
    if (wavFile.available() < bytesNeeded || bytesReadTotal >= dataChunkSize) {
        Serial.println("Playback finished. Looping track...");
        wavFile.seek(44); // 跳过 WAV 头回到数据区
        bytesReadTotal = 0;
        audioDSP->reset();
        return;
    }

    size_t readBytes = wavFile.read(sampleBuffer, bytesNeeded);
    bytesReadTotal += readBytes;

    float inL = 0.0f, inR = 0.0f;

    // 解析 16-bit 或 24-bit PCM
    if (bitsPerSample == 16) {
        int16_t rawL = *((int16_t*)&sampleBuffer[0]);
        inL = (float)rawL / 32768.0f;
        if (channels == 2) {
            int16_t rawR = *((int16_t*)&sampleBuffer[2]);
            inR = (float)rawR / 32768.0f;
        } else {
            inR = inL; // 单声道复制到双声道
        }
    } 
    else if (bitsPerSample == 24) {
        int32_t valL = (sampleBuffer[0] << 8) | (sampleBuffer[1] << 16) | (sampleBuffer[2] << 24);
        inL = (float)(valL >> 8) / 8388608.0f;
        if (channels == 2) {
            int32_t valR = (sampleBuffer[3] << 8) | (sampleBuffer[4] << 16) | (sampleBuffer[5] << 24);
            inR = (float)(valR >> 8) / 8388608.0f;
        } else {
            inR = inL;
        }
    }

    // 核心 DSP 流水线：1x 样点 -> 4x 经 Catmull-Rom 插值样点
    float outL4[4], outR4[4];
    audioDSP->process1xTo4x(inL, inR, outL4, outR4);

    // 通过 Core 3.x 的 I2S 接口以帧形式写入立体声数据
    for (int i = 0; i < 4; ++i) {
        int16_t dacL = (int16_t)(outL4[i] * 32767.0f);
        int16_t dacR = (int16_t)(outR4[i] * 32767.0f);

        int16_t frame[2] = {dacL, dacR};
        I2S.write(frame, sizeof(frame));
    }
}