#ifndef APODIZING_PRE_FIR_H
#define APODIZING_PRE_FIR_H

#include <cmath>
#include <algorithm>

class ApodizingPreFIR {
public:
    enum Mode { BYPASS, NATURAL_AIR, BRILLIANCE, ADVANCED };

    struct Config {
        float freq;
        float gainDb;
        float q;
        float asymmetry;
    };

    ApodizingPreFIR(double sampleRate = 44100.0) : sampleRate(sampleRate), bufIdx(0), isBypass(true), currentMode(BYPASS) {
        std::fill(bufferL, bufferL + N, 0.0f);
        std::fill(bufferR, bufferR + N, 0.0f);
        std::fill(currentTaps, currentTaps + N, 0.0);
        currentTaps[N / 2] = 1.0;
        setMode(NATURAL_AIR);
    }

    void setMode(Mode mode, const Config* advConfig = nullptr) {
        currentMode = mode;
        switch (mode) {
            case BYPASS:
                isBypass = true;
                std::fill(currentTaps, currentTaps + N, 0.0);
                currentTaps[N / 2] = 1.0;
                break;
            case NATURAL_AIR:
                isBypass = false;
                generateApodizingTaps({10000.0f, 1.8f, 0.5f, 0.0f});
                break;
            case BRILLIANCE:
                isBypass = false;
                generateApodizingTaps({8000.0f, 2.6f, 0.55f, 0.15f});
                break;
            case ADVANCED:
                isBypass = false;
                if (advConfig) advancedConfig = *advConfig;
                else advancedConfig = {10000.0f, 1.8f, 0.5f, 0.0f};
                generateApodizingTaps(advancedConfig);
                break;
        }
    }

    void reset() {
        std::fill(bufferL, bufferL + N, 0.0f);
        std::fill(bufferR, bufferR + N, 0.0f);
        bufIdx = 0;
    }

    void process(float sampleL, float sampleR, float &outL, float &outR) {
        float inL = sampleL * 0.85f;
        float inR = sampleR * 0.85f;

        if (isBypass) {
            outL = inL;
            outR = inR;
            return;
        }

        bufferL[bufIdx] = inL;
        bufferR[bufIdx] = inR;

        double sumL = 0.0, sumR = 0.0;
        for (int i = 0; i < N; ++i) {
            int readIdx = (bufIdx - i + N) % N;
            sumL += bufferL[readIdx] * currentTaps[i];
            sumR += bufferR[readIdx] * currentTaps[i];
        }

        bufIdx = (bufIdx + 1) % N;
        outL = (float)sumL;
        outR = (float)sumR;
    }

private:
    static const int N = 31;
    double sampleRate;
    float bufferL[N];
    float bufferR[N];
    int bufIdx;
    double currentTaps[N];
    bool isBypass;
    Mode currentMode;
    Config advancedConfig;

    void generateApodizingTaps(const Config &config) {
        if (config.gainDb == 0.0f) {
            isBypass = true;
            return;
        }

        double A = std::pow(10.0, config.gainDb / 40.0);
        double w0 = 2.0 * M_PI * config.freq / sampleRate;
        double alpha = std::sin(w0) / (2.0 * config.q);

        double b0 = A * ((A + 1.0) + (A - 1.0) * std::cos(w0) + 2.0 * std::sqrt(A) * alpha);
        double b1 = -2.0 * A * ((A - 1.0) + (A + 1.0) * std::cos(w0));
        double b2 = A * ((A + 1.0) + (A - 1.0) * std::cos(w0) - 2.0 * std::sqrt(A) * alpha);
        double a0 = (A + 1.0) - (A - 1.0) * std::cos(w0) + 2.0 * std::sqrt(A) * alpha;
        double a1 = 2.0 * ((A - 1.0) - (A + 1.0) * std::cos(w0));
        double a2 = (A + 1.0) - (A - 1.0) * std::cos(w0) - 2.0 * std::sqrt(A) * alpha;

        double nb0 = b0 / a0, nb1 = b1 / a0, nb2 = b2 / a0;
        double na1 = a1 / a0, na2 = a2 / a0;

        auto magnitude = [&](double w) {
            double cosW = std::cos(w), cos2W = std::cos(2.0 * w);
            double sinW = std::sin(w), sin2W = std::sin(2.0 * w);
            double numRe = nb0 + nb1 * cosW + nb2 * cos2W;
            double numIm = -(nb1 * sinW + nb2 * sin2W);
            double denRe = 1.0 + na1 * cosW + na2 * cos2W;
            double denIm = -(na1 * sinW + na2 * sin2W);
            return std::sqrt((numRe * numRe + numIm * numIm) / (denRe * denRe + denIm * denIm));
        };

        auto idealH = [&](double t) {
            int steps = 250;
            double dw = M_PI / steps;
            double sum = 0.0;
            for (int i = 0; i < steps; ++i) {
                double w = (i + 0.5) * dw;
                sum += magnitude(w) * std::cos(w * t);
            }
            return (sum * dw) / M_PI;
        };

        double centerPeak = (N - 1) / 2.0;
        double peakPos = centerPeak * (1.0 - config.asymmetry * 2.0);

        double dcSum = 0.0;
        double newTaps[N];

        for (int i = 0; i < N; ++i) {
            double t = (double)i - peakPos;
            double window = (i <= peakPos) ? 
                ((peakPos <= 0.0) ? 1.0 : 0.5 * (1.0 - std::cos(M_PI * i / peakPos))) : 
                ((peakPos >= (N - 1.0)) ? 1.0 : 0.5 * (1.0 - std::cos(M_PI * (N - 1.0 - i) / (N - 1.0 - peakPos))));

            newTaps[i] = idealH(t) * window;
            dcSum += newTaps[i];
        }

        for (int i = 0; i < N; ++i) {
            currentTaps[i] = newTaps[i] / dcSum;
        }
    }
};

#endif