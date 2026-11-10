#ifndef SFF_AUDIO_DSP_H
#define SFF_AUDIO_DSP_H

#include "ApodizingPreFIR.h"
#include "CatmullRom4x.h"

class SFFAudioDSP {
public:
    SFFAudioDSP(double sampleRate = 44100.0) : preFIR(sampleRate) {}

    void setMode(ApodizingPreFIR::Mode mode, const ApodizingPreFIR::Config* config = nullptr) {
        preFIR.setMode(mode, config);
    }

    void reset() {
        preFIR.reset();
        crL.reset();
        crR.reset();
    }

    void process1xTo4x(float inL, float inR, float outL4[4], float outR4[4]) {
        float preL, preR;
        preFIR.process(inL, inR, preL, preR);

        float tempL[4], tempR[4];
        crL.process(preL, tempL);
        crR.process(preR, tempR);

        for (int i = 0; i < 4; ++i) {
            outL4[i] = tempL[i];
            outR4[i] = tempR[i];
        }
    }

private:
    ApodizingPreFIR preFIR;
    CatmullRom4x crL;
    CatmullRom4x crR;
};

#endif