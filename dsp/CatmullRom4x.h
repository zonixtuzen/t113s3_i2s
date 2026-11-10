#ifndef CATMULL_ROM_4X_H
#define CATMULL_ROM_4X_H

#include <algorithm>

class CatmullRom4x {
public:
    CatmullRom4x() { reset(); }

    void reset() { std::fill(history, history + 4, 0.0f); }

    void process(float newSample, float output4x[4]) {
        shiftHistory(newSample);
        output4x[0] = interpolateCatmullRom(0.0f);
        output4x[1] = interpolateCatmullRom(0.25f);
        output4x[2] = interpolateCatmullRom(0.50f);
        output4x[3] = interpolateCatmullRom(0.75f);
    }

private:
    float history[4];

    inline void shiftHistory(float newSample) {
        history[0] = history[1];
        history[1] = history[2];
        history[2] = history[3];
        history[3] = newSample;
    }

    inline float interpolateCatmullRom(float t) {
        float t2 = t * t;
        float t3 = t2 * t;
        return 0.5f * (
            (2.0f * history[1]) +
            (-history[0] + history[2]) * t +
            (2.0f * history[0] - 5.0f * history[1] + 4.0f * history[2] - history[3]) * t2 +
            (-history[0] + 3.0f * history[1] - 3.0f * history[2] + history[3]) * t3
        );
    }
};

#endif