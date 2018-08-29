#ifndef __NOTCHFILTER_H__
#define __NOTCHFILTER_H__

#include <AP_Math.h>
#include <cmath>
#include <inttypes.h>

template <class T>
class NotchFilter {
public:
    void init(float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB);
    T apply(const T &sample);

private:
    bool initialised;
    float b0, b1, b2, a1, a2, a0_inv;
    T ntchsig, ntchsig1, ntchsig2, signal2, signal1;
};


typedef NotchFilter<float> NotchFilterFloat;
typedef NotchFilter<Vector3f> NotchFilterVector3f;

#endif
