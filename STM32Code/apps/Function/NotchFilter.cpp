#include "NotchFilter.h"

float log2f_c(float x)
{
  return (logf(x) / M_LN2);
}

/*
  initialise filter
 */
template <class T>
void NotchFilter<T>::init(float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB)
{
    float omega = 2.0f * M_PI * center_freq_hz / sample_freq_hz;
    float octaves = log2f_c(center_freq_hz  / (center_freq_hz - bandwidth_hz/2)) * 2;
    float A = powf(10, -attenuation_dB/40);
    float Q = sqrtf(powf(2, octaves)) / (powf(2,octaves) - 1);
    float alpha = sinf(omega) / (2 * Q/A);
    b0 =  1.0f + alpha*A;
    b1 = -2.0f * cosf(omega);
    b2 =  1.0f - alpha*A;
    a0_inv =  1.0f/(1.0f + alpha/A);
    a1 = -2.0f * cosf(omega);
    a2 =  1.0f - alpha/A;
    initialised = true;
}

/*
  apply a new input sample, returning new output
 */
template <class T>
T NotchFilter<T>::apply(const T &sample)
{
    if (!initialised) {
        // if we have not been initialised when return the input
        // sample as output
        return sample;
    }
    ntchsig2 = ntchsig1;
    ntchsig1 = ntchsig;
    ntchsig = sample;
    T output = (ntchsig*b0 + ntchsig1*b1 + ntchsig2*b2 - signal1*a1 - signal2*a2) * a0_inv;
    signal2 = signal1;
    signal1 = output;
    return output;
}

/* 
   instantiate template classes
 */
template class NotchFilter<float>;
template class NotchFilter<Vector3f>;
