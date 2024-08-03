#include <cmath>

#include "filters.hpp"

float lowPassFilter(float currentInput, float previousOutput, float alpha)
{
    return alpha * currentInput + (1.0f - alpha) * previousOutput;
}

float highPassFilter(float currentInput, float previousInput, float previousOutput, float alpha)
{
    return alpha * (previousOutput + currentInput - previousInput);
}

void lpf2pSetCutoffFreq(lpf2pData *lpfData, float sample_freq, float cutoff_freq)
{
    float fr = sample_freq / cutoff_freq;
    float ohm = tanf(M_PI / fr);
    float c = 1.0f + 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm;
    lpfData->b0 = ohm * ohm / c;
    lpfData->b1 = 2.0f * lpfData->b0;
    lpfData->b2 = lpfData->b0;
    lpfData->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
    lpfData->a2 = (1.0f - 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm) / c;
    lpfData->delay_element_1 = 0.0f;
    lpfData->delay_element_2 = 0.0f;
}

void lpf2pInit(lpf2pData *lpfData, float sample_freq, float cutoff_freq)
{
    if (lpfData == NULL || cutoff_freq <= 0.0f)
    {
        return;
    }
    lpf2pSetCutoffFreq(lpfData, sample_freq, cutoff_freq);
}

float lpf2pApply(lpf2pData *lpfData, float sample)
{
    float delay_element_0 = sample - lpfData->delay_element_1 * lpfData->a1 - lpfData->delay_element_2 * lpfData->a2;
    if (!std::isfinite(delay_element_0))
    {
        delay_element_0 = sample;
    }

    float output = delay_element_0 * lpfData->b0 + lpfData->delay_element_1 * lpfData->b1 + lpfData->delay_element_2 * lpfData->b2;

    lpfData->delay_element_2 = lpfData->delay_element_1;
    lpfData->delay_element_1 = delay_element_0;

    return output;
}

float lpf2pReset(lpf2pData *lpfData, float sample)
{
    float dval = sample / (lpfData->b0 + lpfData->b1 + lpfData->b2);
    lpfData->delay_element_1 = dval;
    lpfData->delay_element_2 = dval;
    return lpf2pApply(lpfData, sample);
}