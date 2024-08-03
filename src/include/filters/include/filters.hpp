#ifndef __FILTERS_HPP__
#define __FILTERS_HPP__
#include "filters_type.hpp"

/**
 * @brief Simple First Order Low Pass Filter
 * y[n] = α ⋅ x[n] + (1 − α) ⋅ y[n−1]
 * 
 * @param currentInput The current input value
 * @param previousOutput The previous output value
 * @param alpha The filter constant (value between 0 and 1)
 * @return float The filtered output value
 */
float lowPassFilter(float currentInput, float previousOutput, float alpha);

/**
 * @brief Simple First Order High Pass Filter
 * y[n] = α ⋅ (y[n−1] + x[n] − x[n−1])
 * 
 * @param currentInput The current input value
 * @param previousInput The previous input value
 * @param previousOutput The previous output value
 * @param alpha The filter constant (value between 0 and 1)
 * @return float The filtered output value
 */
float highPassFilter(float currentInput, float previousInput, float previousOutput, float alpha);

/**
 * @brief Sets the cutoff frequency for the 2-Pole Low Pass Filter
 * 
 * @param lpfData Pointer to the low pass filter data
 * @param sample_freq The sampling frequency
 * @param cutoff_freq The cutoff frequency
 */
void lpf2pSetCutoffFreq(lpf2pData* lpfData, float sample_freq, float cutoff_freq);

/**
 * @brief Initializes the 2-Pole Low Pass Filter
 * 
 * @param lpfData Pointer to the low pass filter data
 * @param sample_freq The sampling frequency
 * @param cutoff_freq The cutoff frequency
 */
void lpf2pInit(lpf2pData* lpfData, float sample_freq, float cutoff_freq);

/**
 * @brief Applies the 2-Pole Low Pass Filter to the given sample
 * 
 * @param lpfData Pointer to the low pass filter data
 * @param sample The input sample
 * @return float The filtered output value
 */
float lpf2pApply(lpf2pData* lpfData, float sample);

/**
 * @brief Resets the 2-Pole Low Pass Filter with the given sample
 * 
 * @param lpfData Pointer to the low pass filter data
 * @param sample The input sample
 * @return float The filtered output value
 */
float lpf2pReset(lpf2pData* lpfData, float sample);

#endif