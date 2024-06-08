#ifndef __NUM_TOOLS_HPP__
#define __NUM_TOOLS_HPP__

/**
 * @brief Maps a value from one range to another.
 *
 * @param data The input value to be mapped.
 * @param in_min The minimum value of the input range.
 * @param in_max The maximum value of the input range.
 * @param out_min The minimum value of the output range.
 * @param out_max The maximum value of the output range.
 * @return The mapped value in the output range.
 */
double mapValue(double data, double in_min, 
                double in_max, double out_min, double out_max);

/**
 * @brief Constrain a value within a specified range.
 *
 * @param value The value to be constrained.
 * @param min_value The minimum allowed value (inclusive).
 * @param max_value The maximum allowed value (inclusive).
 * @return The constrained value within the specified range.
 */
float constrainValue(float value, const float min_value, const float max_value);

#endif
