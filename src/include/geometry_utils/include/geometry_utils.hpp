#ifndef __GEOMETRY_UTILS_HPP__
#define __GEOMETRY_UTILS_HPP__

#include <cmath>
#include <Eigen/Dense>

#define F2P(x) (1000 / x)
#define DEG2RAD (M_PI / 180.0f)
#define RAD2DEG (180.0f / M_PI)

typedef enum
{
    X,
    Y,
    Z,
    ALL_CC
} Cartesian_coordinate_e;

typedef enum
{
    RADIUS,
    THETA,
    PHI,
    ALL_SC
} Cylindrical_coordinate_e;

typedef enum
{
    ROLL,
    PITCH,
    YAW,
    R_ALL
} Rotation_angle_t;

/**
 * @brief Generates a rotation matrix for a rotation around the X-axis.
 *
 * @param phi The rotation angle in radians.
 * @return A 3x3 rotation matrix for a rotation around the X-axis.
 */
Eigen::Matrix3f R_x(float phi);

/**
 * @brief Generates a rotation matrix for a rotation around the Y-axis.
 *
 * @param beta The rotation angle in radians.
 * @return A 3x3 rotation matrix for a rotation around the Y-axis.
 */
Eigen::Matrix3f R_y(float beta);

/**
 * @brief Generates a rotation matrix for a rotation around the Z-axis.
 *
 * @param theta The rotation angle in radians.
 * @return A 3x3 rotation matrix for a rotation around the Z-axis.
 */
Eigen::Matrix3f R_z(float theta);

/**
 * @brief Calculates the combined rotation matrix from an array of Euler angles.
 *
 * @param angle An array of three floats representing the Euler angles
 * (in radians) in the order: [ROLL, PITCH, YAW].
 * @return A 3x3 rotation matrix representing the combined rotation.
 */
Eigen::Matrix3f calculateRotationMatrix(float angle[]);

/**
 * @brief Calculates the combined rotation matrix from individual Euler angles.
 *
 * @param roll The rotation angle around the X-axis (in radians).
 * @param pitch The rotation angle around the Y-axis (in radians).
 * @param yaw The rotation angle around the Z-axis (in radians).
 * @return A 3x3 rotation matrix representing the combined rotation.
 */
Eigen::Matrix3f calculateRotationMatrix(float roll, float pitch, float yaw);

/**
 * @brief Applies a rotation transformation to a 3D vector using Euler angles.
 *
 * @param[out] out An array of three float pointers representing the 3D vector
 * to be transformed. The array should contain pointers to three float values.
 * @param[in] angle An array of three floats representing the Euler angles
 * (in radians) in the order: [ROLL, PITCH, YAW].
 */
void rotasyonTransformation(float *out[], float angle[]);

/**
 * @brief Applies a rotation transformation to a 3D vector using roll, pitch, and yaw angles.
 *
 * @param[out] out An array of three float pointers representing the 3D vector
 * to be transformed. The array should contain pointers to three float values.
 * @param[in] roll The roll angle (in radians) for the rotation.
 * @param[in] pitch The pitch angle (in radians) for the rotation.
 * @param[in] yaw The yaw angle (in radians) for the rotation.
 */
void rotasyonTransformation(float *out[], float roll, float pitch, float yaw);

/**
 * @brief Converts Cartesian coordinates to spherical coordinates.
 *
 * @param[in] cart_data A pointer to an array of three floats representing the
 *                      cartesian coordinates (x, y, z).
 * @param[out] spe_data A pointer to an array of three floats where the spherical
 *                      coordinates (radius, theta, phi) will be stored.
 */
void cartesian2Spherical(float *cart_data, float *spe_data);

/**
 * @brief Converts spherical coordinates to Cartesian coordinates.
 *
 * @param[in] spe_data A pointer to an array of three floats representing the
 *                      spherical coordinates (radius, theta, phi).
 * @param[out] cart_data A pointer to an array of three floats where the Cartesian
 *                      coordinates (x, y, z) will be stored.
 */
void spherical2Cartesian(float *spe_data, float *cart_data);

/**
 * @brief Constrains the given angle to a specified range.
 *
 * @param angle The angle to be normalized.
 * @param min_angle The minimum angle of the range. Valid values are 0 and -180.
 * @return The normalized angle constrained to the specified range.
 *
 * @note If min_angle is neither 0 nor -180, it will be set to 0.
 */
float constrainAngle(float angle = 0, float min_angle = 0);

#endif