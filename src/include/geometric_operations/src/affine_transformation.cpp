#include "geometric_operations.hpp"

Eigen::Matrix3f R_x(float phi)
{
    Eigen::Matrix3f rx;
    rx << 1, 0, 0,
        0, cos(phi), -sin(phi),
        0, sin(phi), cos(phi);
    return rx;
}

Eigen::Matrix3f R_y(float beta)
{
    Eigen::Matrix3f ry;
    ry << cos(beta), 0, sin(beta),
        0, 1, 0,
        -sin(beta), 0, cos(beta);
    return ry;
}

Eigen::Matrix3f R_z(float theta)
{
    Eigen::Matrix3f rz;
    rz << cos(theta), -sin(theta), 0,
        sin(theta), cos(theta), 0,
        0, 0, 1;
    return rz;
}

Eigen::Matrix3f rotationMatrix(float angle[])
{
    return R_z(angle[YAW]) * R_y(angle[PITCH]) * R_x(angle[ROLL]);
}

Eigen::Matrix3f rotationMatrix(float roll, float pitch, float yaw)
{
    return R_z(yaw) * R_y(pitch) * R_x(roll);
}

Eigen::Matrix4f translationMatrix(float state[])
{
    Eigen::Matrix4f t;
    t << 1, 0, 0, state[0],   // X
        0, 1, 0, state[1],    // Y
        0, 0, 1, state[2],    // Z
        0, 0, 0, 1;
    return t;
}

Eigen::Matrix4f translationMatrix(float x, float y, float z)
{
    Eigen::Matrix4f t;
    t << 1, 0, 0, x,
        0, 1, 0, y,
        0, 0, 1, z,
        0, 0, 0, 1;
    return t;
}

Eigen::Matrix4f scaleMatrix(float s_x, float s_y, float s_z)
{
    Eigen::Matrix4f S;
    S << s_x, 0, 0, 0,
        0, s_y, 0, 0,
        0, 0, s_z, 0;
    0, 0, 0, 1;
    return S;
}

Eigen::Matrix4f scaleMatrix(float scale[])
{
    Eigen::Matrix4f S;
    S << scale[0], 0, 0, 0,   // X
        0, scale[1], 0, 0,    // Y
        0, 0, scale[2], 0,    // Z
        0, 0, 0, 1;
    return S;
}

Eigen::Matrix4f extendRotationMatrix(const Eigen::Matrix3f &rotationMatrix)
{
    Eigen::Matrix4f extendedMatrix = Eigen::Matrix4f::Identity();
    extendedMatrix.block<3, 3>(0, 0) = rotationMatrix;
    return extendedMatrix;
}

void transformation(float out[], float angle[], float state[])
{
    Eigen::Matrix<float, 4, 1> input;
    Eigen::Matrix<float, 4, 1> result;

    Eigen::Matrix3f rotation_matrix = rotationMatrix(angle);
    Eigen::Matrix4f translation_matrix = translationMatrix(state);

    Eigen::Matrix4f extended_rotationMatrix = extendRotationMatrix(rotation_matrix);

    input << out[0], out[1], out[2], 1.0;
    result = translation_matrix * extended_rotationMatrix * input;

    out[0] = result(0, 0);
    out[1] = result(1, 0);
    out[2] = result(2, 0);
}
