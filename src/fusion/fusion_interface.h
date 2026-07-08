/**
 * @file    fusion_interface.h
 * @brief   融合层：姿态解算算法统一接口
 */

#ifndef FUSION_INTERFACE_H
#define FUSION_INTERFACE_H

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

class FusionAlgorithm {
public:
    virtual ~FusionAlgorithm() = default;

    virtual void update(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float dt) = 0;

    virtual void getQuaternion(float& w, float& x, float& y, float& z) const = 0;
    virtual void getEuler(float& roll, float& pitch, float& yaw) const = 0;
    virtual void reset() = 0;
    virtual const char* name() const = 0;
    
    static float radToDeg(float rad) { return rad * 180.0f / M_PI; }
    static float degToRad(float deg) { return deg * M_PI / 180.0f; }

    static void quatToEuler(float qw, float qx, float qy, float qz,
                            float& roll, float& pitch, float& yaw) {
        // Roll: 绕X轴
        float sinr = 2.0f * (qw * qx + qy * qz);
        float cosr = 1.0f - 2.0f * (qx * qx + qy * qy);
        roll = atan2f(sinr, cosr);

        // Pitch: 绕Y轴
        float sinp = 2.0f * (qw * qy - qz * qx);
        if (fabsf(sinp) >= 1.0f) {
            pitch = copysignf(M_PI / 2.0f, sinp);
        } else {
            pitch = asinf(sinp);
        }

        // Yaw: 绕Z轴
        float siny = 2.0f * (qw * qz + qx * qy);
        float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);
        yaw = atan2f(siny, cosy);
    }
};

#endif // FUSION_INTERFACE_H
