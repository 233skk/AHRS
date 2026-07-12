/**
 * @file    iekf.h
 * @brief   Left-Invariant EKF on SO(3) — 6轴IMU姿态估计
 *
 * 使用左不变误差的EKF，状态6维（姿态误差3 + 陀螺偏置3）。
 * 参考文献：Barrau & Bonnabel (2016)
 */

#ifndef IEKF_FILTER_H
#define IEKF_FILTER_H

#include "fusion_interface.h"

class IEKFFilter : public FusionAlgorithm {
public:
    IEKFFilter(float sigma_gyro  = 0.05f,
               float sigma_accel = 0.5f,
               float sigma_bias  = 0.0003f,
               float gain        = 2.0f,
               float sigma_mag   = 0.1f,
               float dt          = 0.005f);

    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float dt) override;

    void getQuaternion(float& w, float& x, float& y, float& z) const override;
    void getEuler(float& roll, float& pitch, float& yaw) const override;
    void reset() override;
    const char* name() const override { return "IEKF"; }

    /// 磁力计测量更新 — 使 yaw 可观
    void updateMag(float mx, float my, float mz);

    void getBias(float& bx, float& by, float& bz) const {
        bx = bx_; by = by_; bz = bz_;
    }
    float getAttitudeVariance() const {
        return P_[0] + P_[7] + P_[14];
    }

private:
    float qw_, qx_, qy_, qz_;       // 单位四元数
    float bx_, by_, bz_;            // 陀螺偏置估计
    float sigma_gyro_, sigma_accel_, sigma_bias_, sigma_mag_, gain_;
    float P_[36];                   // 6×6 协方差
    bool  initialized_;

    // 磁力计参考方向 (世界系), 首次调用 updateMag 时自动标定
    float mw_x_, mw_y_, mw_z_;
    bool  mag_ref_set_;

    void predict(float gx, float gy, float gz, float dt, float q_scale = 1.0f);
    void updateAccel(float ax, float ay, float az);
    void initFromAccel(float ax, float ay, float az);
    void applyCorrection(float dxi_x, float dxi_y, float dxi_z,
                         float dze_x, float dze_y, float dze_z);
};

#endif
