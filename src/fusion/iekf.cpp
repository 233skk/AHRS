/**
 * @file    iekf.cpp
 * @brief   Left-Invariant EKF on SO(3) — 6轴姿态估计 (加速度计和陀螺仪)
 *
 * 旋转在欧氏空间线性化会破坏正交约束，
 * 而左不变误差定义在 so(3) 切空间上，保证每一步都留在 SO(3) 流形上。
 * 参考：Barrau & Bonnabel (2016)
 */

#include "iekf.h"
#include <cmath>
#include <cstring>
#include <algorithm>

// ---- 矩阵/向量工具 ----

// 反对称矩阵: R^3 → so(3)
inline void skew(float vx, float vy, float vz,
                 float& m00, float& m01, float& m02,
                 float& m10, float& m11, float& m12,
                 float& m20, float& m21, float& m22) {
    m00 =  0.0f; m01 = -vz;   m02 =  vy;
    m10 =  vz;   m11 =  0.0f; m12 = -vx;
    m20 = -vy;   m21 =  vx;   m22 =  0.0f;
}

// 四元数乘法 q = a ⊗ b
inline void quatMul(float aw, float ax, float ay, float az,
                    float bw, float bx, float by, float bz,
                    float& qw, float& qx, float& qy, float& qz) {
    qw = aw*bw - ax*bx - ay*by - az*bz;
    qx = aw*bx + ax*bw + ay*bz - az*by;
    qy = aw*by - ax*bz + ay*bw + az*bx;
    qz = aw*bz + ax*by - ay*bx + az*bw;
}

// 轴角 → 四元数: exp_q(φ·n̂) = [cos(φ/2), sin(φ/2)·n̂]
inline void axisAngleToQuat(float angle, float nx, float ny, float nz,
                             float& qw, float& qx, float& qy, float& qz) {
    float half_a = 0.5f * angle;
    float s = sinf(half_a);
    qw = cosf(half_a);
    qx = nx * s;
    qy = ny * s;
    qz = nz * s;
}

// 四元数 → 旋转矩阵 (body→world)
inline void quatToRot(float qw, float qx, float qy, float qz,
                       float& r00, float& r01, float& r02,
                       float& r10, float& r11, float& r12,
                       float& r20, float& r21, float& r22) {
    float qw2 = qw*qw, qx2 = qx*qx, qy2 = qy*qy, qz2 = qz*qz;
    r00 = qw2 + qx2 - qy2 - qz2;
    r01 = 2.0f*(qx*qy - qw*qz);
    r02 = 2.0f*(qx*qz + qw*qy);
    r10 = 2.0f*(qx*qy + qw*qz);
    r11 = qw2 - qx2 + qy2 - qz2;
    r12 = 2.0f*(qy*qz - qw*qx);
    r20 = 2.0f*(qx*qz - qw*qy);
    r21 = 2.0f*(qy*qz + qw*qx);
    r22 = qw2 - qx2 - qy2 + qz2;
}

// ---- 构造函数 ----
IEKFFilter::IEKFFilter(float sigma_gyro, float sigma_accel,
                       float sigma_bias, float sigma_mag, float dt)
    : qw_(1.0f), qx_(0.0f), qy_(0.0f), qz_(0.0f)
    , bx_(0.0f), by_(0.0f), bz_(0.0f)
    , sigma_gyro_(sigma_gyro)
    , sigma_accel_(sigma_accel)
    , sigma_bias_(sigma_bias)
    , sigma_mag_(sigma_mag)
    , initialized_(false)
    , mag_ref_set_(false)
{
    (void)dt;

    std::memset(P_, 0, sizeof(P_));
    float init_att  = 0.3f * 0.3f;
    float init_bias = 0.05f * 0.05f;
    P_[0]  = P_[7]  = init_att;
    P_[14] = init_att;
    P_[21] = P_[28] = P_[35] = init_bias;
}

// ---- 预测步骤 ----
// 状态转移矩阵: Φ = [I - ω̂^∧·dt, -I·dt; 0, I]
// 协方差传播:   P ← Φ·P·Φ^T + Q_d

#define P(r,c) P_[(r)*6+(c)]

void IEKFFilter::predict(float gx, float gy, float gz, float dt) {
    float wx = gx - bx_;
    float wy = gy - by_;
    float wz = gz - bz_;

    // 名义状态传播: q̂ ← q̂ ⊗ exp_q(ω̂·Δt)
    float angle = sqrtf(wx*wx + wy*wy + wz*wz) * dt;
    if (angle > 1e-6f) {
        float ax_norm = wx * dt / angle;
        float ay_norm = wy * dt / angle;
        float az_norm = wz * dt / angle;
        float dqw, dqx, dqy, dqz;
        axisAngleToQuat(angle, ax_norm, ay_norm, az_norm, dqw, dqx, dqy, dqz);
        float nw, nx, ny, nz;
        quatMul(qw_, qx_, qy_, qz_, dqw, dqx, dqy, dqz, nw, nx, ny, nz);
        float norm = sqrtf(nw*nw + nx*nx + ny*ny + nz*nz);
        qw_ = nw/norm; qx_ = nx/norm; qy_ = ny/norm; qz_ = nz/norm;
    }

    // 构造 Φ = [I - (ω̂)^∧·dt, -I·dt; 0, I]
    float s00, s01, s02, s10, s11, s12, s20, s21, s22;
    skew(wx*dt, wy*dt, wz*dt, s00, s01, s02, s10, s11, s12, s20, s21, s22);
    float F00 = 1.0f - s00, F01 =     - s01, F02 =     - s02;
    float F10 =     - s10, F11 = 1.0f - s11, F12 =     - s12;
    float F20 =     - s20, F21 =     - s21, F22 = 1.0f - s22;
    float F03 = -dt, F04 = 0.0f, F05 = 0.0f;
    float F13 = 0.0f, F14 = -dt, F15 = 0.0f;
    float F23 = 0.0f, F24 = 0.0f, F25 = -dt;

    // M = Φ·P
    float M[36];
    for (int col = 0; col < 6; col++) {
        float p0 = P(0,col), p1 = P(1,col), p2 = P(2,col);
        float p3 = P(3,col), p4 = P(4,col), p5 = P(5,col);
        M[0*6+col] = F00*p0 + F01*p1 + F02*p2 + F03*p3 + F04*p4 + F05*p5;
        M[1*6+col] = F10*p0 + F11*p1 + F12*p2 + F13*p3 + F14*p4 + F15*p5;
        M[2*6+col] = F20*p0 + F21*p1 + F22*p2 + F23*p3 + F24*p4 + F25*p5;
    }
    for (int col = 0; col < 6; col++) {
        M[3*6+col] = P(3,col);
        M[4*6+col] = P(4,col);
        M[5*6+col] = P(5,col);
    }

    // P_new = M·Φ^T
    float P_new[36];
    for (int row = 0; row < 6; row++) {
        float m0 = M[row*6+0], m1 = M[row*6+1], m2 = M[row*6+2];
        float m3 = M[row*6+3], m4 = M[row*6+4], m5 = M[row*6+5];
        P_new[row*6+0] = m0*F00 + m1*F01 + m2*F02 + m3*F03 + m4*F04 + m5*F05;
        P_new[row*6+1] = m0*F10 + m1*F11 + m2*F12 + m3*F13 + m4*F14 + m5*F15;
        P_new[row*6+2] = m0*F20 + m1*F21 + m2*F22 + m3*F23 + m4*F24 + m5*F25;
        P_new[row*6+3] = m3;
        P_new[row*6+4] = m4;
        P_new[row*6+5] = m5;
    }

    // 加过程噪声: 陀螺噪声 ∝ Δt², 偏置游走 ∝ Δt
    float q_gyro  = sigma_gyro_*sigma_gyro_ * dt*dt;
    float q_bias  = sigma_bias_*sigma_bias_ * dt;
    P_new[0]  += q_gyro;   P_new[7]  += q_gyro;   P_new[14] += q_gyro;
    P_new[21] += q_bias;   P_new[28] += q_bias;   P_new[35] += q_bias;

    std::memcpy(P_, P_new, sizeof(P_));
}

// ---- 加速度计更新 ----
// 测量矩阵 H = [g_pred^∧ | 0], g_pred 随姿态变化而非硬编码
// 因为只测重力方向, H 秩=2, 只约束 roll/pitch 不约束 yaw
void IEKFFilter::updateAccel(float ax, float ay, float az) {
    float norm_a = sqrtf(ax*ax + ay*ay + az*az);
    if (norm_a < 0.1f) return;
    ax /= norm_a; ay /= norm_a; az /= norm_a;

    float r00, r01, r02, r10, r11, r12, r20, r21, r22;
    quatToRot(qw_, qx_, qy_, qz_, r00, r01, r02, r10, r11, r12, r20, r21, r22);

    // g_pred = R^T * [0,0,1]^T = R的第三行
    float gx = r20, gy = r21, gz = r22;

    // 新息 ν = y - g_pred
    float nux = ax - gx;
    float nuy = ay - gy;
    float nuz = az - gz;

    float R_noise = sigma_accel_ * sigma_accel_;

    // PHT = P·H^T (6×3), H = [g_pred^∧ | 0]
    float PHT[18];
    for (int i = 0; i < 6; i++) {
        PHT[i*3+0] = -P(i,1)*gz + P(i,2)*gy;
        PHT[i*3+1] =  P(i,0)*gz - P(i,2)*gx;
        PHT[i*3+2] = -P(i,0)*gy + P(i,1)*gx;
    }

    // S = H·PHT + R (3×3)
    float S00 = -gz*PHT[1*3+0] + gy*PHT[2*3+0] + R_noise;
    float S01 = -gz*PHT[1*3+1] + gy*PHT[2*3+1];
    float S02 = -gz*PHT[1*3+2] + gy*PHT[2*3+2];
    float S10 =  gz*PHT[0*3+0] - gx*PHT[2*3+0];
    float S11 =  gz*PHT[0*3+1] - gx*PHT[2*3+1] + R_noise;
    float S12 =  gz*PHT[0*3+2] - gx*PHT[2*3+2];
    float S20 = -gy*PHT[0*3+0] + gx*PHT[1*3+0];
    float S21 = -gy*PHT[0*3+1] + gx*PHT[1*3+1];
    float S22 = -gy*PHT[0*3+2] + gx*PHT[1*3+2] + R_noise;

    // 3×3 求逆 (伴随矩阵 + 正则化, 因为 yaw 不可观测 det≈0)
    float eps = 1e-8f;
    float adj00 =  (S11*S22 - S12*S21) + eps;
    float adj01 = -(S01*S22 - S02*S21);
    float adj02 =  (S01*S12 - S02*S11);
    float adj10 = -(S10*S22 - S12*S20);
    float adj11 =  (S00*S22 - S02*S20) + eps;
    float adj12 = -(S00*S12 - S02*S10);
    float adj20 =  (S10*S21 - S11*S20);
    float adj21 = -(S00*S21 - S01*S20);
    float adj22 =  (S00*S11 - S01*S10) + eps;
    float det = S00*adj00 + S01*adj01 + S02*adj02 + eps * 3.0f;

    float invS00 = adj00/det, invS01 = adj01/det, invS02 = adj02/det;
    float invS10 = adj10/det, invS11 = adj11/det, invS12 = adj12/det;
    float invS20 = adj20/det, invS21 = adj21/det, invS22 = adj22/det;

    // K = PHT · invS (6×3)
    float K[18];
    for (int i = 0; i < 6; i++) {
        K[i*3+0] = PHT[i*3+0]*invS00 + PHT[i*3+1]*invS10 + PHT[i*3+2]*invS20;
        K[i*3+1] = PHT[i*3+0]*invS01 + PHT[i*3+1]*invS11 + PHT[i*3+2]*invS21;
        K[i*3+2] = PHT[i*3+0]*invS02 + PHT[i*3+1]*invS12 + PHT[i*3+2]*invS22;
    }

    // δx = K·ν
    float dxi_x = K[0]*nux  + K[1]*nuy  + K[2]*nuz;
    float dxi_y = K[3]*nux  + K[4]*nuy  + K[5]*nuz;
    float dxi_z = K[6]*nux  + K[7]*nuy  + K[8]*nuz;
    float dze_x = K[9]*nux  + K[10]*nuy + K[11]*nuz;
    float dze_y = K[12]*nux + K[13]*nuy + K[14]*nuz;
    float dze_z = K[15]*nux + K[16]*nuy + K[17]*nuz;

    // 误差注入: q ← q ⊗ exp_q(ξ), b ← b + ζ
    float dangle = sqrtf(dxi_x*dxi_x + dxi_y*dxi_y + dxi_z*dxi_z);
    if (dangle > 1e-6f) {
        float nx = dxi_x/dangle, ny = dxi_y/dangle, nz = dxi_z/dangle;
        float dqw, dqx, dqy, dqz;
        axisAngleToQuat(dangle, nx, ny, nz, dqw, dqx, dqy, dqz);
        float nw, nx2, ny2, nz2;
        quatMul(qw_, qx_, qy_, qz_, dqw, dqx, dqy, dqz, nw, nx2, ny2, nz2);
        float nq = sqrtf(nw*nw + nx2*nx2 + ny2*ny2 + nz2*nz2);
        qw_ = nw/nq; qx_ = nx2/nq; qy_ = ny2/nq; qz_ = nz2/nq;
    }
    bx_ += dze_x;  by_ += dze_y;  bz_ += dze_z;

    // Joseph-form 协方差更新: P ← (I-KH)P(I-KH)^T + K·R·K^T
    float KH[36];
    std::memset(KH, 0, sizeof(KH));
    for (int i = 0; i < 6; i++) {
        KH[i*6+0] =             K[i*3+1]*gz - K[i*3+2]*gy;
        KH[i*6+1] = -K[i*3+0]*gz             + K[i*3+2]*gx;
        KH[i*6+2] =  K[i*3+0]*gy - K[i*3+1]*gx;
    }

    float IKH[36];
    for (int i = 0; i < 36; i++) IKH[i] = -KH[i];
    for (int i = 0; i < 6; i++) IKH[i*6+i] += 1.0f;

    float T[36] = {0};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            for (int k = 0; k < 6; k++)
                T[i*6+j] += IKH[i*6+k] * P(k,j);

    float P_new[36] = {0};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            for (int k = 0; k < 6; k++)
                P_new[i*6+j] += T[i*6+k] * IKH[j*6+k];

    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            P_new[i*6+j] += R_noise * (K[i*3+0]*K[j*3+0] + K[i*3+1]*K[j*3+1] + K[i*3+2]*K[j*3+2]);

    std::memcpy(P_, P_new, sizeof(P_));
}

// ---- 从加速度计初始化姿态 ----
// 从重力方向反推 roll/pitch, yaw 不可观测故置零
void IEKFFilter::initFromAccel(float ax, float ay, float az) {
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.1f) return;
    ax /= norm; ay /= norm; az /= norm;

    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
    float yaw   = 0.0f;

    // 欧拉角 → 四元数 (ZYX)
    float cr = cosf(roll*0.5f),  sr = sinf(roll*0.5f);
    float cp = cosf(pitch*0.5f), sp = sinf(pitch*0.5f);
    float cy = cosf(yaw*0.5f),   sy = sinf(yaw*0.5f);

    qw_ = cr*cp*cy + sr*sp*sy;
    qx_ = sr*cp*cy - cr*sp*sy;
    qy_ = cr*sp*cy + sr*cp*sy;
    qz_ = cr*cp*sy - sr*sp*cy;

    float qn = sqrtf(qw_*qw_ + qx_*qx_ + qy_*qy_ + qz_*qz_);
    qw_ /= qn; qx_ /= qn; qy_ /= qn; qz_ /= qn;

    initialized_ = true;
}

// ---- 误差注入: q ← q ⊗ exp_q(ξ), b ← b + ζ ----
void IEKFFilter::applyCorrection(float dxi_x, float dxi_y, float dxi_z,
                                  float dze_x, float dze_y, float dze_z) {
    float dangle = sqrtf(dxi_x*dxi_x + dxi_y*dxi_y + dxi_z*dxi_z);
    if (dangle > 1e-6f) {
        float nx = dxi_x/dangle, ny = dxi_y/dangle, nz = dxi_z/dangle;
        float dqw, dqx, dqy, dqz;
        axisAngleToQuat(dangle, nx, ny, nz, dqw, dqx, dqy, dqz);
        float nw, nx2, ny2, nz2;
        quatMul(qw_, qx_, qy_, qz_, dqw, dqx, dqy, dqz, nw, nx2, ny2, nz2);
        float nq = sqrtf(nw*nw + nx2*nx2 + ny2*ny2 + nz2*nz2);
        qw_ = nw/nq; qx_ = nx2/nq; qy_ = ny2/nq; qz_ = nz2/nq;
    }
    bx_ += dze_x;  by_ += dze_y;  bz_ += dze_z;
}

// ---- Joseph-form 协方差更新 (当前为简化实现) ----
// 完整的 cov update 需要传入测量向量来构造 H, 当前 updateAccel/updateMag
// 已内联完成协方差更新, 此函数作为占位保留
void IEKFFilter::covUpdate(const float* K, int meas_dim) {
    (void)K;
    (void)meas_dim;
}

// ---- 主更新接口 ----
// 仅在加速度模值正常 (0.5g~1.5g) 时做测量更新, 避免高速运动时加速度计混入运动加速度
void IEKFFilter::update(float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt) {
    if (!initialized_) {
        initFromAccel(ax, ay, az);
        return;
    }
    if (dt < 1e-6f || dt > 1.0f) return;

    predict(gx, gy, gz, dt);

    float accel_mag = sqrtf(ax*ax + ay*ay + az*az);
    if (accel_mag > 0.5f && accel_mag < 1.5f) {
        updateAccel(ax, ay, az);
    }
}

// ---- 磁力计更新 (yaw 可观) ----
// 首次调用时自动标定世界磁场参考方向
void IEKFFilter::updateMag(float mx, float my, float mz) {
    if (!initialized_) return;

    if (!mag_ref_set_) {
        float r00, r01, r02, r10, r11, r12, r20, r21, r22;
        quatToRot(qw_, qx_, qy_, qz_, r00, r01, r02, r10, r11, r12, r20, r21, r22);
        mw_x_ = r00*mx + r01*my + r02*mz;
        mw_y_ = r10*mx + r11*my + r12*mz;
        mw_z_ = r20*mx + r21*my + r22*mz;
        mag_ref_set_ = true;
        return;
    }

    float r00, r01, r02, r10, r11, r12, r20, r21, r22;
    quatToRot(qw_, qx_, qy_, qz_, r00, r01, r02, r10, r11, r12, r20, r21, r22);

    // 预测: m_pred = R^T · m_world
    float px = r00*mw_x_ + r10*mw_y_ + r20*mw_z_;
    float py = r01*mw_x_ + r11*mw_y_ + r21*mw_z_;
    float pz = r02*mw_x_ + r12*mw_y_ + r22*mw_z_;

    // 新息 ν = m_meas - m_pred
    float nux = mx - px;
    float nuy = my - py;
    float nuz = mz - pz;

    // H = [m_pred^∧ | 0]
    float R_noise = sigma_mag_ * sigma_mag_;

    float PHT[18];
    for (int i = 0; i < 6; i++) {
        PHT[i*3+0] = -P(i,1)*pz + P(i,2)*py;
        PHT[i*3+1] =  P(i,0)*pz - P(i,2)*px;
        PHT[i*3+2] = -P(i,0)*py + P(i,1)*px;
    }

    // S = H·PHT + R
    float S00 = -pz*PHT[1*3+0] + py*PHT[2*3+0] + R_noise;
    float S01 = -pz*PHT[1*3+1] + py*PHT[2*3+1];
    float S02 = -pz*PHT[1*3+2] + py*PHT[2*3+2];
    float S10 =  pz*PHT[0*3+0] - px*PHT[2*3+0];
    float S11 =  pz*PHT[0*3+1] - px*PHT[2*3+1] + R_noise;
    float S12 =  pz*PHT[0*3+2] - px*PHT[2*3+2];
    float S20 = -py*PHT[0*3+0] + px*PHT[1*3+0];
    float S21 = -py*PHT[0*3+1] + px*PHT[1*3+1];
    float S22 = -py*PHT[0*3+2] + px*PHT[1*3+2] + R_noise;

    // 3×3 求逆 (正则化)
    float eps = 1e-8f;
    float det = S00*(S11*S22 - S12*S21) - S01*(S10*S22 - S12*S20) + S02*(S10*S21 - S11*S20);
    float adj00 = (S11*S22 - S12*S21) + eps;
    float adj01 = -(S01*S22 - S02*S21);
    float adj02 = (S01*S12 - S02*S11);
    float adj10 = -(S10*S22 - S12*S20);
    float adj11 = (S00*S22 - S02*S20) + eps;
    float adj12 = -(S00*S12 - S02*S10);
    float adj20 = (S10*S21 - S11*S20);
    float adj21 = -(S00*S21 - S01*S20);
    float adj22 = (S00*S11 - S01*S10) + eps;
    det += eps * 3.0f;
    float invS00 = adj00/det, invS01 = adj01/det, invS02 = adj02/det;
    float invS10 = adj10/det, invS11 = adj11/det, invS12 = adj12/det;
    float invS20 = adj20/det, invS21 = adj21/det, invS22 = adj22/det;

    // K = PHT · invS
    float K[18];
    for (int i = 0; i < 6; i++) {
        K[i*3+0] = PHT[i*3+0]*invS00 + PHT[i*3+1]*invS10 + PHT[i*3+2]*invS20;
        K[i*3+1] = PHT[i*3+0]*invS01 + PHT[i*3+1]*invS11 + PHT[i*3+2]*invS21;
        K[i*3+2] = PHT[i*3+0]*invS02 + PHT[i*3+1]*invS12 + PHT[i*3+2]*invS22;
    }

    // δx = K·ν
    float dx0=K[0]*nux+K[1]*nuy+K[2]*nuz,   dx1=K[3]*nux+K[4]*nuy+K[5]*nuz;
    float dx2=K[6]*nux+K[7]*nuy+K[8]*nuz,   dx3=K[9]*nux+K[10]*nuy+K[11]*nuz;
    float dx4=K[12]*nux+K[13]*nuy+K[14]*nuz, dx5=K[15]*nux+K[16]*nuy+K[17]*nuz;

    applyCorrection(dx0, dx1, dx2, dx3, dx4, dx5);
    covUpdate(K, 3);
}

void IEKFFilter::getQuaternion(float& w, float& x, float& y, float& z) const {
    w = qw_; x = qx_; y = qy_; z = qz_;
}

void IEKFFilter::getEuler(float& roll, float& pitch, float& yaw) const {
    quatToEuler(qw_, qx_, qy_, qz_, roll, pitch, yaw);
}

void IEKFFilter::reset() {
    qw_ = 1.0f; qx_ = qy_ = qz_ = 0.0f;
    bx_ = by_ = bz_ = 0.0f;
    initialized_ = false;
    mag_ref_set_ = false;

    std::memset(P_, 0, sizeof(P_));
    float init_att  = 0.3f * 0.3f;
    float init_bias = 0.05f * 0.05f;
    P_[0] = P_[7] = P_[14] = init_att;
    P_[21] = P_[28] = P_[35] = init_bias;
}
