/**
 * @file    main.cpp
 * @brief   MPU6050 6轴姿态解算 — 用法: ./main iekf [σ_gyro] [σ_accel] [σ_bias]
 */

#include <iostream>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <csignal>

#include "platform/iic_linux.cpp"
#include "driver/mpu6050.cpp"
#include "driver/hmc5883l.cpp"
#include "fusion/iekf.cpp"
#include "debug/debug_logger.h"

static FusionAlgorithm* g_algo = nullptr;
static volatile bool    g_run  = true;
static bool    g_gyro_init  = false;
// 纯陀螺四元数积分（用于和 IEKF 做公平对比，而非简单欧拉角累加）
static float   g_gyro_qw = 1, g_gyro_qx = 0, g_gyro_qy = 0, g_gyro_qz = 0;
static FILE*   g_csv_file  = nullptr;

void openCSV() {
    g_csv_file = fopen("/tmp/mpu6050_fusion.csv", "w");
    if (g_csv_file) {
        fprintf(g_csv_file, "t_ms,gyr_roll,gyr_pitch,gyr_yaw,fus_roll,fus_pitch,fus_yaw\n");
        fflush(g_csv_file);
    }
}

void writeCSV(float t_ms,
              float gyr_r, float gyr_p, float gyr_y,
              float fus_r, float fus_p, float fus_y) {
    if (g_csv_file) {
        fprintf(g_csv_file, "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
                t_ms, gyr_r, gyr_p, gyr_y, fus_r, fus_p, fus_y);
        fflush(g_csv_file);
    }
}

void closeCSV() {
    if (g_csv_file) {
        fclose(g_csv_file);
        g_csv_file = nullptr;
    }
}

void sigHandler(int) { g_run = false; }

FusionAlgorithm* createAlgorithm(int argc, char* argv[],
                                  float& p1, float& p2, float& p3, float& p4) {
    std::string name = (argc > 1) ? argv[1] : "iekf";

    p1 = (argc > 2) ? atof(argv[2]) : 0.05f;
    p2 = (argc > 3) ? atof(argv[3]) : 0.5f;
    p3 = (argc > 4) ? atof(argv[4]) : 0.0003f;
    p4 = (argc > 5) ? atof(argv[5]) : 2.0f;  // gain: 校正力度, 1=默认, 2~10=激进
    std::cout << "IEKF | σ_gyro=" << p1 << " σ_accel=" << p2
              << " σ_bias=" << p3 << " gain=" << p4 << "\n";
    return new IEKFFilter(p1, p2, p3, p4);
}

int main(int argc, char* argv[]) {
    signal(SIGINT, sigHandler);
    signal(SIGTERM, sigHandler);

    float p1=0, p2=0, p3=0, p4=0;
    g_algo = createAlgorithm(argc, argv, p1, p2, p3, p4);
    try {
        IICLinux bus("/dev/i2c-3");
        MPU6050 imu(&bus);
        if (!imu.init()) { std::cerr << "MPU6050 失败!\n"; return -1; }
        std::cout << "WHO_AM_I: " << (imu.whoAmI() ? "OK" : "FAIL")
                  << " | 200Hz | Ctrl+C 退出\n";

        // GY-87: 使能 MPU6050 I2C bypass, 让 HMC5883L 直连到主 I2C 总线
        bus.writeRegister(imu.getDeviceAddr(), REG_INT_PIN_CFG, 0x02);

        HMC5883L mag(&bus);
        if (!mag.init()) { std::cerr << "HMC5883L 初始化失败!\n"; return -1; }
        std::cout << "HMC5883L WHO_AM_I: " << (mag.whoAmI() ? "OK" : "FAIL")
                  << " | gain=" << mag.getGainScale() << " LSB/Ga\n\n";

        openCSV();

        int16_t ax, ay, az, gx, gy, gz;
        struct timespec ts_start, ts_prev, ts_now;
        clock_gettime(CLOCK_MONOTONIC, &ts_start); ts_prev = ts_start;

        while (g_run) {
            if (imu.getMotion6(ax, ay, az, gx, gy, gz)) {
                clock_gettime(CLOCK_MONOTONIC, &ts_now);
                float dt = (ts_now.tv_sec - ts_prev.tv_sec) +
                           (ts_now.tv_nsec - ts_prev.tv_nsec) / 1e9f;
                ts_prev = ts_now;
                float elapsed = (ts_now.tv_sec - ts_start.tv_sec)*1000.0f +
                                (ts_now.tv_nsec - ts_start.tv_nsec)/1e6f;

                float gx_r = imu.gyroRawToRadPerSec(gx);
                float gy_r = imu.gyroRawToRadPerSec(gy);
                float gz_r = imu.gyroRawToRadPerSec(gz);
                float ax_g = imu.accelRawToG(ax);
                float ay_g = imu.accelRawToG(ay);
                float az_g = imu.accelRawToG(az);

                g_algo->update(gx_r, gy_r, gz_r, ax_g, ay_g, az_g, dt);

                // 纯加速度计角度
                float acc_roll  = atan2f(ay_g, az_g);
                float acc_pitch = atan2f(-ax_g, sqrtf(ay_g*ay_g + az_g*az_g));

                // 纯陀螺四元数积分（从加速度计角度初始化）
                if (!g_gyro_init) {
                    // 欧拉角 → 四元数 (ZYX: roll→pitch→yaw)
                    float cr = cosf(acc_roll*0.5f),  sr = sinf(acc_roll*0.5f);
                    float cp = cosf(acc_pitch*0.5f), sp = sinf(acc_pitch*0.5f);
                    float cy = cosf(0.0f),           sy = sinf(0.0f);
                    g_gyro_qw = cr*cp*cy + sr*sp*sy;
                    g_gyro_qx = sr*cp*cy - cr*sp*sy;
                    g_gyro_qy = cr*sp*cy + sr*cp*sy;
                    g_gyro_qz = cr*cp*sy - sr*sp*cy;
                    g_gyro_init = true;
                }

                // 纯陀螺四元数传播: q ← q ⊗ exp_q(ω·dt)
                {
                    float w_dt = sqrtf(gx_r*gx_r + gy_r*gy_r + gz_r*gz_r) * dt;
                    if (w_dt > 1e-6f) {
                        float half_a = 0.5f * w_dt;
                        float s = sinf(half_a);
                        float dqw = cosf(half_a);
                        float dqx = (gx_r * dt) / w_dt * s;
                        float dqy = (gy_r * dt) / w_dt * s;
                        float dqz = (gz_r * dt) / w_dt * s;
                        // q ← q ⊗ dq
                        float nw = g_gyro_qw*dqw - g_gyro_qx*dqx - g_gyro_qy*dqy - g_gyro_qz*dqz;
                        float nx = g_gyro_qw*dqx + g_gyro_qx*dqw + g_gyro_qy*dqz - g_gyro_qz*dqy;
                        float ny = g_gyro_qw*dqy - g_gyro_qx*dqz + g_gyro_qy*dqw + g_gyro_qz*dqx;
                        float nz = g_gyro_qw*dqz + g_gyro_qx*dqy - g_gyro_qy*dqx + g_gyro_qz*dqw;
                        float nrm = sqrtf(nw*nw + nx*nx + ny*ny + nz*nz);
                        g_gyro_qw = nw/nrm; g_gyro_qx = nx/nrm;
                        g_gyro_qy = ny/nrm; g_gyro_qz = nz/nrm;
                    }
                }

                // 从纯陀螺四元数提取欧拉角
                float g_roll, g_pitch, g_yaw;
                FusionAlgorithm::quatToEuler(g_gyro_qw, g_gyro_qx, g_gyro_qy, g_gyro_qz,
                                             g_roll, g_pitch, g_yaw);

                float qw, qx, qy, qz, roll, pitch, yaw;
                g_algo->getQuaternion(qw, qx, qy, qz);
                g_algo->getEuler(roll, pitch, yaw);
                float rd = FusionAlgorithm::radToDeg(roll);
                float pd = FusionAlgorithm::radToDeg(pitch);
                float yd = FusionAlgorithm::radToDeg(yaw);

                // 写CSV对比日志（6列：时间+纯陀螺仪R/P/Y+融合R/P/Y）
                writeCSV(elapsed,
                    FusionAlgorithm::radToDeg(g_roll),
                    FusionAlgorithm::radToDeg(g_pitch),
                    FusionAlgorithm::radToDeg(g_yaw),
                    rd, pd, yd);

                // 磁力计：读取 → 硬铁/软铁校正 → IEKF 融合
                int16_t mx=0, my=0, mz=0;
                float mag_x_cal=0, mag_y_cal=0, mag_z_cal=0;
                bool mag_ok = mag.getField(mx, my, mz);
                if (mag_ok) {
                    // 原始值 (Gauss) — 未经校准，直接用于 IEKF 融合
                    mag_x_cal = mag.rawToGauss(mx);
                    mag_y_cal = mag.rawToGauss(my);
                    mag_z_cal = mag.rawToGauss(mz);

                    // IEKF 磁力计融合 → yaw 可观
                    auto* iekf = dynamic_cast<IEKFFilter*>(g_algo);
                    if (iekf) {
                        iekf->updateMag(mag_x_cal, mag_y_cal, mag_z_cal);
                    }
                }

                auto* iekf = dynamic_cast<IEKFFilter*>(g_algo);
                float bx=0, by=0, bz=0;
                if (iekf) { iekf->getBias(bx, by, bz); }

                // 校准后的磁力计航向 (平面投影)
                float mag_hdg = FusionAlgorithm::radToDeg(atan2f(mag_y_cal, mag_x_cal));

                printf("\r[%.1fs] IEKF | R:%6.1f P:%6.1f Y:%6.1f | Q:[%.4f %.4f %.4f %.4f] | Mag:%+6.0f %+6.0f %+6.0f mGa H:%6.1f° | bias:[%+.3f %+.3f %+.3f] | σ²=%.4f    ",
                       elapsed/1000, rd, pd, yd,
                       qw, qx, qy, qz,
                       mag_x_cal*1000, mag_y_cal*1000, mag_z_cal*1000, mag_hdg,
                       bx, by, bz, iekf ? iekf->getAttitudeVariance() : 0.0f);
                fflush(stdout);

            }
        }
    } catch (const std::exception& e) { std::cerr << "\n异常: " << e.what() << "\n"; }

    closeCSV();
    delete g_algo;
    std::cout << "\n已退出\n";
    return 0;
}
