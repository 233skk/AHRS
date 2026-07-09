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
#include "fusion/iekf.cpp"
#include "debug/debug_logger.h"

static FusionAlgorithm* g_algo = nullptr;
static volatile bool    g_run  = true;
static bool    g_gyro_init  = false;
static float   g_gyro_roll = 0, g_gyro_pitch = 0, g_gyro_yaw = 0;
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
                                  float& p1, float& p2, float& p3) {
    std::string name = (argc > 1) ? argv[1] : "iekf";

    p1 = (argc > 2) ? atof(argv[2]) : 0.05f;
    p2 = (argc > 3) ? atof(argv[3]) : 0.5f;
    p3 = (argc > 4) ? atof(argv[4]) : 0.0003f;
    std::cout << "IEKF | σ_gyro=" << p1 << " σ_accel=" << p2 << " σ_bias=" << p3 << "\n";
    return new IEKFFilter(p1, p2, p3);
}

int main(int argc, char* argv[]) {
    signal(SIGINT, sigHandler);
    signal(SIGTERM, sigHandler);

    float p1=0, p2=0, p3=0;
    g_algo = createAlgorithm(argc, argv, p1, p2, p3);
    try {
        IICLinux bus("/dev/i2c-3");
        MPU6050 imu(&bus);
        if (!imu.init()) { std::cerr << "MPU6050 失败!\n"; return -1; }
        std::cout << "WHO_AM_I: " << (imu.whoAmI() ? "OK" : "FAIL")
                  << " | 200Hz | Ctrl+C 退出\n\n";

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

                // 纯陀螺积分（从加速度计角度初始化）
                if (!g_gyro_init) {
                    g_gyro_roll  = acc_roll;
                    g_gyro_pitch = acc_pitch;
                    g_gyro_yaw  = 0.0f;
                    g_gyro_init  = true;
                }
                g_gyro_roll  += gx_r * dt;
                g_gyro_pitch += gy_r * dt;
                g_gyro_yaw   += gz_r * dt;

                float qw, qx, qy, qz, roll, pitch, yaw;
                g_algo->getQuaternion(qw, qx, qy, qz);
                g_algo->getEuler(roll, pitch, yaw);
                float rd = FusionAlgorithm::radToDeg(roll);
                float pd = FusionAlgorithm::radToDeg(pitch);
                float yd = FusionAlgorithm::radToDeg(yaw);

                // 写CSV对比日志（6列：时间+纯陀螺仪R/P/Y+融合R/P/Y）
                writeCSV(elapsed,
                    FusionAlgorithm::radToDeg(g_gyro_roll),
                    FusionAlgorithm::radToDeg(g_gyro_pitch),
                    FusionAlgorithm::radToDeg(g_gyro_yaw),
                    rd, pd, yd);

                auto* iekf = dynamic_cast<IEKFFilter*>(g_algo);
                float bx=0, by=0, bz=0;
                if (iekf) { iekf->getBias(bx, by, bz); }

                if (iekf)
                    printf("\r[%.1fs] IEKF | R:%6.1f P:%6.1f Y:%6.1f | Q:[%.3f %.3f %.3f %.3f] | bias:[%+.3f %+.3f %+.3f] | σ²=%.4f    ",
                           elapsed/1000, rd, pd, yd, qw, qx, qy, qz, bx, by, bz, iekf->getAttitudeVariance());
                else
                    printf("\r[%.1fs] %s | R:%6.1f P:%6.1f Y:%6.1f | Q:[%.3f %.3f %.3f %.3f]    ",
                           elapsed/1000, g_algo->name(), rd, pd, yd, qw, qx, qy, qz);
                fflush(stdout);

            }
        }
    } catch (const std::exception& e) { std::cerr << "\n异常: " << e.what() << "\n"; }

    closeCSV();
    delete g_algo;
    std::cout << "\n已退出\n";
    return 0;
}
