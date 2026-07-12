/**
 * @file    mag_calib_main.cpp
 * @brief   磁力计椭球校准工具 — 旋转设备采集数据 → 拟合 → 输出校正矩阵
 *
 * 用法: ./mag_calib
 *
 * 操作:
 *   1. 缓慢旋转设备，覆盖 3D 各方向（画 ∞ 字 + 翻转）
 *   2. 观察终端显示的 X/Y/Z 最小/最大值，确保每个轴都扫过正负范围
 *   3. 采样满 2048 点或 Ctrl+C 停止 → 自动拟合
 *   4. 将打印的 C++ 常量拷入 main.cpp
 *
 * 采样间隔固定 ~50ms (20Hz)，2048 点约 100 秒，有充足时间旋转设备。
 */

#include <iostream>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <csignal>
#include <unistd.h>

#include "platform/iic_linux.cpp"
#include "driver/mpu6050.cpp"
#include "driver/hmc5883l.cpp"
#include "calibration/mag_calibration.h"

static volatile bool g_run = true;
void sigHandler(int) { g_run = false; }

int main() {
    signal(SIGINT,  sigHandler);
    signal(SIGTERM, sigHandler);

    try {
        IICLinux bus("/dev/i2c-3");

        // 初始化 MPU6050 — GY-87 需要 bypass 才能访问磁力计
        MPU6050 imu(&bus);
        if (!imu.init()) {
            std::cerr << "MPU6050 初始化失败!\n";
            return -1;
        }
        std::cout << "MPU6050 WHO_AM_I: " << (imu.whoAmI() ? "OK" : "FAIL") << "\n";

        // 使能 I2C bypass
        bus.writeRegister(imu.getDeviceAddr(), 0x37, 0x02);  // REG_INT_PIN_CFG

        // 初始化 HMC5883L
        HMC5883L mag(&bus);
        if (!mag.init()) {
            std::cerr << "HMC5883L 初始化失败!\n";
            return -1;
        }
        std::cout << "HMC5883L WHO_AM_I: " << (mag.whoAmI() ? "OK" : "FAIL") << "\n\n";

        // 采集
        MagCalibration calib;
        std::cout << "============================================\n";
        std::cout << "  磁力计椭球校准 — 采集阶段\n";
        std::cout << "============================================\n";
        std::cout << "操作: 缓慢旋转设备覆盖 3D 各个方向\n";
        std::cout << "  1) 水平画 ∞ 字\n";
        std::cout << "  2) 侧立翻转，让芯片朝上/下/左/右/前/后\n";
        std::cout << "  3) 随机慢速转圈\n";
        std::cout << "\n采样间隔 50ms (20Hz), 2048 点约 100 秒\n";
        std::cout << "观察下面的 min/max: 每个轴正负都要扫到!\n";
        std::cout << "按 Ctrl+C 停止采集并自动拟合\n\n";

        int16_t mx, my, mz;
        struct timespec ts_start, ts_now, ts_last_sample;
        clock_gettime(CLOCK_MONOTONIC, &ts_start);
        ts_last_sample = ts_start;

        // 记录各轴范围，用于显示覆盖质量
        float x_min = 1e9, x_max = -1e9;
        float y_min = 1e9, y_max = -1e9;
        float z_min = 1e9, z_max = -1e9;

        while (g_run && calib.sampleCount() < 2048) {
            // 固定 50ms 采样间隔，给用户充足时间旋转设备
            clock_gettime(CLOCK_MONOTONIC, &ts_now);
            float since_last = (ts_now.tv_sec - ts_last_sample.tv_sec) +
                              (ts_now.tv_nsec - ts_last_sample.tv_nsec) / 1e9f;
            if (since_last < 0.05f) {
                usleep(5000);  // 还没到 50ms，等一等
                continue;
            }

            if (!mag.getField(mx, my, mz)) {
                usleep(5000);
                continue;
            }
            ts_last_sample = ts_now;

            float gx = mag.rawToGauss(mx);
            float gy = mag.rawToGauss(my);
            float gz = mag.rawToGauss(mz);

            // 更新各轴范围
            if (gx < x_min) x_min = gx;
            if (gx > x_max) x_max = gx;
            if (gy < y_min) y_min = gy;
            if (gy > y_max) y_max = gy;
            if (gz < z_min) z_min = gz;
            if (gz > z_max) z_max = gz;

            calib.addSample(gx, gy, gz);

            float elapsed = (ts_now.tv_sec - ts_start.tv_sec) +
                           (ts_now.tv_nsec - ts_start.tv_nsec) / 1e9f;
            float total = sqrtf(gx*gx + gy*gy + gz*gz) * 1000.0f;
            int n = calib.sampleCount();

            // 显示当前读数和各轴覆盖范围
            printf("\r[%5.1fs] #%4d | X:%+7.1f Y:%+7.1f Z:%+7.1f mGa | |B|:%5.0f mGa | "
                   "X:[%+6.0f~%+6.0f] Y:[%+6.0f~%+6.0f] Z:[%+6.0f~%+6.0f]  ",
                   elapsed, n, gx*1000, gy*1000, gz*1000, total,
                   x_min*1000, x_max*1000,
                   y_min*1000, y_max*1000,
                   z_min*1000, z_max*1000);
            fflush(stdout);
        }

        std::cout << "\n\n采集完成, 共 " << calib.sampleCount() << " 个采样点\n";

        // 检查各轴覆盖范围
        float x_range = x_max - x_min;
        float y_range = y_max - y_min;
        float z_range = z_max - z_min;
        printf("\n各轴覆盖范围 (mGa): X=%.0f  Y=%.0f  Z=%.0f\n",
               x_range*1000, y_range*1000, z_range*1000);

        // 检查是否正负都扫到
        bool x_ok = (x_min < 0) && (x_max > 0);
        bool y_ok = (y_min < 0) && (y_max > 0);
        bool z_ok = (z_min < 0) && (z_max > 0);
        int axes_ok = (x_ok ? 1 : 0) + (y_ok ? 1 : 0) + (z_ok ? 1 : 0);

        printf("过零点检查: X:%s  Y:%s  Z:%s  (%d/3 轴扫过正负)\n",
               x_ok ? "✓" : "✗", y_ok ? "✓" : "✗", z_ok ? "✓" : "✗", axes_ok);

        if (axes_ok < 2) {
            std::cerr << "\n⚠ 警告: 大部分轴没有扫过正负范围，拟合可能失败!\n"
                      << "  请重新采集，确保每个轴都经历正向和负向的磁场分量。\n"
                      << "  多翻转设备，让芯片朝向不同的方向。\n";
        }

        if (calib.sampleCount() < 30) {
            std::cerr << "采样太少, 至少需要 30 个点\n";
            return -1;
        }

        std::cout << "\n正在拟合椭球...\n";
        if (!calib.compute()) {
            std::cerr << "拟合失败! 请重新采集, 确保设备旋转覆盖了 3D 空间\n";
            std::cerr << "提示: 如果各轴范围都很窄, 说明旋转不够全面。\n";
            std::cerr << "      试着把设备翻转 90°/180°, 让芯片朝不同方向。\n";
            return -1;
        }

        calib.printParameters();
        calib.verifyCalibration();

        std::cout << "\n将上面的 C++ 常量拷入 main.cpp 即可使用\n";

    } catch (const std::exception& e) {
        std::cerr << "\n异常: " << e.what() << "\n";
        return -1;
    }

    return 0;
}
