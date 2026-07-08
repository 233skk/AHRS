/**
 * @file    mpu6050.h
 * @brief   MPU6050 6轴传感器驱动，仅依赖 IICInterface 抽象接口
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "../platform/iic_interface.h"
#include "mpu6050_reg.h"

class MPU6050 {
public:
    MPU6050(IICInterface* bus, uint8_t addr = MPU6050_ADDR_LOW);

    bool init(GyroRange gyro_range = GyroRange::DPS_250,
              AccelRange accel_range = AccelRange::G_2,
              DLPFBandwidth dlpf = DLPFBandwidth::BW_44HZ);

    bool whoAmI();
    bool getMotion6(int16_t& ax, int16_t& ay, int16_t& az,
                    int16_t& gx, int16_t& gy, int16_t& gz);
    bool getTemperature(float& celsius);

    float gyroRawToRadPerSec(int16_t raw) const { return raw / gyro_scale_ * (M_PI / 180.0f); }
    float accelRawToG(int16_t raw)       const { return raw / accel_scale_; }

    float getGyroScale()   const { return gyro_scale_; }
    float getAccelScale()  const { return accel_scale_; }

    uint8_t getDeviceAddr() const { return dev_addr_; }
    GyroRange  getGyroRange()  const { return gyro_range_; }
    AccelRange getAccelRange() const { return accel_range_; }

private:
    IICInterface* bus_;
    uint8_t      dev_addr_;
    GyroRange    gyro_range_;
    AccelRange   accel_range_;
    float        gyro_scale_;
    float        accel_scale_;
};

#endif // MPU6050_H
