#include "mpu6050.h"
#include <cmath>

MPU6050::MPU6050(IICInterface* bus, uint8_t addr)
    : bus_(bus)
    , dev_addr_(addr)
    , gyro_range_(GyroRange::DPS_250)
    , accel_range_(AccelRange::G_2)
    , gyro_scale_(131.0f)
    , accel_scale_(16384.0f)
{
}

bool MPU6050::init(GyroRange gyro_range, AccelRange accel_range, DLPFBandwidth dlpf) {
    gyro_range_  = gyro_range;
    accel_range_ = accel_range;

    // 唤醒设备: 退出睡眠, 时钟源选 PLL_X
    if (!bus_->writeRegister(dev_addr_, REG_PWR_MGMT_1, CLOCK_PLL_X)) {
        return false;
    }
    bus_->delayMs(100);

    if (!bus_->writeRegister(dev_addr_, REG_GYRO_CONFIG,
                             static_cast<uint8_t>(gyro_range))) {
        return false;
    }

    switch (gyro_range) {
        case GyroRange::DPS_250:  gyro_scale_ = 131.0f;   break;
        case GyroRange::DPS_500:  gyro_scale_ = 65.5f;    break;
        case GyroRange::DPS_1000: gyro_scale_ = 32.8f;    break;
        case GyroRange::DPS_2000: gyro_scale_ = 16.4f;    break;
    }

    if (!bus_->writeRegister(dev_addr_, REG_ACCEL_CONFIG,
                             static_cast<uint8_t>(accel_range))) {
        return false;
    }

    switch (accel_range) {
        case AccelRange::G_2:  accel_scale_ = 16384.0f;  break;
        case AccelRange::G_4:  accel_scale_ = 8192.0f;   break;
        case AccelRange::G_8:  accel_scale_ = 4096.0f;   break;
        case AccelRange::G_16: accel_scale_ = 2048.0f;   break;
    }

    if (!bus_->writeRegister(dev_addr_, REG_CONFIG, static_cast<uint8_t>(dlpf))) {
        return false;
    }

    // SMPLRT_DIV=4, 采样率 = 1kHz/(1+4) = 200Hz
    if (!bus_->writeRegister(dev_addr_, REG_SMPLRT_DIV, 0x04)) {
        return false;
    }

    return true;
}

bool MPU6050::whoAmI() {
    uint8_t id = 0;
    if (!bus_->readRegisters(dev_addr_, REG_WHO_AM_I, &id, 1)) {
        return false;
    }
    return id == 0x68;
}

bool MPU6050::getMotion6(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz) {
    uint8_t buf[14];
    if (!bus_->readRegisters(dev_addr_, REG_ACCEL_XOUT_H, buf, 14)) {
        return false;
    }

    ax = (buf[0]  << 8) | buf[1];
    ay = (buf[2]  << 8) | buf[3];
    az = (buf[4]  << 8) | buf[5];
    gx = (buf[8]  << 8) | buf[9];
    gy = (buf[10] << 8) | buf[11];
    gz = (buf[12] << 8) | buf[13];

    return true;
}

bool MPU6050::getTemperature(float& celsius) {
    uint8_t buf[2];
    if (!bus_->readRegisters(dev_addr_, REG_TEMP_OUT_H, buf, 2)) {
        return false;
    }
    int16_t raw = (buf[0] << 8) | buf[1];
    celsius = raw / 340.0f + 36.53f;
    return true;
}
