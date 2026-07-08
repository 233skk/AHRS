/**
 * @file    mpu6050_reg.h
 * @brief   MPU6050 寄存器地址定义 (参考 MPU-6000/MPU-6050 Register Map Rev 4.2)
 */

#ifndef MPU6050_REG_H
#define MPU6050_REG_H

#include <cstdint>

// I2C 设备地址
constexpr uint8_t MPU6050_ADDR_LOW  = 0x68;
constexpr uint8_t MPU6050_ADDR_HIGH = 0x69;

// 寄存器映射
constexpr uint8_t REG_SELF_TEST_X      = 0x0D;
constexpr uint8_t REG_SELF_TEST_Y      = 0x0E;
constexpr uint8_t REG_SELF_TEST_Z      = 0x0F;
constexpr uint8_t REG_SELF_TEST_A      = 0x10;
constexpr uint8_t REG_SMPLRT_DIV       = 0x19;

constexpr uint8_t REG_CONFIG           = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG      = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG     = 0x1C;
constexpr uint8_t REG_ACCEL_CONFIG2    = 0x1D;

constexpr uint8_t REG_FIFO_EN          = 0x23;
constexpr uint8_t REG_I2C_MST_CTRL     = 0x24;
constexpr uint8_t REG_I2C_SLV0_ADDR    = 0x25;
constexpr uint8_t REG_I2C_SLV0_REG     = 0x26;
constexpr uint8_t REG_I2C_SLV0_CTRL    = 0x27;
constexpr uint8_t REG_I2C_SLV1_ADDR    = 0x28;
constexpr uint8_t REG_I2C_SLV1_REG     = 0x29;
constexpr uint8_t REG_I2C_SLV1_CTRL    = 0x2A;
constexpr uint8_t REG_I2C_SLV2_ADDR    = 0x2B;
constexpr uint8_t REG_I2C_SLV2_REG     = 0x2C;
constexpr uint8_t REG_I2C_SLV2_CTRL    = 0x2D;
constexpr uint8_t REG_I2C_SLV3_ADDR    = 0x2E;
constexpr uint8_t REG_I2C_SLV3_REG     = 0x2F;
constexpr uint8_t REG_I2C_SLV3_CTRL    = 0x30;
constexpr uint8_t REG_I2C_SLV4_ADDR    = 0x31;
constexpr uint8_t REG_I2C_SLV4_REG     = 0x32;
constexpr uint8_t REG_I2C_SLV4_DO      = 0x33;
constexpr uint8_t REG_I2C_SLV4_CTRL    = 0x34;
constexpr uint8_t REG_I2C_SLV4_DI      = 0x35;
constexpr uint8_t REG_I2C_MST_STATUS   = 0x36;

constexpr uint8_t REG_INT_PIN_CFG      = 0x37;
constexpr uint8_t REG_INT_ENABLE       = 0x38;
constexpr uint8_t REG_INT_STATUS       = 0x3A;

// 传感器数据 (14字节, 大端序)
constexpr uint8_t REG_ACCEL_XOUT_H     = 0x3B;
constexpr uint8_t REG_ACCEL_XOUT_L     = 0x3C;
constexpr uint8_t REG_ACCEL_YOUT_H     = 0x3D;
constexpr uint8_t REG_ACCEL_YOUT_L     = 0x3E;
constexpr uint8_t REG_ACCEL_ZOUT_H     = 0x3F;
constexpr uint8_t REG_ACCEL_ZOUT_L     = 0x40;
constexpr uint8_t REG_TEMP_OUT_H       = 0x41;
constexpr uint8_t REG_TEMP_OUT_L       = 0x42;
constexpr uint8_t REG_GYRO_XOUT_H      = 0x43;
constexpr uint8_t REG_GYRO_XOUT_L      = 0x44;
constexpr uint8_t REG_GYRO_YOUT_H      = 0x45;
constexpr uint8_t REG_GYRO_YOUT_L      = 0x46;
constexpr uint8_t REG_GYRO_ZOUT_H      = 0x47;
constexpr uint8_t REG_GYRO_ZOUT_L      = 0x48;

constexpr uint8_t REG_SIGNAL_PATH_RESET= 0x68;
constexpr uint8_t REG_USER_CTRL        = 0x6A;
constexpr uint8_t REG_PWR_MGMT_1       = 0x6B;
constexpr uint8_t REG_PWR_MGMT_2       = 0x6C;
constexpr uint8_t REG_FIFO_COUNT_H     = 0x72;
constexpr uint8_t REG_FIFO_COUNT_L     = 0x73;
constexpr uint8_t REG_FIFO_R_W         = 0x74;
constexpr uint8_t REG_WHO_AM_I         = 0x75;

// 陀螺仪量程 (REG_GYRO_CONFIG[4:3])
enum class GyroRange : uint8_t {
    DPS_250  = 0x00,
    DPS_500  = 0x08,
    DPS_1000 = 0x10,
    DPS_2000 = 0x18
};

// 加速度计量程 (REG_ACCEL_CONFIG[4:3])
enum class AccelRange : uint8_t {
    G_2  = 0x00,
    G_4  = 0x08,
    G_8  = 0x10,
    G_16 = 0x18
};

// 低通滤波 DLPF (REG_CONFIG[2:0])
enum class DLPFBandwidth : uint8_t {
    BW_260HZ = 0x00,
    BW_184HZ = 0x01,
    BW_94HZ  = 0x02,
    BW_44HZ  = 0x03,
    BW_21HZ  = 0x04,
    BW_10HZ  = 0x05,
    BW_5HZ   = 0x06
};

// 时钟源 (REG_PWR_MGMT_1[2:0])
constexpr uint8_t CLOCK_INTERNAL = 0x00;
constexpr uint8_t CLOCK_PLL_X    = 0x01;
constexpr uint8_t CLOCK_PLL_Y    = 0x02;
constexpr uint8_t CLOCK_PLL_Z    = 0x03;

#endif // MPU6050_REG_H
