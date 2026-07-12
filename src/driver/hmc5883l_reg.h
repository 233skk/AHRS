/**
 * @file    hmc5883l_reg.h
 * @brief   HMC5883L 三轴磁力计寄存器地址定义 (参考 HMC5883L Data Sheet Rev E)
 */

#ifndef HMC5883L_REG_H
#define HMC5883L_REG_H

#include <cstdint>

// I2C 设备地址 (7-bit)
constexpr uint8_t HMC5883L_ADDR = 0x1E;

// 寄存器映射
constexpr uint8_t HMC_CONFIG_A    = 0x00;  // 采样平均 + 输出速率 + 测量模式
constexpr uint8_t HMC_CONFIG_B    = 0x01;  // 增益/量程
constexpr uint8_t HMC_MODE        = 0x02;  // 工作模式
constexpr uint8_t HMC_DATA_X_H    = 0x03;  // X 轴高字节
constexpr uint8_t HMC_DATA_X_L    = 0x04;
constexpr uint8_t HMC_DATA_Z_H    = 0x05;  // Z 轴高字节 (注意: Z 在 Y 之前)
constexpr uint8_t HMC_DATA_Z_L    = 0x06;
constexpr uint8_t HMC_DATA_Y_H    = 0x07;  // Y 轴高字节
constexpr uint8_t HMC_DATA_Y_L    = 0x08;
constexpr uint8_t HMC_STATUS      = 0x09;  // 状态 (0x01=ready, 0x00=busy)
constexpr uint8_t HMC_ID_A        = 0x0A;  // 器件 ID: 'H' (0x48)
constexpr uint8_t HMC_ID_B        = 0x0B;  // 器件 ID: '4' (0x34)
constexpr uint8_t HMC_ID_C        = 0x0C;  // 器件 ID: '3' (0x33)

// ---- 采样平均 (CONFIG_A[6:5]) ----
enum class HMCAveraging : uint8_t {
    AVG_1  = 0x00,   // 默认
    AVG_2  = 0x20,
    AVG_4  = 0x40,
    AVG_8  = 0x60
};

// ---- 输出速率 (CONFIG_A[4:2]) ----
enum class HMCOutputRate : uint8_t {
    RATE_0_75HZ = 0x00,
    RATE_1_5HZ  = 0x04,
    RATE_3HZ    = 0x08,
    RATE_7_5HZ  = 0x0C,
    RATE_15HZ   = 0x10,   // 默认
    RATE_30HZ   = 0x14,
    RATE_75HZ   = 0x18
};

// ---- 测量模式 (CONFIG_A[1:0]) — 正常模式下这两位无影响 ----
constexpr uint8_t HMC_MEAS_NORMAL    = 0x00;  // 正常测量
constexpr uint8_t HMC_MEAS_POS_BIAS  = 0x01;  // 正偏置自检
constexpr uint8_t HMC_MEAS_NEG_BIAS  = 0x02;  // 负偏置自检

// ---- 增益/量程 (CONFIG_B[6:5]) ----
// 推荐 ±1.3Ga (GN_1_9), 地球磁场约 0.25~0.65Ga
enum class HMCGain : uint8_t {
    GN_0_88 = 0x00,   // ±0.88 Ga,   1370 LSB/Ga  (默认)
    GN_1_3  = 0x20,   // ±1.3 Ga,    1090 LSB/Ga  (推荐)
    GN_1_9  = 0x40,   // ±1.9 Ga,     820 LSB/Ga
    GN_2_5  = 0x60,   // ±2.5 Ga,     660 LSB/Ga
    GN_4_0  = 0x80,   // ±4.0 Ga,     440 LSB/Ga
    GN_4_7  = 0xA0,   // ±4.7 Ga,     390 LSB/Ga
    GN_5_6  = 0xC0,   // ±5.6 Ga,     330 LSB/Ga
    GN_8_1  = 0xE0    // ±8.1 Ga,     230 LSB/Ga
};

// ---- 工作模式 (MODE[1:0]) ----
constexpr uint8_t HMC_MODE_CONTINUOUS = 0x00;  // 连续测量
constexpr uint8_t HMC_MODE_SINGLE     = 0x01;  // 单次测量 (默认)
constexpr uint8_t HMC_MODE_IDLE       = 0x02;  // 空闲 (上电默认)
constexpr uint8_t HMC_MODE_IDLE2      = 0x03;  // 空闲

// ---- 状态寄存器 ----
constexpr uint8_t HMC_STATUS_RDY  = 0x01;  // 数据就绪
constexpr uint8_t HMC_STATUS_LOCK = 0x02;  // 寄存器锁定

// ---- 器件 ID ----
constexpr uint8_t HMC_ID_A_VAL = 0x48;  // 'H'
constexpr uint8_t HMC_ID_B_VAL = 0x34;  // '4'
constexpr uint8_t HMC_ID_C_VAL = 0x33;  // '3'

#endif // HMC5883L_REG_H
