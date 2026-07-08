# AHRS

> MPU6050 六轴姿态航向参考系统，基于左不变扩展卡尔曼滤波（Left-Invariant EKF）

## 快速开始

### 编译

```bash
make
```

### 运行

```bash
# IEKF（默认噪声参数）
./main iekf

# IEKF 自定义噪声参数
./main iekf <σ_gyro> <σ_accel> <σ_bias>

# 示例
./main iekf 0.05 0.5 0.0003
```

### 参数说明

| 参数 | 含义 | 典型值 |
|---|---|---|
| `σ_gyro` | 陀螺仪噪声密度 (rad/s/√Hz) | 0.01（优）~ 0.1（差） |
| `σ_accel` | 加速度计噪声 (m/s²) | 0.2（优）~ 1.0（差） |
| `σ_bias` | 陀螺仪偏置随机游走 (rad/s/√s) | 0.0001 ~ 0.001 |

## 项目结构

```
AHRS/
├── Makefile                 # 交叉编译（Rockchip RV1106）
├── README.md
└── src/
    ├── main.cpp             # 入口，命令行解析，主循环
    ├── platform/
    │   ├── iic_interface.h  # I2C 抽象接口（3 个纯虚方法）
    │   └── iic_linux.cpp    # Linux I2C 实现（/dev/i2c-N）
    ├── driver/
    │   ├── mpu6050_reg.h    # MPU6050 全部寄存器 & 配置枚举
    │   ├── mpu6050.h        # 传感器驱动类
    │   └── mpu6050.cpp      # 初始化、6轴读取、温度
    ├── fusion/
    │   ├── fusion_interface.h  # 融合算法统一接口 + quat→euler + rad↔deg
    │   ├── iekf.h              # Left-Invariant EKF 声明
    │   └── iekf.cpp            # 6×6 协方差、预测/更新、陀螺仪偏置估计
    └── debug/
        └── debug_logger.h      # NDJSON 调试日志 (/tmp/mpu6050_debug.ndjson)
```

## 融合算法

当前实现了 **Left-Invariant EKF（左不变扩展卡尔曼滤波）**：

- 状态向量 6 维：姿态误差（3 维）+ 陀螺仪偏置（3 维）
- 基于 IMU 运动学模型的左不变误差状态传播
- 加速度计量测用于校正俯仰/横滚角
- 实时估计并补偿陀螺仪零偏

## 移植

换平台只需实现 `IICInterface` 三个方法：

```cpp
class IICInterface {
    virtual bool writeRegister(uint8_t dev_addr, uint8_t reg, uint8_t data) = 0;
    virtual bool readRegisters(uint8_t dev_addr, uint8_t reg, uint8_t* buffer, size_t length) = 0;
    virtual void delayMs(unsigned int ms) = 0;
};
```

驱动层和融合层代码完全不动。已有实现：

- `iic_linux.cpp` — Linux / Raspberry Pi / Luckfox
- 可扩展：Arduino Wire、STM32 HAL、ESP-IDF

## License

MIT
