/**
 * @file    hmc5883l.h
 * @brief   HMC5883L 三轴磁力计驱动，仅依赖 IICInterface 抽象接口
 */

#ifndef HMC5883L_H
#define HMC5883L_H

#include "../platform/iic_interface.h"
#include "hmc5883l_reg.h"

class HMC5883L {
public:
    HMC5883L(IICInterface* bus, uint8_t addr = HMC5883L_ADDR);

    bool init(HMCAveraging averaging = HMCAveraging::AVG_1,
              HMCOutputRate rate     = HMCOutputRate::RATE_15HZ,
              HMCGain gain          = HMCGain::GN_1_3);

    bool whoAmI();
    bool isDataReady();

    /// 读取 3 轴原始值 (12-bit, 符号扩展后的 int16)
    bool getField(int16_t& mx, int16_t& my, int16_t& mz);

    /// 原始值 → 高斯 (milli-Gauss 更方便: 实际使用 ×1000)
    float rawToGauss(int16_t raw) const { return raw / gain_scale_; }

    /// 返回当前增益 (LSB/Ga)
    float getGainScale()  const { return gain_scale_; }

    /// 返回当前量程
    HMCGain getGain()     const { return gain_; }

    uint8_t getDeviceAddr() const { return dev_addr_; }

private:
    IICInterface* bus_;
    uint8_t       dev_addr_;
    HMCGain       gain_;
    float         gain_scale_;
};

#endif // HMC5883L_H
