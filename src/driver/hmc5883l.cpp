#include "hmc5883l.h"

HMC5883L::HMC5883L(IICInterface* bus, uint8_t addr)
    : bus_(bus)
    , dev_addr_(addr)
    , gain_(HMCGain::GN_1_3)
    , gain_scale_(1090.0f)
{
}

bool HMC5883L::init(HMCAveraging averaging, HMCOutputRate rate, HMCGain gain) {
    gain_ = gain;

    // Config A: 采样平均 | 输出速率 | 正常测量模式
    uint8_t config_a = static_cast<uint8_t>(averaging)
                     | static_cast<uint8_t>(rate)
                     | HMC_MEAS_NORMAL;
    if (!bus_->writeRegister(dev_addr_, HMC_CONFIG_A, config_a)) {
        return false;
    }

    // Config B: 增益
    if (!bus_->writeRegister(dev_addr_, HMC_CONFIG_B, static_cast<uint8_t>(gain))) {
        return false;
    }

    switch (gain) {
        case HMCGain::GN_0_88: gain_scale_ = 1370.0f; break;
        case HMCGain::GN_1_3:  gain_scale_ = 1090.0f; break;
        case HMCGain::GN_1_9:  gain_scale_ =  820.0f; break;
        case HMCGain::GN_2_5:  gain_scale_ =  660.0f; break;
        case HMCGain::GN_4_0:  gain_scale_ =  440.0f; break;
        case HMCGain::GN_4_7:  gain_scale_ =  390.0f; break;
        case HMCGain::GN_5_6:  gain_scale_ =  330.0f; break;
        case HMCGain::GN_8_1:  gain_scale_ =  230.0f; break;
    }

    // Mode: 连续测量
    if (!bus_->writeRegister(dev_addr_, HMC_MODE, HMC_MODE_CONTINUOUS)) {
        return false;
    }

    return true;
}

bool HMC5883L::whoAmI() {
    uint8_t id_a = 0, id_b = 0, id_c = 0;
    if (!bus_->readRegisters(dev_addr_, HMC_ID_A, &id_a, 1)) return false;
    if (!bus_->readRegisters(dev_addr_, HMC_ID_B, &id_b, 1)) return false;
    if (!bus_->readRegisters(dev_addr_, HMC_ID_C, &id_c, 1)) return false;
    return id_a == HMC_ID_A_VAL && id_b == HMC_ID_B_VAL && id_c == HMC_ID_C_VAL;
}

bool HMC5883L::isDataReady() {
    uint8_t status = 0;
    if (!bus_->readRegisters(dev_addr_, HMC_STATUS, &status, 1)) {
        return false;
    }
    return (status & HMC_STATUS_RDY) != 0;
}

bool HMC5883L::getField(int16_t& mx, int16_t& my, int16_t& mz) {
    // HMC5883L 数据顺序: X, Z, Y (注意 Z 在 Y 之前)
    uint8_t buf[6];
    if (!bus_->readRegisters(dev_addr_, HMC_DATA_X_H, buf, 6)) {
        return false;
    }

    int16_t raw_x = (buf[0] << 8) | buf[1];
    int16_t raw_z = (buf[2] << 8) | buf[3];
    int16_t raw_y = (buf[4] << 8) | buf[5];

    mx = raw_x;
    my = raw_y;
    mz = raw_z;

    return true;
}
