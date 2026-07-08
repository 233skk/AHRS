/**
 * @file    iic_interface.h
 * @brief   I2C 总线抽象接口 — 换平台只需实现这三个方法
 */

#ifndef IIC_INTERFACE_H
#define IIC_INTERFACE_H

#include <cstdint>
#include <cstddef>

class IICInterface {
public:
    virtual ~IICInterface() = default;

    virtual bool writeRegister(uint8_t dev_addr, uint8_t reg, uint8_t data) = 0;
    virtual bool readRegisters(uint8_t dev_addr, uint8_t reg, uint8_t* buffer, size_t length) = 0;
    virtual void delayMs(unsigned int ms) = 0;
};

#endif // IIC_INTERFACE_H
