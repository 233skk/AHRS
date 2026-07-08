/**
 * @file    iic_linux.cpp
 * @brief   Linux I2C 总线实现 (Luckfox / Raspberry Pi 等)
 */

#include "iic_interface.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdexcept>
#include <cerrno>
#include <cstring>

class IICLinux : public IICInterface {
private:
    int fd;
    const char* bus_path;

public:
    explicit IICLinux(const char* bus) : fd(-1), bus_path(bus) {
        fd = open(bus, O_RDWR);
        if (fd < 0) {
            throw std::runtime_error(
                std::string("无法打开I2C总线: ") + bus +
                " (" + strerror(errno) + ")"
            );
        }
    }

    ~IICLinux() override {
        if (fd >= 0) {
            close(fd);
        }
    }

    // 文件描述符不可共享
    IICLinux(const IICLinux&) = delete;
    IICLinux& operator=(const IICLinux&) = delete;

    bool writeRegister(uint8_t dev_addr, uint8_t reg, uint8_t data) override {
        if (ioctl(fd, I2C_SLAVE, dev_addr) < 0) return false;
        uint8_t buf[2] = {reg, data};
        return write(fd, buf, 2) == 2;
    }

    bool readRegisters(uint8_t dev_addr, uint8_t reg,
                       uint8_t* buffer, size_t length) override {
        if (ioctl(fd, I2C_SLAVE, dev_addr) < 0) return false;
        if (write(fd, &reg, 1) != 1) return false;
        return read(fd, buffer, length) == static_cast<ssize_t>(length);
    }

    void delayMs(unsigned int ms) override {
        usleep(ms * 1000);
    }
};
