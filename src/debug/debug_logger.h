/**
 * @file    debug_logger.h
 * @brief   NDJSON 调试日志, 输出到 /tmp/mpu6050_debug.ndjson
 */

#ifndef DEBUG_LOGGER_H
#define DEBUG_LOGGER_H

#include <string>
#include <cstdio>
#include <ctime>
#include <cstdarg>

namespace debug {

inline FILE* getFile() {
    static FILE* f = fopen("/tmp/mpu6050_debug.ndjson", "a");
    return f;
}

inline void log(const std::string& json) {
    FILE* f = getFile();
    if (f) { fprintf(f, "%s\n", json.c_str()); fflush(f); }
}

inline std::string fmt(const char* tmpl, ...) {
    char buf[512];
    va_list args; va_start(args, tmpl);
    vsnprintf(buf, sizeof(buf), tmpl, args);
    va_end(args);
    return std::string(buf);
}

// 预定义日志事件
inline void logRawSensor(int16_t ax, int16_t ay, int16_t az,
                         int16_t gx, int16_t gy, int16_t gz) {
    log(fmt("{\"event\":\"raw\",\"ax\":%d,\"ay\":%d,\"az\":%d,\"gx\":%d,\"gy\":%d,\"gz\":%d}",
            ax, ay, az, gx, gy, gz));
}

inline void logConverted(float ax, float ay, float az,
                         float gx, float gy, float gz) {
    log(fmt("{\"event\":\"converted\",\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,\"gx\":%.6f,\"gy\":%.6f,\"gz\":%.6f}",
            ax, ay, az, gx, gy, gz));
}

inline void logAlgoInput(const char* algo, float dt) {
    log(fmt("{\"event\":\"algo_input\",\"algo\":\"%s\",\"dt\":%.6f}", algo, dt));
}

inline void logQuaternion(const char* algo, float w, float x, float y, float z) {
    log(fmt("{\"event\":\"quat\",\"algo\":\"%s\",\"w\":%.4f,\"x\":%.4f,\"y\":%.4f,\"z\":%.4f}",
            algo, w, x, y, z));
}

inline void logEuler(const char* algo, float roll, float pitch, float yaw) {
    log(fmt("{\"event\":\"euler\",\"algo\":\"%s\",\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f}",
            algo, roll, pitch, yaw));
}

} // namespace debug
#endif // DEBUG_LOGGER_H
