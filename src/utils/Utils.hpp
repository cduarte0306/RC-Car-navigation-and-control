#ifndef UTILS_HPP
#define UTILS_HPP
#include <string>
#include <cstddef>
#include "types.h"
#include <cassert>

#include "logger.hpp"

#define CHECK(x) if (!(x)) { Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Check failed: %s\r\n", #x); assert(false); }

namespace Utils {

    std::string GetOEVersion();

    int ConfigCores();

    void install_crash_handler();

    uint32 xCRC32(const uint8 data[], size_t len);
}

#endif