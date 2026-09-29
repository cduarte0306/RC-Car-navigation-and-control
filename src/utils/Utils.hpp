#ifndef UTILS_HPP
#include <string>
#include <cstddef>
#include "types.h"

namespace Utils {

    std::string GetOEVersion();

    int ConfigCores();

    void install_crash_handler();

    uint32 xCRC32(const uint8 data[], size_t len);
}

#endif