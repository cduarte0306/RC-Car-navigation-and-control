#ifndef UTILS_HPP
#include <string>

namespace Utils {

    std::string GetOEVersion();

    int ConfigCores();

    void install_crash_handler();
}

#endif