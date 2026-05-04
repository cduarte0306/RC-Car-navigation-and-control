#include "RegisterMap.hpp"


static RegisterMap* g_registerMap = nullptr;


/**
 * @brief Get the singleton instance of the RegisterMap
 * 
 * @return RegisterMap* Pointer to the singleton RegisterMap instance
 */
RegisterMap* RegisterMap::getInstance() {
    if (!g_registerMap) {
        static std::mutex instanceMutex;
        std::lock_guard<std::mutex> lock(instanceMutex);
        if (!g_registerMap) {
            g_registerMap = new RegisterMap();
        }
    }
    return g_registerMap;
}