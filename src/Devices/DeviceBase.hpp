#pragma once

namespace Device {
    class DeviceBase
    {
    private:
        /* data */
    public:
        DeviceBase(/* args */) {}
        ~DeviceBase() {}

        DeviceBase* getDev() {
            return this;
        }
    };
}

#pragma endregion