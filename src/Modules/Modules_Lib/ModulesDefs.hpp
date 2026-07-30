#pragma once

namespace ModuleDefs {
    enum class DeviceType {
        NullModule  = 0,
        MotorControllerModule,
        CameraControllerModule,
        UpdaterModule,
        CommsModule,
        CliModule,
        TelemetryModule
    };

    enum class AdapterId {
        CliAdapterID = 1,
        CommsAdapterID,
        MotorAdapterID, 
        CameraAdapterID,
        TlmAdapterID,
        UpdateAdapterID,
    };
    enum class NetworkPorts : unsigned int {
        // Command/control listener used by CommandController over ETH/WLAN.
        CommandDispatcherPort = 65000,

        // Video streaming ports.
        StreamPort = 5005,
        StreamOutPort = 5006,

        // Telemetry uplink port.
        TelemetryPort = 6000,

        // NetworkComms handshake listener.
        HandshakePort = 8192
    };
} // namespace ModuleDefs

namespace WebAppIface
{
    enum
    {
        INITIATE_UPDATE,
        READ_UPDATE_STATUS
    };

    constexpr static uint16_t WEB_APP_PROXY_PORT  = 8080; /*!< Web application proxy port */
    constexpr static uint16_t MAIN_APP_PROXY_PORT = 9090; /*!< Main application proxy port */
} // namespace WebAppIface

#pragma endregion