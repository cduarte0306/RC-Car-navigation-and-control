#pragma once

namespace ModuleDefs {
    enum class DeviceType {
        NullModule  = 0,
        CommsModule,
        CommandAdapterModule,
        MotorControllerModule,
        TelemetryModule,
        CameraControllerModule,
        CliModule,
        UpdaterModule
    };

    enum class AdapterId {
        CliAdapterID = 1,
        CommsAdapterID,
        CommandAdapterID,
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
}

#pragma endregion