#pragma once

namespace ModuleDefs {
    enum class DeviceType {
        WIRELESS_COMMS,
        COMMAND_CONTROLLER,
        MOTOR_CONTROLLER,
        TELEMETRY_MODULE,
        CAMERA_CONTROLLER,
        VIDEO_STREAMER,
        CLI_INTERFACE,
        UPDATER_MODULE
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
}

#pragma endregion