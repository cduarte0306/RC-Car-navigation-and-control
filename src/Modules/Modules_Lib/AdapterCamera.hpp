#pragma once
#ifndef ADAPTERCAMERA_HPP
#define ADAPTERCAMERA_HPP

#include "AdapterBase.hpp"

namespace Adapter {
    class CameraAdapter : public AdapterBase {
    public:

        // Camera module commands
        enum {
            CmdStartStream,           // start video stream
            CmdStopStream,            // stop video stream
            CmdSelCameraStream,       // select camera stream (Normal or training)
            CmdSetFps,                // set streaming parameters (host IP, port, etc)
            CmdSetQuality,            // Set the stream compresison quality
        
            // Stereo parameters
            CmdSetMinDisparities,      // Set maximum disparities
            CmdSetMaxDisparities,
            CmdSetConfidenceThreshold,
            CmdSetUniquenessRatio,    // Sets the uniqueness ratio
            CmdSetP1,                 // Sets StereoSGM P1
            CmdSetP2,                 // Sets StereoSGM P2
            CmdSetZMax,
            CmdSetZMin,
            CmdSetDepthThreshold,
            CmdSetMinAgreeingPixels,
            CmdSetColorThreshold,

            CmdRdParams,              // Command to read all configured parameteres
            CmdClrVideoRec,           // clear video recording buffer
            CmdSaveVideo,             // save video recording to disks
            CmdLoadStoredVideos,      // Read stored videos from disk
            CmdLoadSelectedVideo,     // load selected video from disk
            CmdDeleteVideo,           // Delete video
            CmdCalibrationSetState,   // Start video calibration
            CmdCalibrationWrtParams,  // Sets calibration paramters
            CmdCalibrationReset,      // Reset calibration
            CmdCalibrationSave,       // Save calibration to disk
        };

        CameraAdapter(std::string parentName_="");

        int setCameraState(bool state);

        int StartStreaming();

        int StopStreaming();

        int SetStreamingState(bool state);

        /**
         * @brief Read stats from module
         * 
         * @return std::string 
         */
        virtual std::string readStats() override;

    protected:
        std::function<int(bool)>                    setCameraStateCommand    = nullptr;
        std::function<int(void)>                    startStreamingCommand    = nullptr;
        std::function<int(void)>                    stopStreamingCommand     = nullptr;

        virtual int bind_(AdapterBase* Adapter) override;

        void bindInterface(CameraAdapter* adapter);

        virtual int startStreaming_();

        virtual int stopStreaming_();

        virtual int setCameraState_(int direction);

        virtual int configurePipeline_(const std::string& host);
    };
}

#endif // ADAPTERCAMERA_HPP
