#include "../include/spectacularAI/rtabmap/camera_replay.h"
#include "../include/spectacularAI/rtabmap/util.h"

#include <rtabmap/utilite/UThread.h>

#ifdef SPECTACULARAI_CORE
#include <spectacularAI/vio.hpp>
#include <spectacularAI/replay.hpp>
#endif

namespace rtabmap {

bool CameraReplay::available() {
#ifdef SPECTACULARAI_CORE
    return true;
#else
    return false;
#endif
}

CameraReplay::CameraReplay(
    const std::string& dataFolder,
    float imageRate,
    const rtabmap::Transform &localTransform) :
    CameraSpectacularAI(imageRate, localTransform),
    dataFolder(dataFolder) {}

CameraReplay::~CameraReplay() {
    shouldQuit = true;

    if (replayThread.joinable()) {
        replayThread.join();
    }
}

bool CameraReplay::init(const std::string &calibrationFolder, const std::string &cameraName) {
    (void)calibrationFolder, (void)cameraName;

    replayThread = std::thread(&CameraReplay::startReplay, this);

    return true;
}

bool CameraReplay::isCalibrated() const {
    return true;
}

std::string CameraReplay::getSerial() const {
    return "";
}

SensorData CameraReplay::captureImage(CameraInfo *info) {
    SensorData data;

    // Wait until new keyframe is available from mapping API.
    while (!shouldQuit && !keyFrameData.isValid()) {
        uSleep(1);
    }

    std::lock_guard<std::mutex> guard(dataMutex);
    if (keyFrameData.isValid()) {
        // Send keyframe data to RTAB-Map.
        data = keyFrameData;
        info->odomPose = keyFramePose;

        // Clear old data
        keyFrameData = SensorData();
    }

    return data;
}

void CameraReplay::startReplay() {
    #ifdef SPECTACULARAI_CORE

    // Mapping API callback function to receive new, updated and deleted keyframes.
    auto mapperFn = [&](std::shared_ptr<const spectacularAI::mapping::MapperOutput> output) {
       this->mappingApiCallback(output);
    };

    // Replay API callback function to capture vio poses for each frame (for smooth visualisation).
    auto vioFn = [&](spectacularAI::VioOutputPtr vioOutput) {
        if (vioOutput) {
            auto &position = vioOutput->pose.position;
            auto &rotation = vioOutput->pose.orientation;
            Transform pose = Transform(position.x, position.y, position.z,
                rotation.x, rotation.y, rotation.z, rotation.w);

            this->post(new PoseEvent(pose));
        }
    };

    std::string configYaml = "applyLoopClosures: False\n" // Disable loop closures & let RTAB-Map handle them.
        "skipFirstNCandidates: 10"; // Skip couple first keyframes to ensure gravity estimate is accurate.

    spectacularAI::Vio::Builder vioBuilder = spectacularAI::Vio::builder()
        .setConfigurationYAML(configYaml)
        .setMapperCallback(mapperFn);
    auto replayApi = spectacularAI::Replay::builder(dataFolder, vioBuilder).build();
    replayApi->setOutputCallback(vioFn);

    // Main loop, reads data from replay one line at a time.
    bool moreData = true;
    while (moreData && !shouldQuit) {
        if (!keyFrameData.isValid()) {
            moreData = replayApi->replayOneLine();
        } else {
            uSleep(1);
        }
    }

    #endif
}

} // namespace rtabmap
