#include "../include/spectacularAI/rtabmap/camera_k4a.h"
#include "../include/spectacularAI/rtabmap/util.h"

#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UThread.h>

#ifdef SPECTACULARAI_CAMERA_K4A
#include <spectacularAI/k4a/plugin.hpp>
#include <spectacularAI/mapping.hpp>
#endif

namespace rtabmap {

bool CameraK4A::available() {
#ifdef SPECTACULARAI_CAMERA_K4A
    return true;
#else
    return false;
#endif
}

CameraK4A::CameraK4A(
    const std::string &recordingFolder,
    float imageRate,
    const rtabmap::Transform &localTransform) :
    CameraSpectacularAI(imageRate, localTransform)
#ifdef SPECTACULARAI_CAMERA_K4A
    ,
    recordingFolder(recordingFolder)
#endif
    {}

CameraK4A::~CameraK4A() {
#ifdef SPECTACULARAI_CAMERA_K4A
    session = nullptr;
#endif
}

bool CameraK4A::init(const std::string &calibrationFolder, const std::string &cameraName) {
    (void)calibrationFolder, (void)cameraName;

#ifdef SPECTACULARAI_CAMERA_K4A
    // Mapping API callback function to receive new, updated and deleted keyframes.
    auto mapperFn = [&](std::shared_ptr<const spectacularAI::mapping::MapperOutput> output) {
       this->mappingApiCallback(output);
    };

    // Create config for VIO and k4a device.
    spectacularAI::k4aPlugin::Configuration config;
    std::map<std::string, std::string> internalParameters;
    internalParameters.insert(std::make_pair("applyLoopClosures", "False")); // Let RTAB-Map handle loop closures
    internalParameters.insert(std::make_pair("skipFirstNCandidates", "10")); // Skip couple first keyframes to ensure gravity estimate is accurate.
    config.internalParameters = internalParameters;
    config.k4aConfig = spectacularAI::k4aPlugin::getK4AConfiguration(colorResolution, depthMode, frameRate);
    config.recordingFolder = recordingFolder;

    // Create vio pipeline using the config, and then start k4a device and VIO.
    vioPipeline = std::make_unique<spectacularAI::k4aPlugin::Pipeline >(config);
    vioPipeline->setMapperCallback(mapperFn);
    session = vioPipeline->startSession();

    return true;
#else
    return false;
#endif
}

bool CameraK4A::isCalibrated() const {
#ifdef SPECTACULARAI_CAMERA_K4A
    return model.isValidForProjection();
#else
    return false;
#endif
}

std::string CameraK4A::getSerial() const {
    return "";
}

SensorData CameraK4A::captureImage(CameraInfo *info) {
    SensorData data;

#ifdef SPECTACULARAI_CAMERA_K4A
    // Wait until new keyframe is available from mapping API.
    UTimer timer;
    while (!keyFrameData.isValid() && timer.elapsed() < NO_MORE_IMAGES_DELAY) {
        postPoseEvent();
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
#endif

    return data;
}

void CameraK4A::postPoseEvent() {
#ifdef SPECTACULARAI_CAMERA_K4A
    // Get the most recent vio pose estimate from the queue.
    std::shared_ptr<const spectacularAI::VioOutput> vioOutput;
    while (session->hasOutput()) {
        vioOutput = session->getOutput();
    }

    if (vioOutput) {
        auto &position = vioOutput->pose.position;
        auto &rotation = vioOutput->pose.orientation;
        Transform pose = Transform(position.x, position.y, position.z,
            rotation.x, rotation.y, rotation.z, rotation.w);

        this->post(new PoseEvent(pose));
    }
#endif
}

} // namespace rtabmap
