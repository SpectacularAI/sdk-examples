#include "../include/spectacularAI/rtabmap/camera_oak.h"
#include "../include/spectacularAI/rtabmap/util.h"

#include <rtabmap/utilite/UThread.h>

namespace rtabmap {

bool CameraOAK::available() {
#ifdef SPECTACULARAI_CAMERA_OAK
    return true;
#else
    return false;
#endif
}

CameraOAK::CameraOAK(
    const std::string &recordingFolder,
    float imageRate,
    const rtabmap::Transform &localTransform) :
    CameraSpectacularAI(imageRate, localTransform)
#ifdef SPECTACULARAI_CAMERA_OAK
    ,
    recordingFolder(recordingFolder)
#endif
    {}

CameraOAK::~CameraOAK() {
#ifdef SPECTACULARAI_CAMERA_OAK
    shouldQuit = true;
    session = nullptr;
#endif
}

bool CameraOAK::init(const std::string &calibrationFolder, const std::string &cameraName) {
    (void)calibrationFolder, (void)cameraName;

#ifdef SPECTACULARAI_CAMERA_OAK
    // Mapping API callback function to receive new, updated and deleted keyframes.
    auto mapperFn = [&](std::shared_ptr<const spectacularAI::mapping::MapperOutput> output) {
       this->mappingApiCallback(output);
    };

    // Create Depth AI (OAK-D) pipeline
    dai::Pipeline pipeline;

    // Optional configuration
    spectacularAI::daiPlugin::Configuration config;
    std::map<std::string, std::string> internalParameters;
    internalParameters.insert(std::make_pair("applyLoopClosures", "False")); // Let RTAB-Map handle loop closures
    internalParameters.insert(std::make_pair("skipFirstNCandidates", "10")); // Skip couple first keyframes to ensure gravity estimate is accurate.
    internalParameters.insert(std::make_pair("computeStereoPointCloud", "true"));
    internalParameters.insert(std::make_pair("pointCloudNormalsEnabled", "true"));
    config.internalParameters = internalParameters;
    config.recordingFolder = recordingFolder;
    // Example: enable these to support fisheye lenses (SDK 0.16+)
    // config.meshRectification = true;
    // config.depthScaleCorrection = true;

    vioPipeline = std::make_unique<spectacularAI::daiPlugin::Pipeline>(pipeline, config, mapperFn);

    // Connect to device and start pipeline
    device = std::make_shared<dai::Device>(pipeline);
    session = vioPipeline->startSession(*device);

    return true;
#else
    return false;
#endif
}

bool CameraOAK::isCalibrated() const {
#ifdef SPECTACULARAI_CAMERA_OAK
    return model.isValidForProjection();
#else
    return false;
#endif
}

std::string CameraOAK::getSerial() const {
    return "";
}

SensorData CameraOAK::captureImage(CameraInfo *info) {
    SensorData data;

#ifdef SPECTACULARAI_CAMERA_OAK
    // Wait until new keyframe is available from mapping API.
    while (!shouldQuit && !keyFrameData.isValid()) {
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

void CameraOAK::postPoseEvent() {
#ifdef SPECTACULARAI_CAMERA_OAK
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
