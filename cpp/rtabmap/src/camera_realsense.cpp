#include "../include/spectacularAI/rtabmap/camera_realsense.h"
#include "../include/spectacularAI/rtabmap/util.h"

#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UThread.h>

#ifdef SPECTACULARAI_CAMERA_REALSENSE
#include <librealsense2/rs.hpp>
#include <spectacularAI/realsense/plugin.hpp>
#include <spectacularAI/mapping.hpp>
#endif

namespace rtabmap {

bool CameraRealsense::available() {
#ifdef SPECTACULARAI_CAMERA_REALSENSE
    return true;
#else
    return false;
#endif
}

CameraRealsense::CameraRealsense(
    const std::string &recordingFolder,
    float imageRate,
    const rtabmap::Transform &localTransform) :
    CameraSpectacularAI(imageRate, localTransform)
#ifdef SPECTACULARAI_CAMERA_REALSENSE
    ,
    recordingFolder(recordingFolder)
#endif
    {}

CameraRealsense::~CameraRealsense() {
#ifdef SPECTACULARAI_CAMERA_REALSENSE
    session = nullptr;
#endif
}

bool CameraRealsense::init(const std::string &calibrationFolder, const std::string &cameraName) {
    (void)calibrationFolder, (void)cameraName;
#ifdef SPECTACULARAI_CAMERA_REALSENSE
    // Mapping API callback function to receive new, updated and deleted keyframes.
    auto mapperFn = [&](std::shared_ptr<const spectacularAI::mapping::MapperOutput> output) {
       this->mappingApiCallback(output);
    };

    spectacularAI::rsPlugin::Configuration config;
    config.recordingFolder = recordingFolder;
    config.useSlam = true;
    spectacularAI::rsPlugin::Pipeline vioPipeline(config);
    {
        // Find RealSense device
        rs2::context rsContext;
        rs2::device_list devices = rsContext.query_devices();
        if (devices.size() != 1) {
            UERROR("Connect exactly one RealSense device!");
            return false;
        }
        rs2::device device = devices.front();
        vioPipeline.configureDevice(device);
    }

    // Start pipeline
    rs2::config rsConfig;
    vioPipeline.configureStreams(rsConfig);
    vioPipeline.setMapperCallback(mapperFn);
    session = vioPipeline.startSession(rsConfig);

    return true;
#else
    return false;
#endif
}

bool CameraRealsense::isCalibrated() const {
#ifdef SPECTACULARAI_CAMERA_REALSENSE
    return model.isValidForProjection();
#else
    return false;
#endif
}

std::string CameraRealsense::getSerial() const {
    return "";
}

SensorData CameraRealsense::captureImage(CameraInfo *info) {
    SensorData data;

#ifdef SPECTACULARAI_CAMERA_REALSENSE
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

void CameraRealsense::postPoseEvent() {
#ifdef SPECTACULARAI_CAMERA_REALSENSE
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
