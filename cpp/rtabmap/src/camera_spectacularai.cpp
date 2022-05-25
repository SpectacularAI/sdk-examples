#include "../include/spectacularAI/rtabmap/camera_spectacularai.h"
#include "../include/spectacularAI/rtabmap/util.h"

#include <rtabmap/utilite/ULogger.h>

namespace rtabmap {

bool CameraSpectacularAI::available() {
    return true;
}

CameraSpectacularAI::CameraSpectacularAI(
    float imageRate,
    const rtabmap::Transform &localTransform) :
    Camera(imageRate, localTransform) {}

CameraSpectacularAI::~CameraSpectacularAI() {}

void CameraSpectacularAI::mappingApiCallback(std::shared_ptr<const spectacularAI::mapping::MapperOutput> output) {
    if (output->map->keyFrames.empty()) return;

    // Get the newest keyframe from mapping API.
    auto latestKeyFrameIt = output->map->keyFrames.rbegin();
    int64_t latestKeyFrameId = latestKeyFrameIt->first;
    auto &frameSet = latestKeyFrameIt->second->frameSet;

    // Ensure the latest key frame is new.
    if (latestKeyFrameId <= latestProcessedKeyFrameId)
        return;

    UINFO("New keyframe with id=%ld", latestKeyFrameId);

    double timestamp = frameSet->rgbFrame->cameraPose.pose.time;
    cv::Mat bgr;
    cv::Mat depth;

    if (!frameSet->rgbFrame || !frameSet->depthFrame) {
        UWARN("Keyframe %ld has color/depth image! Discarding...", latestKeyFrameId);
        return;
    }

    std::shared_ptr<spectacularAI::mapping::Frame> rgbFrame = frameSet->getUndistortedFrame(frameSet->rgbFrame);
    std::shared_ptr<spectacularAI::mapping::Frame> depthFrame = frameSet->getAlignedDepthFrame(rgbFrame);

    if (!model.isValidForProjection()) {
        model = util::convert(rgbFrame->cameraPose.camera, rgbFrame->image->getWidth(), rgbFrame->image->getHeight());
        assert(model.isValidForProjection());
    }

    bgr = util::convertColor(rgbFrame->image);
    depth = util::convertDepth(depthFrame->image);

    SensorData data = SensorData(bgr, depth, model, latestKeyFrameId, timestamp);

    if (latestKeyFrameIt->second->pointCloud) {
        LaserScan scan = util::laserScanFromPointCloud(latestKeyFrameIt->second->pointCloud);
        data.setLaserScan(scan);
    }

    std::lock_guard<std::mutex> guard(dataMutex);
    latestProcessedKeyFrameId = latestKeyFrameId;
    keyFramePose = util::convert(frameSet->rgbFrame->cameraPose.pose);
    keyFrameData = data;
}

} // namespace rtabmap
