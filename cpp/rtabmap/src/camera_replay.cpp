#include "../include/spectacularAI/rtabmap/camera_replay.h"
#include "../include/spectacularAI/rtabmap/util3d.h"

#include <opencv2/imgproc/types_c.h>

#include <spectacularAI/vio.hpp>
#include <spectacularAI/replay.hpp>
#include <rtabmap/utilite/ULogger.h>

namespace rtabmap {
namespace {

int colorFormatToOpenCVType(spectacularAI::ColorFormat colorFormat) {
    switch (colorFormat) {
        case spectacularAI::ColorFormat::GRAY: return CV_8UC1;
        case spectacularAI::ColorFormat::RGB: return CV_8UC3;
        case spectacularAI::ColorFormat::RGBA: return CV_8UC4;
        default: UFATAL("Unknown color format!"); return -1;
    }
}

cv::Mat convert(const spectacularAI::Matrix3d &m) {
    cv::Mat mat = cv::Mat::zeros(3, 3, CV_64FC1);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
	    	mat.at<double>(i, j) = m[i][j];
        }
    }

    return mat;
}

Transform convert(const spectacularAI::Pose &pose) {
    const spectacularAI::Vector3d& p = pose.position;
    const spectacularAI::Quaternion& q = pose.orientation;

    return Transform(p.x, p.y, p.z, q.x, q.y, q.z, q.w);
}

cv::Mat convertColor(const std::shared_ptr<spectacularAI::mapping::Frame> &frame) {
    assert(frame);
    cv::Mat bgr;
    const std::uint8_t* data = frame->image->getDataReadOnly();
    int type = colorFormatToOpenCVType(frame->image->getColorFormat());

    cv::Mat color = cv::Mat(
        frame->image->getHeight(),
        frame->image->getWidth(),
        type,
        const_cast<uint8_t *>(data)).clone();

    if (type == CV_8UC4) {
        cv::cvtColor(color, bgr, cv::COLOR_RGBA2BGR);
    } else if (type == CV_8UC3) {
        cv::cvtColor(color, bgr, cv::COLOR_RGB2BGR);
    } else if (type == CV_8UC1) {
        cv::cvtColor(color, bgr, cv::COLOR_GRAY2BGR);
    }

    return bgr;
}

cv::Mat convertDepth(const std::shared_ptr<spectacularAI::mapping::Frame> &frame) {
    assert(frame);
    assert(frame->image->getColorFormat() == spectacularAI::ColorFormat::GRAY16);
    const std::uint8_t* data = frame->image->getDataReadOnly();

    return cv::Mat(
        frame->image->getHeight(),
        frame->image->getWidth(),
        CV_16UC1,
        const_cast<uint8_t *>(data)).clone();
}

CameraModel convert(
    const std::string &cameraName,
    const std::shared_ptr<const spectacularAI::Camera> &camera,
    int width,  int height,
    const Transform &localTransform
) {
    // K is the camera intrinsic 3x3 CV_64FC1
    // D is the distortion coefficients 1x5 CV_64FC1
    // R is the rectification matrix 3x3 CV_64FC1 (computed from stereo or Identity)
    // P is the projection matrix 3x4 CV_64FC1 (computed from stereo or equal to [K [0 0 1]'])
    cv::Mat K = convert(camera->getIntrinsicMatrix());
    cv::Mat D = cv::Mat::zeros(1, 5, CV_64FC1);
    cv::Mat R = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat P = cv::Mat::zeros(3, 4, CV_64FC1);
    P.at<double>(0, 0) = K.at<double>(0, 0);
    P.at<double>(1, 1) = K.at<double>(1, 1);
    P.at<double>(0, 2) = K.at<double>(0, 2);
    P.at<double>(1, 2) = K.at<double>(1, 2);
    P.at<double>(2, 3) = 1;

    return CameraModel(
        cameraName,
        cv::Size(width, height),
        K, D, R, P,
        localTransform);
}

} // anonymous namespace

bool CameraReplay::available() {
    return true;
}

CameraReplay::CameraReplay(
    const std::string& dataFolder,
    float imageRate,
    const rtabmap::Transform &localTransform) :
    Camera(imageRate, localTransform),
    dataFolder(dataFolder) {}

CameraReplay::~CameraReplay() {}

bool CameraReplay::init(const std::string &calibrationFolder, const std::string &cameraName) {
    (void)calibrationFolder;

    auto mapperFn = [&](std::shared_ptr<const spectacularAI::mapping::MapperOutput> output) {
        for (int64_t frameId : output->updatedKeyframes) {
            auto search = output->map->keyframes.find(frameId);

            // Remove deleted keyframes.
            if (search == output->map->keyframes.end()) {
                keyFrames.erase(frameId);
                poses.erase(frameId);
                UINFO("Deleted keyframe with id=%ld.", frameId);
                continue;
            }

            // Update keyframe pose.
            auto& frameSet = search->second->frameSet;
            poses[frameId] = convert(frameSet.rgbFrame->cameraPose.pose);

            // Only save images & point cloud for new keyframes.
            if (keyFrames.find(frameId) != keyFrames.end()) continue;
            UINFO("New keyframe with id=%ld", frameId);

            double timestamp = frameSet.rgbFrame->cameraPose.pose.time;
            cv::Mat bgr;
            cv::Mat depth;

            if (frameSet.rgbFrame) {
                bgr = convertColor(frameSet.rgbFrame);
            } else {
                UWARN("Keyframe %ld has no color image! Discarding...", frameId);
                continue;
            }

            if (frameSet.depthFrame) {
                depth = convertDepth(frameSet.depthFrame);
            } else {
                UWARN("Keyframe %ld has no depth image! Discarding...", frameId);
                continue;
            }

            if (!model.isValidForProjection()) {
                model = convert(
                    cameraName,
                    frameSet.rgbFrame->cameraPose.camera,
                    frameSet.rgbFrame->image->getWidth(),
                    frameSet.rgbFrame->image->getHeight(),
                    Transform::getIdentity());

                assert(model.isValidForProjection());
            }

            SensorData data = SensorData(bgr, depth, model, frameId, timestamp);

            if (search->second->pointCloud) {
                LaserScan scan = util3d::laserScanFromPointCloud(search->second->pointCloud);
                data.setLaserScan(scan);
            }

            keyFrames[frameId] = data;
        }
    };

    spectacularAI::Vio::Builder vioBuilder = spectacularAI::Vio::builder().setMapperCallback(mapperFn);
    auto replayApi = spectacularAI::Replay::builder(dataFolder, vioBuilder).build();
    replayApi->startReplay();

    return true;
}

bool CameraReplay::isCalibrated() const {
    return true;
}

std::string CameraReplay::getSerial() const {
    return "";
}

SensorData CameraReplay::takeImage(CameraInfo *info) {
    return captureImage(info);
}

SensorData CameraReplay::captureImage(CameraInfo *info) {
    SensorData data;

    if (!keyFrames.empty()) {
        auto it = keyFrames.begin();
        int64_t id = it->first;
        data = it->second;

        auto poseIt = poses.find(id);
        if (poseIt != poses.end()) {
            if (info != nullptr)
                info->odomPose = poseIt->second;
            poses.erase(poseIt);
        } else {
            UWARN("No pose available for keyframe with id=%ld", id);
        }

        keyFrames.erase(it);
    }

    return data;
}

} // namespace rtabmap
