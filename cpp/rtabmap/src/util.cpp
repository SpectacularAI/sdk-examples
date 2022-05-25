#include "../include/spectacularAI/rtabmap/util.h"

#include <opencv2/imgproc/types_c.h>
#include <rtabmap/utilite/ULogger.h>

namespace rtabmap {
namespace util {
namespace {

LaserScan laserScanFromPointCloudXYZ(const std::shared_ptr<const spectacularAI::mapping::PointCloud> &cloud) {
    cv::Mat laserScan = cv::Mat(1, (int)cloud->size(), CV_32FC3);
    int oi = 0;

    for (unsigned int i = 0; i < cloud->size(); ++i) {
        float *ptr = laserScan.ptr<float>(0, oi++);
        const spectacularAI::Vector3d &p = cloud->getPosition(i);
        ptr[0] = p.x;
        ptr[1] = p.y;
        ptr[2] = p.z;
    }

    return LaserScan(laserScan(cv::Range::all(), cv::Range(0, oi)), 0, 0.0f, LaserScan::kXYZ);
}

LaserScan laserScanFromPointCloudXYZRGB(const std::shared_ptr<const spectacularAI::mapping::PointCloud> &cloud) {
    cv::Mat laserScan = cv::Mat(1, (int)cloud->size(), CV_32FC(4));
    int oi = 0;

    for (unsigned int i = 0; i < cloud->size(); ++i) {
        float *ptr = laserScan.ptr<float>(0, oi++);
        const spectacularAI::Vector3d &p = cloud->getPosition(i);
        const std::array<std::uint8_t, 3> &rgb = cloud->getRGB24(i);
        ptr[0] = p.x;
        ptr[1] = p.y;
        ptr[2] = p.z;
        int *ptrInt = (int *)ptr;
        ptrInt[3] = int(rgb[2]) | (int(rgb[1]) << 8) | (int(rgb[0]) << 16);
    }

    return LaserScan(laserScan(cv::Range::all(), cv::Range(0, oi)), 0, 0.0f, LaserScan::kXYZRGB);
}

LaserScan laserScanFromPointCloudXYZNormal(const std::shared_ptr<const spectacularAI::mapping::PointCloud> &cloud) {
    cv::Mat laserScan = cv::Mat(1, (int)cloud->size(), CV_32FC(6));
    int oi = 0;

    for (unsigned int i = 0; i < cloud->size(); ++i) {
        float *ptr = laserScan.ptr<float>(0, oi++);
        const spectacularAI::Vector3d &p = cloud->getPosition(i);
        const spectacularAI::Vector3d &n = cloud->getNormal(i);
        ptr[0] = p.x;
        ptr[1] = p.y;
        ptr[2] = p.z;
        ptr[3] = n.x;
        ptr[4] = n.y;
        ptr[5] = n.z;
    }

    return LaserScan(laserScan(cv::Range::all(), cv::Range(0, oi)), 0, 0.0f, LaserScan::kXYZNormal);
}

LaserScan laserScanFromPointCloudXYZRGBNormal(const std::shared_ptr<const spectacularAI::mapping::PointCloud> &cloud) {
    cv::Mat laserScan = cv::Mat(1, (int)cloud->size(), CV_32FC(7));
    int oi = 0;

    for (unsigned int i = 0; i < cloud->size(); ++i) {
        float *ptr = laserScan.ptr<float>(0, oi++);
        const spectacularAI::Vector3d &p = cloud->getPosition(i);
        const spectacularAI::Vector3d &n = cloud->getNormal(i);
        const std::array<std::uint8_t, 3> &rgb = cloud->getRGB24(i);
        ptr[0] = p.x;
        ptr[1] = p.y;
        ptr[2] = p.z;
        ptr[4] = n.x;
        ptr[5] = n.y;
        ptr[6] = n.z;
        int *ptrInt = (int *)ptr;
        ptrInt[3] = int(rgb[2]) | (int(rgb[1]) << 8) | (int(rgb[0]) << 16);
    }

    return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYZRGBNormal);
}

} // anonymous namespace

LaserScan laserScanFromPointCloud(const std::shared_ptr<const spectacularAI::mapping::PointCloud> &cloud) {
    if (cloud->empty())
        return LaserScan();
    else if (cloud->hasNormals() && cloud->hasColors())
        return laserScanFromPointCloudXYZRGBNormal(cloud);
    else if (cloud->hasNormals())
        return laserScanFromPointCloudXYZNormal(cloud);
    else if (cloud->hasColors())
        return laserScanFromPointCloudXYZRGB(cloud);
    else
        return laserScanFromPointCloudXYZ(cloud);
}

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

CameraModel convert(
    const std::shared_ptr<const spectacularAI::Camera> &camera,
    int width,  int height
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
        "",
        cv::Size(width, height),
        K, D, R, P,
        Transform::getIdentity());
}

cv::Mat convertColor(const std::shared_ptr<const spectacularAI::Bitmap> &image) {
    assert(image);
    cv::Mat bgr;
    const std::uint8_t* data = image->getDataReadOnly();
    int type = colorFormatToOpenCVType(image->getColorFormat());

    cv::Mat color = cv::Mat(
        image->getHeight(),
        image->getWidth(),
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

cv::Mat convertDepth(const std::shared_ptr<const spectacularAI::Bitmap> &image) {
    assert(image);
    assert(image->getColorFormat() == spectacularAI::ColorFormat::GRAY16);
    const std::uint8_t* data = image->getDataReadOnly();

    return cv::Mat(
        image->getHeight(),
        image->getWidth(),
        CV_16UC1,
        const_cast<uint8_t *>(data)).clone();
}

} // namespace util
} // namespace rtabmap
