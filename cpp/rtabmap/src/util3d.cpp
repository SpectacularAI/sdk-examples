#include "../include/spectacularAI/rtabmap/util3d.h"

namespace rtabmap {
namespace util3d {
namespace {

LaserScan laserScanFromPointCloudXYZ(std::shared_ptr<const spectacularAI::mapping::PointCloud> &cloud) {
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

LaserScan laserScanFromPointCloudXYZRGB(std::shared_ptr<const spectacularAI::mapping::PointCloud> &cloud) {
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

LaserScan laserScanFromPointCloudXYZNormal(std::shared_ptr<const spectacularAI::mapping::PointCloud> &cloud) {
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

LaserScan laserScanFromPointCloudXYZRGBNormal(std::shared_ptr<const spectacularAI::mapping::PointCloud> &cloud) {
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

LaserScan laserScanFromPointCloud(std::shared_ptr<const spectacularAI::mapping::PointCloud> &cloud) {
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

} // namespace util3d
} // namespace rtabmap
