#ifndef SPECTACULAR_AI_UTIL3D_HPP
#define SPECTACULAR_AI_UTIL3D_HPP

#include <string>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/utilite/UEvent.h>
#include <rtabmap/core/Transform.h>

#include <spectacularAI/types.hpp>
#include <spectacularAI/mapping.hpp>

namespace rtabmap {

class PoseEvent: public UEvent {
public:
	PoseEvent(const Transform & pose) : pose_(pose) {}
	virtual std::string getClassName() const { return "PoseEvent"; }
	const Transform & pose() const { return pose_; }

private:
	Transform pose_;
};

namespace util {

LaserScan laserScanFromPointCloud(const std::shared_ptr<const spectacularAI::mapping::PointCloud> &cloud);
int colorFormatToOpenCVType(spectacularAI::ColorFormat colorFormat);
cv::Mat convert(const spectacularAI::Matrix3d &m);
Transform convert(const spectacularAI::Pose &pose);
CameraModel convert(const std::shared_ptr<const spectacularAI::Camera> &camera, int width,  int height);
cv::Mat convertColor(const std::shared_ptr<const spectacularAI::Bitmap> &image);
cv::Mat convertDepth(const std::shared_ptr<const spectacularAI::Bitmap> &image);

} // namespace util
} // namespace rtabmap

#endif // SPECTACULAR_AI_UTIL3D_HPP
