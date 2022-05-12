#ifndef SPECTACULAR_AI_UTIL3D_HPP
#define SPECTACULAR_AI_UTIL3D_HPP

#include <spectacularAI/vio.hpp>
#include <rtabmap/core/LaserScan.h>

namespace rtabmap {
namespace util3d {

LaserScan laserScanFromPointCloud(std::shared_ptr<const spectacularAI::mapping::PointCloud> &cloud);

} // namespace util3d
} // namespace rtabmap

#endif // SPECTACULAR_AI_UTIL3D_HPP
