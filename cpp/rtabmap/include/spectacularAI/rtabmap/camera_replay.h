#ifndef SPECTACULAR_AI_CAMERA_REPLAY_HPP
#define SPECTACULAR_AI_CAMERA_REPLAY_HPP

#include <string>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Camera.h>
#include <spectacularAI/vio.hpp>

namespace rtabmap {

class CameraReplay : public rtabmap::Camera {
public:
    static bool available();

public:
    CameraReplay(
        const std::string &dataFolder,
        float imageRate = 0,
        const Transform &localTransform = Transform::getIdentity());
    virtual ~CameraReplay();

    virtual bool init(const std::string &calibrationFolder = ".", const std::string &cameraName = "");
    virtual bool isCalibrated() const;
    virtual std::string getSerial() const;

    SensorData takeImage(CameraInfo *info = 0);

protected:
    virtual SensorData captureImage(CameraInfo *info = 0);

private:
    const std::string dataFolder;
    CameraModel model;

    std::map<int64_t, Transform> poses;
    std::map<int64_t, SensorData> keyFrames;
};

} // namespace rtabmap

#endif // SPECTACULAR_AI_CAMERA_REPLAY_HPP
