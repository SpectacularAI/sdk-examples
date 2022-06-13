#ifndef SPECTACULAR_AI_CAMERA_SPECTACULARAI_HPP
#define SPECTACULAR_AI_CAMERA_SPECTACULARAI_HPP

#include <string>
#include <thread>
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/utilite/UEventsSender.h>

#include <spectacularAI/mapping.hpp>

namespace rtabmap {

class CameraSpectacularAI : public Camera, public UEventsSender {
public:
    static bool available();

public:
    CameraSpectacularAI(
        float imageRate = 0,
        const Transform &localTransform = Transform::getIdentity());
    virtual ~CameraSpectacularAI();

    virtual bool init(const std::string &calibrationFolder = ".", const std::string &cameraName = "") = 0;
    virtual bool isCalibrated() const = 0;
    virtual std::string getSerial() const = 0;

protected:
    virtual void mappingApiCallback(std::shared_ptr<const spectacularAI::mapping::MapperOutput> output);

    CameraModel model;

    bool shouldQuit = false;
    uint64_t latestProcessedKeyFrameId = 0;
    SensorData keyFrameData;
    Transform keyFramePose;
    std::mutex dataMutex;
};

} // namespace rtabmap

#endif // SPECTACULAR_AI_CAMERA_SPECTACULARAI_HPP
