#ifndef SPECTACULAR_AI_CAMERA_K4A_HPP
#define SPECTACULAR_AI_CAMERA_K4A_HPP

#include "camera_availability.h"
#include "camera_spectacularai.h"

#ifdef SPECTACULARAI_CAMERA_K4A
#include <spectacularAI/k4a/plugin.hpp>
#endif

namespace rtabmap {

class CameraK4A : public CameraSpectacularAI {
public:
    static bool available();

public:
    CameraK4A(
        float imageRate = 0,
        const Transform &localTransform = Transform::getIdentity());
    virtual ~CameraK4A();

    virtual bool init(const std::string &calibrationFolder = ".", const std::string &cameraName = "");
    virtual bool isCalibrated() const;
    virtual std::string getSerial() const;

protected:
    virtual SensorData captureImage(CameraInfo *info = 0);

private:
    void postPoseEvent();

#ifdef SPECTACULARAI_CAMERA_K4A
    std::shared_ptr<spectacularAI::k4aPlugin::Session> session;
#endif
};

} // namespace rtabmap

#endif // SPECTACULAR_AI_CAMERA_K4A_HPP
