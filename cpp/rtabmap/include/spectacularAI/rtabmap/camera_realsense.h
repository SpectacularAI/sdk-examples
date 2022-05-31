#ifndef SPECTACULAR_AI_CAMERA_REALSENSE_HPP
#define SPECTACULAR_AI_CAMERA_REALSENSE_HPP

#include "camera_availability.h"
#include "camera_spectacularai.h"

#ifdef SPECTACULARAI_CAMERA_REALSENSE
#include <spectacularAI/realsense/plugin.hpp>
#endif

namespace rtabmap {

class CameraRealsense : public CameraSpectacularAI {
public:
    static bool available();

public:
    CameraRealsense(
        const std::string &recordingFolder,
        float imageRate = 0,
        const Transform &localTransform = Transform::getIdentity());
    virtual ~CameraRealsense();

    virtual bool init(const std::string &calibrationFolder = ".", const std::string &cameraName = "");
    virtual bool isCalibrated() const;
    virtual std::string getSerial() const;

protected:
    virtual SensorData captureImage(CameraInfo *info = 0);

private:
    void postPoseEvent();

#ifdef SPECTACULARAI_CAMERA_REALSENSE
    std::unique_ptr<spectacularAI::rsPlugin::Session> session;
    const std::string recordingFolder;
#endif
};

} // namespace rtabmap

#endif // SPECTACULAR_AI_CAMERA_REALSENSE_HPP
