#ifndef SPECTACULAR_AI_CAMERA_OAK_HPP
#define SPECTACULAR_AI_CAMERA_OAK_HPP

#include "camera_spectacularai.h"

#ifdef SPECTACULARAI_CAMERA_OAK
#include <spectacularAI/depthai/plugin.hpp>
#include <depthai/depthai.hpp>
#endif

namespace rtabmap {

class CameraOAK : public CameraSpectacularAI {
public:
    static bool available();

public:
    CameraOAK(
        const std::string &recordingFolder,
        float imageRate = 0,
        const Transform &localTransform = Transform::getIdentity());
    virtual ~CameraOAK();

    virtual bool init(const std::string &calibrationFolder = ".", const std::string &cameraName = "");
    virtual bool isCalibrated() const;
    virtual std::string getSerial() const;

protected:
    virtual SensorData captureImage(CameraInfo *info = 0);

private:
    void postPoseEvent();

#ifdef SPECTACULARAI_CAMERA_OAK
    std::shared_ptr<dai::Device> device;
    std::unique_ptr<spectacularAI::daiPlugin::Session> session;
    std::unique_ptr<spectacularAI::daiPlugin::Pipeline> vioPipeline;

    const std::string recordingFolder;
#endif
};

} // namespace rtabmap

#endif // SPECTACULAR_AI_CAMERA_OAK_HPP
