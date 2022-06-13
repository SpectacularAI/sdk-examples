#ifndef SPECTACULAR_AI_CAMERA_REPLAY_HPP
#define SPECTACULAR_AI_CAMERA_REPLAY_HPP

#include "camera_spectacularai.h"

namespace rtabmap {

class CameraReplay : public CameraSpectacularAI {
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

protected:
    virtual SensorData captureImage(CameraInfo *info = 0);

private:
    void startReplay();

    const std::string dataFolder;

    std::thread replayThread;
};

} // namespace rtabmap

#endif // SPECTACULAR_AI_CAMERA_REPLAY_HPP
