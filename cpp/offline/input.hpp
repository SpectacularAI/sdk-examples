#ifndef SPECTACULAR_AI_OFFLINE_INPUT_HPP
#define SPECTACULAR_AI_OFFLINE_INPUT_HPP

#include <cstdint>
#include <memory>

#include <spectacularAI/vio.hpp>

class Input {
public:
    struct Data {
        double timestamp = 0.0;
        uint8_t *video0 = nullptr;
        uint8_t *video1 = nullptr;
        std::vector<spectacularAI::MonocularFeature> features0;
        int width = -1;
        int height = -1;
        std::shared_ptr<spectacularAI::Vector3d> accelerometer;
        std::shared_ptr<spectacularAI::Vector3d> gyroscope;
        size_t nFrames = 0;
    };
    virtual bool next(Data &data) = 0;
    virtual std::string getConfig() const = 0;
    virtual std::string getCalibration() const = 0;
    virtual ~Input() {};

    static std::unique_ptr<Input> buildJsonl(const std::string &inputFolderPath);
    static std::unique_ptr<Input> buildMock();
};

#endif
