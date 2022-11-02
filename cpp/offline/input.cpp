#include "input.hpp"

#include "ffmpeg.hpp"

#include <fstream>
#include <sstream>

#include <nlohmann/json.hpp>
#include <lodepng/lodepng.h>

namespace {

const std::string SEPARATOR = "/";

using json = nlohmann::json;

// Format strings in printf style.
template<typename ... Args>
std::string stringFormat(const std::string &f, Args ... args) {
    int n = std::snprintf(nullptr, 0, f.c_str(), args ...) + 1;
    assert(n > 0);
    auto nn = static_cast<size_t>(n);
    auto buf = std::make_unique<char[]>(nn);
    std::snprintf(buf.get(), nn, f.c_str(), args ...);
    return std::string(buf.get(), buf.get() + nn - 1);
}

std::vector<std::string> findVideos(const std::string &folderPath, bool &isImageFolder) {
    isImageFolder = false;
    std::vector<std::string> videos;
    for (size_t cameraInd = 0; cameraInd < 2; ++cameraInd) {
        std::string videoPathNoSuffix = folderPath + SEPARATOR + "data";
        if (cameraInd > 0) videoPathNoSuffix += std::to_string(cameraInd + 1);
        for (std::string suffix : { "mov", "avi", "mp4", "mkv" }) {
            const std::string videoPath = videoPathNoSuffix + "." + suffix;
            std::ifstream testFile(videoPath);
            if (testFile.is_open()) videos.push_back(videoPath);
        }

        const std::string imageFolder = stringFormat("%s/frames%d", folderPath.c_str(), cameraInd + 1);
        const std::string firstImage = stringFormat("%s/%08d.png", imageFolder.c_str(), 0);
        std::ifstream testFile(firstImage);
        if (testFile.is_open()) isImageFolder = true;
    }
    return videos;
}

// Read PNG image to buffer.
bool readImage(
    const std::string &filePath,
    std::vector<uint8_t> &data,
    std::vector<uint8_t> &tmpBuffer,
    int &width,
    int &height
) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        printf("No such file %s\n", filePath.c_str());
        return false;
    }
    tmpBuffer.clear();
    unsigned w, h;
    unsigned error = lodepng::decode(tmpBuffer, w, h, filePath);
    if (error) {
        printf("Error %s\n", lodepng_error_text(error));
        return false;
    }
    assert(tmpBuffer.size() == 4 * w * h);
    data.resize(w * h);
    for (size_t i = 0; i < w * h; ++i) {
        // Get green channel from RGBA. Any reasonable gray-scale conversion should work for VIO.
        data[i] = tmpBuffer.at(4 * i + 1);
    }
    width = static_cast<int>(w);
    height = static_cast<int>(h);
    return true;
}

class InputJsonl : public Input {
public:
    InputJsonl(const std::string &inputFolderPath) :
        inputFolderPath(inputFolderPath)
    {
        imu = std::make_shared<spectacularAI::Vector3d>(spectacularAI::Vector3d{ 0.0, 0.0, 0.0 });
        jsonlFile.open(inputFolderPath + SEPARATOR + "data.jsonl");
        if (!jsonlFile.is_open()) {
            printf("No data.jsonl file found. Does `%s` exist?\n", inputFolderPath.c_str());
            assert(false);
        }
        videoPaths = findVideos(inputFolderPath, useImageInput);
        assert(!videoPaths.empty() || useImageInput);
        for (const std::string &videoPath : videoPaths) {
            videoInputs.push_back(std::make_unique<VideoInput>(videoPath));
        }
    }

    std::string getConfig() const final {
        std::ifstream configFile(inputFolderPath + SEPARATOR + "vio_config.yaml");
        if (!configFile.is_open()) {
            printf("No vio_config.yaml provided, using default config.\n");
            return "";
        }
        std::ostringstream oss;
        oss << configFile.rdbuf();
        return oss.str();
    }

    std::string getCalibration() const final {
        std::ifstream calibrationFile(inputFolderPath + SEPARATOR + "calibration.json");
        // Calibration is always required.
        assert(calibrationFile.is_open());
        std::ostringstream oss;
        oss << calibrationFile.rdbuf();
        return oss.str();
    }

    bool next(Data &data) final {
        if (!std::getline(jsonlFile, line)) return false;
        data.video0 = nullptr;
        data.video1 = nullptr;
        data.accelerometer = nullptr;
        data.gyroscope = nullptr;

        json j = json::parse(line, nullptr, false); // stream, callback, allow_exceptions
        data.timestamp = j["time"].get<double>();

        if (j.find("sensor") != j.end()) {
            std::array<double, 3> v = j["sensor"]["values"];
            *imu = { .x = v[0], .y = v[1], .z = v[2] };
            const std::string sensorType = j["sensor"]["type"];
            if (sensorType == "gyroscope") {
                data.gyroscope = imu;
            }
            else if (sensorType == "accelerometer") {
                data.accelerometer = imu;
            }
        }
        else if (j.find("frames") != j.end()) {
            json jFrames = j["frames"];
            size_t cameraCount = jFrames.size();
            assert(cameraCount >= 1);
            int number = j["number"].get<int>();
            for (size_t cameraInd = 0; cameraInd < cameraCount; ++cameraInd) {
                std::vector<uint8_t> &video = cameraInd == 0 ? video0 : video1;
                if (useImageInput) {
                    std::string filePath = stringFormat("%s/frames%zu/%08zu.png",
                        inputFolderPath.c_str(), cameraInd + 1, number);
                    bool success = readImage(filePath, video, tmpBuffer, data.width, data.height);
                    assert(success);
                }
                else {
                    bool success = videoInputs.at(cameraInd)->read(video, data.width, data.height);
                    assert(success);
                }
                uint8_t *&dataVideo = cameraInd == 0 ? data.video0 : data.video1;
                dataVideo = video.data();
            }
        }
        return true;
    }

private:
    std::ifstream jsonlFile;
    std::string line;
    std::shared_ptr<spectacularAI::Vector3d> imu;
    const std::string inputFolderPath;
    std::vector<uint8_t> video0, video1, tmpBuffer;
    bool useImageInput = false;
    std::vector<std::string> videoPaths;
    std::vector<std::unique_ptr<VideoInput>> videoInputs;
};

class InputMock : public Input {
public:
    int n = 0;
    const int width;
    const int height;
    std::vector<uint8_t> video0, video1;
    std::shared_ptr<spectacularAI::Vector3d> imu;

    InputMock() : width(640), height(480),
        video0(width * height), video1(width * height)
    {
        imu = std::make_shared<spectacularAI::Vector3d>(spectacularAI::Vector3d{ 0.0, 0.0, 0.0 });
        printf("Using mock input, VIO may not output anything.\n");
    }

    bool next(Data &data) final {
        const size_t ITERATIONS = 100;
        data.video0 = video0.data();
        data.video1 = video1.data();
        data.width = width;
        data.height = height;
        data.accelerometer = imu;
        data.gyroscope = imu;
        data.timestamp += 0.1;
        return n++ < ITERATIONS;
    }

    std::string getConfig() const final {
        return "";
    }

    std::string getCalibration() const final {
        return R"({ "cameras": [
            { "focalLengthX": 1.0, "focalLengthY": 1.0, "model": "pinhole", "imuToCamera": [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]] },
            { "focalLengthX": 1.0, "focalLengthY": 1.0, "model": "pinhole", "imuToCamera": [[1,0,0,1],[0,1,0,0],[0,0,1,0],[0,0,0,1]] }
        ] })";
    }
};

} // anonymous namespace

std::unique_ptr<Input> Input::buildJsonl(const std::string &inputFolderPath) {
    return std::unique_ptr<Input>(
        new InputJsonl(inputFolderPath));
}

std::unique_ptr<Input> Input::buildMock() {
    return std::unique_ptr<Input>(
        new InputMock());
}
