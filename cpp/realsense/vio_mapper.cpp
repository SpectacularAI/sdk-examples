#include <iostream>
#include <fstream>
#include <librealsense2/rs.hpp>
#include <spectacularAI/realsense/plugin.hpp>
#include <spectacularAI/mapping.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cassert>
#include <string>
#include <cstdint>
#include <thread>
#include <chrono>
#include <deque>
#include <atomic>
#include <cstdlib>
#include <set>

namespace {
struct ImageToSave {
    std::string fileName;
    cv::Mat mat;
};

int colorFormatToOpenCVType(spectacularAI::ColorFormat colorFormat) {
    switch (colorFormat) {
        case spectacularAI::ColorFormat::GRAY: return CV_8UC1;
        case spectacularAI::ColorFormat::GRAY16: return CV_16UC1;
        case spectacularAI::ColorFormat::RGB: return CV_8UC3;
        case spectacularAI::ColorFormat::RGBA: return CV_8UC4;
        default: return -1;
    }
}

std::function<void()> buildImageWriter(
    std::deque<ImageToSave> &queue,
    std::mutex &mutex,
    std::atomic<bool> &shouldQuit)
{
    return [&queue, &mutex, &shouldQuit]() {
        while (!shouldQuit) {
            std::unique_lock<std::mutex> lock(mutex);
            constexpr int LOOP_SLEEP_MS = 10;

            if (queue.empty()) {
                lock.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_SLEEP_MS));
                continue;
            }

            auto img = queue.front();
            queue.pop_front();
            lock.unlock();
            cv::imwrite(img.fileName.c_str(), img.mat);
        }
    };
}

cv::Mat copyImage(std::shared_ptr<const spectacularAI::Bitmap> bitmap) {
        int cvType = colorFormatToOpenCVType(bitmap->getColorFormat());
        return cv::Mat(
            bitmap->getHeight(),
            bitmap->getWidth(),
            cvType,
            const_cast<std::uint8_t *>(bitmap->getDataReadOnly())
        ).clone();
}

std::string matrix4ToString(const spectacularAI::Matrix4d &matrix) {
    std::stringstream ss;
    ss << "[";
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            ss << matrix[i][j];
    ss << "]";
    return ss.str();
}

void serializePosesToFile(std::ofstream& posesFile, std::shared_ptr<spectacularAI::mapping::KeyFrame> keyframe) {
    auto& frameSet = keyframe->frameSet;
    std::stringstream ss;
    ss << "{\"frameId\": " << (keyframe->id) << ","
        << "\"poses\": {";
    if (frameSet.rgbFrame) {
        ss << "\"rgb\": " << matrix4ToString(frameSet.rgbFrame->cameraPose.pose.asMatrix());
    }
    if (frameSet.depthFrame) {
        if (frameSet.rgbFrame) ss << ",";
        ss << "\"depth\": " << matrix4ToString(frameSet.depthFrame->cameraPose.pose.asMatrix());
    }
    ss << "}}";
    std::cout << "saving " << ss.str() << std::endl;
    posesFile << ss.str() << std::endl;
}
} // namespace

int main(int argc, char** argv) {
    // If a folder is given as an argument, record session there
    std::string recordingFolder;
    if (argc >= 2) {
        recordingFolder = argv[1];
    } else {
        std::cerr
            << "Usage: " << argv[0] << " /path/to/recording/folder" << std::endl;
        return 1;
    }

    spectacularAI::rsPlugin::Configuration vioConfig;
    vioConfig.useSlam = true;
    spectacularAI::rsPlugin::Pipeline vioPipeline(vioConfig);

    {
        // Find RealSense device
        rs2::context rsContext;
        rs2::device_list devices = rsContext.query_devices();
        if (devices.size() != 1) {
            std::cout << "Connect exactly one RealSense device." << std::endl;
            return EXIT_SUCCESS;
        }
        rs2::device device = devices.front();
        vioPipeline.configureDevice(device);
    }

    // Start pipeline
    rs2::config rsConfig;
    vioPipeline.configureStreams(rsConfig);

    // VIO works fine with BGR-flipped data too
    rsConfig.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGR8);

    // The RS callback thread should not be blocked for long.
    // Using worker threads for image encoding and disk I/O
    constexpr int N_WORKER_THREADS = 4;
    std::mutex queueMutex;
    std::deque<ImageToSave> imageQueue;
    std::atomic<bool> shouldQuit(false);

    std::vector<std::thread> imageWriterThreads;
    for (int i = 0; i < N_WORKER_THREADS; ++i) {
        imageWriterThreads.emplace_back(buildImageWriter(imageQueue, queueMutex, shouldQuit));
    }

    int frameCounter = 0;
    std::vector<char> fileNameBuf;
    fileNameBuf.resize(1000, 0);
    std::shared_ptr<spectacularAI::rsPlugin::Session> vioSession;

    std::ofstream posesFile = std::ofstream(recordingFolder + "/poses.jsonl");
    std::set<int64_t> savedFrames;
    vioPipeline.setMapperCallback([&](std::shared_ptr<const spectacularAI::mapping::MapperOutput> output){
        for (int64_t frameId : output->updatedKeyframes) {
            auto search = output->map->keyframes.find(frameId);
            if (search == output->map->keyframes.end()) {
                continue; // deleted frame
            }

            auto& frameSet = search->second->frameSet;

            if (savedFrames.count(frameId) == 0) {
                // Only save images once, despide frames pose possibly being updated several times
                savedFrames.insert(frameId);
                std::lock_guard<std::mutex> lock(queueMutex);
                char *fileName = fileNameBuf.data();
                // Copy images to ensure they are in memory later for saving
                if (frameSet.rgbFrame && frameSet.rgbFrame->image) {
                    std::snprintf(fileName, fileNameBuf.size(), "%s/rgb_%04ld.png", recordingFolder.c_str(), frameId);
                    imageQueue.push_back(ImageToSave {fileName, copyImage(frameSet.rgbFrame->image)});
                }
                if (frameSet.depthFrame && frameSet.depthFrame->image) {
                    std::snprintf(fileName, fileNameBuf.size(), "%s/depth_%04ld.png", recordingFolder.c_str(), frameId);
                    imageQueue.push_back(ImageToSave {fileName, copyImage(frameSet.depthFrame->image)});
                }
                // TODO: Save pointclouds as JSON?
            }
        }

        // Save only final fully optimized poses, might not contain poses for all frames in case they were deleted
        if (output->finalMap) {
            for (auto it = output->map->keyframes.begin(); it != output->map->keyframes.end(); it++) {
                serializePosesToFile(posesFile, it->second);
            }
        }
    });

    vioSession = vioPipeline.startSession(rsConfig);

    std::thread inputThread([&]() {
        std::cerr << "Press Enter to quit." << std::endl << std::endl;
        std::getchar();
        shouldQuit = true;
    });

    while (!shouldQuit) {
        auto vioOut = vioSession->waitForOutput();
    }

    vioSession = nullptr; // Ensure Vio is done before we quit

    inputThread.join();
    for (auto &t : imageWriterThreads) t.join();
    std::cerr << "Bye!" << std::endl;
    return 0;
}

