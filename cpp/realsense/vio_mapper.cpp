#include <iostream>
#include <fstream>
#include <sstream>
#include <librealsense2/rs.hpp>
#include <spectacularAI/realsense/plugin.hpp>
#include <spectacularAI/mapping.hpp>
#include <lodepng/lodepng.h>
#include <cassert>
#include <string>
#include <cstdint>
#include <thread>
#include <chrono>
#include <deque>
#include <atomic>
#include <cstdlib>
#include <mutex>
#include <functional>
#include <set>
#include <utility>
#include <vector>

#include "helpers.hpp"

namespace {
struct ImageToSave {
    std::string fileName;
    unsigned width;
    unsigned height;
    LodePNGColorType colorType;
    unsigned bitDepth;
    std::vector<std::uint8_t> data;
};

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

            auto img = std::move(queue.front());
            queue.pop_front();
            lock.unlock();

            lodepng::encode(img.fileName, img.data, img.width, img.height, img.colorType, img.bitDepth);
        }
    };
}

ImageToSave copyImage(const std::string &fileName, std::shared_ptr<const spectacularAI::Bitmap> bitmap) {
    const unsigned width = bitmap->getWidth();
    const unsigned height = bitmap->getHeight();
    LodePNGColorType colorType = LCT_RGB;
    unsigned bitDepth = 8;
    std::size_t bytesPerPixel = 3;
    switch (bitmap->getColorFormat()) {
        case spectacularAI::ColorFormat::GRAY:
            colorType = LCT_GREY;
            bitDepth = 8;
            bytesPerPixel = 1;
            break;
        case spectacularAI::ColorFormat::GRAY16:
            colorType = LCT_GREY;
            bitDepth = 16;
            bytesPerPixel = 2;
            break;
        case spectacularAI::ColorFormat::RGB:
            colorType = LCT_RGB;
            bitDepth = 8;
            bytesPerPixel = 3;
            break;
        case spectacularAI::ColorFormat::RGBA:
            colorType = LCT_RGBA;
            bitDepth = 8;
            bytesPerPixel = 4;
            break;
        default:
            assert(false && "Unsupported color format");
            break;
    }

    const std::uint8_t *src = bitmap->getDataReadOnly();
    const std::size_t sizeBytes = static_cast<std::size_t>(width) * height * bytesPerPixel;
    std::vector<std::uint8_t> data(src, src + sizeBytes);
    if (bitDepth == 16) {
        for (std::size_t i = 0; i + 1 < data.size(); i += 2) {
            std::swap(data[i], data[i + 1]);
        }
    }
    return ImageToSave { fileName, width, height, colorType, bitDepth, std::move(data) };
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

void serializePosesToFile(std::ofstream& posesFile, std::shared_ptr<const spectacularAI::mapping::KeyFrame> keyframe) {
    auto& frameSet = keyframe->frameSet;
    std::stringstream ss;
    ss << "{\"frameId\": " << (keyframe->id) << ","
        << "\"poses\": {";
    if (frameSet->rgbFrame) {
        ss << "\"rgb\": " << matrix4ToString(frameSet->rgbFrame->cameraPose.pose.asMatrix());
    }
    if (frameSet->depthFrame) {
        if (frameSet->rgbFrame) ss << ",";
        ss << "\"depth\": " << matrix4ToString(frameSet->depthFrame->cameraPose.pose.asMatrix());
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
        createFolders(recordingFolder);
    } else {
        std::cerr
            << "Usage: " << argv[0] << " /path/to/recording/folder" << std::endl;
        return 1;
    }

    // The RS callback thread should not be blocked for long.
    // Using worker threads for image encoding and disk I/O
    std::atomic<bool> shouldQuit(false);
    constexpr int N_WORKER_THREADS = 4;
    std::mutex queueMutex;
    std::deque<ImageToSave> imageQueue;
    std::ofstream posesFile = std::ofstream(recordingFolder + "/poses.jsonl");
    std::set<int64_t> savedFrames;
    std::vector<char> fileNameBuf;
    fileNameBuf.resize(1000, 0);

    // Create vio pipeline with mapper callback
    spectacularAI::rsPlugin::Configuration vioConfig;
    spectacularAI::rsPlugin::Pipeline vioPipeline(vioConfig, [&](std::shared_ptr<const spectacularAI::mapping::MapperOutput> output) {
        for (int64_t frameId : output->updatedKeyFrames) {
            auto search = output->map->keyFrames.find(frameId);
            if (search == output->map->keyFrames.end()) {
                continue; // deleted frame
            }

            auto& frameSet = search->second->frameSet;

            if (savedFrames.count(frameId) == 0) {
                // Only save images once, despide frames pose possibly being updated several times
                savedFrames.insert(frameId);
                std::lock_guard<std::mutex> lock(queueMutex);
                char *fileName = fileNameBuf.data();
                // Copy images to ensure they are in memory later for saving
                if (frameSet->rgbFrame && frameSet->rgbFrame->image) {
                    std::snprintf(fileName, fileNameBuf.size(), "%s/rgb_%04ld.png", recordingFolder.c_str(), frameId);
                    imageQueue.push_back(copyImage(fileName, frameSet->rgbFrame->image));
                }
                if (frameSet->depthFrame && frameSet->depthFrame->image) {
                    std::snprintf(fileName, fileNameBuf.size(), "%s/depth_%04ld.png", recordingFolder.c_str(), frameId);
                    imageQueue.push_back(copyImage(fileName, frameSet->depthFrame->image));
                }
                // TODO: Save pointclouds as JSON?
            }
        }

        // Save only final fully optimized poses, might not contain poses for all frames in case they were deleted
        if (output->finalMap) {
            for (auto it = output->map->keyFrames.begin(); it != output->map->keyFrames.end(); it++) {
                serializePosesToFile(posesFile, it->second);
            }
        }
    });

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

    std::vector<std::thread> imageWriterThreads;
    for (int i = 0; i < N_WORKER_THREADS; ++i) {
        imageWriterThreads.emplace_back(buildImageWriter(imageQueue, queueMutex, shouldQuit));
    }

    auto vioSession = vioPipeline.startSession(rsConfig);
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

