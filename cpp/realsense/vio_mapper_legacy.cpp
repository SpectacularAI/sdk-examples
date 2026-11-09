#include <iostream>
#include <fstream>
#include <librealsense2/rs.hpp>
#include <spectacularAI/realsense/plugin.hpp>
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
}

int main(int argc, char** argv) {
    // If a folder is given as an argument, record session there
    std::string recordingFolder;
    int keyFrameInterval = 10;
    if (argc >= 2) {
        recordingFolder = argv[1];
        createFolders(recordingFolder);
        if (argc >= 3) {
            keyFrameInterval = std::stoi(argv[2]);
        }
    } else {
        std::cerr
            << "Usage: " << argv[0] << " /path/to/recording/folder [N]" << std::endl
            << "where N is the frame sampling interval, default: " << keyFrameInterval << std::endl;
        return 1;
    }

    spectacularAI::rsPlugin::Pipeline vioPipeline;

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

    auto callback = [
        &frameCounter,
        &fileNameBuf,
        &vioSession,
        &queueMutex,
        &imageQueue,
        &shouldQuit,
        keyFrameInterval,
        recordingFolder
    ](const rs2::frame &frame)
    {
        if (shouldQuit) return;
        auto frameset = frame.as<rs2::frameset>();
        if (frameset && frameset.get_profile().stream_type() == RS2_STREAM_DEPTH) {
            auto vio = vioSession; // atomic
            if (!vio) return;

            if ((frameCounter++ % keyFrameInterval) != 0) return;
            int keyFrameNumber = ((frameCounter - 1) / keyFrameInterval) + 1;

            vio->addTrigger(frame.get_timestamp() * 1e-3, keyFrameNumber);

            rs2_stream depthAlignTarget = RS2_STREAM_COLOR;
            rs2::align alignDepth(depthAlignTarget);

            // This line can be commented out to disable aligning.
            frameset = alignDepth.process(frameset);

            const rs2::video_frame &depth = frameset.get_depth_frame();
            const rs2::video_frame &color = frameset.get_color_frame();
            assert(depth.get_profile().format() == RS2_FORMAT_Z16);
            assert(color.get_profile().format() == RS2_FORMAT_RGB8);

            const unsigned colorW = color.get_width();
            const unsigned colorH = color.get_height();
            const std::uint8_t *colorData = reinterpret_cast<const std::uint8_t*>(color.get_data());
            const std::size_t colorBytes = static_cast<std::size_t>(colorW) * colorH * 3;

            const unsigned depthW = depth.get_width();
            const unsigned depthH = depth.get_height();
            const std::uint8_t *depthData = reinterpret_cast<const std::uint8_t*>(depth.get_data());
            const std::size_t depthBytes = static_cast<std::size_t>(depthW) * depthH * 2;

            char *fileName = fileNameBuf.data();
            std::snprintf(fileName, fileNameBuf.size(), "%s/depth_%04d.png", recordingFolder.c_str(), keyFrameNumber);

            ImageToSave depthImg;
            depthImg.fileName = fileName;
            depthImg.width = depthW;
            depthImg.height = depthH;
            depthImg.colorType = LCT_GREY;
            depthImg.bitDepth = 16;
            depthImg.data.assign(depthData, depthData + depthBytes);
            for (std::size_t i = 0; i + 1 < depthImg.data.size(); i += 2) {
                std::swap(depthImg.data[i], depthImg.data[i + 1]);
            }

            ImageToSave colorImg;
            std::snprintf(fileName, fileNameBuf.size(), "%s/rgb_%04d.png", recordingFolder.c_str(), keyFrameNumber);
            colorImg.fileName = fileName;
            colorImg.width = colorW;
            colorImg.height = colorH;
            colorImg.colorType = LCT_RGB;
            colorImg.bitDepth = 8;
            colorImg.data.assign(colorData, colorData + colorBytes);

            std::lock_guard<std::mutex> lock(queueMutex);
            imageQueue.push_back(std::move(depthImg));
            imageQueue.push_back(std::move(colorImg));
        }
    };

    vioSession = vioPipeline.startSession(rsConfig, callback);
    std::ofstream vioOutJsonl(recordingFolder + "/vio.jsonl");

    std::thread inputThread([&]() {
        std::cerr << "Press Enter to quit." << std::endl << std::endl;
        std::getchar();
        shouldQuit = true;
    });

    while (!shouldQuit) {
        auto vioOut = vioSession->waitForOutput();
        if (vioOut->tag > 0) {
            vioOutJsonl << "{\"tag\":" << vioOut->tag << ",\"vio\":" << vioOut->asJson() << "}" << std::endl;
        }
    }

    inputThread.join();
    for (auto &t : imageWriterThreads) t.join();
    std::cerr << "Bye!" << std::endl;
    return 0;
}
