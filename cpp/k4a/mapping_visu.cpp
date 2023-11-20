#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
#include <fstream>
#include <thread>

#include <spectacularAI/k4a/plugin.hpp>
#include "serialize_output.hpp"

const std::string SEPARATOR = "/";

void showUsage() {
    std::cout << "Valid input arguments are:\n "
                 "-o output_file,\n "
                 "-r recording_folder,\n "
                 "-fps [5, 15, 30 (default)]\n "
                 "-h"
              << std::endl;
}

int main(int argc, char *argv[]) {
    std::vector<std::string> arguments(argv, argv + argc);
    std::string outputFile;
    spectacularAI::k4aPlugin::Configuration config;
    config.internalParameters = {
        {"computeStereoPointCloud", "true"} // enables point cloud colors
    };

    for (size_t i = 1; i < arguments.size(); ++i) {
        const std::string &argument = arguments.at(i);
        if (argument == "-o") outputFile = arguments.at(++i);
        else if (argument == "-r") config.recordingFolder = arguments.at(++i);
        else if (argument == "-fps") {
            int fps = std::stoi(arguments.at(++i));
            if (fps == 5) {
                config.k4aConfig.camera_fps = K4A_FRAMES_PER_SECOND_5;
            } else if (fps == 15) {
                config.k4aConfig.camera_fps = K4A_FRAMES_PER_SECOND_15;
            } else if (fps == 30) {
                config.k4aConfig.camera_fps = K4A_FRAMES_PER_SECOND_30;
            } else {
                std::cout << "Valid camera FPS options are [5, 15, 30 (default)]: " << fps << std::endl;
                exit(1);
            }
        } else if (argument == "-h") {
            showUsage();
            exit(0);
        } else {
            std::cout << "Unknown argument: " << argument << std::endl;
            showUsage();
            exit(1);
        }
    }

    if (outputFile.empty()) {
        std::cout << "Please specify output `-o`."<< std::endl;
        return 0;
    }

    std::ofstream outputStream(outputFile.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
    if (!outputStream.is_open()) {
        std::cerr << "Failed to open file: " << outputFile << std::endl;
        return EXIT_FAILURE;
    }

    // Vio and Mapping outputs can come from different threads (`separateOutputThreads: True`), prevent mixing output stream by mutex
    std::mutex m;
    Serializer serializer;

    // Create vio pipeline using the config, and then start k4a device and vio with a mapper callback.
    spectacularAI::k4aPlugin::Pipeline vioPipeline(config,
        [&](spectacularAI::mapping::MapperOutputPtr mappingOutput) {
            // Serialize mapping output for point cloud
            std::lock_guard<std::mutex> lock(m);
            serializer.serializeMappingOutput(outputStream, mappingOutput);
        }
    );

    std::atomic<bool> shouldQuit(false);
    std::thread inputThread([&]() {
        std::cout << "Press Enter to quit." << std::endl << std::endl;
        getchar();
        shouldQuit = true;
    });

    // Add scope so that session dtor is called and final SLAM map is also serialized.
    {
        // Start k4a device and vio
        auto session = vioPipeline.startSession();

        while (!shouldQuit) {
            if (session->hasOutput()) {
                auto vioOutput = session->getOutput();
                std::lock_guard<std::mutex> lock(m);
                serializer.serializeVioOutput(outputStream, vioOutput);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    std::cout << "Exiting." << std::endl;
    inputThread.join();
    outputStream.close();

    return 0;
}
