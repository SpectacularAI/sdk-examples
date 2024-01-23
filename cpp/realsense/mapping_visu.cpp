#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
#include <fstream>
#include <thread>
#include <librealsense2/rs.hpp>
#include <spectacularAI/realsense/plugin.hpp>

#include "serialize_output.hpp"

const std::string SEPARATOR = "/";

void showUsage() {
    std::cout << "Valid input arguments are:\n "
                 "-o output_file,\n "
                 "-r recording_folder,\n "
                 "-h"
              << std::endl;
}

int main(int argc, char *argv[]) {
    std::vector<std::string> arguments(argv, argv + argc);
    std::string outputFile;

    // Create Spectacular AI realsnse plugin configuration
    spectacularAI::rsPlugin::Configuration config;
    config.internalParameters = {
        {"computeStereoPointCloud", "true"} // enables point cloud colors
    };

    for (size_t i = 1; i < arguments.size(); ++i) {
        const std::string &argument = arguments.at(i);
        if (argument == "-o") outputFile = arguments.at(++i);
        else if (argument == "-r") config.recordingFolder = arguments.at(++i);
        else if (argument == "-h") {
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
        return EXIT_SUCCESS;
    }

    std::ofstream outputStream(outputFile.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
    if (!outputStream.is_open()) {
        std::cerr << "Failed to open file: " << outputFile << std::endl;
        return EXIT_FAILURE;
    }

    // Vio and Mapping outputs can come from different threads (`separateOutputThreads: True`), prevent mixing output stream by mutex
    std::mutex m;
    Serializer serializer;

    // Create vio pipeline using the config
    spectacularAI::rsPlugin::Pipeline vioPipeline(config,
        [&](spectacularAI::mapping::MapperOutputPtr mappingOutput) {
            // Serialize mapping output for point cloud
            std::lock_guard<std::mutex> lock(m);
            serializer.serializeMappingOutput(outputStream, mappingOutput);
        }
    );

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

    std::atomic<bool> shouldQuit(false);
    std::thread inputThread([&]() {
        std::cout << "Press Enter to quit." << std::endl << std::endl;
        getchar();
        shouldQuit = true;
    });

    // Add scope so that session dtor is called and final SLAM map is also serialized.
    {
        // Start pipeline
        rs2::config rsConfig;
        vioPipeline.configureStreams(rsConfig);
        auto session = vioPipeline.startSession(rsConfig);

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
