#include <cassert>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <mutex>
#include <fcntl.h>

#include <spectacularAI/k4a/plugin.hpp>

#include "serialize_output.hpp"

const std::string SEPARATOR = "/";

int main(int argc, char *argv[]) {
    std::vector<std::string> arguments(argv, argv + argc);
    std::string outputFile;
    spectacularAI::k4aPlugin::Configuration config;

    for (size_t i = 1; i < arguments.size(); ++i) {
        const std::string &argument = arguments.at(i);
        if (argument == "-o") outputFile = arguments.at(++i);
        else if (argument == "-r") config.recordingFolder = arguments.at(++i);
        else {
            std::cout << "Unknown argument: " << argument << std::endl;
            exit(1);
        }
    }

    if (outputFile.empty()) {
        std::cout << "Please specify output `-o`."<< std::endl;
        return 0;
    }

    Serializer serializer;
    // Vio and Mapping outputs come from different threads, prevent mixing output stream by mutex
    std::mutex m;

    int fileDescriptor = open(outputFile.c_str(), O_RDWR);
    FILE *outputStream = fdopen(fileDescriptor, "w+");
    if (fileDescriptor == -1) {
        std::cerr << "Failed to open file: " << outputFile;
        exit(EXIT_FAILURE);
    }

    config.k4aConfig.camera_fps = K4A_FRAMES_PER_SECOND_15;

    config.internalParameters = {
        {"computeStereoPointCloud", "true"}
    };

    // Create vio pipeline using the config, and then start k4a device and vio.
    spectacularAI::k4aPlugin::Pipeline vioPipeline(config);


    // Serialize mapping output for point cloud
    vioPipeline.setMapperCallback([&](spectacularAI::mapping::MapperOutputPtr mappingOutput) {
       std::lock_guard<std::mutex> lock(m);
       serializer.serializeMappingOutput(outputStream, mappingOutput);
   });

    auto session = vioPipeline.startSession();

    while (true) {
        // Serialize vio output for more frequent camera pose updates
        auto vioOut = session->waitForOutput();
        {
            std::lock_guard<std::mutex> lock(m);
            serializer.serializeVioOutput(outputStream, vioOut);
        }
    }

    return 0;
}
