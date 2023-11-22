#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <mutex>

#include <spectacularAI/vio.hpp>
#include <spectacularAI/replay.hpp>

#include "serialize_output.hpp"

const std::string SEPARATOR = "/";

std::string readFileToString(const std::string &filename) {
    std::ifstream f(filename);
    std::ostringstream oss;
    oss << f.rdbuf();
    return oss.str();
}

bool fileExists(const std::string &filePath) {
    std::ifstream dataFile(filePath);
    return dataFile.is_open();
}

int main(int argc, char *argv[]) {
    std::vector<std::string> arguments(argv, argv + argc);
    std::string inputFolder;
    std::string outputFile;

    for (size_t i = 1; i < arguments.size(); ++i) {
        const std::string &argument = arguments.at(i);
        if (argument == "-i") inputFolder = arguments.at(++i);
        else if (argument == "-o") outputFile = arguments.at(++i);
        else {
            std::cout << "Unknown argument: " << argument << std::endl;
            exit(1);
        }
    }
    if (inputFolder.empty()) {
        std::cout << "Please specify input directory using `-i`."<< std::endl;
        return 0;
    }

    if (outputFile.empty()) {
        std::cout << "Please specify output `-o`."<< std::endl;
        return 0;
    }

    std::ostringstream configurationYaml;
    std::string dataConfigurationYamlPath = inputFolder + SEPARATOR + "vio_config.yaml";
    if (fileExists(dataConfigurationYamlPath)) {
        configurationYaml << readFileToString(dataConfigurationYamlPath) << std::endl;
    }
    // Ensure pointcloud output is enabled
    configurationYaml << "computeDenseStereoDepthKeyFramesOnly: True\n";
    configurationYaml << "computeStereoPointCloud: True\n";
    configurationYaml << "useRectification: True\n"; // TODO: Should check for alreadyRectified

    Serializer serializer;
    // Vio and Mapping outputs come from different threads, prevent mixing output stream by mutex
    std::mutex m;

    std::ofstream outputStream(outputFile.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
    if (!outputStream.is_open()) {
        std::cerr << "Failed to open file: " << outputFile << std::endl;
        return EXIT_FAILURE;
    }

    spectacularAI::Vio::Builder vioBuilder = spectacularAI::Vio::builder()
        .setConfigurationYAML(configurationYaml.str());

   vioBuilder.setMapperCallback([&](spectacularAI::mapping::MapperOutputPtr mappingOutput) {
       std::lock_guard<std::mutex> lock(m);
       serializer.serializeMappingOutput(outputStream, mappingOutput);
   });

    std::unique_ptr<spectacularAI::Replay> replay
        = spectacularAI::Replay::builder(inputFolder, vioBuilder).build();
    replay->setPlaybackSpeed(-1);

    replay->setOutputCallback([&](spectacularAI::VioOutputPtr vioOutput) {
        std::lock_guard<std::mutex> lock(m);
        serializer.serializeVioOutput(outputStream, vioOutput);
    });

    replay->runReplay();

    outputStream.close();

    return 0;
}
