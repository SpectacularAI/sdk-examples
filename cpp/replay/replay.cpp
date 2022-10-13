#include <cassert>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>

#include <spectacularAI/vio.hpp>
#include <spectacularAI/replay.hpp>

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
    std::unique_ptr<std::ofstream> outputFile;
    std::string inputFolder;
    std::string userConfigurationYaml;
    bool realtime = false;
    bool print = false;
    for (size_t i = 1; i < arguments.size(); ++i) {
        const std::string &argument = arguments.at(i);
        if (argument == "-o") {
            outputFile = std::make_unique<std::ofstream>(arguments.at(++i));
            assert(outputFile->is_open());
        }
        else if (argument == "-i") inputFolder = arguments.at(++i);
        else if (argument == "-c") userConfigurationYaml = readFileToString(arguments.at(++i));
        else if (argument == "--realtime") realtime = true;
        else if (argument == "--print") print = true;
    }
    if (inputFolder.empty()) {
        std::cout << "Please specify input directory using `-i`."<< std::endl;
        return 0;
    }

    std::ostringstream configurationYaml;
    std::string dataConfigurationYamlPath = inputFolder + SEPARATOR + "vio_config.yaml";
    if (fileExists(dataConfigurationYamlPath)) {
        // Vio::Builder::setConfigurationYAML overwrites the configuration in the data
        // directory, but in most cases that configuration is needed for the best VIO
        // performance, so we concatenate our changes to it.
        configurationYaml << readFileToString(dataConfigurationYamlPath) << std::endl;
    }
    // The `processingQueueSize` option makes the data input wait for VIO to finish to
    // avoid dropping frames. It should not be used in real-time scenarios.
    configurationYaml << "processingQueueSize: 0\n";
    configurationYaml << userConfigurationYaml;

    // The Replay API builder takes as input the main API builder as a way to share
    // configuration methods.
    spectacularAI::Vio::Builder vioBuilder = spectacularAI::Vio::builder()
        .setConfigurationYAML(configurationYaml.str());

    std::unique_ptr<spectacularAI::Replay> replay
        = spectacularAI::Replay::builder(inputFolder, vioBuilder).build();
    replay->setPlaybackSpeed(realtime ? 1.0 : -1.0);

    replay->setOutputCallback([&](spectacularAI::VioOutputPtr vioOutput) {
        if (outputFile) *outputFile << vioOutput->asJson().c_str() << std::endl;
        if (print) std::cout << vioOutput->asJson().c_str() << std::endl;
    });

    const auto t0 = std::chrono::steady_clock::now();

    replay->runReplay();

    const auto time = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    printf("Replay took %.2fs\n", 1e-3 * time);

    return 0;
}
