#include <iostream>
#include <vector>
#include <spectacularAI/k4a/plugin.hpp>

int main(int argc, char *argv[]) {
    std::vector<std::string> arguments(argv, argv + argc);
    spectacularAI::k4aPlugin::Configuration config;

    // If a folder is given as an argument, record session there
    if (argc >= 2) {
        config.recordingFolder = argv[1];
        config.recordingOnly = true;
    }

    // Create vio pipeline using the config, and then start k4a device and vio.
    spectacularAI::k4aPlugin::Pipeline vioPipeline(config);
    auto session = vioPipeline.startSession();

    while (true) {
        auto vioOut = session->waitForOutput();
        std::cout << vioOut->asJson() << std::endl;
    }

    return EXIT_SUCCESS;
}
