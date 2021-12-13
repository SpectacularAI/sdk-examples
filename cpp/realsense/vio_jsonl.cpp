#include <iostream>
#include <librealsense2/rs.hpp>
#include <spectacularAI/realsense/plugin.hpp>

int main(int argc, char** argv) {
    spectacularAI::rsPlugin::Configuration config;

    // If a folder is given as an argument, record session there
    if (argc >= 2) {
        config.recordingFolder = argv[1];
        config.recordingOnly = true;
    }

    spectacularAI::rsPlugin::Pipeline vioPipeline(config);

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
    auto vioSession = vioPipeline.startSession(rsConfig);

    while (true) {
        auto vioOut = vioSession->waitForOutput();
        std::cout << vioOut->asJson() << std::endl;
    }

    return 0;
}
