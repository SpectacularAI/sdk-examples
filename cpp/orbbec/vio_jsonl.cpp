#include <iostream>
#include <vector>
#include <libobsensor/ObSensor.hpp>
#include <spectacularAI/orbbec/plugin.hpp>

int main(int argc, char *argv[]) {
    std::vector<std::string> arguments(argv, argv + argc);
    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_OFF);

    // Create OrbbecSDK pipeline (with default device).
    ob::Pipeline obPipeline;

    // Create Spectacular AI orbbec plugin configuration (depends on device type).
    spectacularAI::orbbecPlugin::Configuration config(obPipeline);

    // If a folder is given as an argument, record session there
    if (argc >= 2) {
        config.recordingFolder = argv[1];
        config.recordingOnly = true;
    }

    // Create VIO pipeline & setup orbbec pipeline
    spectacularAI::orbbecPlugin::Pipeline vioPipeline(obPipeline, config);

    // and then start orbbec device and vio.
    auto session = vioPipeline.startSession();

    while (true) {
        auto vioOut = session->waitForOutput();
        std::cout << vioOut->asJson() << std::endl;
    }

    return EXIT_SUCCESS;
}
