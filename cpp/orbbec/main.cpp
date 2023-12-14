#include <atomic>
#include <iostream>
#include <sstream>
#include <memory>
#include <thread>
#include <vector>

#include <libobsensor/ObSensor.hpp>
#include <spectacularAI/orbbec/plugin.hpp>

void showUsage() {
    std::cout << "Valid input arguments are:\n "
                 "-color_res [width,height]\n "
                 "-depth_res [width,height]\n "
                 "-frame_rate [fps]\n "
                 "-align (use orbbec sdk to align depth images to color images, note color camera might have smaller fov!)\n "
                 "-record [path/to/recording]\n "
                 "-record_only\n "
                 "-fast\n "
                 "-vio_only (-useSlam=false)\n "
                 "-mono (-useStereo=false, rgb only & depth camera disabled)\n "
                 "-print\n "
                 "-exposure [value]\n "
                 "-gain [value]\n "
                 "-whitebalance [kelvins]\n "
                 "-brightness [value]\n"
              << std::endl;
}

bool setCameraProperty(std::shared_ptr<ob::Device> device, OBPropertyID propertyId, int value, const std::string &propertyName) {
    try {
        if(device->isPropertySupported(propertyId, OB_PERMISSION_READ)) {
            OBIntPropertyRange valueRange = device->getIntPropertyRange(propertyId);

            if(device->isPropertySupported(propertyId, OB_PERMISSION_WRITE)) {
                if (value >= valueRange.min && value <= valueRange.max) {
                    device->setIntProperty(propertyId, value);
                    return true;
                }
                std::cerr << propertyName << " range is [" << valueRange.min << "-" << valueRange.max << "], requested value: " << value << std::endl;
            } else {
                std::cerr << propertyName << " set property is not supported." << std::endl;
            }
        } else {
            std::cerr << propertyName << " get property is not supported." << std::endl;
        }
    } catch(ob::Error &e) {
        std::cerr << propertyName << " set property is not supported." << std::endl;
    }

    return false;
}

bool setCameraProperty(std::shared_ptr<ob::Device> device, OBPropertyID propertyId, bool value, const std::string &propertyName) {
    try {
        if(device->isPropertySupported(propertyId, OB_PERMISSION_READ)) {
            if (value == device->getBoolProperty(propertyId)) return true;

            if(device->isPropertySupported(propertyId, OB_PERMISSION_WRITE)) {
                device->setBoolProperty(propertyId, !value);
                return true;
            }
            std::cerr << propertyName << " set property is not supported." << std::endl;
        } else {
            std::cerr << propertyName << " get property is not supported." << std::endl;
        }
    } catch(ob::Error &e) {
        std::cerr << propertyName << " set property is not supported." << std::endl;
    }

    return false;
}

std::pair<int, int> tryParseResolution(const std::string &s) {
    std::istringstream iss(s);

    int width, height;
    char comma;
    if (iss >> width >> comma >> height && comma == ',') {
        return std::make_pair(width, height);
    } else {
        std::cerr << "Failed to parse resolution from " << s << std::endl;
        exit(1);
    }
}

int main(int argc, char *argv[]) {
    std::vector<std::string> arguments(argv, argv + argc);
    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_OFF);

    // Create OrbbecSDK pipeline (with default device).
    ob::Pipeline obPipeline;

    // Create Spectacular AI orbbec plugin configuration (depends on device type).
    spectacularAI::orbbecPlugin::Configuration config(obPipeline);

    int exposureValue = -1;
    int whiteBalanceKelvins = -1;
    int gain = -1;
    int brightness = -1;
    bool print = false;
    for (size_t i = 1; i < arguments.size(); ++i) {
        const std::string &argument = arguments.at(i);
        if (argument == "-color_res")
            config.rgbResolution = tryParseResolution(arguments.at(++i));
        else if (argument == "-depth_res")
            config.depthResolution = tryParseResolution(arguments.at(++i));
        else if (argument == "-frame_rate")
            config.cameraFps = std::stoi(arguments.at(++i));
        else if (argument == "-align")
            config.alignedDepth = true;
        else if (argument == "-mono")
            config.useStereo = false;
        else if (argument == "-record")
            config.recordingFolder = arguments.at(++i);
        else if (argument == "-record_only")
            config.recordingOnly = true;
        else if (argument == "-fast")
            config.fastVio = true;
        else if (argument == "-vio_only")
            config.useSlam = false;
        else if (argument == "-print")
            print = true;
        else if (argument == "-exposure")
            exposureValue = std::stoi(arguments.at(++i));
        else if (argument == "-whitebalance")
            whiteBalanceKelvins = std::stoi(arguments.at(++i));
        else if (argument == "-gain")
            gain = std::stoi(arguments.at(++i));
        else if (argument == "-brightness")
            brightness = std::stoi(arguments.at(++i));
        else if (argument == "-help" || argument == "-h") {
            showUsage();
            exit(0);
        } else {
            showUsage();
            std::cerr << "Unknown argument: " << argument << std::endl;
            exit(1);
        }
    }

    // Require recording folder when using recording only mode.
    if (config.recordingOnly && config.recordingFolder.empty()) {
        std::cerr << "Record only but recording folder is not set!" << std::endl;
        exit(1);
    }

    // Create vio pipeline using the config & setup orbbec pipeline
    spectacularAI::orbbecPlugin::Pipeline vioPipeline(obPipeline, config);

    auto device = obPipeline.getDevice();
    if (exposureValue >= 0) {
        if (!setCameraProperty(device, OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, false, "OB_PROP_COLOR_AUTO_EXPOSURE_BOOL")) exit(1);
        if (!setCameraProperty(device, OB_PROP_COLOR_EXPOSURE_INT, exposureValue, "OB_PROP_COLOR_EXPOSURE_INT")) exit(1);
    }

    if (gain >= 0) {
        if (!setCameraProperty(device, OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, false, "OB_PROP_COLOR_AUTO_EXPOSURE_BOOL")) exit(1);
        if (!setCameraProperty(device, OB_PROP_COLOR_GAIN_INT, gain, "OB_PROP_COLOR_GAIN_INT")) exit(1);
    }

    if (whiteBalanceKelvins >= 0) {
        if (!setCameraProperty(device, OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, false, "OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL")) exit(1);
        if (!setCameraProperty(device, OB_PROP_COLOR_WHITE_BALANCE_INT, whiteBalanceKelvins, "OB_PROP_COLOR_WHITE_BALANCE_INT")) exit(1);
    }

    if (brightness >= 0) {
        if (!setCameraProperty(device, OB_PROP_COLOR_BRIGHTNESS_INT, brightness, "OB_PROP_COLOR_BRIGHTNESS_INT")) exit(1);
    }

    // Start orbbec device and vio.
    auto session = vioPipeline.startSession();

    std::atomic<bool> shouldQuit(false);
    std::thread inputThread([&]() {
        std::cout << "Press Enter to quit." << std::endl << std::endl;
        getchar();
        shouldQuit = true;
    });

    while (!shouldQuit) {
        if (session->hasOutput()) {
            auto out = session->getOutput();
            if (print) {
                std::cout << "Vio API pose: " << out->pose.time << ", " << out->pose.position.x
                          << ", " << out->pose.position.y << ", " << out->pose.position.z << ", "
                          << out->pose.orientation.x << ", " << out->pose.orientation.y << ", "
                          << out->pose.orientation.z << ", " << out->pose.orientation.w
                          << std::endl;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "Exiting." << std::endl;
    if (shouldQuit) inputThread.join();

    return EXIT_SUCCESS;
}
