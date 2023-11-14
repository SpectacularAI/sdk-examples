#include <iostream>
#include <librealsense2/rs.hpp>
#include <spectacularAI/realsense/plugin.hpp>

void showUsage() {
    std::cout << "Supported arguments:" << std::endl
        << "  -h, --help Help" << std::endl
        << "  --output <recording_fodler>, recorded output" << std::endl
        << "  --recording_only, disables Vio" << std::endl
        << "  --brightness <value>" << std::endl
        << "  --contrast <value>" << std::endl
        << "  --exposure <value>" << std::endl
        << "  --gain <value>" << std::endl
        << "  --gamma <value>" << std::endl
        << "  --hue <value>" << std::endl
        << "  --saturation <value>" << std::endl
        << "  --sharpness <value>" << std::endl
        << "  --white_balance <value>" << std::endl
        << std::endl;
}

struct ColorCameraConfig {
    int brightness = -1;
    int contrast = -1;
    int exposure = -1;
    int gain = -1;
    int gamma = -1;
    int hue = -1;
    int saturation = -1;
    int sharpness = -1;
    int whiteBalance = -1;
};

int main(int argc, char** argv) {
    spectacularAI::rsPlugin::Configuration config;
    ColorCameraConfig colorConfig;

    std::vector<std::string> arguments(argv, argv + argc);
    for (size_t i = 1; i < arguments.size(); ++i) {
        const std::string &argument = arguments.at(i);
        if (argument == "--output")
            config.recordingFolder = arguments.at(++i);
        else if (argument == "--recording_only")
            config.recordingOnly = true;
        else if (argument == "--brightness")
            colorConfig.brightness = std::stoi(arguments.at(++i));
        else if (argument == "--contrast")
            colorConfig.contrast = std::stoi(arguments.at(++i));
        else if (argument == "--exposure")
            colorConfig.exposure = std::stoi(arguments.at(++i));
        else if (argument == "--gain")
            colorConfig.gain = std::stoi(arguments.at(++i));
        else if (argument == "--gamma")
            colorConfig.gamma = std::stoi(arguments.at(++i));
        else if (argument == "--hue")
            colorConfig.hue = std::stoi(arguments.at(++i));
        else if (argument == "--saturation")
            colorConfig.saturation = std::stoi(arguments.at(++i));
        else if (argument == "--sharpness")
            colorConfig.sharpness = std::stoi(arguments.at(++i));
        else if (argument == "--white_balance")
            colorConfig.whiteBalance = std::stoi(arguments.at(++i));
        else if (argument == "--help" || argument == "-h") {
            showUsage();
            return EXIT_SUCCESS;
        } else {
            showUsage();
            std::cerr << "Unknown argument: " <<  argument << std::endl;
            return EXIT_FAILURE;
        }
    }

    if (config.recordingFolder.empty()) {
            std::cerr << "  You must provide output folder with --output <folder> argument argument." << std::endl;;
            return EXIT_FAILURE;
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

#define SET_OPTION(SENSOR, FIELD, VALUE) \
            if (VALUE >= 0 && SENSOR.supports(FIELD)) do { SENSOR.set_option(FIELD, VALUE); } while (false)
        for (const rs2::sensor &sensor : device.query_sensors()) {
            if (sensor.as<rs2::color_sensor>()) {
                SET_OPTION(sensor, RS2_OPTION_BRIGHTNESS, colorConfig.brightness);
                SET_OPTION(sensor, RS2_OPTION_CONTRAST, colorConfig.contrast);
                SET_OPTION(sensor, RS2_OPTION_EXPOSURE, colorConfig.exposure);
                SET_OPTION(sensor, RS2_OPTION_GAIN, colorConfig.gain);
                SET_OPTION(sensor, RS2_OPTION_GAMMA, colorConfig.gamma);
                SET_OPTION(sensor, RS2_OPTION_HUE, colorConfig.hue);
                SET_OPTION(sensor, RS2_OPTION_SATURATION, colorConfig.saturation);
                SET_OPTION(sensor, RS2_OPTION_SHARPNESS, colorConfig.sharpness);
                SET_OPTION(sensor, RS2_OPTION_WHITE_BALANCE, colorConfig.whiteBalance);
            }
        }
#undef SET_OPTION
    }

    // Start pipeline
    rs2::config rsConfig;
    vioPipeline.configureStreams(rsConfig);
    auto vioSession = vioPipeline.startSession(rsConfig);

    std::cout << "Recording, use Ctrl+C to stop" << std::endl;

    while (true) {
        vioSession->waitForOutput();
    }

    return 0;
}
