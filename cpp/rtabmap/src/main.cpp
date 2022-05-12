#include <iostream>
#include <string>

#include <QApplication>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/ULogger.h>

#include "../include/spectacularAI/rtabmap/map_builder.h"
#include "../include/spectacularAI/rtabmap/camera_replay.h"

void showUsage() {
    std::cout << "Usage: rtabmap_mapper --input (-i) path/to/dataset --output (-o) path/to/output/database.db \n"
                 "Optional parameters: --config (-c) path/to/rtabmap_config.ini" << std::endl;
}

int main(int argc, char *argv[]) {
    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kWarning);

    const std::vector<std::string> arguments(argv, argv + argc);
    std::string dataFolder;
    std::string outputPath;
    std::string rtabmapConfigFile;

    for (size_t i = 1; i < arguments.size(); ++i) {
        const std::string &argument = arguments.at(i);
        if (argument == "-i" || argument == "--input")
            dataFolder = arguments.at(++i);
        else if (argument == "-o" || argument == "--output")
            outputPath = arguments.at(++i);
        else if (argument == "-c" || argument == "--config")
            rtabmapConfigFile = arguments.at(++i);
        else if (argument == "-h" || argument == "--help") {
            showUsage();
            return EXIT_SUCCESS;
        } else {
            showUsage();
            UFATAL("Unknown argument: %s", argument.c_str());
        }
    }

    if (dataFolder.empty()) {
        showUsage();
        UFATAL("Empty input path!");
    }

    if (outputPath.empty()) {
        showUsage();
        UFATAL("Empty output path!");
    }

    // Create CameraReplay, which is a RTAB-Map wrapper around the SpectacularAI mapping API.
    // Essentially it just captures all keyframes (RGB-D image, point cloud, pose) and converts them
    // to RTAB-Map format.
    CameraReplay camera(dataFolder);
    if(camera.init()) {
        // Initialize Rtabmap instance (with given configuration file).
        Rtabmap rtabmap;
        rtabmap.init(rtabmapConfigFile);

        QApplication app(argc, argv);
        MapBuilder mapBuilder;
        mapBuilder.show();
        QApplication::processEvents();

        // Image loop
        CameraInfo info;
        SensorData data = camera.takeImage(&info);
        while (data.isValid() && mapBuilder.isVisible()) {
            QApplication::processEvents();

            // Input data to RTAB-Map
            if (rtabmap.process(data, info.odomPose)) {
                mapBuilder.processStatistics(rtabmap.getStatistics());

                if (rtabmap.getLoopClosureId() > 0) {
                    std::cout << "RTAB-Map loop closure detected: " << rtabmap.getLastLocationId()
                              << "->" << rtabmap.getLoopClosureId() << std::endl;
                }
            }

            while (mapBuilder.isPaused() && mapBuilder.isVisible()) {
                uSleep(100);
                QApplication::processEvents();
            }

            data = camera.takeImage(&info);
        }

        if (mapBuilder.isVisible()) {
            std::cout << "Processed all frames" << std::endl;
            app.exec();
        }

        // Save RTAB-Map database
        rtabmap.close(true, outputPath);
        std::cout << "Saved RTAB-Map database to " + outputPath << std::endl;
    }

    return EXIT_SUCCESS;
}
