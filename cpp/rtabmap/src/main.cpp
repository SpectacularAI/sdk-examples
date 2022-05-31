#include <iostream>
#include <string>

#include <QApplication>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/RtabmapThread.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/ULogger.h>

#include "../include/spectacularAI/rtabmap/map_builder.h"
#include "../include/spectacularAI/rtabmap/camera_replay.h"
#include "../include/spectacularAI/rtabmap/camera_k4a.h"
#include "../include/spectacularAI/rtabmap/camera_realsense.h"

void showUsage() {
    std::cout << "Usage: rtabmap_mapper driver\n"
        << "  driver options: replay, k4a, realsense"
        << "  Optional parameters:\n"
        << "  --output (-o) path/to/output/database.db  [If set, RTAB-Map database is saved to this file]\n"
        << "  --config (-c) path/to/rtabmap_config.ini  [If set, RTAB-Map settings are overriden by this file]\n"
        << "  --record (-r) path/to/recording           [If set, captured data is saved to this location]\n"
        << "  --input (-i) path/to/dataset              [Input dataset path for replay camera driver]\n"
        << std::endl;

    exit(EXIT_SUCCESS);
}

int main(int argc, char *argv[]) {
    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kWarning);

    const std::vector<std::string> arguments(argv, argv + argc);
    std::string driver;
    std::string outputPath;
    std::string rtabmapConfigFile;
    std::string inputDataFolder;
    std::string recordingFolder;
    float imageRate = 0; // as fast as possible

    if (arguments.size() < 2) {
        showUsage();
        return EXIT_SUCCESS;
    }

    for (size_t i = 2; i < arguments.size(); ++i) {
        const std::string &argument = arguments.at(i);
        if (argument == "-i" || argument == "--input")
            inputDataFolder = arguments.at(++i);
        else if (argument == "-o" || argument == "--output")
            outputPath = arguments.at(++i);
        else if (argument == "-c" || argument == "--config")
            rtabmapConfigFile = arguments.at(++i);
        else if (argument == "-r" || argument == "--record")
            recordingFolder = arguments.at(++i);
        else if (argument == "-h" || argument == "--help") {
            showUsage();
        } else {
            UERROR("Unknown argument: %s", argument.c_str());
            showUsage();
        }
    }

    Camera *camera = 0;
    driver = arguments.at(1);
    if (driver == "replay") {
        if(!CameraReplay::available()) {
            UERROR("Not built with CameraReplay support...");
            exit(EXIT_FAILURE);
        }
        camera = new CameraReplay(inputDataFolder, imageRate);
    } else if (driver == "k4a") {
        if(!CameraK4A::available()) {
            UERROR("Not built with CameraK4A support...");
            exit(EXIT_FAILURE);
        }
        camera = new CameraK4A(recordingFolder, imageRate);
    } else if (driver == "realsense") {
        if (!CameraRealsense::available()) {
            UERROR("Not built with CameraRealsense support...");
            exit(EXIT_FAILURE);
        }
        camera = new CameraRealsense(recordingFolder, imageRate);
    } else {
        UERROR("Unknown camera driver: %s", driver.c_str());
        showUsage();
    }

    if(!camera->init()) {
   	    UFATAL("Camera init failed!");
    }

    CameraThread* cameraThread = new CameraThread(camera);

    // GUI stuff, there the handler will receive RtabmapEvent and construct the map
    // We give it the camera so the GUI can pause/resume the camera
    QApplication app(argc, argv);
    MapBuilder mapBuilder(cameraThread);

    // Create RTAB-Map to process OdometryEvent
    Rtabmap *rtabmap = new Rtabmap();
    rtabmap->init(rtabmapConfigFile);
    RtabmapThread rtabmapThread(rtabmap); // ownership is transfered

    // Setup handlers
    rtabmapThread.registerToEventsManager();
    mapBuilder.registerToEventsManager();

    // Let's start the threads
    rtabmapThread.start();
    cameraThread->start();

    mapBuilder.show();
    app.exec(); // main loop

    // remove handlers
    mapBuilder.unregisterFromEventsManager();
    rtabmapThread.unregisterFromEventsManager();

    // Kill all threads
    cameraThread->join(true);
    rtabmapThread.join(true);
    delete cameraThread;

    // Close RTAB-Map and optionally save RTAB-Map database
    bool save = !outputPath.empty();
    rtabmap->close(save, outputPath);
    if (save) {
        std::cout << "Saved RTAB-Map database to " + outputPath << std::endl;
    }
    std::cout << "Finished!" << std::endl;

    return EXIT_SUCCESS;
}
