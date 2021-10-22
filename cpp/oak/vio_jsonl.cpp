#include <iostream>
#include <depthai/depthai.hpp>
#include <spectacularAI/depthai/plugin.hpp>
#ifdef EXAMPLE_USE_OPENCV
#include <opencv2/opencv.hpp>
#endif

int main() {
    // Create Depth AI (OAK-D) pipeline
    dai::Pipeline pipeline;

    // Optional configuration
    spectacularAI::daiPlugin::Configuration config;
    // config.useStereo = false;
    // config.useFeatureTracker = false;

    spectacularAI::daiPlugin::Pipeline vioPipeline(pipeline, config);

#ifdef EXAMPLE_USE_OPENCV
    // note: must set useFeatureTracker = false or mono cam will not be read
    vioPipeline.hooks.monoPrimary = [&](std::shared_ptr<dai::ImgFrame> img) {
        // Note: typically not main thread
        cv::imshow("primary mono cam", img->getCvFrame());
        cv::waitKey(1);
    };
#endif

    // Connect to device and start pipeline
    dai::Device device(pipeline);
    auto vioSession = vioPipeline.startSession(device);

    while (true) {
        auto vioOut = vioSession->waitForOutput();
        std::cout << vioOut->asJson() << std::endl;
    }

    return 0;
}
