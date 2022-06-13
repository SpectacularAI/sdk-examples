#include <spectacularAI/vio.hpp>
#include <spectacularAI/replay.hpp>

#include <iostream>
#ifdef EXAMPLE_USE_OPENCV
#include <opencv2/opencv.hpp>
#endif

int main(int argc, char** argv) {
    if (argc < 2) return 1;

    std::string dataFolder = argv[1];

    spectacularAI::Vio::Builder vioBuilder = spectacularAI::Vio::builder();
    auto replayApi = spectacularAI::Replay::builder(dataFolder, vioBuilder)
        .build();

    replayApi->setExtendedOutputCallback([&](spectacularAI::VioOutputPtr output, spectacularAI::FrameSet frames) {
        std::cout << output->asJson() << std::endl;
        #ifdef EXAMPLE_USE_OPENCV
        for (int i = 0; i < frames.size(); i++) {
            // Note: typically not main thread
            auto &frame = frames[i];
            if (frame->image) {
                cv::Mat img = frame->image->asOpenCV();
                cv::imshow("Video " + std::to_string(i), img);
                cv::waitKey(1);
            }
        }
        #else
        (void)frames;
        #endif
    });

    replayApi->startReplay();

    return 0;
}
