#include <iostream>
#include <spectacularAI/replay.hpp>
#ifdef EXAMPLE_USE_OPENCV
#include <opencv2/opencv.hpp>
#endif

int main(int argc, char** argv) {
    if (argc < 2) return 1;

    std::string dataFolder = argv[1];

    auto replayApi = spectacularAI::Replay::builder()
            .setDataFolder(dataFolder)
            .build();

    replayApi->setExtendedOutputCallback([&](spectacularAI::VioOutputPtr output, spectacularAI::FrameSet frames){
        std::cout << output->asJson() << std::endl;
        #ifdef EXAMPLE_USE_OPENCV
        for (int i = 0; i < frames.size(); i++) {
            // Note: typically not main thread
            auto &frame = frames[i];
            if (frame->image) {
                cv::Mat img;
                if (frame->image->getColorFormat() == spectacularAI::ColorFormat::RGB)
                    cv::cvtColor(frame->image->asOpenCV(), img, cv::COLOR_RGB2BGR);
                else
                    img = frame->image->asOpenCV();
                cv::imshow("Video " + std::to_string(i), img);
                cv::waitKey(1);
            }
        }
        #endif
    });

    replayApi->startReplay();

    return 0;
}
