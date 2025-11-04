#include "input.hpp"

#include <spectacularAI/vio.hpp>

#include <cassert>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

void addFrameSet(spectacularAI::Vio &vio, const Input::Data &data) {
    if (data.nFrames == 0) return;
    if (!data.video0 && !data.video0 && data.features0.empty()) return;

    spectacularAI::InputFrameSet inputFrameSet;
    inputFrameSet.timestamp = data.timestamp;

    std::vector<spectacularAI::InputFrame> inputFrames(data.nFrames);
    inputFrameSet.frames = inputFrames.data();
    inputFrameSet.nFrames = inputFrames.size();
    for (size_t i = 0; i < data.nFrames; ++i) {
        spectacularAI::InputFrame *inputFrame = &inputFrames.at(i);
        inputFrame->width = data.width;
        inputFrame->height = data.height;
        if (i == 0 && !data.features0.empty()) {
            inputFrame->features = data.features0.data();
            inputFrame->nFeatures = data.features0.size();
        }
        if (i == 0 && data.video0) {
            inputFrame->data = data.video0;
            inputFrame->colorFormat = spectacularAI::ColorFormat::GRAY;
        }
        if (i == 1 && data.video1) {
            inputFrame->data = data.video1;
            inputFrame->colorFormat = spectacularAI::ColorFormat::GRAY;
        }
    }
    vio.addFrameSet(&inputFrameSet);
}

int main(int argc, char *argv[]) {
    std::vector<std::string> arguments(argv, argv + argc);
    std::unique_ptr<std::ofstream> outputFile;
    std::unique_ptr<Input> input;
    std::string recordingFolder = "";
    bool useFrameSets = false;
    for (size_t i = 1; i < arguments.size(); ++i) {
        const std::string &argument = arguments.at(i);
        if (argument == "-o") {
            outputFile = std::make_unique<std::ofstream>(arguments.at(++i));
            assert(outputFile->is_open());
        }
        else if (argument == "-r") recordingFolder = arguments.at(++i);
        else if (argument == "-i") input = Input::buildJsonl(arguments.at(++i));
        else if (argument == "-f") useFrameSets = true;
    }
    std::ostream &output = outputFile ? *outputFile : std::cout;
    if (!input) input = Input::buildMock();

    std::ostringstream config;
    // This option makes the data input wait for VIO to finish to avoid dropping frames.
    // It should not be used in real-time scenarios.
    config << "blockingReplay: True\n";

    config << input->getConfig();
    auto builder = spectacularAI::Vio::builder()
        .setConfigurationYAML(config.str())
        .setCalibrationJSON(input->getCalibration());
    if (!recordingFolder.empty()) builder.setRecordingFolder(recordingFolder);
    std::unique_ptr<spectacularAI::Vio> vio = builder.build();

    vio->setOutputCallback([&](spectacularAI::VioOutputPtr vioOutput) {
        output << vioOutput->asJson().c_str() << std::endl;
    });

    Input::Data data;
    while (input->next(data)) {
        if (useFrameSets) {
            addFrameSet(*vio, data);
        }
        else if (data.video0 && data.video1) {
            vio->addFrameStereo(data.timestamp, data.width, data.height, data.video0, data.video1,
                spectacularAI::ColorFormat::GRAY);
        }
        else if (data.video0) {
            vio->addFrameMono(data.timestamp, data.width, data.height, data.video0,
                spectacularAI::ColorFormat::GRAY);
        }

        if (data.accelerometer) {
            vio->addAcc(data.timestamp, *data.accelerometer);
        }
        if (data.gyroscope) {
            vio->addGyro(data.timestamp, *data.gyroscope);
        }
    }

    return 0;
}
