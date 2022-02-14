#ifndef SPECTACULAR_AI_OFFLINE_FFMPEG_HPP
#define SPECTACULAR_AI_OFFLINE_FFMPEG_HPP

#include <cassert>
#include <sstream>

// Run a shell command and return its stdout (not stderr).
std::string exec(const std::string &cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
    assert(pipe);
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
            result += buffer.data();
    }
    return result;
}

bool ffprobeResolution(const std::string &videoPath, int &width, int &height) {
    std::string cmd = "ffprobe -v error -select_streams v:0 -show_entries stream=width,height -of csv=s=x:p=0 " + videoPath;
    std::string resolutionText = exec(cmd + " 2>/dev/null");
    if (sscanf(resolutionText.c_str(), "%dx%d", &width, &height) == 2) {
        return true;
    }
    assert(false);
    return false;
}

struct VideoInput {
private:
    FILE *pipe = nullptr;

public:
    int width = 0;
    int height = 0;

    VideoInput(const std::string &videoPath) {
        bool success = ffprobeResolution(videoPath, width, height);
        assert(success && width > 0 && height > 0);
        std::stringstream ss;
        ss << "ffmpeg -i " << videoPath
            << " -f rawvideo -vcodec rawvideo -vsync vfr -pix_fmt gray - 2>/dev/null";
        pipe = popen(ss.str().c_str(), "r");
        assert(pipe);
    }

    VideoInput(const VideoInput&) = delete;

    ~VideoInput() {
        assert(pipe);
        fflush(pipe);
        pclose(pipe);
    }

    bool read(std::vector<uint8_t> &video, int &width, int &height) {
        width = this->width;
        height = this->height;
        assert(pipe);
        int n = width * height;
        video.resize(n);
        int count = std::fread(video.data(), 1, n, pipe);
        return count == n;
    }
};

#endif
