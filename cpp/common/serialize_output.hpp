#include <array>
#include <fstream>

#include <spectacularAI/output.hpp>
#include <spectacularAI/mapping.hpp>

// Random number indicating start of a MessageHeader
#define MAGIC_BYTES 2727221974

using Matrix3d = std::array<std::array<double, 3>, 3>;
using Matrix4d = std::array<std::array<double, 4>, 4>;
using SAI_BOOL = uint8_t;

struct MessageHeader {
    uint32_t magicBytes;
    uint32_t messageId; // Counter for debugging
    uint32_t jsonSize;
    uint32_t binarySize;
};

class Serializer {
public:
    void serializeVioOutput(std::ofstream &outputStream, spectacularAI::VioOutputPtr vioOutput);
    void serializeMappingOutput(std::ofstream &outputStream, spectacularAI::mapping::MapperOutputPtr mapperOutput);

private:
    uint32_t messageIdCounter = 0;
};
