#include <cstdint>
#include <array>
#include <spectacularAI/output.hpp>
#include <spectacularAI/mapping.hpp>

// Random number indicating start of a MessageHeader
#define MAGIC_BYTES 2727221974

using Matrix4d = std::array<std::array<double, 4>, 4>;
using SAI_BOOL = uint8_t;

#pragma pack(push)
#pragma pack(1)
struct MessageHeader {
    uint32_t magicBytes;
    uint32_t messageId; // Counter for debugging
    uint32_t jsonSize;
    uint32_t binarySize;
};
#pragma pack(pop)

class Serializer {
public:
    void serializeVioOutput(FILE *out, spectacularAI::VioOutputPtr vioOutput);
    void serializeMappingOutput(FILE *out, spectacularAI::mapping::MapperOutputPtr mapperOutput);

private:
    uint32_t messageIdCounter = 0;
};
