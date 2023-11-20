#include <sstream>
#include <iomanip>
#include <nlohmann/json.hpp>

#include "serialize_output.hpp"

namespace {

    nlohmann::json serializeCamera(const spectacularAI::Camera &camera) {
        float near = 0.01f, far = 100.0f;
        const Matrix3d &intrinsics = camera.getIntrinsicMatrix();
        const Matrix4d &projectionMatrixOpenGL = camera.getProjectionMatrixOpenGL(near, far);

        nlohmann::json json;
        json["intrinsics"] = intrinsics;
        json["projectionMatrixOpenGL"] = projectionMatrixOpenGL;
        return json;
    }

} // anonymous namespace

void Serializer::serializeVioOutput(FILE *out, spectacularAI::VioOutputPtr vioOutput) {
    const spectacularAI::Camera &camera = *vioOutput->getCameraPose(0).camera;
    const Matrix4d &cameraToWorld = vioOutput->getCameraPose(0).getCameraToWorldMatrix();

    // Only properties used in current visualization are serialized, i.e. add more stuff if needed.
    nlohmann::json json;
    json["cameraPoses"] = {
        {
            {"camera", serializeCamera(camera)},
            {"cameraToWorld", cameraToWorld}
        }
    };

    std::string jsonStr = json.dump();
    uint32_t jsonLength = jsonStr.length();
    MessageHeader header = {
        .magicBytes = MAGIC_BYTES,
        .messageId = messageIdCounter,
        .jsonSize = jsonLength,
        .binarySize = 0
    };
    fwrite(&header, sizeof(MessageHeader), 1, out);
    fprintf(out, "%s", jsonStr.c_str());
    messageIdCounter++;
}

void Serializer::serializeMappingOutput(FILE *out, spectacularAI::mapping::MapperOutputPtr mapperOutput) {
    std::map<std::string, nlohmann::json> jsonKeyFrames;
    std::size_t binaryLength = 0;

    for (auto keyFrameId : mapperOutput->updatedKeyFrames) {
        auto search = mapperOutput->map->keyFrames.find(keyFrameId);
        if (search == mapperOutput->map->keyFrames.end()) continue; // deleted frame, skip
        auto& frameSet = search->second->frameSet;
        auto& pointCloud = search->second->pointCloud;
        const spectacularAI::Camera &camera = *frameSet->primaryFrame->cameraPose.camera;
        const Matrix4d &cameraToWorld = frameSet->primaryFrame->cameraPose.getCameraToWorldMatrix();
        nlohmann::json keyFrameJson;
        keyFrameJson["id"] = std::to_string(keyFrameId);
        keyFrameJson["frameSet"] = {
            {"primaryFrame", {
                {"cameraPose", {
                    {"camera", serializeCamera(camera)},
                    {"cameraToWorld", cameraToWorld}
                }}
            }}
        };
        std::size_t points = pointCloud->size();
        if (points > 0) {
            keyFrameJson["pointCloud"] = {
                {"size", points },
                {"hasNormals", pointCloud->hasNormals() },
                {"hasColors", pointCloud->hasColors() },
            };
            binaryLength += points * sizeof(spectacularAI::Vector3f);
            if (pointCloud->hasNormals()) binaryLength += points * sizeof(spectacularAI::Vector3f);
            if (pointCloud->hasColors()) binaryLength += points * sizeof(std::uint8_t) * 3;
        }
        jsonKeyFrames[keyFrameJson["id"]] = keyFrameJson;
    }

    nlohmann::json json;
    json["updatedKeyFrames"] = mapperOutput->updatedKeyFrames;
    json["map"] = {{"keyFrames", jsonKeyFrames}};
    json["finalMap"] = mapperOutput->finalMap;

    std::string jsonStr = json.dump();
    uint32_t jsonLength = jsonStr.length();
    MessageHeader header = {
        .magicBytes = MAGIC_BYTES,
        .messageId = messageIdCounter,
        .jsonSize = jsonLength,
        .binarySize = (uint32_t)binaryLength
    };

    fwrite(&header, sizeof(MessageHeader), 1, out);
    fprintf(out, "%s", jsonStr.c_str());

    for (auto keyFrameId : mapperOutput->updatedKeyFrames) {
        auto search = mapperOutput->map->keyFrames.find(keyFrameId);
        if (search == mapperOutput->map->keyFrames.end()) continue;
        auto& pointCloud = search->second->pointCloud;
        std::size_t points = pointCloud->size();
        if (points > 0) {
            fwrite(pointCloud->getPositionData(), sizeof(spectacularAI::Vector3f), points, out);
            if (pointCloud->hasNormals())
                fwrite(pointCloud->getNormalData(), sizeof(spectacularAI::Vector3f), points, out);
            if (pointCloud->hasColors())
                fwrite(pointCloud->getRGB24Data(), sizeof(std::uint8_t) * 3, points, out);
        }
    }

    messageIdCounter++;
}

