#include <nlohmann/json.hpp>

#include "serialize_output.hpp"

void Serializer::serializeVioOutput(std::ofstream &outputStream, spectacularAI::VioOutputPtr vioOutput) {
    Matrix4d camMatrix = vioOutput->getCameraPose(0).getCameraToWorldMatrix();
    nlohmann::json json;
    json["cameraPoses"] = { {{"cameraToWorld", camMatrix}} };

    std::string jsonStr = json.dump();
    uint32_t jsonLength = jsonStr.length();

    MessageHeader header = {
        .magicBytes = MAGIC_BYTES,
        .messageId = messageIdCounter,
        .jsonSize = jsonLength,
        .binarySize = 0
    };

    outputStream.write(reinterpret_cast<char*>(&header), sizeof(MessageHeader));
    outputStream.write(jsonStr.c_str(), jsonStr.size());
    outputStream.flush();

    messageIdCounter++;
}

void Serializer::serializeMappingOutput(std::ofstream &outputStream, spectacularAI::mapping::MapperOutputPtr mapperOutput) {
    std::map<std::string, nlohmann::json> jsonKeyFrames;
    std::size_t binaryLength = 0;

    for (auto keyFrameId : mapperOutput->updatedKeyFrames) {
        auto search = mapperOutput->map->keyFrames.find(keyFrameId);
        if (search == mapperOutput->map->keyFrames.end()) continue; // deleted frame, skip
        auto& frameSet = search->second->frameSet;
        auto& pointCloud = search->second->pointCloud;
        Matrix4d camMatrix = frameSet->primaryFrame->cameraPose.getCameraToWorldMatrix();
        nlohmann::json keyFrameJson;
        keyFrameJson["id"] = std::to_string(keyFrameId);
        keyFrameJson["frameSet"] = {
            {"primaryFrame", {
                {"cameraPose", {{"cameraToWorld", camMatrix }} }
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

    outputStream.write(reinterpret_cast<char*>(&header), sizeof(MessageHeader));
    outputStream.write(jsonStr.c_str(), jsonStr.size());

    for (auto keyFrameId : mapperOutput->updatedKeyFrames) {
        auto search = mapperOutput->map->keyFrames.find(keyFrameId);
        if (search == mapperOutput->map->keyFrames.end()) continue;
        auto& pointCloud = search->second->pointCloud;
        std::size_t points = pointCloud->size();
        if (points > 0) {
            outputStream.write(
                reinterpret_cast<const char*>(pointCloud->getPositionData()),
                sizeof(spectacularAI::Vector3f) * points);

            if (pointCloud->hasNormals()) {
                outputStream.write(
                    reinterpret_cast<const char*>(pointCloud->getNormalData()),
                    sizeof(spectacularAI::Vector3f) * points);
            }

            if (pointCloud->hasColors()) {
                outputStream.write(
                    reinterpret_cast<const char*>(pointCloud->getRGB24Data()),
                    sizeof(std::uint8_t) * 3 * points);
            }
        }
    }

    outputStream.flush();
    messageIdCounter++;
}

