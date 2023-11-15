#!/bin/bash

set -eux

: "${OUTPUT_FOLDER:=recordings/$(date "+%Y%m%d-%H%M%S")}"

mkdir -p "$OUTPUT_FOLDER"

echo "------------- Press Q to stop recording -------------"
python mapping_visu.py --keyFrameCandidateInterval=4 --recordingFolder="$OUTPUT_FOLDER"
python replay_to_nerf.py "$OUTPUT_FOLDER" "$OUTPUT_FOLDER"/nerfstudio --device_preset=oak-d --preview

echo "Now run this Nerfstudio command:

    ns-train nerfacto --data $OUTPUT_FOLDER/nerfstudio
"
