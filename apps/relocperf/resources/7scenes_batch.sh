#! /usr/bin/env bash

set -e

# cd to the script folder, to allow relative paths later
cd "${0%/*}"

tag=${1:-Default_NoRank} # The tag can be specified as first parameter of the script.
tag=${tag%.ini} # Remove trailing .ini from the tag if present

sequences='chess fire heads office pumpkin redkitchen stairs'

dataset_root=${2:-/media/data_ssd/datasets/7scenes} # The dataset root can be specified as second parameter of the script.
config_file="$tag.ini"
spaintgui_folder="../../spaintgui"
relocperf_folder=".."

for seq in $sequences; do
  time $spaintgui_folder/spaintgui -s "$dataset_root/$seq/train" -t Disk -s "$dataset_root/$seq/test" -t ForceFail --pipelineType slam --experimentTag "$tag"_"$seq" -f "$config_file" --batch --headless -v # --saveMeshOnExit
done

$relocperf_folder/relocperf -d "$dataset_root" -r $spaintgui_folder/reloc_poses -s $spaintgui_folder/reloc_times -t "$tag"
