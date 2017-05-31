#! /usr/bin/env bash

# Parameters are: iniPath outputPath datasetPath
# spaintgui is located in the parent folder

set -e

tag='Relocopt_Batch'
# We skip chess heads and office since they are already over 99%
sequences='fire pumpkin redkitchen stairs'
# sequences='chess fire heads office pumpkin redkitchen stairs'
# sequences='stairs'

parameters_file=$1
output_file=$2
dataset_root=$3

# Move to the script folder, to have relative paths.
cd `dirname $0`

# Concatenate the default parameters file with the current parameters file
default_parameters_file="default_parameters.ini"
ini_file="$tag.ini"

cat "$parameters_file" > $ini_file
cat "$default_parameters_file" >> $ini_file

spaint_path="../../spaintgui/spaintgui"
reloc_poses_path="../../spaintgui/reloc_poses/"
relocperf_path="../../relocperf/relocperf"

training_sequence='train'
evaluation_sequence='validation'

depth_mask='frame-%06d.depth.png'
color_mask='frame-%06d.color.png'
pose_mask='frame-%06d.pose.txt'

# Run spaintgui on every sequence.
for seq in $sequences; do
  CUDA_VISIBLE_DEVICES=1 time "$spaint_path" -c "$dataset_root/calib.txt" -d "$dataset_root/$seq/$training_sequence/$depth_mask" -r "$dataset_root/$seq/$training_sequence/$color_mask" -p "$dataset_root/$seq/$training_sequence/$pose_mask" -t Disk -d "$dataset_root/$seq/$evaluation_sequence/$depth_mask" -r "$dataset_root/$seq/$evaluation_sequence/$color_mask" -t ForceFail --pipelineType slam --batch --experimentTag "$tag"_"$seq" -f "$ini_file" # --saveMeshOnExit
done

# Now evaluate the run.
$relocperf_path -d "$dataset_root" -r "$reloc_poses_path" -t "$tag" --useValidation > $output_file

# Remove ini file.
rm $ini_file
