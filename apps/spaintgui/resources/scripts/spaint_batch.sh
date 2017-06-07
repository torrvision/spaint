#! /usr/bin/env bash

set -e

tag='Default'
sequences='chess fire heads office pumpkin redkitchen stairs'
# sequences='pumpkin redkitchen stairs'
# sequences='heads office'
# sequences='stairs'
dataset_root='/media/data/7scenes'
depth_mask='frame-%06d.depth.png'
color_mask='frame-%06d.color.png'
pose_mask='frame-%06d.pose.txt'


for seq in $sequences; do
#  CUDA_VISIBLE_DEVICES=1 time ../../spaintgui -c "$dataset_root/calib.txt" -d "$dataset_root/$seq/train/$depth_mask" -r "$dataset_root/$seq/train/$color_mask" -p "$dataset_root/$seq/train/$pose_mask" -t Disk -d "$dataset_root/$seq/test/$depth_mask" -r "$dataset_root/$seq/test/$color_mask" -t ForceFail --pipelineType slam --batch --experimentTag "$tag"_"$seq" -f "7scenes.ini" # --saveMeshOnExit
  CUDA_VISIBLE_DEVICES=0 time ./bin/apps/spaintgui/spaintgui -c "$dataset_root/calib.txt" -d "$dataset_root/$seq/train/$depth_mask" -r "$dataset_root/$seq/train/$color_mask" -p "$dataset_root/$seq/train/$pose_mask" -t Disk -d "$dataset_root/$seq/test/$depth_mask" -r "$dataset_root/$seq/test/$color_mask" -t ForceFail --pipelineType slam --batch --experimentTag "$tag"_"$seq" -f "7scenes.ini" # --saveMeshOnExit
done

