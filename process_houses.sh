#!/bin/bash

housedir="/usr0/home/Datasets/SUNCG/house"
toolboxdir="/usr1/home/wyuan1/Repos/SUNCGtoolbox"
catfile="$toolboxdir/metadata/ModelCategoryMapping.csv"

echo "Processing $1"
cd $1
rm -rf imgs
mkdir -p imgs
mkdir -p full_pcds
mkdir -p partial_pcds

# $toolboxdir/gaps/bin/x86_64/scn2scn house.json house.obj
$toolboxdir/gaps/bin/x86_64/scn2cam house.json cameras.txt -categories $catfile -output_camera_names camera_names.txt \
    -output_camera_extrinsics extrinsics.txt -output_camera_intrinsics intrinsics.txt
$toolboxdir/gaps/bin/x86_64/scn2img house.json cameras.txt imgs -categories $catfile -capture_color_images \
    -capture_depth_images -capture_kinect_images -capture_normal_images -capture_node_images -capture_category_images
$toolboxdir/scn2pcd/build/scn2pcd house.json cameras.txt imgs full_pcds
$toolboxdir/depth2pcd/build/depth2pcd .