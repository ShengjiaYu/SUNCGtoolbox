#!/bin/bash

housedir="/usr0/home/Datasets/SUNCG/house"
toolboxdir="/usr1/home/wyuan1/Repos/SUNCGtoolbox"
catfile="$toolboxdir/metadata/ModelCategoryMapping.csv"

echo "Processing $1"
cd $1

#$toolboxdir/gaps/bin/x86_64/scn2scn house.json house.obj
#$toolboxdir/gaps/bin/x86_64/scn2cam house.json cameras.txt -categories $catfile -output_camera_names camera_names.txt \
#    -output_camera_extrinsics extrinsics.txt -output_camera_intrinsics intrinsics.txt -v
rm -rf imgs
mkdir -p imgs
$toolboxdir/gaps/bin/x86_64/scn2img house.json cameras.txt imgs -categories $catfile -capture_color_images \
        -capture_depth_images -capture_kinect_images -capture_node_images -capture_category_images -v
rm -rf object_pcds
mkdir -p object_pcds
$toolboxdir/findobjects/build/findobjects house.json cameras.txt imgs object_pcds -min_visible_ratio 0.25 \
        -min_num_points 1000 -v
rm -rf partial_pcds
mkdir -p partial_pcds
$toolboxdir/depth2pcd/build/depth2pcd intrinsics.txt extrinsics.txt imgs partial_pcds
#rm -rf full_pcds
#mkdir -p full_pcds
#$toolboxdir/scn2pcd/build/scn2pcd house.json cameras.txt imgs full_pcds