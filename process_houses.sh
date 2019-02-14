#!/bin/bash

toolboxdir="//home/mengqing/SUNCGtoolbox"
catfile="/data/SUNCG/metadata/ModelCategoryMapping.csv"
outputdir="."

echo "Processing $1"
cd $1
# if [ -d "$outputdir" ]; then
#     rm -rf $outputdir
# fi
# mkdir -m 775 $outputdir
# chgrp users $outputdir

# $toolboxdir/gaps/bin/x86_64/scn2scn house.json house.obj

# generate cameras
$toolboxdir/gaps/bin/x86_64/scn2cam house.json $outputdir/cameras.txt -categories $catfile \
    -width 320 -height 240 -xfov 0.54 \
    -position_sampling 1.5 -eye_height 1.5 -eye_height_radius 0.1 \
    -min_visible_objects 0 \
    -angle_sampling 0.628 \
    -output_camera_names $outputdir/camera_names.txt \
    -output_camera_extrinsics $outputdir/extrinsics.txt \
    -output_camera_intrinsics $outputdir/intrinsics.txt \
    -output_nodes $outputdir/nodes.txt

# render images
rm -rf $outputdir/imgs
mkdir -p $outputdir/imgs
#$toolboxdir/gaps/bin/x86_64/scn2img house.json $outputdir/cameras.txt $outputdir/imgs \
#    -width 320 -height 240 -capture_color_images -capture_depth_images -capture_node_images -v
$toolboxdir/gaps/bin/x86_64/scn2img house.json $outputdir/cameras.txt $outputdir/imgs \
    -kinect_noise_fraction 0.01 \
    -width 320 -height 240 -capture_color_images -capture_kinect_images -capture_node_images -v

# generate partial point cloud
$toolboxdir/depth2pcd/build/fuse_depths $outputdir/intrinsics.txt $outputdir/extrinsics.txt \
    $outputdir/imgs $outputdir/house_partial.pcd $outputdir/instance.txt -width 320 -height 240 -leaf_size 0.05 -v

# write object poses
$toolboxdir/scn2pcd/build/scn2poses house.json $outputdir/object_poses

# generating complete point cloud
# $toolboxdir/scn2pcd/build/scn2pcd house.json house.pcd -categories $catfile -v
# $toolboxdir/scn2pcd/build/scn2pcd -c -v $datadir

#rm -rf object_pcds
#mkdir -p object_pcds
#$toolboxdir/findobjects/build/findobjects house.json cameras.txt imgs object_pcds -min_visible_ratio 0.25 \
#    -min_num_points 1000 -v
#rm -rf partial_pcds
#mkdir -p partial_pcds
#$toolboxdir/depth2pcd/build/depth2pcd intrinsics.txt extrinsics.txt imgs partial_pcds -v
#rm -rf full_pcds
#mkdir -p full_pcds
#$toolboxdir/view2pcd/build/view2pcd house.json cameras.txt imgs full_pcds
