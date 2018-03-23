#!/bin/bash

housedir="/usr0/home/Datasets/SUNCG/house"
toolboxdir="/usr1/home/wyuan1/Repos/SUNCGtoolbox"
catfile="$toolboxdir/metadata/ModelCategoryMapping.csv"
for dir in $housedir/0a0b*
do (
	echo "Processing $dir"
	cd $dir
	mkdir -p imgs
	mkdir -p pcds

	$toolboxdir/gaps/bin/x86_64/scn2scn house.json house.obj
	$toolboxdir/gaps/bin/x86_64/scn2cam house.json cameras.txt -output_camera_names camera_names.txt \
	-output_camera_extrinsics extrinsics.txt -output_camera_intrinsics intrinsics.txt -categories $catfile -v
	$toolboxdir/gaps/bin/x86_64/scn2img house.json cameras.txt imgs -categories $catfile -v
	$toolboxdir/scn2pcd/build/scn2pcd house.json cameras.txt camera_names.txt pcds -categories $catfile \
	-min_visible_ratio 0.2 -v
	$toolboxdir/depth2pcd/build/depth2pcd .
) done