#!/bin/bash

catfile="../../SUNCGtoolbox/metadata/ModelCategoryMapping.csv"
for dir in house/0a*
do (
	echo "Processing $dir"
	cd $dir
	mkdir -p imgs
	mkdir -p pcds

	../../SUNCGtoolbox/gaps/bin/x86_64/scn2scn house.json house.obj
	../../SUNCGtoolbox/gaps/bin/x86_64/scn2cam house.json cameras.txt -output_camera_names camera_names.txt \
	-output_camera_extrinsics extrinsics.txt -output_camera_intrinsics intrinsics.txt -categories $catfile -v
	../../SUNCGtoolbox/gaps/bin/x86_64/scn2img house.json cameras.txt imgs -categories $catfile -v
	../../scn2pcd/build/scn2pcd house.json cameras.txt camera_names.txt pcds -categories $catfile -min_visible_ratio 0.2 -v
) done