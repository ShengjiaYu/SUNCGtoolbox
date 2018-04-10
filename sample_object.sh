#!/bin/bash

sampling_binary="/usr1/home/wyuan1/Repos/point-cvae/pcl_mod/build/bin/pcl_mesh_sampling"
for file in /usr0/home/Datasets/SUNCG/object/*/*.obj
do (
	$sampling_binary $file ${file//.obj/.pcd} -no_vis_result -leaf_size 0.005
) done