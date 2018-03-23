#!/bin/bash

for file in /usr0/home/Datasets/SUNCG/object/*/*.obj
do (
	pcl_mesh_sampling $file ${file//obj/pcd} -no_vis_result #-write_normals
) done