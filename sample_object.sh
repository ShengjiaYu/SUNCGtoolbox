#!/bin/bash

for file in object/*/*.obj
do (
	pcl_mesh_sampling $file ${file//obj/pcd} -no_vis_result #-write_normals
) done