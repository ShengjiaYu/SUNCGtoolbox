#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include "R3Graphics/R3Graphics.h"


int main(int argc, char **argv) {
    char *scene_dir = argv[1];
    char extrinsics_path[100];
    sprintf(extrinsics_path, "%s/extrinsics.txt", scene_dir);
    FILE *extrinsics = fopen(extrinsics_path, "r");

    int i = 0;
    char input_path[100];
    char output_path[100];
    RNScalar r1, r2, r3, r4, r5, r6, r7, r8, r9, t1, t2, t3;
    while (fscanf(extrinsics, "%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf",
           &r1, &r2, &r3, &t1, &r4, &r5, &r6, &t2, &r7, &r8, &r9, &t3) == 12) {
        sprintf(input_path, "%s/pcds/%06d.pcd", scene_dir, i);
        pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud <pcl::PointXYZ>);
        pcl::io::loadPCDFile(input_path, *input);

        Eigen::Affine3f translation = Eigen::Affine3f::Identity();
        translation.translation() << -t1, -t2, -t3;
        std::cout << translation.matrix() << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr translated;
        pcl::transformPointCloud(*input, *translated, translation);

        Eigen::Matrix4f refelection = Eigen::Matrix4f::Identity();
        refelection(2, 2) = -1;
        std::cout << refelection << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr reflected;
        pcl::transformPointCloud(*translated, *reflected, refelection);

        Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
        rotation << r1, r2, r3, 0, r4, r5, r6, 0, r7, r8, r9, 0, 0, 0, 0, 1;
        std::cout << rotation << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr output;
        pcl::transformPointCloud(*output, *reflected, rotation);

        sprintf(output_path, "%s/full_pcds/%06d.pcd", scene_dir, i);
        pcl::io::savePCDFile(input_path, *output);
        i++;
    }
}
