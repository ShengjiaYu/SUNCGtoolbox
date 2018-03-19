#include <pcl/io/pcd_io.h>
#include "R3Graphics/R3Graphics.h"

static int width = 640;
static int height = 480;


int main(int argc, char **argv) {
    char *scene_dir = argv[1];
    char intrinsics_path[100];
    char extrinsics_path[100];
    sprintf(intrinsics_path, "%s/intrinsics.txt", scene_dir);
    FILE *intrinsics = fopen(intrinsics_path, "r");
//    sprintf(extrinsics_path, "%s/extrinsics.txt", scene_dir);
//    FILE *extrinsics = fopen(extrinsics_path, "r");

    int i = 0;
    char depth_img_path[100];
    char output_pcd_path[100];
    RNScalar fx, fy, s, cx, cy, zero1, zero2, zero3, one;
    while (fscanf(intrinsics, "%lf%lf%lf%lf%lf%lf%lf%lf%lf",
           &fx, &s, &cx, &zero1, &fy, &cy, &zero2, &zero3, &one) == 9) {
        sprintf(depth_img_path, "%s/imgs/%06d_depth.png", scene_dir, i);
        R2Grid depth(width, height);
        depth.ReadFile(depth_img_path);
        depth.Divide(1000);

        // https://stackoverflow.com/questions/31265245/extracting-3d-coordinates-given-2d-image-points-depth-map-and-camera-calibratio
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>(width, height));
        int depth_idx = 0;
        for (int y = 0; y < cloud->height; ++y) {
            for (int x = 0; x < cloud->width; ++x, ++depth_idx) {
                pcl::PointXYZ& pt = cloud->points[depth_idx];
                pt.x = (x - cx) * depth(x, y) / fx;
                pt.y = (y - cy) * depth(x, y) / fy;
                pt.z = depth(x, y);
            }
        }
        sprintf(output_pcd_path, "%s/partial_pcds/%06d.pcd", scene_dir, i);
        pcl::io::savePCDFile(output_pcd_path, *cloud, /* binary_mode */ true);
        i++;
    }
}
