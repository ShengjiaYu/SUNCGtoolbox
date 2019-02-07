#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp>

static int width = 640;
static int height = 480;

int main(int argc, char **argv) {
  if (argc < 5) {
    fprintf(stderr, "Usage: depth2pcd intrinsics_file extrinsics_file input_image_path output_pcd_path\n");
    return 0;
  }
  const char *intrinsics_file = argv[1];
  const char *extrinsics_file = argv[2];
  const char *depth_img_path = argv[3];
  const char *output_pcd_path = argv[4];
  FILE *intrinsics = fopen(intrinsics_file, "r");
  FILE *extrinsics = fopen(extrinsics_file, "r");

  RNScalar fx, fy, s, cx, cy, zero1, zero2, zero3, one;
  RNScalar r1, r2, r3, r4, r5, r6, r7, r8, r9, t1, t2, t3;
  if (fscanf(intrinsics, "%lf%lf%lf%lf%lf%lf%lf%lf%lf",
             &fx, &s, &cx, &zero1, &fy, &cy, &zero2, &zero3, &one) != 9) {
    fprintf(stderr, "Not enough values in intrinsics file\n");
    return 0;
  }
  if (fscanf(extrinsics, "%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf",
             &r1, &r2, &r3, &t1, &r4, &r5, &r6, &t2, &r7, &r8, &r9, &t3) != 12) {
    fprintf(stderr, "Not enough values in extrinsics file\n");
    return 0;
  }

  R2Grid depth(width, height);
  cv::Mat depth_mat = cv::imread(depth_img_path, CV_LOAD_IMAGE_UNCHANGED);
  depth.Divide(1000);

  // https://stackoverflow.com/questions/31265245
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>(width, height));
  int depth_idx = 0;
  for (int y = 0; y < cloud->height; ++y) {
    for (int x = 0; x < cloud->width; ++x, ++depth_idx) {
      pcl::PointXYZ& pt = cloud->points[depth_idx];
      pt.x = (x - cx) * depth(x, y) / fx;
      pt.y = (y - cy) * depth(x, y) / fy;
      pt.z = -depth(x, y);  // Camera's z-axis points into the camera
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud <pcl::PointXYZ>);
  Eigen::Matrix4f transform;
  transform << r1, r2, r3, t1,
               r4, r5, r6, t2,
               r7, r8, r9, t3,
               0, 0, 0, 1;
  std::cout << transform << std::endl;
  pcl::transformPointCloud(*cloud, *output, transform);

  sprintf(output_pcd_path, "%s/frame-%06d.ply", output_pcd_directory, i);
  pcl::io::savePLYFile(output_pcd_path, *output);

  if (print_verbose) {
    printf("Wrote depth point clouds to %s\n", output_pcd_directory);
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Point clouds = %d\n", i);
    fflush(stdout);
  }
}
