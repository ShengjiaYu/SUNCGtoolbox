#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include "R3Graphics/R3Graphics.h"

static int width = 640;
static int height = 480;
static int print_verbose = 0;
static char *intrinsics_file = NULL;
static char *extrinsics_file = NULL;
static char *input_img_directory = NULL;
static char *output_pcd_directory = NULL;


static int
ParseArgs(int argc, char **argv)
{
  // Parse arguments
  argc--; argv++;
  while (argc > 0) {
    if ((*argv)[0] == '-') {
      if (!strcmp(*argv, "-v")) print_verbose = 1;
      else {
        fprintf(stderr, "Invalid program argument: %s", *argv);
        exit(1);
      }
      argv++; argc--;
    }
    else {
      if (!intrinsics_file) intrinsics_file = *argv;
      else if (!extrinsics_file) extrinsics_file = *argv;
      else if (!input_img_directory) input_img_directory = *argv;
      else if (!output_pcd_directory) output_pcd_directory = *argv;
      else { fprintf(stderr, "Invalid program argument: %s", *argv); exit(1); }
      argv++; argc--;
    }
  }

  // Check filenames
  if (!intrinsics_file || !extrinsics_file || !input_img_directory || !output_pcd_directory) {
    fprintf(stderr, "Usage: depth2pcd intrinsicsfile extrinsicsfile inputimgdirectory outputpcddirectory\n");
    return 0;
  }

  // Return OK status
  return 1;
}


int main(int argc, char **argv) {
  // Check number of arguments
  if (!ParseArgs(argc, argv)) exit(1);
  FILE *intrinsics = fopen(intrinsics_file, "r");
  FILE *extrinsics = fopen(extrinsics_file, "r");

  int i = 0;
  char depth_img_path[100];
  char output_pcd_path[100];
  RNScalar fx, fy, s, cx, cy, zero1, zero2, zero3, one;
  RNScalar r1, r2, r3, r4, r5, r6, r7, r8, r9, t1, t2, t3;

  RNTime start_time;
  start_time.Read();
  while (fscanf(intrinsics, "%lf%lf%lf%lf%lf%lf%lf%lf%lf",
                &fx, &s, &cx, &zero1, &fy, &cy, &zero2, &zero3, &one) == 9 &&
         fscanf(extrinsics, "%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf",
                &r1, &r2, &r3, &t1, &r4, &r5, &r6, &t2, &r7, &r8, &r9, &t3) == 12) {
    sprintf(depth_img_path, "%s/%06d_depth.png", input_img_directory, i);
    R2Grid depth(width, height);
    depth.ReadFile(depth_img_path);
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
      pcl::transformPointCloud(*cloud, *output, transform);

      sprintf(output_pcd_path, "%s/%06d.pcd", output_pcd_directory, i);
      pcl::io::savePCDFile(output_pcd_path, *output);
      i++;
  }

  if (print_verbose) {
    printf("Wrote depth point clouds to %s\n", output_pcd_directory);
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Point clouds = %d\n", i);
    fflush(stdout);
  }
}
