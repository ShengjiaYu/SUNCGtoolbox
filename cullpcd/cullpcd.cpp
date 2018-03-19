#include <math.h>
#include <pcl/common/common.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/distances.h>


static char *input_pcd_name = NULL;
static char *input_cameras_name = NULL;
static char *output_directory = NULL;
static bool print_verbose = false;


int
CullPCD(const char *input_pcd_name, const char *input_cameras_name,
  const char *output_directory)
{
  // Read point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr frustum(new pcl::PointCloud<pcl::PointXYZ>);
  char output_filename[1024];

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_name, *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return 0;
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from "
            << input_pcd_name
            << std::endl;

  FILE *fp = fopen(input_cameras_name, "r");
  if (!fp) {
    fprintf(stderr, "Unable to open cameras file %s\n", input_cameras_name);
    return 0;
  }

  pcl::FrustumCulling<pcl::PointXYZ> fc;
  fc.setInputCloud(cloud);
  fc.setNearPlaneDistance(0.1);
  fc.setFarPlaneDistance(10);

  int index = 0;
  float vx, vy, vz, tx, ty, tz, ux, uy, uz, xf, yf, value;
  while (fscanf(fp, "%f%f%f%f%f%f%f%f%f%f%f%f", &vx, &vy, &vz, &tx, &ty, &tz,
      &ux, &uy, &uz, &xf, &yf, &value) == (unsigned int) 12) {
    Eigen::Vector3f viewpoint(vx, vy, vz);
    Eigen::Vector3f towards(tx, ty, tz);
    Eigen::Vector3f up(ux, uy, uz);
    Eigen::Vector3f right = towards.cross(up);
    Eigen::Matrix4f pose;
    pose.block(0,0,3,4) << towards, up, right, viewpoint;
    pose.row(3) << 0,0,0,1;
    if (print_verbose) {
      std::cout << "Camera" << index << std::endl << viewpoint << std::endl
          << towards << std::endl << up << std::endl << right << std::endl
          << pose << std::endl;
    }

    fc.setCameraPose(pose);
    fc.setHorizontalFOV(xf * 360 / M_PI);
    fc.setVerticalFOV(yf * 360 / M_PI);
    fc.filter(*frustum);
    sprintf(output_filename, "%s/%06d.pcd", output_directory, index);
    pcl::io::savePCDFile(output_filename, *frustum, /* binary_mode */ true);
    std::cout << "Saved " << frustum->points.size()
              << " data points to " << output_filename << std::endl;
    index++;
  }

  // Return success
  return 1;
}


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
      if (!input_pcd_name) input_pcd_name = *argv;
      else if (!input_cameras_name) input_cameras_name = *argv;
      else if (!output_directory) output_directory = *argv;
      else { fprintf(stderr, "Invalid program argument: %s", *argv); exit(1); }
      argv++; argc--;
    }
  }

  // Check filenames
  if (!input_pcd_name || !input_cameras_name || !output_directory) {
    fprintf(stderr, "Usage: cullpcd inputpcdfile inputcamerasfile outputdirectory\n");
    return 0;
  }

  // Return OK status 
  return 1;
}


int main(int argc, char **argv)
{
  // Parse program arguments
  if (!ParseArgs(argc, argv)) exit(-1);

  // Cull point cloud
  if (!CullPCD(input_pcd_name, input_cameras_name, output_directory)) exit(-1);

  // Return success 
  return 0;
}