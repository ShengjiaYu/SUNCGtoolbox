#include <math.h>
#include <iostream>
#include <string>
#include <map>
#include <pcl/filters/frustum_culling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "R3Graphics/R3Graphics.h"


static char *input_scene_file = NULL;
static char *input_cameras_file = NULL;
static char *input_img_directory = NULL;
static char *output_pcd_directory = NULL;
static char *input_categories_name = "/usr0/home/Datasets/SUNCG/metadata/ModelCategoryMapping.csv";
static char *model_directory = "/usr0/home/Datasets/SUNCG/object";
static char *valid_model_id_file = "/usr0/home/Datasets/SUNCG/metadata/ValidObjects.csv";
static int print_verbose = 0;
static int width = 640;
static int height = 480;
static float min_visible_ratio = 0;
static int min_num_points = 0;

static R3Scene *scene = NULL;
static RNArray<R3Camera *> cameras;
static RNArray<char *> camera_names;
static std::set<std::string> *valid_model_ids = new std::set<std::string>;


int
ReadScene(const char *filename)
{
  // Start statistics
  RNTime start_time;
  start_time.Read();

  // Allocate scene
  scene = new R3Scene();
  if (!scene) {
    fprintf(stderr, "Unable to allocate scene for %s\n", filename);
    return 0;
  }

  // Read scene from file
  if (!scene->ReadFile(filename)) {
    delete scene;
    return 0;
  }

  // Move referenced models to the current scene tree
  scene->RemoveReferences();

  // Print statistics
  if (print_verbose) {
    printf("Read scene from %s ...\n", filename);
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Nodes = %d\n", scene->NNodes());
    printf("  # Lights = %d\n", scene->NLights());
    printf("  # Materials = %d\n", scene->NMaterials());
    printf("  # Brdfs = %d\n", scene->NBrdfs());
    printf("  # Textures = %d\n", scene->NTextures());
    printf("  # Referenced models = %d\n", scene->NReferencedScenes());
    fflush(stdout);
  }

  // Return success
  return 1;
}


static int
ReadCameras(const char *filename)
{
  // Start statistics
  RNTime start_time;
  start_time.Read();
  int camera_count = 0;

  // Get useful variables
  RNScalar neardist = 0.01 * scene->BBox().DiagonalRadius();
  RNScalar fardist = 100 * scene->BBox().DiagonalRadius();
  RNScalar aspect = (RNScalar) height / (RNScalar) width;

  // Open file
  FILE *fp = fopen(filename, "r");
  if (!fp) {
    fprintf(stderr, "Unable to open cameras file %s\n", filename);
    return 0;
  }

  // Read file
  RNScalar vx, vy, vz, tx, ty, tz, ux, uy, uz, xf, yf, value;
  while (fscanf(fp, "%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf", &vx, &vy, &vz,
        &tx, &ty, &tz, &ux, &uy, &uz, &xf, &yf, &value) == (unsigned int) 12) {
    R3Point viewpoint(vx, vy, vz);
    R3Vector towards(tx, ty, tz);
    R3Vector up(ux, uy, uz);
    R3Vector right = towards % up;
    towards.Normalize();
    up = right % towards;
    up.Normalize();
    yf = atan(aspect * tan(xf));
    R3Camera *camera = new R3Camera(viewpoint, towards, up, xf, yf, neardist, fardist);
    camera->SetValue(value);
    cameras.Insert(camera);
    camera_count++;
  }

  // Close file
  fclose(fp);

  // Print statistics
  if (print_verbose) {
    printf("Read cameras from %s ...\n", filename);
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Cameras = %d\n", camera_count);
    fflush(stdout);
  }

  // Return success
  return 1;
}


static int
ReadCategories(const char *filename)
{
  // Start statistics
  RNTime start_time;
  start_time.Read();

  // Read file
  if (!scene->ReadSUNCGModelFile(filename)) return 0;

  // Print statistics
  if (print_verbose) {
    printf("Read categories from %s ...\n", filename);
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    fflush(stdout);
  }

  // Return success
  return 1;
}


static int
ReadValidModelIds(const char *filename)
{
  // Open file
  FILE *fp = fopen(filename, "r");
  if (!fp) {
    fprintf(stderr, "Unable to open valid object ID file %s\n", filename);
    return 0;
  }

  char id[10];
  char model_id[10];
  char class_name[100];
  while (fscanf(fp, "%s,%s,%s\n", id, model_id, class_name) == 3) {
    valid_model_ids->insert(model_id);
  }

  fclose(fp);

  return 1;
}


static int
ReadInstanceCounts(const char *filename, std::map<int, int> *instance_counts)
{
  R2Grid node(width, height);
  if (!node.ReadFile(filename)) {
    fprintf(stderr, "Unable to read node image %s\n", filename);
    return 0;
  }
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      int id = node(j, i);
      if (instance_counts->find(id) == instance_counts->end()) {
        (*instance_counts)[id] = 1;
      } else {
        (*instance_counts)[id] += 1;
      }
    }
  }
  return 1;
}


int
GetObjectPCD(const std::string& pcd_path,
             pcl::PointCloud<pcl::PointXYZ>::Ptr output,
             const R4Matrix& M)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
    std::cout << "Couldn't read point cloud from " << pcd_path << std::endl;
    return 0;
  }
  Eigen::Matrix4f transform;
  transform << M[0][0], M[0][1], M[0][2], M[0][3],
               M[1][0], M[1][1], M[1][2], M[1][3],
               M[2][0], M[2][1], M[2][2], M[2][3],
               M[3][0], M[3][1], M[3][2], M[3][3];
  RNTime start_time;
  pcl::transformPointCloud(*cloud, *output, transform);
  return 1;
}


void
CullPCD(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
        pcl::PointCloud<pcl::PointXYZ>::Ptr output,
        const R3Camera *camera)
{
  pcl::FrustumCulling<pcl::PointXYZ> fc;
  fc.setInputCloud(input);
  fc.setNearPlaneDistance(camera->Near());
  fc.setFarPlaneDistance(camera->Far());
  fc.setHorizontalFOV(camera->XFOV() * 360 / M_PI);
  fc.setVerticalFOV(camera->YFOV() * 360 / M_PI);
  R3Point origin = camera->Origin();
  R3Vector towards = camera->Towards();
  R3Vector up = camera->Up();
  R3Vector right = camera->Right();
  Eigen::Matrix4f pose;
  pose << towards[0], up[0], right[0], origin[0],
          towards[1], up[1], right[1], origin[1],
          towards[2], up[2], right[2], origin[2],
          0, 0, 0, 1;
  fc.setCameraPose(pose);
  fc.filter(*output);
}


static int
FindObjects(const char* output_directory)
{
  // Start statistics
  RNTime start_time;
  start_time.Read();

  // Write object point clouds
  char node_img_path[100];
  char model_pcd_path[100];
  char output_pcd_path[100];
  pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr visible(new pcl::PointCloud<pcl::PointXYZ>);
  int num_visible_objects = 0;
  for (int i = 0; i < cameras.NEntries(); ++i) {
    sprintf(node_img_path, "%s/%06d_node.png", input_img_directory, i);
    std::map<int, int> instance_counts;
    if (!ReadInstanceCounts(node_img_path, &instance_counts)) continue;
    for (std::map<int, int>::iterator it = instance_counts.begin(); it != instance_counts.end(); ++it) {
      int node_id = it->first;
      int node_count = it->second;
//      std::cout << i << " " << node_id << " " << node_count << std::endl;
      if (node_id > 0 && node_count > min_num_points) {
        R3SceneNode *node = scene->Node(node_id - 1);
        if (!strncmp(node->Name(), "Model", 5)) {
          const char *model_id = strchr(node->Name(), '#') + 1;
          if (valid_model_ids->find(model_id) != valid_model_ids->end()) {
            sprintf(model_pcd_path, "%s/%s/%s.pcd", model_directory, model_id, model_id);
            GetObjectPCD(model_pcd_path, object, node->CumulativeTransformation().Matrix());
            CullPCD(object, visible, cameras[i]);
            float visible_ratio = (float) visible->points.size() / object->points.size();
            if (visible_ratio > min_visible_ratio) {
              sprintf(output_pcd_path, "%s/%06d_%d_%s.pcd", output_directory, i, node_id, model_id);
              pcl::io::savePCDFileBinary(output_pcd_path, *object);
              num_visible_objects++;
            }
          }
        }
      }
    }
  }

  // Print statistics
  if (print_verbose) {
    printf("Wrote visible objects to %s\n", output_directory);
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Objects = %d\n", num_visible_objects);
    fflush(stdout);
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
      else if (!strcmp(*argv, "-categories")) { argc--; argv++; input_categories_name = *argv; }
      else if (!strcmp(*argv, "-model_directory")) {argc--; argv++; model_directory = *argv; }
      else if (!strcmp(*argv, "-valid_model_ids")) { argc--; argv++; valid_model_id_file = *argv; }
      else if (!strcmp(*argv, "-min_visible_ratio")) { argc--; argv++; min_visible_ratio = atof(*argv); }
      else if (!strcmp(*argv, "-min_num_points")) { argc--; argv++; min_num_points = atof(*argv); }
      else {
        fprintf(stderr, "Invalid program argument: %s", *argv);
        exit(1);
      }
      argv++; argc--;
    }
    else {
      if (!input_scene_file) input_scene_file = *argv;
      else if (!input_cameras_file) input_cameras_file = *argv;
      else if (!input_img_directory) input_img_directory = *argv;
      else if (!output_pcd_directory) output_pcd_directory = *argv;
      else { fprintf(stderr, "Invalid program argument: %s", *argv); exit(1); }
      argv++; argc--;
    }
  }

  // Check filenames
  if (!input_scene_file || !input_cameras_file || !input_img_directory || !output_pcd_directory) {
    fprintf(stderr, "Usage: findobjects inputscenefile inputcamerasfile inputimgdirectory outputpcddirectory \n");
    return 0;
  }

  // Return OK status
  return 1;
}


int main(int argc, char **argv)
{
  // Check number of arguments
  if (!ParseArgs(argc, argv)) exit(1);

  // Read scene
  if (!ReadScene(input_scene_file)) exit(-1);

  // Read cameras
  if (!ReadCameras(input_cameras_file)) exit(-1);

  // Read valid object ids
  if (valid_model_id_file) {
    if (!ReadValidModelIds(valid_model_id_file)) exit(-1);
  }

  // Read categories
  if (input_categories_name) {
    if (!ReadCategories(input_categories_name)) exit(-1);
  }

  if (!FindObjects(output_pcd_directory)) exit(-1);

  // Return success
  return 0;
}
