#include <math.h>
#include <fstream>
#include <iostream>
#include <string>
#include <set>
#include <pcl/filters/frustum_culling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "R3Graphics/R3Graphics.h"

static char *input_scene_file = NULL;
static char *input_categories_name = NULL;
static char *data_directory = NULL;
static char *output_pcd_file = NULL;
static int print_verbose = 0;
static int read_categories = 0;

static R3Scene *scene = NULL;
static RNArray<R3Camera *> cameras;
static RNArray<char *> camera_names;


int
ReadScene()
{
  // Start statistics
  RNTime start_time;
  start_time.Read();
  char filename[100];
  sprintf(filename, "house.json");

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
ReadCategories()
{
  // Start statistics
  RNTime start_time;
  start_time.Read();

  // Read file
  char filename[100];
  sprintf(filename, "%s/metadata/ModelCategoryMapping.csv", data_directory);
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
GetLayoutPCD(const char *scene, int floor, int room,
             pcl::PointCloud<pcl::PointXYZL>::Ptr output)
{
  const char *layouts = "wcf";  // Wall, Ceiling, Floor
  char pcd_path[100];
  pcl::Label label;
  pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);
  pcl::PointCloud<pcl::PointXYZL>::Ptr labeled(new pcl::PointCloud<pcl::PointXYZL>);
  for (int i = 0; i < 3; ++i) {
    sprintf(pcd_path, "%s/room/%s/fr_%drm_%d%c.pcd", data_directory, scene, floor, room, layouts[i]);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *points) == -1) {
      continue;
    }
    label.label = i + 1;
    *labels = pcl::PointCloud<pcl::Label>(points->width, points->height, label);
    pcl::concatenateFields(*points, *labels, *labeled);
    *output += *labeled;
  }
  return 1;
}


static int
GetObjectPCD(const char *model_id, const R4Matrix& M,
             pcl::PointCloud<pcl::PointXYZ>::Ptr points)
{
  char pcd_path[100];
  pcl::Label label;
  pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);
  // pcl::PointCloud<pcl::PointXYZL>::Ptr labeled(new pcl::PointCloud<pcl::PointXYZL>);

  sprintf(pcd_path, "%s/object/%s/%s.pcd", data_directory, model_id, model_id);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *model) == -1) {
    return 0;
  }

  Eigen::Matrix4f transform;
  transform << M[0][0], M[0][1], M[0][2], M[0][3],
               M[1][0], M[1][1], M[1][2], M[1][3],
               M[2][0], M[2][1], M[2][2], M[2][3],
               M[3][0], M[3][1], M[3][2], M[3][3];
  pcl::transformPointCloud(*model, *points, transform);

  // label.label = index;
  // *labels = pcl::PointCloud<pcl::Label>(points->width, points->height, label);
  // *labels_agg += *labels;
  // pcl::concatenateFields(*points, *labels, *labeled);
  // *output += *labeled;
  return 1;
}


static int
WritePCD()
{
  char points_path[100];
  sprintf(points_path, "house.pcd");
  std::ofstream labels_file;
  labels_file.open("instances.txt");

  const char *scene_id = scene->Node(0)->Name();
  pcl::PointCloud<pcl::PointXYZ>::Ptr points_agg(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Label>::Ptr labels_agg(new pcl::PointCloud<pcl::Label>);

  RNTime start_time;
  int index = 0;
  start_time.Read();
  for (int j=0; j<scene->NNodes(); ++j) {
    R3SceneNode *node = scene->Node(j);
    if (!strncmp(node->Name(), "Room", 4)) {
      int floor, room;
      sscanf(strchr(node->Name(), '#') + 1, "%d_%d", &floor, &room);
      // if (!GetLayoutPCD(scene_id, floor, room, output)) return 0;
    }
    if (!strncmp(node->Name(), "Object", 6)) {
      if (strncmp(node->Parent()->Name(), "Room", 4)) continue;
      R3Scene *ref = node->Reference(0)->ReferencedScene();
      // int index = atoi(ref->Node(0)->Info("index"));
      const char *model_id = ref->Name();
      pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
      if (!GetObjectPCD(model_id, node->CumulativeTransformation().Matrix(), points)) return 0;
      *points_agg += *points;
      for (int k=0; k<points->width; ++k) {
        labels_file << index << std::endl;
      }
      index++;
    }
  }

  pcl::io::savePCDFile(points_path, *points_agg, /* binary_mode */ true);
  labels_file.close();
  if (print_verbose) {
    printf("Saved %lu points to %s\n", points_agg->size(), points_path);
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    fflush(stdout);
  }

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
      else if (!strcmp(*argv, "-c")) read_categories = 1;
      else {
        fprintf(stderr, "Invalid program argument: %s", *argv);
        exit(1);
      }
      argv++; argc--;
    }
    else {
      if (!data_directory) data_directory = *argv;
      else { fprintf(stderr, "Invalid program argument: %s", *argv); exit(1); }
      argv++; argc--;
    }
  }

  // Check filenames
  if (!data_directory) {
    fprintf(stderr, "Usage: scn2pcd datadirectory\n");
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
  if (!ReadScene()) exit(-1);

  // Read categories
  if (read_categories) {
    if (!ReadCategories()) exit(-1);
  }

  // Write point clouds
  if (!WritePCD()) exit(-1);

  // Return success
  return 0;
}
