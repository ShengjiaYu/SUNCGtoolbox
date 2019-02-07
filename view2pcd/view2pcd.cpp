#include <math.h>
#include <iostream>
#include <string>
#include <set>
#include <pcl/filters/frustum_culling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "R3Graphics/R3Graphics.h"


static char *input_scene_file = NULL;
static char *input_cameras_file = NULL;
static char *input_cameranames_file = NULL;
static char *input_categories_name = NULL;
static char *input_img_directory = NULL;
static char *object_directory = "/usr0/home/Datasets/SUNCG/object";
static char *output_pcd_directory = NULL;
static int print_verbose = 0;
static int width = 640;
static int height = 480;
static float min_visible_ratio = 0;

static R3Scene *scene = NULL;
static RNArray<R3Camera *> cameras;
static RNArray<char *> camera_names;


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
    R3Camera *camera = new R3Camera(viewpoint, towards, up, xf, yf,
        neardist, fardist);
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
ReadCameraNames(const char *filename)
{
  // Open file
  FILE *fp = fopen(filename, "r");
  if (!fp) {
    fprintf(stderr, "Unable to open camera names file %s\n", filename);
    return 0;
  }

  char name[1024];
  while (fscanf(fp, "%s\n", name) == 1) {
    camera_names.Insert(strdup(name));
  }

  fclose(fp);

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
ReadInstanceIds(const char *filename, std::set<int> *instance_ids)
{
  R2Grid node(width, height);
  if (!node.ReadFile(filename)) {
    fprintf(stderr, "Unable to read node image %s\n", filename);
    return 0;
  }
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      if (node(j, i) > 0) {
        instance_ids->insert(node(j, i));
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


int
WritePCD(const char* output_pcd_directory)
{
  char node_img_path[100];
  std::string scene_name = scene->Node(0)->Name();
  for (int i=0; i<cameras.NEntries(); ++i) {
    pcl::PointCloud<pcl::PointXYZL>::Ptr output(new pcl::PointCloud<pcl::PointXYZL>);
    sprintf(node_img_path, "%s/%06d_node.png", input_img_directory, i);
    std::set<int> instance_ids;
    if (!ReadInstanceIds(node_img_path, &instance_ids)) continue;
    for (std::set<int>::iterator j = instance_ids.begin(); j != instance_ids.end(); ++j) {
      R3SceneNode *node = scene->Node(*j - 1);
      if (!strncmp(node->Name(), "Model", 5)) {    // If node is an object
        pcl::PointCloud<pcl::PointXYZL>::Ptr labeled(new pcl::PointCloud<pcl::PointXYZL>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr visible(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);
        char pcd_name[100];
        const char *model_id = strchr(node->Name(), '#') + 1;
        sprintf(pcd_name, "%s/%s/%s.pcd", object_directory, model_id, model_id);
        GetObjectPCD(pcd_name, object, node->CumulativeTransformation().Matrix());
        pcl::Label label;
        label.label = atoi(model_id);
        *labels = pcl::PointCloud<pcl::Label>(object->width, object->height, label);
        pcl::concatenateFields(*object, *labels, *labeled);
        *output += *labeled;
      }
//      if (!node->Parent()) continue;
      // Objects not in the same room as the camera
//      if (strncmp(node->Parent()->Name(), camera_names[i],
//          strlen(node->Parent()->Name()))) continue;

      // Floors, walls and ceilings do not have a reference model
//      if (node->NReferences() == 0) {
//        std::string file_name = node->Name();
//        file_name.insert(file_name.find('_'), "rm");
//        file_name = "fr_" + file_name.substr(file_name.find('#')+1);
//        if (!strncmp(node->Name(), "Ceiling", 7)) {
//          file_name += "c";
//        } else if (!strncmp(node->Name(), "Floor", 5)) {
//          file_name += "f";
//        } else if (!strncmp(node->Name(), "Wall", 4)) {
//          file_name += "w";
//        }
//        std::string pcd_name = "../../room/" + scene_name + "/" + file_name + ".pcd";
//        GetObjectPCD(pcd_name, object, node->CumulativeTransformation().Matrix());
//        CullPCD(object, visible, cameras[i]);
//        pcl::Label label;
//        label.label = atoi(node->Info("index"));
//        *labels = pcl::PointCloud<pcl::Label>(visible->width, visible->height, label);
//        pcl::concatenateFields(*visible, *labels, *labeled);
//        *output += *labeled;
//      } else {
//        R3Scene *ref = node->Reference(0)->ReferencedScene();
//        std::string obj_name = ref->Filename();
//        std::string pcd_name = obj_name.substr(0, obj_name.rfind('.')) + ".pcd";
//        CullPCD(object, visible, cameras[i]);
//        float visible_ratio = (float) visible->points.size() / object->points.size();
//        if (visible_ratio > min_visible_ratio) {
//          label.label = atoi(ref->Node(0)->Info("index"));
//        }
//      }
    }
    char output_filename[1024];
    sprintf(output_filename, "%s/%06d.pcd", output_pcd_directory, i);
    pcl::io::savePCDFile(output_filename, *output, /* binary_mode */ true);
    if (print_verbose) {
      std::cout << "Saved " << output->points.size() << " points to " << output_filename << std::endl;
    }
  }

  return 1;
}


static int
ParseArgs(int argc, char **argv)
{
  // Parse arguments
  // Parse arguments
  argc--; argv++;
  while (argc > 0) {
    if ((*argv)[0] == '-') {
      if (!strcmp(*argv, "-v")) print_verbose = 1;
      else if (!strcmp(*argv, "-categories")) { argc--; argv++; input_categories_name = *argv; }
      else if (!strcmp(*argv, "-min_visible_ratio")) { argc--; argv++; min_visible_ratio = atof(*argv); }
      else if (!strcmp(*argv, "-object_directory")) {argc--; argv++; object_directory = *argv; }
      else {
        fprintf(stderr, "Invalid program argument: %s", *argv);
        exit(1);
      }
      argv++; argc--;
    }
    else {
      if (!input_scene_file) input_scene_file = *argv;
      else if (!input_cameras_file) input_cameras_file = *argv;
//      else if (!input_cameranames_file) input_cameranames_file = *argv;
      else if (!input_img_directory) input_img_directory = *argv;
      else if (!output_pcd_directory) output_pcd_directory = *argv;
      else { fprintf(stderr, "Invalid program argument: %s", *argv); exit(1); }
      argv++; argc--;
    }
  }

  // Check filenames
  if (!input_scene_file || !input_cameras_file || !input_img_directory || !output_pcd_directory) {
    fprintf(stderr, "Usage: scn2pcd inputscenefile inputcamerasfile inputimgdirectory outputpcddirectory\n");
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
//  if (!ReadCameraNames(input_cameranames_file)) exit(-1);

  // Read categories
  if (input_categories_name) {
    if (!ReadCategories(input_categories_name)) exit(-1);
  }

  if (!WritePCD(output_pcd_directory)) exit(-1);

  // Return success 
  return 0;
}
