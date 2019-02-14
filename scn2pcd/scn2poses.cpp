#include <fstream>
#include <iostream>
#include "R3Graphics/R3Graphics.h"


static char *input_scene_file = NULL;
static char *output_poses_dir = NULL;
static int print_verbose = 0;
static int read_categories = 0;
static R3Scene *scene = NULL;


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

  // Remove references
  scene->RemoveReferences();

  // Return success
  return 1;
}


static int
WriteObjects(const char *directory)
{
  RNTime start_time;
  start_time.Read();

  char cmd[1024];
  sprintf(cmd, "mkdir -p %s", directory);
  system(cmd);

  int n = 0;
  for (int j=0; j<scene->NNodes(); ++j) {
    R3SceneNode *node = scene->Node(j);
    if (!strncmp(node->Name(), "Model", 5)) {
      char filename[1024];
      sprintf(filename, "%s/%d_%s.txt", directory, node->SceneIndex() + 1, node->Name() + 6);
      R4Matrix M = node->CumulativeTransformation().Matrix();
      std::ofstream output_file;
      output_file.open(filename);
      output_file << M[0][0] << " " << M[0][1] << " " << M[0][2] << " " << M[0][3] << std::endl
                  << M[1][0] << " " << M[1][1] << " " << M[1][2] << " " << M[1][3] << std::endl
                  << M[2][0] << " " << M[2][1] << " " << M[2][2] << " " << M[2][3] << std::endl
                  << M[3][0] << " " << M[3][1] << " " << M[3][2] << " " << M[3][3] << std::endl;
      output_file.close();
    }
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
      else {
        fprintf(stderr, "Invalid program argument: %s", *argv);
        exit(1);
      }
      argv++; argc--;
    }
    else {
      if (!input_scene_file) input_scene_file = *argv;
      else if (!output_poses_dir) output_poses_dir = *argv;
      else { fprintf(stderr, "Invalid program argument: %s", *argv); exit(1); }
      argv++; argc--;
    }
  }

  // Check filenames
  if (!input_scene_file || !output_poses_dir) {
    fprintf(stderr, "Usage: scn2poses inputscenefile outputposesdirectory\n");
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

  // Write point clouds
  if (!WriteObjects(output_poses_dir)) exit(-1);

  // Return success
  return 0;
}
