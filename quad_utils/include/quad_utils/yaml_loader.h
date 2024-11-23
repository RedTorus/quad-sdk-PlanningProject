#ifndef YAML_LOADER_H
#define YAML_LOADER_H

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "quad_utils/bounding_boxes.h"

// Helper function to load link sizes from a YAML file
std::vector<LinkSize> loadLinkSizes(const std::string& yaml_path);

#endif // YAML_LOADER_H
