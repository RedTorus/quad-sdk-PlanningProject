#include "quad_utils/yaml_loader.h"
#include <yaml-cpp/yaml.h>

std::vector<LinkSize> loadLinkSizes(const std::string& yaml_path) {
    std::vector<LinkSize> link_sizes;
    YAML::Node config = YAML::LoadFile(yaml_path);

    for (const auto& node : config["link_sizes"]) {
        LinkSize size;
        size.link_name = node["link_name"].as<std::string>();
        size.length = node["length"].as<double>();
        size.width = node["width"].as<double>();
        size.height = node["height"].as<double>();
        link_sizes.push_back(size);
    }
    return link_sizes;
}
