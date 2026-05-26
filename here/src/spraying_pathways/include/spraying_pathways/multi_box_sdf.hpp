#pragma once
#include "spraying_pathways/types.hpp"
#include <sstream>
#include <string>
#include <vector>

namespace sp {
inline std::string generate_multi_box_sdf(const std::vector<Cube>& cubes,
                                          double cube_size_x, double cube_size_y,
                                          const std::string& uri="package://spraying_pathways/materials/scripts",
                                          const std::string& mat="My/Seaweed",
                                          bool is_static=true) {
  std::ostringstream sdf;
  sdf << "<?xml version='1.0'?>\n<sdf version='1.7'>\n<model name='multi_cube'>\n";
  sdf << "  <static>" << (is_static ? "true" : "false") << "</static>\n";
  for (const auto& c : cubes) {
    sdf << "  <link name='cube_" << c.id << "'>\n";
    sdf << "    <pose>" << c.pose.position.x << " "
                       << c.pose.position.y << " "
                       << c.pose.z << " 0 0 0</pose>\n";
    sdf << "    <collision name='collision'>\n"
           "      <geometry>\n"
           "        <box><size>" << cube_size_x << " " << cube_size_y << " " << c.height << "</size></box>\n"
           "      </geometry>\n"
           "    </collision>\n";
    sdf << "    <visual name='visual'>\n"
           "      <geometry>\n"
           "        <box><size>" << cube_size_x << " " << cube_size_y << " " << c.height << "</size></box>\n"
           "      </geometry>\n"
           "      <material>\n"
           "        <script>\n"
           "          <uri>" << uri << "</uri>\n"
           "          <name>" << mat << "</name>\n"
           "        </script>\n"
           "        <ambient>1 1 1 0.7</ambient>\n"
           "        <diffuse>1 1 1 0.7</diffuse>\n"
           "      </material>\n"
           "    </visual>\n"
           "  </link>\n";
  }
  sdf << "</model>\n</sdf>\n";
  return sdf.str();
}
} // namespace sp
