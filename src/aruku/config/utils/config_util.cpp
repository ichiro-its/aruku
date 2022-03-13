// Copyright (c) 2021 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <fstream>
#include <iomanip>
#include <string>

#include "aruku/config/utils/config_util.hpp"
#include "nlohmann/json.hpp"

namespace aruku
{

ConfigUtil::ConfigUtil(const std::string & path)
: path(path)
{
  file_name = {
    {"ratio", "kinematic.json"},
    {"offset", "kinematic.json"},
    {"length", "kinematic.json"},
    {"balance", "walking.json"},
    {"pid", "walking.json"},
    {"odometry", "walking.json"},
    {"init_angles", "walking.json"},
    {"angles_direction", "walking.json"}
  };
}

std::string ConfigUtil::get_config() const
{
  std::ifstream walking_file(path + "walking.json");
  nlohmann::json walking_data = nlohmann::json::parse(walking_file);

  walking_file.close();

  std::ifstream kinematic_file(path + "kinematic.json");
  nlohmann::json kinematic_data = nlohmann::json::parse(kinematic_file);

  kinematic_file.close();

  walking_data.merge_patch(kinematic_data);

  return walking_data.dump();
}

void ConfigUtil::set_config(const std::string & name, const std::string & key, double value)
{
  if (file_name.find(name) != file_name.end()) {
    std::ifstream file(path + file_name.at(name));
    nlohmann::json data = nlohmann::json::parse(file);

    file.close();

    data[name][key] = value;

    std::ofstream output(path + file_name.at(name), std::ios::out | std::ios::trunc);
    output << std::setw(2) << data << std::endl;
  }
}

}  // namespace aruku
