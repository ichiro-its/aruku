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

#include "aruku/config/utils/config.hpp"
#include "nlohmann/json.hpp"

namespace aruku
{

Config::Config(const std::string & path)
: path(path)
{
}

std::string Config::get_config(const std::string & key) const
{
  if (key == "walking") {
    std::ifstream walking_file(path + "walking.json");
    nlohmann::json walking_data = nlohmann::json::parse(walking_file);
    walking_file.close();

    walking_data.erase("angles_direction");
    walking_data.erase("pid");

    return walking_data.dump();
  } else if (key == "kinematic") {
    std::ifstream kinematic_file(path + "kinematic.json");
    nlohmann::json kinematic_data = nlohmann::json::parse(kinematic_file);

    kinematic_data.erase("length");

    return kinematic_data.dump();
  }

  return "";
}

void Config::set_config(const nlohmann::json & kinematic_data, const nlohmann::json & walking_data)
{
  std::ofstream kinematic_file(path + "kinematic.json", std::ios::out | std::ios::trunc);
  kinematic_file << std::setw(2) << kinematic_data << std::endl;
  kinematic_file.close();

  std::ofstream walking_file(path + "walking.json", std::ios::out | std::ios::trunc);
  walking_file << std::setw(2) << walking_data << std::endl;
  walking_file.close();
}

}  // namespace aruku
