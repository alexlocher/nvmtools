/**
 * This file is part of NVMTOOLS.
 *
 * Copyright (C) 2017 Alex Locher <alocher at ethz dot ch> (ETH Zuerich)
 * For more information see <https://github.com/alexlocher/nvmtools>
 *
 * NVMTOOLS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NVMTOOLS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NVMTOOLS. If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "CameraModel.h"

namespace nvmtools {

struct NVM_Measurement {
  int imgIndex;
  int featIndex;
  Eigen::Vector2d xy;
};

struct NVM_Point {
  Eigen::Vector3d xyz;
  Eigen::Vector3d rgb;
  std::vector<NVM_Measurement> measurements;
};

struct NVM_Camera {
  std::string filename;
  double f;           // focal length
  Eigen::Vector4d rq; // rotation quaternion <wxyz>
  Eigen::Vector3d c;  // camera center
  double r;           // radial distortion

  // optional intrinsics which are only supported
  // in the NVM version 4
  std::shared_ptr<CameraModel> intrinsics;
};

struct NVM_Model {
  int version;
  std::vector<NVM_Camera> cameras;
  std::vector<NVM_Point> points;
};

class NVMFile {
public:
  static bool readFile(const char *path, std::vector<NVM_Model> &models,
                       bool fixPath = false);
  static bool saveNVM(const char *path, const std::vector<NVM_Model> &models,
                      const int version = 3);

  static void saveAsPly(const NVM_Model &model, const char *file);

  static void setRelativePaths(const std::string &outfolder,
                               std::vector<NVM_Model> &models);
};

std::istream &operator>>(std::istream &istr, NVM_Measurement &rhs);
std::ostream &operator<<(std::ostream &ostr, NVM_Measurement &rhs);

std::istream &operator>>(std::istream &istr, NVM_Point &rhs);
std::ostream &operator<<(std::ostream &ostr, NVM_Point &rhs);

std::istream &operator>>(std::istream &istr, NVM_Camera &rhs);
std::ostream &operator<<(std::ostream &ostr, NVM_Camera &rhs);

std::istream &operator>>(std::istream &istr, NVM_Model &rhs);
std::ostream &operator<<(std::ostream &ostr, NVM_Model &rhs);

} /* namespace nvmtools */
