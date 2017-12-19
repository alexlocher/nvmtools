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

#include <memory>
#include <string>

#include <Eigen/Dense>

namespace nvmtools {

enum class IntrinsicsType {
  UNKNOWN = -1,
  NVM_DEFAULT = 0,
  PINHOLE = 1,
};

inline IntrinsicsType StringToIntrinsicsType(const std::string &s) {
  if (s == "PINHOLE") {
    return IntrinsicsType::PINHOLE;
  } else if (s == "NVM_DEFAULT") {
    return IntrinsicsType::NVM_DEFAULT;
  }
  return IntrinsicsType::UNKNOWN;
}

inline std::string IntrinsicsTypeToString(const IntrinsicsType &t) {
  switch (t) {
  case IntrinsicsType::PINHOLE:
    return "PINHOLE";
  case IntrinsicsType::NVM_DEFAULT:
    return "NVM_DEFAULT";
  default:
    return "";
  }
}

/**
 * Abstract camera interface for all supported intrinsics
 */
class CameraModel {
public:
  static std::shared_ptr<CameraModel> Create(const IntrinsicsType &type);

  CameraModel() = default;
  virtual ~CameraModel() = default;

  virtual IntrinsicsType type() const = 0;
  virtual bool hasDistortion() const = 0;
  virtual double focal() const = 0;
  virtual Eigen::Vector2d principalPoint() const = 0;

  virtual Eigen::Vector3d pixelToCamera(const Eigen::Vector2d &pixel) const = 0;

  virtual Eigen::Vector2d
  cameraToPixel(const Eigen::Vector3d &cameraRay) const = 0;

  virtual Eigen::Vector2d
  distortPoint(const Eigen::Vector2d &undistortedPoint) const = 0;

  virtual Eigen::Vector2d
  undistortPoint(const Eigen::Vector2d &distortedPoint) const = 0;

  bool readAscii(std::istream &istr);
  bool writeAscii(std::ostream &ostr) const;

  template <typename idx_t> void set(const idx_t &idx, const double value) {
    const int intIdx = static_cast<int>(idx);
    if (intIdx >= 0 && intIdx < parameters_.size()) {
      parameters_[intIdx] = value;
    }
  }

  template <typename idx_t> double get(const idx_t &idx) const {
    const int intIdx = static_cast<int>(idx);
    if (intIdx >= 0 && intIdx < parameters_.size()) {
      return parameters_[intIdx];
    }
    return 0.0;
  }

protected:
  std::vector<double> parameters_;
};

} // namespace nvmtools
