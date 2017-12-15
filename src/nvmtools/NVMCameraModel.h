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

#include <string>

#include "CameraModel.h"

namespace nvmtools {

class NVMCameraModel : public CameraModel {

public:
  NVMCameraModel();
  ~NVMCameraModel() {}

  enum IntrinsicIndex {
    FOCAL_LENGTH = 0,
    PRINCIPAL_POINT_X = 1,
    PRINCIPAL_POINT_Y = 2,
    RADIAL_DISTORTION = 3
  };

  IntrinsicsType type() const override { return IntrinsicsType::NVM_DEFAULT; }

  double focal() const override { return parameters_[FOCAL_LENGTH]; }
  Eigen::Vector2d principalPoint() const override {
    return Eigen::Vector2d(parameters_[PRINCIPAL_POINT_X],
                           parameters_[PRINCIPAL_POINT_Y]);
  }

  Eigen::Vector3d pixelToCamera(const Eigen::Vector2d &pixel) const override;

  Eigen::Vector2d
  cameraToPixel(const Eigen::Vector3d &cameraRay) const override;

  Eigen::Vector2d
  distortPoint(const Eigen::Vector2d &undistortedPoint) const override;

  Eigen::Vector2d
  undistortPoint(const Eigen::Vector2d &distortedPoint) const override;
};

} // namespace nvmtools
