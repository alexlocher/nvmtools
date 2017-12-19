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

class PinholeModel : public CameraModel {

public:
  PinholeModel();
  ~PinholeModel() {}

  enum IntrinsicIndex {
    FOCAL_LENGTH = 0,
    ASPECT_RATIO = 1,
    SKEW = 2,
    PRINCIPAL_POINT_X = 3,
    PRINCIPAL_POINT_Y = 4,
    RADIAL_DISTORTION_1 = 5,
    RADIAL_DISTORTION_2 = 6
  };

  IntrinsicsType type() const override { return IntrinsicsType::PINHOLE; }

  bool hasDistortion() const override {
    return parameters_[RADIAL_DISTORTION_1] *
               parameters_[RADIAL_DISTORTION_2] !=
           0.0;
  }

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
