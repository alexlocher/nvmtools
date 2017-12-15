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

#include "NVMCameraModel.h"

#include <iostream>

namespace nvmtools {

NVMCameraModel::NVMCameraModel() {
  parameters_.resize(4);
  parameters_[FOCAL_LENGTH] = 1.0;
  parameters_[PRINCIPAL_POINT_X] = 0.0;
  parameters_[PRINCIPAL_POINT_Y] = 0.0;
  parameters_[RADIAL_DISTORTION] = 0.0;
}

Eigen::Vector3d
NVMCameraModel::pixelToCamera(const Eigen::Vector2d &pixel) const {
  const double f = parameters_[FOCAL_LENGTH];
  Eigen::Vector2d normalized_pixel;
  normalized_pixel[0] = (pixel.x() - parameters_[PRINCIPAL_POINT_X]) / f;
  normalized_pixel[1] = (pixel.y() - parameters_[PRINCIPAL_POINT_Y]) / f;

  Eigen::Vector3d ray;
  if (parameters_[RADIAL_DISTORTION] != 0.0)
    ray << undistortPoint(normalized_pixel), 1;
  else
    ray << normalized_pixel, 1;
  return ray;
}

Eigen::Vector2d
NVMCameraModel::cameraToPixel(const Eigen::Vector3d &cameraRay) const {
  const double f = parameters_[FOCAL_LENGTH];
  Eigen::Vector2d normalized_pixel(cameraRay.x() / cameraRay.z(),
                                   cameraRay.y() / cameraRay.z());

  if (parameters_[RADIAL_DISTORTION] != 0.0)
    normalized_pixel = distortPoint(normalized_pixel);

  Eigen::Vector2d pixel;
  pixel[0] = (normalized_pixel.x() * f) + parameters_[PRINCIPAL_POINT_X];
  pixel[1] = (normalized_pixel.y() * f) + parameters_[PRINCIPAL_POINT_Y];
  return pixel;
}

Eigen::Vector2d
NVMCameraModel::distortPoint(const Eigen::Vector2d &undistortedPoint) const {
  const unsigned kNumIter = 100;
  const double kEpsilon = 1e-10;

  const double rn = parameters_[RADIAL_DISTORTION] * parameters_[FOCAL_LENGTH] *
                    parameters_[FOCAL_LENGTH];

  Eigen::Vector2d distortedPoint;
  Eigen::Vector2d prevDistortedPoint;

  distortedPoint = undistortedPoint;
  for (unsigned ii = 0; ii < kNumIter; ii++) {
    prevDistortedPoint = distortedPoint;

    const double r2 = rn * distortedPoint.squaredNorm();
    distortedPoint = undistortedPoint / (1.0 + r2);

    if (std::abs(distortedPoint.x() - prevDistortedPoint.x()) < kEpsilon &&
        std::abs(distortedPoint.y() - prevDistortedPoint.y()) < kEpsilon) {
      break;
    }
  }

  return distortedPoint;
}

Eigen::Vector2d
NVMCameraModel::undistortPoint(const Eigen::Vector2d &distortedPoint) const {
  // according to http://ccwu.me/vsfm/doc.html#nvm => the distortion is inverse
  // to
  // what we usually have: "The undistorted measurement is (1 + r2) * (mx, my)"
  const double f = parameters_[FOCAL_LENGTH];
  const double r2 =
      parameters_[RADIAL_DISTORTION] * f * f * distortedPoint.squaredNorm();
  return (1.0 + r2) * distortedPoint;
}

} // namespace nvmtools
