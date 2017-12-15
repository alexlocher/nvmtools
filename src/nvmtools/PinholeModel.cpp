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

#include "PinholeModel.h"

namespace nvmtools {

PinholeModel::PinholeModel() {
  parameters_.resize(7);
  parameters_[FOCAL_LENGTH] = 1.0;
  parameters_[ASPECT_RATIO] = 1.0;
  parameters_[SKEW] = 0.0;
  parameters_[PRINCIPAL_POINT_X] = 0.0;
  parameters_[PRINCIPAL_POINT_Y] = 0.0;
  parameters_[RADIAL_DISTORTION_1] = 0.0;
  parameters_[RADIAL_DISTORTION_2] = 0.0;
}

Eigen::Vector3d
PinholeModel::pixelToCamera(const Eigen::Vector2d &pixel) const {
  const double f = parameters_[FOCAL_LENGTH];
  const double a = parameters_[ASPECT_RATIO];
  const double s = parameters_[SKEW];

  Eigen::Vector2d normalized_pixel;
  normalized_pixel[1] = (pixel.y() - parameters_[PRINCIPAL_POINT_Y]) / (f * a);
  normalized_pixel[0] =
      (pixel.x() - parameters_[PRINCIPAL_POINT_X] - s * normalized_pixel[1]) /
      f;

  Eigen::Vector3d ray;
  if (parameters_[RADIAL_DISTORTION_1] != 0.0 ||
      parameters_[RADIAL_DISTORTION_2] != 0.0)
    ray << undistortPoint(normalized_pixel), 1;
  else
    ray << normalized_pixel, 1;
  return ray;
}

Eigen::Vector2d
PinholeModel::cameraToPixel(const Eigen::Vector3d &cameraRay) const {
  const double f = parameters_[FOCAL_LENGTH];
  const double a = parameters_[ASPECT_RATIO];
  const double s = parameters_[SKEW];
  Eigen::Vector2d normalized_pixel(cameraRay.x() / cameraRay.z(),
                                   cameraRay.y() / cameraRay.z());

  if (parameters_[RADIAL_DISTORTION_1] != 0.0 ||
      parameters_[RADIAL_DISTORTION_2] != 0.0)
    normalized_pixel = distortPoint(normalized_pixel);

  Eigen::Vector2d pixel;
  pixel[0] = (normalized_pixel.x() * f) + (normalized_pixel.y() * s) +
             parameters_[PRINCIPAL_POINT_X];
  pixel[1] = (normalized_pixel.y() * f * a) + parameters_[PRINCIPAL_POINT_Y];
  return pixel;
}

Eigen::Vector2d
PinholeModel::distortPoint(const Eigen::Vector2d &undistortedPoint) const {
  const double r2 = undistortedPoint.squaredNorm();
  const double d = 1.0 +
                   r2 * (parameters_[RADIAL_DISTORTION_1] +
                         parameters_[RADIAL_DISTORTION_2] * r2);

  return d * undistortedPoint;
}

Eigen::Vector2d
PinholeModel::undistortPoint(const Eigen::Vector2d &distortedPoint) const {
  const unsigned kNumIter = 100;
  const double kEpsilon = 1e-10;

  Eigen::Vector2d undistortedPoint;
  Eigen::Vector2d prevUndistortedPoint;

  undistortedPoint = distortedPoint;
  for (unsigned ii = 0; ii < kNumIter; ii++) {
    prevUndistortedPoint = undistortedPoint;

    const double r2 = undistortedPoint.squaredNorm();
    const double d = 1.0 +
                     r2 * (parameters_[RADIAL_DISTORTION_1] +
                           parameters_[RADIAL_DISTORTION_2] * r2);

    undistortedPoint = distortedPoint / d;

    if (std::abs(undistortedPoint.x() - prevUndistortedPoint.x()) < kEpsilon &&
        std::abs(undistortedPoint.y() - prevUndistortedPoint.y()) < kEpsilon) {
      break;
    }
  }

  return undistortedPoint;
}

} // namespace nvmtools
