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
#include "NVMFileVersion.h"

#include <Eigen/Dense>
#include <glog/logging.h>

#define cimg_display 0
#define cimg_use_jpeg
#define WIN32_LEAN_AND_MEAN
#include <CImg.h>

#include "NVMCameraModel.h"

namespace nvmtools {

// ----------------------------------------------------------------------------

bool ReadImageSizes(const NVM_Model &model,
                    std::vector<Eigen::Vector2d> &sizes) {
  cimg_library::CImg<unsigned char> img;
  const int nCams = model.cameras.size();
  sizes.resize(nCams);
  for (unsigned ii = 0; ii < nCams; ii++) {
    img.load(model.cameras[ii].filename.c_str());
    if (img.width() <= 0 || img.height() <= 0) {
      LOG(WARNING) << "cannot load img >" << model.cameras[ii].filename << "<";
      return false;
    }

    sizes[ii][0] = img.width();
    sizes[ii][1] = img.height();
  }

  return true;
}

// ----------------------------------------------------------------------------

bool ConvertV3toV4(NVM_Model &model) {
  if (model.version != 3) {
    return false;
  }
  std::vector<Eigen::Vector2d> sizes;
  if (!ReadImageSizes(model, sizes))
    return false;

  // since this is a model 3, all cameras should be nvm default and the
  // principal points in the middle => we anyway have the image sizes loaded so
  // lets set them regardless if already set or not
  for (unsigned ii = 0; ii < model.cameras.size(); ii++) {
    auto &c = model.cameras[ii];
    if (!c.intrinsics)
      c.intrinsics = CameraModel::Create(IntrinsicsType::NVM_DEFAULT);
    if (c.intrinsics && c.intrinsics->type() != IntrinsicsType::NVM_DEFAULT) {
      LOG(WARNING) << " inconsistent model expected NVM_DEFAULT in V3 file";
      return false;
    }

    // set the principal point
    c.intrinsics->set(NVMCameraModel::PRINCIPAL_POINT_X, sizes[ii].x() / 2.0);
    c.intrinsics->set(NVMCameraModel::PRINCIPAL_POINT_Y, sizes[ii].y() / 2.0);

    // set focal and radial
    c.intrinsics->set(NVMCameraModel::FOCAL_LENGTH, c.f);
    c.intrinsics->set(NVMCameraModel::RADIAL_DISTORTION, c.r);
  }

  // now convert the measurements (add the principal point)
  for (auto &p : model.points) {
    for (auto &m : p.measurements) {
      m.xy += model.cameras[m.imgIndex].intrinsics->principalPoint();
    }
  }
  model.version = 4;
  return true;
}

// ----------------------------------------------------------------------------

bool ConvertV4toV3(NVM_Model &model) {
  if (model.version != 4) {
    return false;
  }

  std::vector<Eigen::Vector2d> sizes;
  if (!ReadImageSizes(model, sizes))
    return false;

  // we can only convert the version backwards to V3 if all cameras are nvm
  // cameras and the principal point is in the image center
  for (unsigned ii = 0; ii < model.cameras.size(); ii++) {
    auto &c = model.cameras[ii];
    if (c.intrinsics) {
      if (c.intrinsics->type() != IntrinsicsType::NVM_DEFAULT) {
        LOG(WARNING) << "Camera model not NVM_DEFAULT => cannot convert to V3";
        return false;
      }

      if (c.intrinsics->principalPoint() != sizes[ii] / 2.0) {
        LOG(WARNING) << "Principal point not in center => cannot convert to V3";
        return false;
      }
    }

    if (!c.intrinsics)
      c.intrinsics = CameraModel::Create(IntrinsicsType::NVM_DEFAULT);

    // we are good, make sure the values are set!
    c.intrinsics->set(NVMCameraModel::PRINCIPAL_POINT_X, sizes[ii].x() / 2.0);
    c.intrinsics->set(NVMCameraModel::PRINCIPAL_POINT_Y, sizes[ii].y() / 2.0);
    c.intrinsics->set(NVMCameraModel::FOCAL_LENGTH, c.f);
    c.intrinsics->set(NVMCameraModel::RADIAL_DISTORTION, c.r);
  }

  // now convert the measurements (subtract the principal point)
  for (auto &p : model.points) {
    for (auto &m : p.measurements) {
      m.xy -= model.cameras[m.imgIndex].intrinsics->principalPoint();
    }
  }

  model.version = 3;
  return true;
}

// ----------------------------------------------------------------------------

bool ConvertToVersion(std::vector<NVM_Model> &models, const int targetVersion) {
  for (auto &m : models) {
    if (!ConvertToVersion(m, targetVersion)) {
      return false;
    }
  }
  return true;
}

// ----------------------------------------------------------------------------

bool ConvertToVersion(NVM_Model &model, const int targetVersion) {
  if (model.version == targetVersion) {
    return true;
  }

  if (model.version == 3 && targetVersion == 4) {
    return ConvertV3toV4(model);
  }

  if (model.version == 4 && targetVersion == 3) {
    return ConvertV4toV3(model);
  }

  LOG(WARNING) << "cannot convert V" << model.version << " to "
               << targetVersion;
  return false;
}

// ----------------------------------------------------------------------------

} // namespace nvmtools
