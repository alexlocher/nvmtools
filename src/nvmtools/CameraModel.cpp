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

#include "CameraModel.h"

#include <glog/logging.h>

#include "NVMCameraModel.h"
#include "PinholeModel.h"

namespace nvmtools {

std::shared_ptr<CameraModel> CameraModel::Create(const IntrinsicsType &type) {
  switch (type) {
  case IntrinsicsType::PINHOLE:
    return std::make_shared<PinholeModel>();
  case IntrinsicsType::NVM_DEFAULT:
    return std::make_shared<NVMCameraModel>();
  default:
    LOG(FATAL) << "cannot create unknown model";
  }
  return nullptr;
}

bool CameraModel::readAscii(std::istream &istr) {
  for (size_t ii = 0; ii < parameters_.size(); ii++) {
    istr >> parameters_[ii];
  }
  return istr.good();
}

bool CameraModel::writeAscii(std::ostream &ostr) const {
  for (size_t ii = 0; ii < parameters_.size(); ii++) {
    ostr << " " << parameters_[ii];
  }
  return ostr.good();
}

} // namespace nvmtools
