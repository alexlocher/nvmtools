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

#include "NVMFile.h"

#define cimg_display 0
#define cimg_use_jpeg
#define WIN32_LEAN_AND_MEAN
#include <CImg.h>

namespace nvmtools {

bool UndistortImage(const NVM_Camera &camDist,
                    const cimg_library::CImg<unsigned char> &imgDist,
                    const NVM_Camera &camRect,
                    cimg_library::CImg<unsigned char> *imgRect);

bool setNVMCameraIntrinsics(NVM_Camera &camera);

NVM_Camera CreateUndistortedCamera(const NVM_Camera &distortedCam);

std::string UndistortedImagePath(const std::string &inImage,
                                 const std::string &outImgFolder);

bool UndistortModel(const NVM_Model &inModel, const std::string &outImgFolder,
                    bool saveImages = true);

} // namespace nvmtools
