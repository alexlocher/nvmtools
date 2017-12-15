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

#include "UndistortModel.h"

#include <glog/logging.h>
#include <stlplus3/file_system.hpp>

#include "NVMCameraModel.h"
#include "NVMFileVersion.h"

namespace nvmtools {

bool UndistortImage(const NVM_Camera &camDist,
                    const cimg_library::CImg<unsigned char> &imgDist,
                    const NVM_Camera &camRect,
                    cimg_library::CImg<unsigned char> *imgRect) {

  if (!camDist.intrinsics) {
    LOG(ERROR) << "intrinsics of distorted camera not set..";
    return false;
  }

  if (camDist.intrinsics->type() == IntrinsicsType::NVM_DEFAULT &&
      camDist.intrinsics->get(NVMCameraModel::RADIAL_DISTORTION) == 0.0) {
    // no need to undistort
    imgRect->assign(imgDist);
    return true;
  }

  CHECK_NOTNULL(imgRect)->resize(imgDist.width(), imgDist.height(),
                                 imgDist.depth(), imgDist.spectrum());

  // loop through the pixels
  for (unsigned y = 0; y < imgRect->height(); y++) {
    for (unsigned x = 0; x < imgRect->width(); x++) {
      Eigen::Vector3d camRay;
      camRay[0] = ((x + 0.0) - imgRect->width() / 2.0) / camRect.f;
      camRay[1] = ((y + 0.0) - imgRect->height() / 2.0) / camRect.f;
      camRay[2] = 1.0;

      Eigen::Vector2d distorted_pixel =
          camDist.intrinsics->cameraToPixel(camRay);

      //
      // LOG(INFO) << "pixel [" << x << ", " << y << "] corresponds to ["
      //           << distorted_pixel.x() << ", " << distorted_pixel.y() << "]";

      bool inBounds = imgDist.containsXYZC((int)distorted_pixel.x(),
                                           (int)distorted_pixel.y());

      for (unsigned c = 0; c < imgDist.spectrum(); c++) {
        imgRect->atXY(x, y, c) =
            inBounds
                ? imgDist.linear_atXY(distorted_pixel.x(), distorted_pixel.y(),
                                      c)
                : 0;
      }
    }
  }

  return true;
}

// -------------------------------------------------------------------

bool setNVMCameraIntrinsics(NVM_Camera &camera) {
  if (!camera.intrinsics ||
      camera.intrinsics->type() != IntrinsicsType::NVM_DEFAULT) {
    return false;
  }

  // load the image size
  cimg_library::CImg<unsigned char> img;
  img.load(camera.filename.c_str());
  if (img.width() * img.height() == 0) {
    LOG(WARNING) << "could not load image >" << camera.filename << "<";
    return false;
  }

  // set the principal point
  camera.intrinsics->set(NVMCameraModel::PRINCIPAL_POINT_X, img.width() / 2.0);
  camera.intrinsics->set(NVMCameraModel::PRINCIPAL_POINT_Y, img.height() / 2.0);

  // set focal and radial
  camera.intrinsics->set(NVMCameraModel::FOCAL_LENGTH, camera.f);
  camera.intrinsics->set(NVMCameraModel::RADIAL_DISTORTION, camera.r);

  return true;
}

// -------------------------------------------------------------------

NVM_Camera CreateUndistortedCamera(const NVM_Camera &distortedCam) {
  NVM_Camera rectCam = distortedCam;
  rectCam.f = distortedCam.intrinsics->focal();
  rectCam.r = 0.0;

  if (rectCam.intrinsics)
    rectCam.intrinsics.reset();

  rectCam.intrinsics = CameraModel::Create(IntrinsicsType::NVM_DEFAULT);
  CHECK(setNVMCameraIntrinsics(rectCam)) << "failed to set nvm intrinsics";

  return rectCam;
}

// -------------------------------------------------------------------

std::string UndistortedImagePath(const std::string &inImage,
                                 const std::string &outImgFolder) {
  std::string base = stlplus::basename_part(inImage);
  std::string extension = stlplus::extension_part(inImage);
  return stlplus::create_filespec(outImgFolder, base + "-rect", extension);
}

// -------------------------------------------------------------------

Eigen::Vector2d UndistortNVMMeasurement(const NVM_Camera &distCam,
                                        const Eigen::Vector2d &measurement,
                                        const NVM_Camera &rectCam,
                                        const int distVersion = 3,
                                        const int rectVersion = 3) {
  CHECK(distCam.intrinsics) << " intrinsics not set";
  CHECK(rectCam.intrinsics) << " intrinsics not set";

  Eigen::Vector2d distPixel = measurement;
  if (distVersion == 3)
    distPixel += distCam.intrinsics->principalPoint();

  auto unDistRay = distCam.intrinsics->pixelToCamera(distPixel);
  auto unDistPixel = rectCam.intrinsics->cameraToPixel(unDistRay);

  if (rectVersion == 3)
    unDistPixel -= rectCam.intrinsics->principalPoint();

  return unDistPixel;
}

// -------------------------------------------------------------------

bool UndistortModel(const NVM_Model &inModel, const std::string &outFolder,
                    bool saveImages) {
  const size_t nCams = inModel.cameras.size();
  NVM_Model outModel;
  outModel.version = inModel.version;
  cimg_library::CImg<unsigned char> distImg;
  cimg_library::CImg<unsigned char> rectImg;

  std::string outImgFolder = stlplus::create_filespec(outFolder, "images");
  if (!stlplus::folder_exists(outImgFolder) && saveImages) {
    stlplus::folder_create(outImgFolder);
  }

  // first undistort the cameras & images
  std::vector<Eigen::Vector2i> imgSizesBefore(nCams);
  std::vector<Eigen::Vector2i> imgSizesAfter(nCams);
  outModel.cameras.resize(nCams);
  for (size_t camIdx = 0; camIdx < nCams; camIdx++) {
    const NVM_Camera &inCam = inModel.cameras[camIdx];

    // load the distorted image
    distImg.load(inCam.filename.c_str());
    if (distImg.width() * distImg.height() == 0) {
      LOG(WARNING) << "could not load imgae >" << inCam.filename << "< of cam "
                   << camIdx;
      return false;
    }

    imgSizesBefore[camIdx] << distImg.width(), distImg.height();

    // create the output image
    imgSizesAfter[camIdx] = imgSizesBefore[camIdx];
    rectImg.resize(imgSizesAfter[camIdx].x(), imgSizesAfter[camIdx].y());
    outModel.cameras[camIdx] = CreateUndistortedCamera(inCam);

    // undistort the image
    if (saveImages) {
      if (!UndistortImage(inModel.cameras[camIdx], distImg,
                          outModel.cameras[camIdx], &rectImg)) {
        return false;
      }

      outModel.cameras[camIdx].filename =
          UndistortedImagePath(inCam.filename, outImgFolder);
      rectImg.save(outModel.cameras[camIdx].filename.c_str());
      LOG(INFO) << "undistorted img of cam " << camIdx << " and saved it to >"
                << outModel.cameras[camIdx].filename;
      distImg.save(stlplus::create_filespec(
                       outImgFolder, stlplus::filename_part(inCam.filename))
                       .c_str());
    }
  }

  // now we can tackle the points and measurements
  const size_t nPoints = inModel.points.size();
  outModel.points.resize(nPoints);
  for (size_t ptIdx = 0; ptIdx < nPoints; ptIdx++) {
    const NVM_Point &inPoint = inModel.points[ptIdx];
    NVM_Point &outPoint = outModel.points[ptIdx];
    outPoint.xyz = inPoint.xyz;
    outPoint.rgb = inPoint.rgb;
    outPoint.measurements = inPoint.measurements;
    for (auto &m : outPoint.measurements) {
      m.xy = UndistortNVMMeasurement(inModel.cameras[m.imgIndex], m.xy,
                                     outModel.cameras[m.imgIndex],
                                     inModel.version, outModel.version);
    }
  }

  // save as nvm 3 file
  CHECK(ConvertToVersion(outModel, 3)) << "failed to convert to V3";

  std::vector<NVM_Model> outModels;
  outModels.push_back(outModel);
  NVMFile::setRelativePaths(outFolder, outModels);
  NVMFile::saveNVM(stlplus::create_filespec(outFolder, "model.nvm").c_str(),
                   outModels, 3);

  return true;
}

// -------------------------------------------------------------------

} // namespace nvmtools
