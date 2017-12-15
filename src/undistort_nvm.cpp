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

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <stlplus3/file_system.hpp>

#include <nvmtools/NVMFile.h>
#include <nvmtools/UndistortModel.h>

// ---------------------------------------------------------
// command line flags
DEFINE_string(nvm, "", "input nvm file");
DEFINE_string(
    outdir, "/tmp",
    "output directory where the undisorted project should be saved to");
DEFINE_bool(skip_images, false, "do not undistort images");

// ---------------------------------------------------------
int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);

  std::vector<nvmtools::NVM_Model> models;
  nvmtools::NVMFile::readFile(FLAGS_nvm.c_str(), models);

  LOG(INFO) << "loaded " << models.size() << " models from >" << FLAGS_nvm
            << "<";
  if (models.empty()) {
    LOG(WARNING) << "no models -> nothing to do.";
    return EXIT_SUCCESS;
  }

  if (!stlplus::folder_exists(FLAGS_outdir)) {
    stlplus::folder_create(FLAGS_outdir);
  }
  for (size_t idx = 0; idx < models.size(); idx++) {
    LOG(INFO) << "model " << idx << " has " << models[idx].cameras.size()
              << " cameras and " << models[idx].points.size() << " points";

    for (auto &camera : models[idx].cameras) {
      nvmtools::setNVMCameraIntrinsics(camera);
    }

    CHECK(
        nvmtools::UndistortModel(models[idx], FLAGS_outdir, !FLAGS_skip_images))
        << "failed to undistort model " << idx;

    // int camIdx = 0;
    // for (const auto &camera : models[idx].cameras) {
    //   if (camera.intrinsics) {
    //     LOG(INFO) << "   camera " << camIdx << " has "
    //               <<
    //               nvmtools::IntrinsicsTypeToString(camera.intrinsics->type())
    //               << " model";
    //   } else {
    //     LOG(INFO) << "   camera " << camIdx << " has NO "
    //               << " model";
    //
    //   }
    //
    //   camIdx++;
    // }
  }

  return EXIT_SUCCESS;
}
