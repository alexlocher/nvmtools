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
#include "NVMFile.h"
#include <glog/logging.h>
#include <iomanip>
#include <stlplus3/file_system.hpp>

#include "NVMCameraModel.h"

using namespace std;

namespace nvmtools {

std::istream &operator>>(std::istream &istr, NVM_Measurement &rhs) {
  istr >> rhs.imgIndex >> rhs.featIndex >> rhs.xy[0] >> rhs.xy[1];
  return istr;
}
std::ostream &operator<<(std::ostream &ostr, NVM_Measurement &rhs) {
  ostr << " " << rhs.imgIndex << " " << rhs.featIndex << " " << rhs.xy[0] << " "
       << rhs.xy[1];
  return ostr;
}

std::istream &operator>>(std::istream &istr, NVM_Point &rhs) {
  istr >> rhs.xyz[0] >> rhs.xyz[1] >> rhs.xyz[2] >> rhs.rgb[0] >> rhs.rgb[1] >>
      rhs.rgb[2];
  int nrMeasurements;
  istr >> nrMeasurements;
  rhs.measurements.resize(nrMeasurements);
  for (size_t ii = 0; ii < nrMeasurements; ii++) {
    istr >> rhs.measurements[ii];
  }
  return istr;
}

std::ostream &operator<<(std::ostream &ostr, NVM_Point &rhs) {
  ostr << rhs.xyz[0] << " " << rhs.xyz[1] << " " << rhs.xyz[2] << " "
       << ((int)rhs.rgb[0]) << " " << ((int)rhs.rgb[1]) << " "
       << ((int)rhs.rgb[2]);
  int nrMeasurements = rhs.measurements.size();
  ostr << " " << nrMeasurements;
  for (size_t ii = 0; ii < nrMeasurements; ii++) {
    ostr << rhs.measurements[ii];
  }
  ostr << endl;
  return ostr;
}

std::istream &operator>>(std::istream &istr, NVM_Camera &rhs) {
  istr >> rhs.filename;
  istr >> rhs.f;
  istr >> rhs.rq[0] >> rhs.rq[1] >> rhs.rq[2] >> rhs.rq[3];
  istr >> rhs.c[0] >> rhs.c[1] >> rhs.c[2];
  istr >> rhs.r;
  rhs.r = rhs.r / (rhs.f * rhs.f);
  int camera_type;
  istr >> camera_type;
  rhs.intrinsics =
      CameraModel::Create(static_cast<IntrinsicsType>(camera_type));
  if (rhs.intrinsics) {
    // compatibility to NVM_V3 -> default camera not written
    if (rhs.intrinsics->type() == IntrinsicsType::NVM_DEFAULT) {
      rhs.intrinsics->set(NVMCameraModel::IntrinsicIndex::FOCAL_LENGTH, rhs.f);
      rhs.intrinsics->set(NVMCameraModel::IntrinsicIndex::RADIAL_DISTORTION,
                          rhs.r);
    } else {
      rhs.intrinsics->readAscii(istr);
    }
  }

  // assert(check == 0 && "last camera parameter shoud be 0");
  replace(rhs.filename.begin(), rhs.filename.end(), '"', ' ');
  return istr;
}
std::ostream &operator<<(std::ostream &ostr, NVM_Camera &rhs) {
  ostr << rhs.filename << " ";
  ostr << rhs.f << " ";
  ostr << rhs.rq[0] << " " << rhs.rq[1] << " " << rhs.rq[2] << " " << rhs.rq[3]
       << " ";
  ostr << rhs.c[0] << " " << rhs.c[1] << " " << rhs.c[2] << " ";
  ostr << (rhs.r * rhs.f * rhs.f) << " ";
  if (rhs.intrinsics && rhs.intrinsics->type() != IntrinsicsType::NVM_DEFAULT) {
    ostr << static_cast<int>(rhs.intrinsics->type());
    rhs.intrinsics->writeAscii(ostr);
  } else {
    int check = 0;
    ostr << check;
  }
  ostr << endl;
  return ostr;
}

std::istream &operator>>(std::istream &istr, NVM_Model &rhs) {
  int nrCameras = 0, nrPoints = 0;
  istr >> nrCameras;
  rhs.cameras.resize(nrCameras);
  for (size_t ii = 0; ii < nrCameras; ii++)
    istr >> rhs.cameras[ii];

  // handle empty case
  if (nrCameras > 0)
    istr >> nrPoints;
  rhs.points.resize(nrPoints);
  for (size_t ii = 0; ii < nrPoints; ii++)
    istr >> rhs.points[ii];
  return istr;
}
std::ostream &operator<<(std::ostream &ostr, NVM_Model &rhs) {
  int nrCameras = rhs.cameras.size(), nrPoints = rhs.points.size();
  ostr << endl << nrCameras << endl;
  for (size_t ii = 0; ii < nrCameras; ii++)
    ostr << rhs.cameras[ii];

  // handle empty case
  if (nrCameras > 0)
    ostr << endl << nrPoints << endl;
  for (size_t ii = 0; ii < nrPoints; ii++)
    ostr << rhs.points[ii];
  return ostr;
}

bool NVMFile::readFile(const char *path, std::vector<NVM_Model> &models,
                       bool fixPath) {
  models.clear();
  std::ifstream infile(path);
  if (!infile.good()) {
    LOG(WARNING) << "cannot read from <" << path << ">";
    return false;
  }

  // extract the folder of this nvm file
  string nvmfolder(stlplus::folder_part(path));

  // check header (or version)
  string header;
  int version = -1;
  infile >> header;
  if (strcasecmp("NVM_V3", header.c_str()) == 0) {
    version = 3;
  } else if (strcasecmp("NVM_V4", header.c_str()) == 0) {
    version = 4;
  } else {
    LOG(WARNING) << "<" << path << "> is no valid nvm file [TAG = " << header
                 << "]";
    return false;
  }

  do {
    models.emplace_back();
    infile >> models.back();
    models.back().version = version;

    if (fixPath) {
      for (size_t ii = 0; ii < models.back().cameras.size(); ii++) {
        string name = models.back().cameras[ii].filename;
        if (stlplus::is_relative_path(name))
          models.back().cameras[ii].filename =
              stlplus::create_filespec(nvmfolder, name);
      }
    }

  } while (infile.good() && models.back().cameras.size() > 0);
  if (models.size() > 0)
    models.pop_back(); // remove empty model at the back

  LOG(INFO) << "read " << models.size() << " models from <" << path << ">";
  for (size_t ii = 0; ii < models.size(); ii++)
    LOG(INFO) << "Model " << ii << ": " << models[ii].cameras.size()
              << " cameras and " << models[ii].points.size() << " points";

  return models.size();
}

bool NVMFile::saveNVM(const char *path, const std::vector<NVM_Model> &models,
                      const int version) {

  // make sure the measurements in the model match the requested version
  for (const auto &m : models) {
    if (m.version != version) {
      LOG(WARNING) << "version of model (" << m.version
                   << ") does not match output version " << version;
      return false;
    }
  }

  std::ofstream outfile(path);
  if (!outfile.good()) {
    LOG(WARNING) << "cannot write to <" << path << ">";
    return false;
  }

  // more precision
  outfile << std::setprecision(12);

  // extract the folder of this nvm file
  string nvmfolder(stlplus::folder_part(path));

  // header
  outfile << "NVM_V" << version << endl;

  for (auto model : models)
    outfile << model;

  // empty model at the end
  outfile << "0";
  bool success = outfile.good();
  outfile.close();

  if (success) {
    LOG(INFO) << "saved " << models.size() << " models to <" << path << ">";
    return true;
  } else {
    LOG(WARNING) << "failed to save nvm file to >" << path << "<";
    return false;
  }
}

void NVMFile::saveAsPly(const NVM_Model &model, const char *file) {
  ofstream ofstr;
  ofstr.open(file);
  ofstr << "ply" << '\n'
        << "format ascii 1.0" << '\n'
        << "element vertex " << (int)model.points.size() << '\n'
        << "property float x" << '\n'
        << "property float y" << '\n'
        << "property float z" << '\n'
        << "property uchar diffuse_red" << '\n'
        << "property uchar diffuse_green" << '\n'
        << "property uchar diffuse_blue" << '\n'
        << "end_header" << '\n';

  for (const auto &p : model.points) {
    ofstr << p.xyz[0] << " ";
    ofstr << p.xyz[1] << " ";
    ofstr << p.xyz[2] << " ";

    ofstr << p.rgb[0] << " ";
    ofstr << p.rgb[0] << " ";
    ofstr << p.rgb[0] << "\n";
  }

  ofstr.close();
}

void NVMFile::setRelativePaths(const std::string &outfolder,
                               std::vector<NVM_Model> &models) {
  for (auto &model : models) {
    for (auto &camera : model.cameras) {
      camera.filename =
          stlplus::filespec_to_relative_path(outfolder, camera.filename);
    }
  }
}

} /* namespace nvmtools */
