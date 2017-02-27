/**
 * Copyright (C) 2017 Lehigh University.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 *
 * Author: Ting Xu (xuting.bme@gmail.com)
 *
 * This file implements the viewpoint class of the SOAX program.
 */


#include "include/viewpoint.h"
#include <sstream>
#include <fstream>
#include <QFileInfo>  // NOLINT(build/include_order)

namespace soax {

Viewpoint::Viewpoint() : path_(".."), camera_(vtkCamera::New()) {}

Viewpoint::~Viewpoint() {
  camera_->Delete();
}

std::string Viewpoint::ToString() const {
  std::ostringstream os;
  os << "ClippingRange\n"
     << camera_->GetClippingRange()[0] << " "
     << camera_->GetClippingRange()[1]
     << "\nCameraPosition\n"
     << camera_->GetPosition()[0] << " "
     << camera_->GetPosition()[1] << " "
     << camera_->GetPosition()[2]
     << "\nCameraFocalPoint\n"
     << camera_->GetFocalPoint()[0] << " "
     << camera_->GetFocalPoint()[1] << " "
     << camera_->GetFocalPoint()[2]
     << "\nCameraViewUp\n"
     << camera_->GetViewUp()[0] << " "
     << camera_->GetViewUp()[1] << " "
     << camera_->GetViewUp()[2] << std::endl;

  return os.str();
}

void Viewpoint::SaveToFile(const QString &filename) {
  std::ofstream outfile(filename.toStdString().c_str());
  if (!outfile) return;
  outfile << ToString();
  outfile.close();
  path_ = QFileInfo(filename).absolutePath();
}

void Viewpoint::LoadFromFile(const QString &filename) {
  std::ifstream infile(filename.toStdString().c_str());
  if (!infile) return;
  Load(infile);
  infile.close();
  path_ = QFileInfo(filename).absolutePath();
}

void Viewpoint::SaveDefault() {
  if (default_viewpoint_str_.empty())
    default_viewpoint_str_ = ToString();
}

void Viewpoint::LoadDefault() {
  if (default_viewpoint_str_.empty()) return;
  std::istringstream buffer(default_viewpoint_str_);
  Load(buffer);
}

void Viewpoint::Load(std::istream &is) {
  std::string name;
  double x, y, z;

  is >> name;
  if (name == "ClippingRange") {
    is >> x >> y;
    camera_->SetClippingRange(x, y);
  }

  is >> name;
  if (name == "CameraPosition") {
    is >> x >> y >> z;
    camera_->SetPosition(x, y, z);
  }

  is >> name;
  if (name == "CameraFocalPoint") {
    is >> x >> y >> z;
    camera_->SetFocalPoint(x, y, z);
  }

  is >> name;
  if (name == "CameraViewUp") {
    is >> x >> y >> z;
    camera_->SetViewUp(x, y, z);
  }
}

std::ostream & operator<<(std::ostream &os, const Viewpoint &vp) {
  return os << vp.ToString();
}



}  // namespace soax
