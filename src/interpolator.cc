/**
 * Copyright (C) 2015 Lehigh University.
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
 */

#include "include/interpolator.h"
#include "vtkImageData.h"
#include "vtkImageInterpolator.h"


namespace soax {

Interpolator::Interpolator() {}

Interpolator::Interpolator(vtkImageData *image) :
    interp_(vtkImageInterpolator::New()) {
  interp_->Initialize(image);
}

Interpolator::~Interpolator() {
  DeleteInterpolator();
}

void Interpolator::Initialize(vtkImageData *image) {
  DeleteInterpolator();
  interp_ = vtkImageInterpolator::New();
  interp_->Initialize(image);
}

bool Interpolator::Interpolate(const double *point,
                               double *voxel) const {
  return interp_->Interpolate(point, voxel);
}

bool Interpolator::Interpolate(const VectorXd &point,
                               VectorXd *voxel) const {
  double p[3] = {0.0, 0.0, 0.0};
  for (int i = 0; i < point.size(); i++) {
    p[i] = point(i);
  }

  double v[3] = {0.0, 0.0, 0.0};
  if (interp_->Interpolate(p, v)) {
    for (int i = 0; i < voxel->size(); i++) {
      (*voxel)(i) = v[i];
    }
    return true;
  }
  return false;
}

bool Interpolator::Interpolate(const Vector2d &point,
                               double *voxel) const {
  double p[3] = {point(0), point(1), 0.0};
  return interp_->Interpolate(p, voxel);
}

bool Interpolator::Interpolate(const Vector3d &point,
                               double *voxel) const {
  double p[3] = {point(0), point(1), point(2)};
  return interp_->Interpolate(p, voxel);
}

bool Interpolator::Interpolate(const Vector2d &point,
                               Vector2d &voxel) const {
  double p[3] = {point(0), point(1), 0.0};
  double v[2] = {0.0, 0.0};
  if (interp_->Interpolate(p, v)) {
    voxel(0) = v[0];
    voxel(1) = v[1];
    return true;
  } else {
    return false;
  }
}

bool Interpolator::Interpolate(const Vector3d &point,
                               Vector3d &voxel) const {
  double p[3] = {point(0), point(1), point(2)};
  double v[3] = {0.0, 0.0, 0.0};
  if (interp_->Interpolate(p, v)) {
    voxel(0) = v[0];
    voxel(1) = v[1];
    voxel(2) = v[2];
    return true;
  } else {
    return false;
  }
}

int Interpolator::GetNumberOfComponents() const {
  return interp_->GetNumberOfComponents();
}

void Interpolator::GetImageSize(int *size) const {
  size[0] = interp_->GetWholeExtent()[1] + 1;
  size[1] = interp_->GetWholeExtent()[3] + 1;
  size[2] = interp_->GetWholeExtent()[5] + 1;
}

int Interpolator::GetImageExtent(int index) const {
  return interp_->GetWholeExtent()[2 * index + 1];
}

/****************** Private Methods ******************/

void Interpolator::DeleteInterpolator() {
  if (interp_) {
    interp_->Delete();
    interp_ = nullptr;
  }
}


} // namespace soax
