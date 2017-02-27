/**
 * Copyright (C) 2017 Lehigh University.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see http://www.gnu.org/licenses/.
 *
 * Author: Ting Xu (xuting.bme@gmail.com)
 *
 * This class is for image interpolation. Image pixels can have multiple
 * components.
 */

#ifndef INTERPOLATOR_H_
#define INTERPOLATOR_H_

#include "./util.h"

class vtkImageData;
class vtkImageInterpolator;

namespace soax {

class Interpolator {
 public:
  Interpolator();
  explicit Interpolator(vtkImageData *image);

  ~Interpolator();

  /*
   * Initialize interpolator for the image.
   */
  void Initialize(vtkImageData *image);

  /*
   * Interpolation for image voxel at POINT. Returns true if POINT is
   * within image bounds.
   */
  bool Interpolate(const double *point, double *voxel) const;
  bool Interpolate(const VectorXd &point, VectorXd *voxel) const;
  bool Interpolate(const Vector2d &point, double *voxel) const;
  bool Interpolate(const Vector3d &point, double *voxel) const;
  bool Interpolate(const Vector2d &point, Vector2d &voxel) const;
  bool Interpolate(const Vector3d &point, Vector3d &voxel) const;

  int GetNumberOfComponents() const;

  void GetImageSize(int *size) const;

  int GetImageExtent(int index) const;

 private:
  /*
   * Delete and free the memory of image interpolator.
   */
  void DeleteInterpolator();

  vtkImageInterpolator *interp_ = nullptr;

  Interpolator(const Interpolator &);
  void operator=(const Interpolator &);
};

}  // namespace soax

#endif  // INTERPOLATOR_H_
