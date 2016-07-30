/**
 * Copyright (C) 2016 Lehigh University.
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
 * ImageResample is a class for interpolate image (in a sequence) to have
 * isotropic pixel/voxel size.
 */

#include <string>

class vtkImageData;

namespace soax {

class ImageResampler {
 public:
  ImageResampler();

  /**
   * Output image dimensions.
   */
  // void set_dimensions(int sx, int sy, int sz);

  /**
   * Set the ratio of voxel spacing along z axis to that of x and y axis.
   */
  // void SetZSpacing(double ratio);

  /**
   * Resample the image to have isotropic voxels. RATIO is the voxel spacing
   * ratio between xy and z axis.
   */
  void Resize(vtkImageData *img, double ratio);

  vtkImageData *GetImage() const {return image_;}

  void WriteImage(const std::string &filename) const;

 private:
  vtkImageData *image_ = nullptr;
};

}  // namespace soax
