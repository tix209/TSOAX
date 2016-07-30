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
 * This file declares utility functions on image processing.
 */

#ifndef IMAGE_H_
#define IMAGE_H_

#include <string>

class vtkImageData;

namespace soax {

int GetImageDimension(vtkImageData *image);

double GetImageIntensity(vtkImageData *image, int x, int y, int z);

double GetMinimumIntensity(vtkImageData *image);
double GetMaximumIntensity(vtkImageData *image);

double *GetImageGradient(vtkImageData *image, int x, int y, int z);
double *GetImageGradient(vtkImageData *image, int *index);

/*
 * Returns true if index is within the bounds specified by EXTENT (x_min,
 * x_max, y_min, y_max, z_min, z_max).
 */
bool IsInside(vtkImageData *image, int *index);

/*
 * Returns a boolen array (voxel of IMAGE).
 */
bool * GetFlag(vtkImageData *image, int *index);
bool * GetFlag(vtkImageData *image, int x, int y, int z);

/*
 * Write image in UNC MetaImage format. Works for 3D.
 */
void WriteMetaImage(vtkImageData *image, const std::string &filename);

/*
 * Write image in TIFF format. Works only for 2D.
 */
void WriteTIFFImage(vtkImageData *image, const std::string &filename);

/*
 * Write image in JPEG format. Works only for 2D.
 */
void WriteJPEGImage(vtkImageData *image, const std::string &filename);


}  // namespace soax

#endif  // IMAGE_H_
