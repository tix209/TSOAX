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
 * This file defines utility functions on image processing.
 */

#include "include/image.h"
#include "vtkImageData.h"
#include "vtkMetaImageWriter.h"
#include "vtkTIFFWriter.h"
#include "vtkJPEGWriter.h"

namespace soax {

int GetImageDimension(vtkImageData *image) {
  return image->GetDataDimension();
}

double GetImageIntensity(vtkImageData *image, int x, int y, int z) {
  int type = image->GetScalarType();
  if (type == 3) {  // unsigned char
    unsigned char *intensity = static_cast<unsigned char*>(
        image->GetScalarPointer(x, y, z));
    return static_cast<double>(intensity[0]);
  } else if (type == 5) {  // unsigned short
    unsigned short *intensity = static_cast<unsigned short*>(
        image->GetScalarPointer(x, y, z));
    return static_cast<double>(intensity[0]);
  } else if (type == 11) {
    return static_cast<double*>(image->GetScalarPointer(x, y, z))[0];
  } else {
    std::cerr << "Unknown pixel type!" << std::endl;
    return 0.0;
  }
}

double *GetImageGradient(vtkImageData *image, int x, int y, int z) {
  return static_cast<double*>(image->GetScalarPointer(x, y, z));
}

double *GetImageGradient(vtkImageData *image, int *index) {
  return static_cast<double*>(image->GetScalarPointer(index));
}


double GetMinimumIntensity(vtkImageData *image) {
  return image->GetScalarRange()[0];
}

double GetMaximumIntensity(vtkImageData *image) {
  return image->GetScalarRange()[1];
}


bool IsInside(vtkImageData *image, int *index) {
  int *extent = image->GetExtent();
  for (int i = 0; i < 3; i++) {
    if (index[i] < extent[2 * i] || index[i] > extent[2 * i + 1])
      return false;
  }
  return true;
}

bool * GetFlag(vtkImageData *image, int *index) {
  return static_cast<bool *>(image->GetScalarPointer(index));
}

bool * GetFlag(vtkImageData *image, int x, int y, int z) {
  return static_cast<bool *>(image->GetScalarPointer(x, y, z));
}

void WriteMetaImage(vtkImageData *image, const std::string &filename) {
  vtkMetaImageWriter *writer = vtkMetaImageWriter::New();
  writer->SetInputData(image);
  writer->SetFileName(filename.c_str());
  std::string raw_name(filename);
  std::string::size_type dot_pos = raw_name.find_last_of(".");
  raw_name.replace(dot_pos + 1, 3, "raw");
  writer->SetRAWFileName(raw_name.c_str());
  writer->Write();
  writer->Delete();
}

void WriteTIFFImage(vtkImageData *image, const std::string &filename) {
  vtkTIFFWriter *writer = vtkTIFFWriter::New();
  writer->SetInputData(image);
  writer->SetFileName(filename.c_str());
  writer->Write();
  writer->Delete();
}

void WriteJPEGImage(vtkImageData *image, const std::string &filename) {
  vtkJPEGWriter *writer = vtkJPEGWriter::New();
  writer->SetInputData(image);
  writer->SetFileName(filename.c_str());
  writer->Write();
  writer->Delete();
}

}  // namespace soax
