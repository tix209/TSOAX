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
 */


#include "include/image_resampler.h"
#include <cassert>
#include "vtkImageData.h"
#include "vtkImageResize.h"
#include "vtkTIFFWriter.h"

namespace soax {

ImageResampler::ImageResampler() {}

// vtkImageResize will magnify or shrink an image with interpolation and
// antialiasing. The resizing is done with a 5-lobe Lanczos-windowed sinc filter
// that is bandlimited to the output sampling frequency in order to avoid
// aliasing when the image size is reduced. This filter utilizes a O(n)
// algorithm to provide good effiency even though the filtering kernel is
// large. The sinc interpolator can be turned off if nearest-neighbor
// interpolation is required, or it can be replaced with a different
// vtkImageInterpolator object.
void ImageResampler::Resize(vtkImageData *img, double ratio) {
  img->SetSpacing(1.0, 1.0, ratio);
  vtkImageResize *resize = vtkImageResize::New();
  resize->SetInputData(img);
  resize->SetResizeMethodToOutputSpacing();
  resize->SetOutputSpacing(1.0, 1.0, 1.0);
  resize->BorderOn();
  resize->Update();

  if (image_)  image_->Delete();
  image_ = vtkImageData::New();
  image_->ShallowCopy(resize->GetOutput());
  resize->Delete();
}


void ImageResampler::WriteImage(const std::string &filename) const {
  assert(image_);
  vtkTIFFWriter *writer = vtkTIFFWriter::New();
  writer->SetFileName(filename.c_str());
  writer->SetInputData(image_);
  writer->Write();
  writer->Delete();
}



}
