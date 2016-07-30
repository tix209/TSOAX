/**
 * Copyright (C) 2015 Lehigh University.
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
 * This file implements the GradientCalculator class.
 */

#include "./gradient_calculator.h"
#include "vtkImageData.h"
#include "vtkImageGradient.h"
#include "vtkImageRGBToHSV.h"
#include "vtkImageExtractComponents.h"
#include "vtkImageShiftScale.h"
#include "vtkImageGaussianSmooth.h"
#include "./util.h"

namespace soax {

GradientCalculator::GradientCalculator() {}

GradientCalculator::GradientCalculator(vtkImageData *image) : image_(image) {}

GradientCalculator::~GradientCalculator() {
  DeleteGradient();
}

void GradientCalculator::Compute(double scale, double sigma) {
  if (!image_) return;

  vtkImageGradient *gradient_filter = vtkImageGradient::New();
  vtkImageData *scaled_image = ScaleImage(image_, scale);
  vtkImageData *image = nullptr;
  if (sigma > kEpsilon) {
    image = SmoothImage(scaled_image, sigma);
    gradient_filter->SetInputData(image);
  } else {
    gradient_filter->SetInputData(scaled_image);
  }

  gradient_filter->SetDimensionality(image_->GetDataDimension());
  gradient_filter->Update();

  DeleteGradient();
  gradient_ = vtkImageData::New();
  gradient_->ShallowCopy(gradient_filter->GetOutput());
  gradient_filter->Delete();

  scaled_image->Delete();
  if (image)
    image->Delete();
}

void GradientCalculator::DeleteGradient() {
  if (gradient_) {
    gradient_->Delete();
    gradient_ = nullptr;
  }
}

void GradientCalculator::Reset() {
  image_ = nullptr;
  DeleteGradient();
}

/**************** Private Methods ****************/
vtkImageData * GradientCalculator::ScaleImage(vtkImageData *image,
                                              double scale) const {
  vtkImageShiftScale *scaler = vtkImageShiftScale::New();
  scaler->SetOutputScalarTypeToDouble();
  vtkImageData *single_channel = nullptr;
  if (image_->GetNumberOfScalarComponents() > 1) {
    single_channel = CondenseToSingleChannel();
    scaler->SetInputData(single_channel);
  } else {
    scaler->SetInputData(image);
  }
  double actual_scale = GetIntensityScaling(scale);
  std::cout << "Actual intensity scaling: " << actual_scale << std::endl;
  scaler->SetScale(actual_scale);
  scaler->Update();
  vtkImageData * output_image = vtkImageData::New();
  output_image->ShallowCopy(scaler->GetOutput());
  scaler->Delete();
  if (single_channel)
    single_channel->Delete();
  return output_image;
}

double GradientCalculator::GetIntensityScaling(double scale) const {
  if (image_ && fabs(scale) < kEpsilon) {
    return 1.0 / image_->GetScalarRange()[1];
  } else {
    return scale;
  }
}

vtkImageData * GradientCalculator::CondenseToSingleChannel() const {
  vtkImageRGBToHSV *hsv = vtkImageRGBToHSV::New();
  hsv->SetInputData(image_);
  vtkImageExtractComponents *extractor = vtkImageExtractComponents::New();
  extractor->SetInputConnection(hsv->GetOutputPort());
  extractor->SetComponents(2);
  extractor->Update();

  vtkImageData * value_image = vtkImageData::New();
  value_image->ShallowCopy(extractor->GetOutput());
  hsv->Delete();
  extractor->Delete();
  return value_image;
}

vtkImageData * GradientCalculator::SmoothImage(vtkImageData *image,
                                               double sigma) const {
  vtkImageGaussianSmooth *smoother = vtkImageGaussianSmooth::New();
  smoother->SetInputData(image);
  smoother->SetDimensionality(image->GetDataDimension());
  smoother->SetStandardDeviation(sigma);
  smoother->SetRadiusFactor(3);
  smoother->Update();
  // double stds[3];
  // smoother->GetStandardDeviations(stds);
  // std::cout << stds[0] << " " << stds[1] << " " << stds[2] << std::endl;
  // double r[3];
  // smoother->GetRadiusFactors(r);
  // std::cout << r[0] << " " << r[1] << " " << r[2] << std::endl;
  vtkImageData *output_image = vtkImageData::New();
  output_image->ShallowCopy(smoother->GetOutput());
  smoother->Delete();
  return output_image;
}

}  // namespace soax
