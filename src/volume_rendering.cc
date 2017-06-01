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
 * This file implements the volume rendering class.
 */


#include "include/volume_rendering.h"
#include "vtkImageData.h"
#include "vtkVolume.h"
#include "vtkVolumeMapper.h"
#include "vtkVolumeProperty.h"
#include "vtkRenderer.h"
#include "vtkPiecewiseFunction.h"
#include "vtkColorTransferFunction.h"
#include "vtkSmartVolumeMapper.h"
#include "vtkFixedPointVolumeRayCastMapper.h"
#include "vtkActor.h"
#include "vtkPolyDataMapper.h"
#include "vtkOutlineSource.h"
#include "vtkProperty.h"

namespace soax {

VolumeRendering::VolumeRendering() {}

VolumeRendering::VolumeRendering(vtkImageData *image) {
  volume_ = vtkVolume::New();

  UpdateDisplayRange(image);
  vtkPiecewiseFunction *opacity_function = vtkPiecewiseFunction::New();
  opacity_function->AddPoint(min_intensity_, 0.0);
  opacity_function->AddPoint(max_intensity_, 1.0);

  vtkColorTransferFunction *color_function =
      vtkColorTransferFunction::New();
  color_function->SetColorSpaceToRGB();
  color_function->AddRGBPoint(0, 1, 1, 1);
  color_function->AddRGBPoint(255, 1, 1, 1);

  vtkVolumeProperty *mip_volume_property = vtkVolumeProperty::New();
  mip_volume_property->SetScalarOpacity(opacity_function);
  mip_volume_property->SetColor(color_function);
  mip_volume_property->SetInterpolationTypeToLinear();
  volume_->SetProperty(mip_volume_property);

  this->SetupSmartVolumeMapper(image);
  opacity_function->Delete();
  color_function->Delete();
  mip_volume_property->Delete();
}

VolumeRendering::~VolumeRendering() {
  if (bounding_box_)
    bounding_box_->Delete();
  if (volume_)
    volume_->Delete();
}

void VolumeRendering::Update(vtkImageData *image) {
  this->SetupSmartVolumeMapper(image);
  UpdateDisplayRange(image);
  SetDisplayRange(min_intensity_, max_intensity_);
}

void VolumeRendering::Show(vtkRenderer *renderer) {
  renderer->AddViewProp(volume_);
}

void VolumeRendering::Hide(vtkRenderer *renderer) {
  renderer->RemoveViewProp(volume_);
}

void VolumeRendering::SetDisplayRange(double min, double max) {
  volume_->GetProperty()->GetScalarOpacity()->RemoveAllPoints();
  volume_->GetProperty()->GetScalarOpacity()->AddPoint(min, 0.0);
  volume_->GetProperty()->GetScalarOpacity()->AddPoint(max, 1.0);
  min_intensity_ = min;
  max_intensity_ = max;
}

void VolumeRendering::ShowBoundingBox(vtkRenderer *renderer) {
  if (!bounding_box_) {
    SetupBoundingBox();
  }
  renderer->AddViewProp(bounding_box_);
}

void VolumeRendering::HideBoundingBox(vtkRenderer *renderer) {
  renderer->RemoveViewProp(bounding_box_);
}

/**************** Private Methods *******************/

void VolumeRendering::UpdateDisplayRange(vtkImageData *image) {
  double range[2];
  image->GetScalarRange(range);
  min_intensity_ = range[0] + ratio_ * (range[1] - range[0]);
  max_intensity_ = range[1];
}

void VolumeRendering::SetupBoundingBox() {
  bounding_box_ = vtkActor::New();
  vtkOutlineSource *outline = vtkOutlineSource::New();
  vtkPolyDataMapper *outline_mapper = vtkPolyDataMapper::New();
  outline_mapper->SetInputConnection(outline->GetOutputPort());
  outline->SetBounds(volume_->GetBounds());
  bounding_box_->PickableOff();
  bounding_box_->DragableOff();
  bounding_box_->SetMapper(outline_mapper);
  bounding_box_->GetProperty()->SetLineWidth(1.0);
  bounding_box_->GetProperty()->SetColor(1, 0, 0);
  bounding_box_->GetProperty()->SetAmbient(1.0);
  bounding_box_->GetProperty()->SetDiffuse(0.0);
  outline->Delete();
  outline_mapper->Delete();
}

void VolumeRendering::SetupSmartVolumeMapper(vtkImageData *image) {
  vtkSmartVolumeMapper *mapper = vtkSmartVolumeMapper::New();
  mapper->SetBlendModeToMaximumIntensity();
  // mapper->SetInterpolationModeToLinear();
  mapper->SetInputData(image);
  volume_->SetMapper(mapper);
  mapper->Delete();
}

}  // namespace soax
