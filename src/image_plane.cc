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
 */


#include "include/image_plane.h"
#include "vtkImageActor.h"
#include "vtkImageData.h"
#include "vtkRenderer.h"
#include "vtkImageProperty.h"

namespace soax {

ImagePlane::ImagePlane() {}

ImagePlane::ImagePlane(vtkImageData *image) {
  actor_ = vtkImageActor::New();
  Update(image);
}

ImagePlane::~ImagePlane() {
  if (actor_) {
    actor_->Delete();
  }
}

void ImagePlane::Update(vtkImageData *image) {
  actor_->SetInputData(image);
  actor_->InterpolateOff();
  UpdateWindowLevel(image);
}

void ImagePlane::Show(vtkRenderer *renderer) {
  renderer->AddViewProp(actor_);
}

void ImagePlane::Hide(vtkRenderer *renderer) {
  renderer->RemoveViewProp(actor_);
}

double ImagePlane::GetWindow() const {
  return actor_->GetProperty()->GetColorWindow();
}

double ImagePlane::GetLevel() const {
  return actor_->GetProperty()->GetColorLevel();
}

void ImagePlane::SetWindowLevel(double window, double level) {
  actor_->GetProperty()->SetColorWindow(window);
  actor_->GetProperty()->SetColorLevel(level);
  window_level_manually_set_ = true;
}

/************************ Private Methods ************************/

void ImagePlane::UpdateWindowLevel(vtkImageData *image) {
  if (!window_level_manually_set_) {
    double range[2];
    image->GetScalarRange(range);
    double window = range[1] - range[0];
    actor_->GetProperty()->SetColorWindow(window);
    actor_->GetProperty()->SetColorLevel(range[0] + window / 2);
  }
}


}  // namespace soax
