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
 * This file implements the SlicePlanes class.
 */

#include "./slice_planes.h"
#include "QVTKInteractor.h"
#include "vtkImagePlaneWidget.h"
#include "vtkImageData.h"


namespace soax {

SlicePlanes::SlicePlanes() {
  for (int i = 0; i < kNPlanes; i++) {
    planes_[i] = nullptr;
  }
}

SlicePlanes::SlicePlanes(vtkImageData *image, QVTKInteractor *interactor) {
  for (int i = 0; i < kNPlanes; i++) {
    planes_[i] = vtkImagePlaneWidget::New();
    planes_[i]->DisplayTextOn();
    planes_[i]->TextureInterpolateOff();
    planes_[i]->SetInteractor(interactor);
    planes_[i]->SetMarginSizeX(0);
    planes_[i]->SetMarginSizeY(0);
    planes_[i]->SetInputData(image);
    planes_[i]->SetPlaneOrientation(i);
    planes_[i]->UpdatePlacement();
    planes_[i]->SetRightButtonAction(
        vtkImagePlaneWidget::VTK_SLICE_MOTION_ACTION);
    planes_[i]->SetMiddleButtonAction(
        vtkImagePlaneWidget::VTK_WINDOW_LEVEL_ACTION);
  }
}

SlicePlanes::~SlicePlanes() {
  for (int i = 0; i < kNPlanes; i++) {
    if (planes_[i])
      planes_[i]->Delete();
  }
}

void SlicePlanes::Update(vtkImageData *image) {
  for (int i = 0; i < kNPlanes; i++) {
    int old_index = planes_[i]->GetSliceIndex();
    planes_[i]->SetInputData(image);
    planes_[i]->SetSliceIndex(old_index);
  }
}

void SlicePlanes::Show() const {
  for (int i = 0; i < kNPlanes; ++i) {
      planes_[i]->On();
  }
}

void SlicePlanes::Hide() const {
  for (int i = 0; i < kNPlanes; ++i) {
      planes_[i]->Off();
  }
}

double SlicePlanes::GetWindow() const {
  return planes_[0]->GetWindow();
}

double SlicePlanes::GetLevel() const {
  return planes_[0]->GetLevel();
}

void SlicePlanes::SetWindowLevel(double window, double level) {
  for (int i = 0; i > kNPlanes; i++) {
    planes_[i]->SetWindowLevel(window, level);
  }
}

}  // namespace soax
