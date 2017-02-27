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
 * This file implements the JunctionActor class.
 */

#include "include/junction_actor.h"
#include "vtkActor.h"
#include "vtkRenderer.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkProperty.h"
#include "vtkSphereSource.h"
#include "include/actor_color.h"

namespace soax {

JunctionActor::JunctionActor() {}

JunctionActor::JunctionActor(const PointContainer &points,
                             double *color, double radius, double opacity) {
  Update(points, color, radius, opacity);
}

JunctionActor::~JunctionActor() {
  Reset();
}

void JunctionActor::Update(const PointContainer &points,
                           double *color, double radius, double opacity) {
  Reset();
  for (auto it = points.begin(); it != points.end(); ++it) {
    vtkActor *sphere = vtkActor::New();
    vtkSphereSource *source = vtkSphereSource::New();
    if (it->size() == 3)
      source->SetCenter((*it)(0), (*it)(1), (*it)(2));
    else
      source->SetCenter((*it)(0), (*it)(1), .0);
    source->SetRadius(radius);
    source->Update();
    vtkPolyDataMapper *mapper = vtkPolyDataMapper::New();
    mapper->SetInputConnection(source->GetOutputPort());
    sphere->SetMapper(mapper);
    sphere->GetProperty()->SetColor(color);
    sphere->GetProperty()->SetOpacity(opacity);
    source->Delete();
    mapper->Delete();
    actor_points_[sphere] = *it;
  }
}


void JunctionActor::Show(vtkRenderer * renderer) const {
  for (const auto &ap : actor_points_) {
    renderer->AddViewProp(ap.first);
  }
}

void JunctionActor::Hide(vtkRenderer * renderer) const {
  for (const auto &ap : actor_points_) {
    renderer->RemoveViewProp(ap.first);
  }
}

void JunctionActor::Reset() {
  for (const auto &ap : actor_points_) {
    ap.first->Delete();
  }
  actor_points_.clear();
}

}  // namespace soax
