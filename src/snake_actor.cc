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
 * This file implements the SnakeActor class.
 */

#include "include/snake_actor.h"
#include "vtkActor.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkProperty.h"
#include "vtkCellArray.h"
#include "vtkPoints.h"
#include "vtkRenderer.h"
#include "include/snake.h"
#include "include/actor_color.h"

namespace soax {


SnakeActor::SnakeActor() {}

SnakeActor::SnakeActor(const SnakeContainer &snakes,
                       double *color, double width, double opacity) :
    color_(color), width_(width), opacity_(opacity) {
  Update(snakes);
}

SnakeActor::~SnakeActor() {
  Reset();
}

void SnakeActor::Update(const SnakeContainer &snakes) {
  Reset();
  for (auto it = snakes.begin(); it != snakes.end(); ++it) {
    vtkActor *actor = SetupSingleActor(*it, 0, static_cast<int>((*it)->GetSize()));
    SetActorProperties(actor);
    actor_snake_map_[actor] = *it;
    snake_actor_map_[*it] = actor;
  }
}

vtkActor * SnakeActor::SetupSingleActor(Snake* snake,
                                        int start, int end) {
  vtkPolyData *curve = MakePolyData(snake, start, end);
  vtkPolyDataMapper *mapper = vtkPolyDataMapper::New();
  mapper->SetInputData(curve);
  vtkActor *actor = vtkActor::New();
  actor->SetMapper(mapper);
  mapper->Update();
  mapper->Delete();
  curve->Delete();
  return actor;
}

void SnakeActor::Show(vtkRenderer * renderer) const {
  for (const auto &as : actor_snake_map_) {
    renderer->AddViewProp(as.first);
  }
}

void SnakeActor::Hide(vtkRenderer * renderer) const {
  for (const auto &as : actor_snake_map_) {
    renderer->RemoveViewProp(as.first);
  }
}

bool SnakeActor::Has(vtkActor *actor) const {
  return actor_snake_map_.find(actor) != actor_snake_map_.end();
}

void SnakeActor::SelectForView(vtkActor *actor) {
  actor->GetProperty()->SetColor(ActorColor::kCyan);
  selected_snake_ = actor_snake_map_[actor];
  selected_snake_->Print(std::cout);
}

void SnakeActor::DeselectForView(vtkActor *actor) {
  actor->GetProperty()->SetColor(color_);
  selected_snake_ = nullptr;
}

void SnakeActor::Highlight(Snake * s) {
  if (!s) return;
  auto it = snake_actor_map_.find(s);
  if (it != snake_actor_map_.end()) {
    it->second->GetProperty()->SetColor(ActorColor::kCyan);
  }
}

void SnakeActor::SetColor(double *color) {
  color_ = color;
  for (auto &as : actor_snake_map_) {
    as.first->GetProperty()->SetColor(color);
  }
}

void SnakeActor::SetWidth(double width) {
  width_ = width;
  for (auto &as : actor_snake_map_) {
    as.first->GetProperty()->SetLineWidth(width);
  }
}

void SnakeActor::SetOpacity(double opacity) {
  opacity_ = opacity;
  for (auto &as : actor_snake_map_) {
    as.first->GetProperty()->SetOpacity(opacity);
  }
}

void SnakeActor::Reset() {
  for (auto &as : actor_snake_map_) {
    as.first->Delete();
  }
  actor_snake_map_.clear();
  snake_actor_map_.clear();
  // selected_snake_ = nullptr;
}

/*********************** Private Methods ***********************/
vtkPolyData * SnakeActor::MakePolyData(Snake *snake,
                                       int start, int end) const {
  vtkPolyData *curve = vtkPolyData::New();
  vtkPoints *points = vtkPoints::New();
  vtkCellArray *cells = vtkCellArray::New();

  vtkIdType cell_index[2];
  double coordinates[3];

  for (int i = start, j = 0; i < end; ++i, ++j) {
    snake->GetVertex(i, coordinates);
    points->InsertPoint(j, coordinates);

    // last point
    if (i == end - 1) {
      if (!snake->open()) {
        cell_index[0] = end - 1 - start;
        cell_index[1] = 0;
        cells->InsertNextCell(2, cell_index);
      }
    } else {
      cell_index[0] = j;
      cell_index[1] = j + 1;
      cells->InsertNextCell(2, cell_index);
    }
  }

  curve->SetPoints(points);
  curve->SetLines(cells);
  points->Delete();
  cells->Delete();
  return curve;
}

// vtkPolyData * SnakeActor::MakePolyData(const SnakeContainer &snakes) const {
//   vtkPolyData *curve = vtkPolyData::New();
//   vtkPoints *points = vtkPoints::New();
//   vtkCellArray *cells = vtkCellArray::New();

//   unsigned index = 0;
//   vtkIdType cell_index[2];
//   vtkFloatingPointType coordinates[3];

//   for (SnakeContainer::const_iterator it = snakes.begin();
//        it != snakes.end(); ++it) {
//     for (int i = 0; i < (*it)->GetSize(); ++i) {
//       (*it)->GetVertex(i, coordinates);
//       points->InsertPoint(index, coordinates);
//       if (i != (*it)->GetSize() - 1) {
//         cell_index[0] = index;
//         cell_index[1] = index + 1;
//         cells->InsertNextCell(2, cell_index);
//       }
//       index++;
//     }
//   }
//   curve->SetPoints(points);
//   curve->SetLines(cells);
//   points->Delete();
//   cells->Delete();

//   return curve;
// }

void SnakeActor::SetActorProperties(vtkActor *actor) {
  actor->GetProperty()->SetInterpolationToPhong();
  actor->GetProperty()->SetOpacity(opacity_);
  actor->GetProperty()->SetAmbient(0.2);
  actor->GetProperty()->SetDiffuse(0.7);
  actor->GetProperty()->SetSpecular(0.6);
  actor->GetProperty()->SetSpecularPower(50);
  actor->GetProperty()->SetColor(color_);
  actor->GetProperty()->SetLineWidth(width_);
}


}  // namespace soax
