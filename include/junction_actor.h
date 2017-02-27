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
 * This class is for the display of network junctions in a VTK render
 * window.
 */

#ifndef JUNCTION_ACTOR_H_
#define JUNCTION_ACTOR_H_

#include <map>
#include "./util.h"

class vtkActor;
class vtkRenderer;

namespace soax {

class JunctionActor {
 public:
  typedef std::map<vtkActor *, PointType> ActorPointMap;
  JunctionActor();
  JunctionActor(const PointContainer &points,
                double *color, double radius, double opacity);
  ~JunctionActor();

  void Update(const PointContainer &points,
              double *color, double radius, double opacity);

  void Show(vtkRenderer *renderer) const;
  void Hide(vtkRenderer *renderer) const;

  void Reset();
 private:
  ActorPointMap actor_points_;
};

}  // namespace soax


#endif  // JUNCTION_ACTOR_H_
