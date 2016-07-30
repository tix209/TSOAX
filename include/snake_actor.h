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
 * This class is for the display of SOACs in a VTK render window.
 */

#ifndef SNAKE_ACTOR_H_
#define SNAKE_ACTOR_H_

#include <map>
#include "./util.h"

class vtkActor;
class vtkPolyData;
class vtkRenderer;

namespace soax {
class Snake;

class SnakeActor {
  friend class SnakeActorTest;

 public:
  typedef std::map<vtkActor *, Snake *> ActorSnakeMap;
  typedef std::map<Snake *, vtkActor *> SnakeActorMap;

  SnakeActor();
  SnakeActor(const SnakeContainer &snakes,
             double *color, double width, double opacity);
  ~SnakeActor();

  /**
   * Update the actor for a new set of SNAKES.
   */
  void Update(const SnakeContainer &snakes);

  /**
   * Show/hide the snakes in RENDERER.
   */
  void Show(vtkRenderer * renderer) const;
  void Hide(vtkRenderer * renderer) const;

  /**
   * Returns true if ACTOR belongs to this SnakeActor.
   */
  bool Has(vtkActor *actor) const;

  /**
   * Select/deselect snake for viewing.
   */
  void SelectForView(vtkActor *actor);
  void DeselectForView(vtkActor *actor);

  void Highlight(Snake * s);

  void SetColor(double *color);
  void SetWidth(double width);
  void SetOpacity(double opacity);

  void Reset();

  Snake * selected_snake() const {return selected_snake_;}
  void set_selected_snake(Snake *s) {selected_snake_ = s;}

 private:
  vtkActor * SetupSingleActor(Snake* snake, int start, int end);

  /*
   * Returns the polydata from snakes. User needs to call Delete() on the
   * returned pointer.
   */
  vtkPolyData * MakePolyData(Snake *snake, int start, int end) const;

  void SetActorProperties(vtkActor *actor);


  ActorSnakeMap actor_snake_map_;
  SnakeActorMap snake_actor_map_;

  double *color_ = nullptr;
  double width_ = 3.0;
  double opacity_ = 1.0;

  Snake * selected_snake_ = nullptr;

  SnakeActor(const SnakeActor &);
  void operator=(const SnakeActor &);
};

}  // namespace soax

#endif  // SNAKE_ACTOR_H_
