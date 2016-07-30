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
 * This file defines the SOAC tip for the network configuration procedure
 * in SOAX.
 */

#ifndef SNAKE_TIP_H_
#define SNAKE_TIP_H_

#include "./util.h"

class Snake;

namespace soax {

class SnakeTip {
 public:
  SnakeTip(Snake *s, bool is_head);

  Snake * snake() const {return snake_;}

  bool is_head() const {return is_head_;}

  SnakeTip * neighbor() const {return neighbor_;}
  void set_neighbor(SnakeTip *neighbor) {neighbor_ = neighbor;}

  /**
   * Returns the tip location.
   */
  PointType GetLocation() const;
  PointType GetOppositeLocation() const;

  /**
   * Returns the tangent vector at the tip.
   */
  VectorXd GetDirection(int delta) const;

  void Print(std::ostream &os) const;  // NOLINT

 private:
  Snake *snake_ = nullptr;
  bool is_head_ = true;
  SnakeTip *neighbor_ = nullptr;
};

/**
 * Compute the distance between SnakeTip T1 and T2.
 */
double ComputeDistance(const SnakeTip *t1, const SnakeTip *t2);

/**
 * Compute the angle bewteen two tangent vectors at snakes tip T1 and T2.
 */
double ComputeAngle(const SnakeTip *t1, const SnakeTip *t2, int delta);

/**
 * Link up snake tip T1 and T2 so that they can be concatenated.
 */
void Link(SnakeTip *t1, SnakeTip *t2);

/**
 * Returns true if SnakeTip T1 and T2 are linked.
 */
bool AreLinked(SnakeTip *t1, SnakeTip *t2);

/**
 * Returns true if the distance between T1 and T2 is smaller than that
 * between opposite of T1 and opposite of T2.
 */
// bool AreTighterThanOppositeLink(const SnakeTip *t1,
//                                 const SnakeTip *t2);

/**
 * Returns true if the distance between T1 and T2 is smaller than any other
 * head/tail combinations.
 */
bool AreTightestLink(const SnakeTip *t1, const SnakeTip *t2);


}  // namespace soax

#endif  // SNAKE_TIP_H_
