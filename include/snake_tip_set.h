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
 * This file defines the SOAC tip set class for the network configuration
 * procedure in SOAX.
 */

#ifndef SNAKE_TIP_SET_H_
#define SNAKE_TIP_SET_H_

#include "include/util.h"
#include "include/munkres.h"


namespace soax {
class SnakeTip;
class Snake;

class SnakeTipSet {
 public:
  typedef std::vector<SnakeTip *> TipContainer;

  explicit SnakeTipSet(SnakeTip *t);

  /**
   * Add SnakeTip T to this set.
   */
  void Add(SnakeTip *t);

  /**
   * Get the average position of the tips in the set.
   */
  PointType centroid() const {return centroid_;}

  /**
   * Returns the smallest distance from a tip in TIPS_ to T.
   */
  double ComputeDistanceTo(SnakeTip *t) const;

  /**
   * Add the snake tips in TS to this set.
   */
  void Combine(SnakeTipSet *ts);

  bool Has(const SnakeTip *t) const;

  /**
   * Update the position of the controid of all the snake tips in the set.
   */
  void UpdateCentroid();

  /**
   * Link the SnakeTips in this set based on their tangent angle. TIPS_
   * becomes empty after this method is called.
   */
  void Configure(int delta, double direction_threshold,
                 double distance_threshold);
  void Configure(const SnakeContainer &previous_snakes, int delta,
                 double direction_threshold,
                 double distance_threshold);

  bool IsJunction(const SnakeContainer &grouped, double threshold) const;

  void Print(std::ostream &os) const;  // NOLINT

 private:
  double FindSmoothestPair(SnakeTip **t1, SnakeTip **t2, int delta,
                           double threshold) const;
  void RemoveTip(const SnakeTip *t);

  void ComputeAngleDifference(Matrix<double> &distance,
                             int delta, double direction_threshold,
                             double distance_threshold);
  void ComputeLabelDifference(const SnakeContainer &previous_snakes,
                              Matrix<double> &label_diffs);
  bool LinkTips(const Matrix<double> &assignment);
  void UnlinkTips();


  TipContainer tips_;

  /**
   * Centroid of all the tip locations.
   */
  PointType centroid_;
};

}  // namespace soax
#endif  // SNAKE_TIP_SET_H_
