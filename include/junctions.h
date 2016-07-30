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
 * This file defines the class of network junctions for network
 * reconfiguration in SOAX.
 */


#ifndef JUNCTIONS_H_
#define JUNCTIONS_H_

#include <vector>
#include "./util.h"

namespace soax {
class SnakeTipSet;
class SnakeTip;

class Junctions {
 public:
  Junctions();
  ~Junctions();

  const PointContainer &locations() const {return locations_;}

  /**
   * Cluster the nearby SnakeTips of SEGMENTS into a network junction.
   */
  void Union(const SnakeContainer *segments);

  /**
   * Link the snake tips in each network junction as smooth as possible.
   */
  void Configure(const SnakeContainer &previous_snakes);

  /**
   * Find the particular tip of snake S.
   */
  SnakeTip * GetTip(Snake *s, bool is_head) const;

  // bool AreClustered(const SnakeTip *t1, const SnakeTip *t2) const;

  SnakeTipSet * GetTipSet(const SnakeTip* t) const;
  SnakeTipSet * GetTipSet(Snake* s, bool is_head) const;

  /**
   * Update the location of network junctions for snakes GROUPED. If the
   * number of snakes that are within distance of THRESHOLD is more than
   * one, the centroid of the corresponding SnakeTipSet is a junction.
   */
  void UpdateLocation(const SnakeContainer &grouped, double threshold);

  void set_distance_threshold(double threshold) {
    distance_threshold_ = threshold;
  }

  void set_direction_threshold(double threshold) {
    direction_threshold_ = threshold;
  }

  void set_delta(int delta) {delta_ = delta;}

 private:
  typedef std::vector<SnakeTipSet *> TipSetContainer;
  typedef std::vector<SnakeTip *> TipContainer;

  void AddTip(Snake *s, bool is_head);

  /**
   * Sets of network junctions (cluster of T-junctions).
   */
  TipSetContainer tip_sets_;

  TipContainer tips_;

  /**
   * Locations of junction points.
   */
  PointContainer locations_;

  /**
   * Distance threshold for clustering T-junctions.
   */
  double distance_threshold_;

  /**
   * Direction threshold for grouping snake segments.
   */
  double direction_threshold_;

  /**
   * Number of vertices apart for estimating snake tip directions during
   * network reconfiguration.
   */
  int delta_;


  Junctions(const Junctions &);
  void operator=(const Junctions &);
};

}  // namespace soax

#endif  // JUNCTIONS_H_
