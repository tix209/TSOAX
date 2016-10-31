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
 * This file implements the network junctions class.
 */

#include "include/junctions.h"
#include "include/snake_tip_set.h"
#include "include/snake_tip.h"
#include "include/snake.h"

namespace soax {

Junctions::Junctions() :
    distance_threshold_(2.0), direction_threshold_(2 * kPi / 3), delta_(16) {}

Junctions::~Junctions() {
  for (auto it = tip_sets_.begin(); it != tip_sets_.end(); ++it) {
    delete *it;
  }

  for (auto it = tips_.begin(); it != tips_.end(); it++) {
      delete *it;
  }

  tip_sets_.clear();
  tips_.clear();
  locations_.clear();
}

void Junctions::Union(const SnakeContainer *segments) {
  for (auto it = segments->begin(); it != segments->end(); ++it) {
    AddTip(*it, true);
    AddTip(*it, false);
  }

  for (auto it = tip_sets_.begin(); it != tip_sets_.end(); ++it) {
    (*it)->UpdateCentroid();
  }
}

SnakeTipSet * Junctions::GetTipSet(const SnakeTip* t) const {
  for (auto it = tip_sets_.begin(); it != tip_sets_.end(); ++it) {
    if ((*it)->Has(t))
      return *it;
  }
  return nullptr;
}

SnakeTipSet * Junctions::GetTipSet(Snake* s, bool is_head) const {
  return GetTipSet(GetTip(s, is_head));
}

void Junctions::Configure(const SnakeContainer &previous_snakes) {
  for (auto it = tip_sets_.begin(); it != tip_sets_.end(); ++it) {
    (*it)->Configure(previous_snakes, delta_, direction_threshold_,
                     distance_threshold_);
    // (*it)->Print(std::cout);
  }
}

SnakeTip * Junctions::GetTip(Snake *s, bool is_head) const {
  for (auto it = tips_.begin(); it != tips_.end(); it++) {
    if ((*it)->snake() == s && (*it)->is_head() == is_head) {
      return *it;
    }
  }
  return nullptr;
}

void Junctions::UpdateLocation(const SnakeContainer &grouped,
                               double threshold) {
  if (grouped.empty()) return;

  for (auto it = tip_sets_.begin(); it != tip_sets_.end(); ++it) {
    if ((*it)->IsJunction(grouped, threshold))
      locations_.push_back((*it)->centroid());
  }
}

/************************ Private Methods ************************/

void Junctions::AddTip(Snake *s, bool is_head) {
  SnakeTip * t = new SnakeTip(s, is_head);
  tips_.push_back(t);
  // Find any TipSets that is close to t and put them to close_tip_sets
  TipSetContainer close_tip_sets;

  for (auto it = tip_sets_.begin(); it != tip_sets_.end(); ++it) {
    double dist = (*it)->ComputeDistanceTo(t);
    if (dist < distance_threshold_) {
      close_tip_sets.push_back(*it);
    }
  }

  // Get consolidated TipSet by merging TipSets which are all close to t
  if (close_tip_sets.empty()) {
    tip_sets_.push_back(new SnakeTipSet(t));
  } else {
    for (auto it = close_tip_sets.begin() + 1; it != close_tip_sets.end();
         ++it) {
      close_tip_sets.front()->Combine(*it);
      auto found = std::find(tip_sets_.begin(), tip_sets_.end(), *it);
      if (found != tip_sets_.end()) {
        delete *found;
        tip_sets_.erase(found);
      }
    }
    close_tip_sets.front()->Add(t);
  }
}

}  // namespace soax
