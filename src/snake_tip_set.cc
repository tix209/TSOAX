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
 * This file implements the SOAC tip set class for the network configuration
 * procedure in SOAX.
 */

#include <iostream>
#include "./snake_tip_set.h"
#include "./snake_tip.h"
#include "./snake.h"


namespace soax {

SnakeTipSet::SnakeTipSet(SnakeTip *t) {
  Add(t);
}

void SnakeTipSet::Add(SnakeTip *t) {
  tips_.push_back(t);
}

bool SnakeTipSet::Has(const SnakeTip *t) const {
  return std::find(tips_.begin(), tips_.end(), t) != tips_.end();
}

double SnakeTipSet::ComputeDistanceTo(SnakeTip *t) const {
  double min_d = ComputeDistance(tips_.front(), t);
  auto it = tips_.begin() + 1;
  while (it != tips_.end()) {
    double d = ComputeDistance(*it, t);
    if (d < min_d) {
      min_d = d;
    }
    it++;
  }
  return min_d;
}

void SnakeTipSet::Combine(SnakeTipSet *ts) {
  for (auto it = ts->tips_.begin(); it != ts->tips_.end(); ++it) {
    Add(*it);
  }
}

void SnakeTipSet::UpdateCentroid() {
  if (tips_.empty()) return;
  int dim = tips_.front()->snake()->dimension();

  PointType sum = PointType::Zero(dim);
  for (auto it = tips_.begin(); it != tips_.end(); it++) {
    sum += (*it)->GetLocation();
  }
  centroid_ = sum / static_cast<double>(tips_.size());
}


void SnakeTipSet::Configure(int delta, double direction_threshold,
                            double distance_threshold) {
  if (tips_.size() < 2) return;

  SnakeTip *t1 = nullptr;
  SnakeTip *t2 = nullptr;

  double angle = FindSmoothestPair(&t1, &t2, delta, distance_threshold);
  if (angle < direction_threshold) {
    return;
  } else {
    Link(t1, t2);
    RemoveTip(t1);
    RemoveTip(t2);
    Configure(delta, direction_threshold, distance_threshold);
  }
}

void SnakeTipSet::Configure(const SnakeContainer &previous_snakes,
                            int delta, double direction_threshold,
                            double distance_threshold) {
  size_t n = tips_.size();
  if (n < 2) return;

  Matrix<double> distance(n, n, -1.0);
  ComputeAngleDifference(distance, delta, direction_threshold,
                         distance_threshold);
  // std::cout << "Angle Diff Matrix\n" << distance << std::endl;
  Matrix<double> angles = distance;
  Munkres solver1;
  solver1.solve(angles);
  // std::cout << "Assignment (angle) Matrix\n" << angles << std::endl;

  if (!previous_snakes.empty()) {
    Matrix<double> label_diffs(n, n, -1.0);
    ComputeLabelDifference(previous_snakes, label_diffs);
    distance += label_diffs;
    // std::cout << "Distance Matrix\n" << distance << std::endl;
    Munkres solver2;
    solver2.solve(distance);
    // std::cout << "Assignment (all) Matrix\n" << distance << std::endl;
  }

  if (previous_snakes.empty()) {
    if (!LinkTips(angles)) {
      std::cerr << "LinkTips error!" << std::endl;
    }
  } else {
    if (!LinkTips(distance)) {
      UnlinkTips();  // don't utilize temporal info if it leads to linktips error
      if (!LinkTips(angles))
        std::cerr << "LinkTips error!" << std::endl;
    }
  }
}

bool SnakeTipSet::IsJunction(const SnakeContainer &grouped,
                             double threshold) const {
  int num = 0;
  for (auto it = grouped.begin(); it != grouped.end(); ++it) {
    if ((*it)->PassThrough(centroid_, threshold))
      num++;
  }
  return num > 1;
}

void SnakeTipSet::Print(std::ostream &os) const {
  os << "************* SnakeTipSet " << this << " *************\n"
     << "centroid: " << centroid_ << std::endl
     << "Snake tips: " << std::endl;
  for (auto it = tips_.begin(); it != tips_.end(); ++it) {
    (*it)->Print(os);
    os << std::endl;
  }
}

/************************ Private Methods ************************/

double SnakeTipSet::FindSmoothestPair(SnakeTip **t1, SnakeTip **t2,
                                      int delta, double threshold) const {
  double max_angle = 0.0;
  for (auto i = tips_.begin(); i != tips_.end(); ++i) {
    for (auto j = i + 1; j != tips_.end(); ++j) {
      if ((*i)->snake() != (*j)->snake() ||
          (*i)->snake()->GetLength() > threshold) {
        double angle = ComputeAngle(*i, *j, delta);
        if (angle > max_angle && AreTightestLink(*i, *j)) {
          max_angle = angle;
          *t1 = *i;
          *t2 = *j;
        }
      }
    }
  }
  return max_angle;
}

void SnakeTipSet::RemoveTip(const SnakeTip *t) {
  auto it = std::find(tips_.begin(), tips_.end(), t);
  if (it != tips_.end()) {
    tips_.erase(it);
  }
}

void SnakeTipSet::ComputeAngleDifference(Matrix<double> &distance,
                                         int delta, double direction_threshold,
                                         double distance_threshold) {
  for (size_t i = 0; i < tips_.size(); i++) {
    distance(i, i) = 1.0 - direction_threshold / kPi;
    for (size_t j = i + 1; j < tips_.size(); j++) {
      if ((tips_[i]->snake() != tips_[j]->snake() ||  // not the same snake
           // same snake with two joining tips (closed curve)
           tips_[i]->snake()->GetLength() > distance_threshold) &&
          // Are these two tips proper to be linked? Some improper grouping
          // threshold may put two tips of a short segment into the same tip
          // set; thus only the two tips closest to the junction are eligible.
          AreTightestLink(tips_[i], tips_[j])) {
        double angle = ComputeAngle(tips_[i], tips_[j], delta);
        distance(i, j) = distance(j, i) = 1.0 - angle / kPi;
      } else {
        distance(i, j) = distance(j, i) = 1.0;
      }
    }
  }
}

void SnakeTipSet::ComputeLabelDifference(const SnakeContainer &previous_snakes,
                                         Matrix<double> &label_diffs) {
  std::vector<Snake *> close_snakes;
  close_snakes.reserve(tips_.size());
  std::vector<double> diff;
  diff.reserve(tips_.size());

  for (size_t i = 0; i < tips_.size(); i++) {
    double d;
    Snake *s = GetClosestSnake(tips_[i]->snake(), previous_snakes, d);
    close_snakes.push_back(s);
    diff.push_back(d);
  }

  for (size_t i = 0; i < tips_.size(); i++) {
    for (size_t j = i + 1; j < tips_.size(); j++) {
      if (tips_[i]->snake() != tips_[j]->snake()) {
        if (close_snakes[i] && close_snakes[j] &&
            close_snakes[i] == close_snakes[j]) {
          label_diffs(i, j) = label_diffs(j, i) = fabs(diff[i] - diff[j]);
          // std::cout << "(" << i << ", " << j << ") common snake: " << close_snakes[i]
          //           << "\t" << diff[i] << "\t" << diff[j] << std::endl;
          // close_snakes[i]->Print(std::cout);
        }
      }
    }
  }
  // std::cout << "Original label diff matrix\n" << label_diffs << std::endl;

  double max_elem = label_diffs.max();
  if (max_elem > 1.0)
    label_diffs /= max_elem;
  for (size_t i = 0; i < label_diffs.rows(); i++) {
    for (size_t j = 0; j < label_diffs.columns(); j++) {
      if (label_diffs(i, j) < 0)
        label_diffs(i, j) = 1.0;
    }
  }
}

bool SnakeTipSet::LinkTips(const Matrix<double> &assignment) {
  for (size_t i = 0; i < assignment.rows(); i++) {
    for (size_t j = i + 1; j < assignment.columns(); j++) {
      if (assignment(i, j) == 0) {
        if (assignment(j, i) == 0) {
          Link(tips_[i], tips_[j]);
        } else {
          return false;
        }
      }
    }
  }
  return true;
}

void SnakeTipSet::UnlinkTips() {
  for (auto t: tips_)
    t->set_neighbor(nullptr);
}

}  // namespace soax
