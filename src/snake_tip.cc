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
 * This file implements the SOAC tip class for the network configuration
 * procedure in SOAX.
 */

#include "./snake_tip.h"
#include "./snake.h"


namespace soax {
SnakeTip::SnakeTip(Snake *s, bool is_head) :
    snake_(s), is_head_(is_head), neighbor_(nullptr) {}

PointType SnakeTip::GetLocation() const {
  return snake_->GetTip(is_head_);
}

PointType SnakeTip::GetOppositeLocation() const {
  return snake_->GetTip(!is_head_);
}

VectorXd SnakeTip::GetDirection(int delta) const {
  return snake_->GetTipTangent(is_head_, delta);
}

void SnakeTip::Print(std::ostream &os) const {
  os << "******** Snake tip " << this << " ********\n"
     << "snake: " << snake_ << "\nis_head: " << is_head_
     << "\nneighbor: " << neighbor_
     << "\nlocation: " << GetLocation() << std::endl;
  snake_->Print(os);
}

double ComputeDistance(const SnakeTip *t1, const SnakeTip *t2) {
  return (t1->GetLocation() - t2->GetLocation()).norm();
}

double ComputeAngle(const SnakeTip *t1, const SnakeTip *t2, int delta) {
  return acos(t1->GetDirection(delta).dot(t2->GetDirection(delta)));
}

void Link(SnakeTip *t1, SnakeTip *t2) {
  t1->set_neighbor(t2);
  t2->set_neighbor(t1);
}

bool AreLinked(SnakeTip *t1, SnakeTip *t2) {
  return t1->neighbor() == t2 && t2->neighbor() == t1;
}

bool AreTightestLink(const SnakeTip *t1, const SnakeTip *t2) {
  if (t1->snake() == t2->snake()) return true;
  PointType p1 = t1->GetLocation();
  PointType p2 = t2->GetLocation();
  PointType p1_op = t1->GetOppositeLocation();
  PointType p2_op = t2->GetOppositeLocation();

  double dist1 = (p1 - p2).norm();
  double dist2 = (p1_op - p2_op).norm();
  double dist3 = (p1_op - p2).norm();
  double dist4 = (p1 - p2_op).norm();

  return dist1 <= dist2 && dist1 <= dist3 && dist1 <= dist4;
}

}  // namespace soax
