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
 * This file implements Stretching Open Active Contour (SOAC) and
 * closed-curve parametric Active Contour.
 */


#include "./snake.h"
#include <iostream>
#include "./interpolator.h"
#include "./util.h"
#include "./snake_parameters.h"

namespace soax {
Snake::Snake() :
    dimension_(3), image_interp_(nullptr), gradient_interp_(nullptr),
    parameters_(nullptr), open_(true) {}

Snake::Snake(const std::vector<double> &coordinates, int dim,
             const Interpolator *image_interp,
             const Interpolator *gradient_interp,
             SnakeParameters *p, bool open, int id) :
    vertices_(coordinates.size() / dim, dim), dimension_(dim),
    image_interp_(image_interp), gradient_interp_(gradient_interp),
    parameters_(p), open_(open), id_(id) {
  CopyCoordinates(coordinates);
}

Snake::Snake(const MatrixXd &coordinates, int dim,
             const Interpolator *image_interp,
             const Interpolator *gradient_interp,
             SnakeParameters *p, bool open, int id) :
    vertices_(coordinates), dimension_(dim),
    image_interp_(image_interp), gradient_interp_(gradient_interp),
    parameters_(p), open_(open), id_(id) {}

Snake::Snake(const std::deque<MatrixXd> &blocks, int dim,
             const Interpolator *image_interp,
             const Interpolator *gradient_interp,
             SnakeParameters *p, bool open, int id) :
    dimension_(dim), image_interp_(image_interp),
    gradient_interp_(gradient_interp), parameters_(p), open_(open), id_(id) {
  CopyCoordinatesFromBlocks(blocks);
}

void Snake::ComputeCentroid() {
  centroid_ = vertices_.colwise().sum() / vertices_.rows();
}

void Snake::GetVertex(int index, double *vertex) const {
  for (int i = 0; i < dimension_; i++) {
    vertex[i] = vertices_(index, i);
  }
}

ConstRowVec Snake::GetVertex(int index) const {
  return vertices_.row(index);
}

ConstRowVec Snake::GetTip(bool is_head) const {
  if (is_head)
    return GetVertex(0);
  else
    return GetVertex(vertices_.rows() - 1);
}

bool Snake::Evolve(const SnakeContainer &snakes) {
  while (iterations_ < parameters_->maximum_iterations()) {
    if (!Resample())
      return false;

    if (iterations_ % parameters_->check_period() == 0) {
      if (IsConverged()) break;
    }

    if (SelfIntersect())
      return false;

    if (!CheckHeadOverlap(snakes))
      return false;
    if (!CheckTailOverlap(snakes))
      return false;

    IterateOnce();
  }
  return CheckBodyOverlap(snakes);
}

bool Snake::EvolveWithTipFixed(int max_iter) {
  assert(iterations_ == 0);
  fixed_head_ = vertices_.row(0);
  fixed_tail_ = vertices_.row(vertices_.rows() - 1);

  while (iterations_ < max_iter) {
    if (iterations_ % parameters_->check_period() == 0) {
      if (IsConverged()) break;
    }

    IterateOnce();
    if (!Resample())
      return false;
  }
  return true;
}

bool Snake::EvolveFinal(const SnakeContainer &snakes, double length) {
  fixed_head_.resize(0);
  fixed_tail_.resize(0);
  if (iterations_ == 0)
    iterations_ = parameters_->check_period();
  while (iterations_ < parameters_->maximum_iterations()) {
    if (iterations_ % parameters_->check_period() == 0) {
      if (IsConverged())
        return true;
    }

    if (!CheckHeadOverlap(snakes))
      return false;
    if (!CheckTailOverlap(snakes))
      return false;
    IterateOnce();
    if (!Resample(length))
      return false;
  }
  return true;
}

void Snake::Print(std::ostream & os) const {
  // os.precision(12);
  os << "======== snake " << this << " ======== " << std::endl;
  os << "id: " << id_ << std::endl;
  os << "dimension: " << dimension_ << std::endl;
  os << "open: " << open_ << std::endl;
  os << "spacing: " << spacing_ << std::endl;
  os << "size: " << vertices_.rows() << std::endl;
  os << "length: " << GetLength() << std::endl;
  os << "iterations: " << iterations_ << std::endl;
  os << "fixed head: " << fixed_head_ << std::endl;
  os << "fixed tail: " << fixed_tail_ << std::endl;
  os << "head hooked snake: " << head_hooked_snake_ << std::endl;
  os << "tail hooked snake: " << tail_hooked_snake_ << std::endl;
  os << "head hooked index: " << head_hooked_index_ << std::endl;
  os << "tail hooked index: " << tail_hooked_index_ << std::endl;
  os << "# of subsnakes: " << subsnakes_.size() << std::endl;
  if (previous_snake_)
    os << "previous snake id: " << previous_snake_->id() << std::endl;
  os << "~~~~~~~~ Current vertices ~~~~~~~~" << std::endl;
  os << vertices_ << std::endl;
  // os << "~~~~~~~~ Last vertices ~~~~~~~~" << std::endl;
  // os << last_vertices_ << std::endl;
}

void Snake::UpdateHookedIndices() {
  if (head_hooked_snake_) {
    head_hooked_snake_->junction_indices_.push_back(head_hooked_index_);
  }

  if (tail_hooked_snake_) {
    tail_hooked_snake_->junction_indices_.push_back(tail_hooked_index_);
  }
}

void Snake::CopySegments(SnakeContainer *segments) {
  int start = 0;
  if (!junction_indices_.empty()) {
    std::sort(junction_indices_.begin(), junction_indices_.end());
    for (size_t i = 0; i < junction_indices_.size(); i++) {
      InitializeFromPart(start, junction_indices_[i] + 1, true, segments);
      start = junction_indices_[i];
    }
  }
  InitializeFromPart(start, vertices_.rows(), true, segments);
}

void Snake::CopyCoordinatesOut(std::deque<MatrixXd> *blocks,
                               bool prepend, bool reverse) const {
  if (prepend) {
    if (reverse) {
      blocks->push_front(vertices_.colwise().reverse());
    } else {
      blocks->push_front(vertices_);
    }
  } else {
    if (reverse) {
      blocks->push_back(vertices_.colwise().reverse());
    } else {
      blocks->push_back(vertices_);
    }
  }
}

void Snake::SaveVertexState() {
  original_vertices_ = vertices_;
}

double Snake::ComputeVertexChange() {
  return 0.0;
}

bool Snake::VertexChanged(double threshold) {
  if (vertices_.rows() != original_vertices_.rows())
    return true;

  for (int i = 0; i < vertices_.rows(); i++) {
    double d = (vertices_.row(i) - original_vertices_.row(i)).norm();
    if (d > threshold)
      return true;
  }
  return false;
}

/************************ Private Methods ************************/

void Snake::CopyCoordinates(const std::vector<double> &coordinates) {
  for (int i = 0; i < vertices_.rows(); i++) {
    for (int j = 0; j < dimension_; j++) {
      vertices_(i, j) = coordinates[dimension_ * i + j];
    }
  }
}

bool Snake::IsConverged() {
  if (vertices_.rows() != last_vertices_.rows()) {
    last_vertices_ = vertices_;
    return false;
  }

  for (int i = 0; i < vertices_.rows(); ++i) {
    double d = (vertices_.row(i) - last_vertices_.row(i)).norm();
    if (d > parameters_->change_threshold()) {
      last_vertices_ = vertices_;
      return false;
    }
  }

  return true;
}

bool Snake::SelfIntersect() {
  if (!open_)
    return false;

  const int min_loop_size = 20;  // todo: remove it
  if (vertices_.rows() <= min_loop_size)
    return false;

  for (int i = 0; i < vertices_.rows() - min_loop_size; i++) {
    for (int j = i + min_loop_size; j < vertices_.rows(); j++) {
      double d = (vertices_.row(i) - vertices_.row(j)).norm();
      if (d < 1.0 && !TipsStopAtSameLocation()) {
        InitializeFromPart(0, i, true, &subsnakes_);
        InitializeFromPart(j, vertices_.rows(), true, &subsnakes_);
        InitializeFromPart(i, j, false, &subsnakes_);
        return true;
      }
    }
  }

  return false;
}

bool Snake::TipsStopAtSameLocation() const {
  if (head_hooked_snake_ && tail_hooked_snake_) {
    if ((fixed_head_ - fixed_tail_).norm() < 1.0)
      return true;
  }
  return false;
}

void Snake::InitializeFromPart(int start, int end, bool is_open,
                               SnakeContainer *snakes) {
  Snake *s = new Snake(vertices_.block(start, 0, end - start, dimension_),
                       dimension_, image_interp_, gradient_interp_,
                       parameters_, is_open, parameters_->GetSnakeId());
  if (s->Resample()) {
    snakes->push_back(s);
  } else {
    delete s;
  }
}

bool Snake::CheckHeadOverlap(const SnakeContainer &snakes) {
  if (snakes.empty())
    return true;

  int first_detach = 0;
  int start = 0;

  if (HeadIsFixed()) {
    start += static_cast<int>(parameters_->overlap_threshold() / spacing_);
  }

  first_detach = FindFirstDetachFromHead(start, snakes);
  if (first_detach != start) {
    if (first_detach == vertices_.rows()) {
      return false;
    } else {
      FindHookedSnake(first_detach - 1, snakes,
                      &head_hooked_snake_, &head_hooked_index_);
      fixed_head_ = head_hooked_snake_->vertices_.row(head_hooked_index_);
      vertices_.row(0) = fixed_head_;
      int n_rows = vertices_.rows() - first_detach + 1;
      vertices_.block(1, 0, vertices_.rows() - first_detach, dimension_) =
          vertices_.block(first_detach, 0, vertices_.rows() - first_detach,
                          dimension_);
      vertices_.conservativeResize(n_rows, Eigen::NoChange);
      if (!open_)
        open_ = true;
      return Resample();
    }
  }
  return true;
}

bool Snake::CheckTailOverlap(const SnakeContainer &snakes) {
  if (snakes.empty())
    return true;

  int first_detach = vertices_.rows() - 1;
  int start = first_detach;

  if (TailIsFixed()) {
    start -= static_cast<int>(parameters_->overlap_threshold() / spacing_);
  }

  first_detach = FindFirstDetachFromTail(start, snakes);
  if (first_detach != start) {
    if (first_detach < 0) {
      return false;
    } else {
      FindHookedSnake(first_detach + 1, snakes,
                      &tail_hooked_snake_, &tail_hooked_index_);
      fixed_tail_ = tail_hooked_snake_->vertices_.row(tail_hooked_index_);
      vertices_.row(first_detach + 1) = fixed_tail_;
      vertices_.conservativeResize(first_detach + 2, Eigen::NoChange);
      if (!open_)
        open_ = true;
      return Resample();
    }
  }
  return true;
}

int Snake::FindFirstDetachFromHead(int start, const SnakeContainer &snakes) {
  int i = start;
  while (i < vertices_.rows() && VertexOverlap(i, snakes)) {
    i++;
  }
  return i;
}

int Snake::FindFirstDetachFromTail(int start, const SnakeContainer &snakes) {
  int i = start;
  while (i >= 0 && VertexOverlap(i, snakes)) {
    i--;
  }
  return i;
}

bool Snake::VertexOverlap(int index, const SnakeContainer &snakes) {
  for (SnakeContainer::const_iterator it = snakes.begin();
       it != snakes.end(); ++it) {
    if ((*it)->PassThrough(vertices_.row(index),
                           parameters_->overlap_threshold()))
      return true;
  }
  return false;
}

bool Snake::PassThrough(const RowVec &point, double threshold) const {
  for (int i = 0; i < vertices_.rows(); i++) {
    double d = (point - vertices_.row(i)).norm();
    if (d < threshold)
      return true;
  }
  return false;
}

bool Snake::PassThrough(const VectorXd &point, double threshold) const {
  for (int i = 0; i < vertices_.rows(); i++) {
    VectorXd vertex = vertices_.row(i);
    double d = (point - vertex).norm();
    if (d < threshold)
      return true;
  }
  return false;
}


void Snake::FindHookedSnake(int last_touch,
                            const SnakeContainer &snakes,
                            Snake **s, int *index) {
  double min_d = kPlusInfinity;
  for (SnakeContainer::const_iterator it = snakes.begin();
       it != snakes.end(); ++it) {
    int i;
    double d = (*it)->FindClosestIndexTo(vertices_.row(last_touch), &i);
    if (d < min_d) {
      min_d = d;
      *index = i;
      *s = *it;
    }
  }
}

double Snake::FindClosestIndexTo(const RowVec &point, int *index) const {
  double min_d = (point - vertices_.row(0)).norm();
  *index = 0;

  for (int i = 1; i < vertices_.rows(); ++i) {
    double d = (point - vertices_.row(i)).norm();
    if (d < min_d) {
      min_d = d;
      *index = i;
    }
  }
  return min_d;
}

bool Snake::CheckBodyOverlap(const SnakeContainer &snakes) {
  if (snakes.empty())
    return true;

  bool last_is_overlap = true;
  int overlap_start = vertices_.rows();
  int overlap_end = vertices_.rows();

  for (int i = 0; i < vertices_.rows(); i++) {
    bool overlap = VertexOverlap(i, snakes);
    if (overlap && !last_is_overlap) {
      overlap_start = i;
    } else if (!overlap && last_is_overlap && i > overlap_start) {
      overlap_end = i;
      break;
    }
    last_is_overlap = overlap;
  }

  if (overlap_end != vertices_.rows()) {
    InitializeFromPart(overlap_end - 1, vertices_.rows(), true, &subsnakes_);
    InitializeFromPart(0, overlap_start + 1, true, &subsnakes_);
    return false;
  }
  return true;
}

void Snake::IterateOnce() {
  // Compute right hand side vector
  MatrixXd rhs = parameters_->gamma() * vertices_;
  AddExternalForce(&rhs);
  AddStretchingForce(&rhs);
  SolveLinearSystem(rhs);

  // const double padding = 0.5;
  // KeepVerticesInsideImage(padding);
  // CheckNaN();

  if (HeadIsFixed())
    vertices_.row(0) = fixed_head_;
  if (TailIsFixed())
    vertices_.row(vertices_.rows() - 1) = fixed_tail_;

  iterations_++;
}

void Snake::SolveLinearSystem(const MatrixXd &rhs) {
  std::vector<SpEntry> entries;
  const int n = vertices_.rows();
  entries.reserve(n * kMinimumEvolvingSize);
  if (open_) {
    FillEntriesOpen(&entries);
  } else {
    FillEntriesClosed(&entries);
  }
  SpMatrix A(n, n);
  A.setFromTriplets(entries.begin(), entries.end());
  Eigen::SimplicialCholesky<SpMatrix> chol(A);
  vertices_ = chol.solve(rhs);
}

bool Snake::CheckNaN() {
  for (int i = 0; i < vertices_.rows(); i++) {
    for (int j = 0; j < dimension_; j++) {
      if (std::isnan(vertices_(i, j))) {
        Print(std::cout);
        return true;
      }
    }
  }
  return false;
}

void Snake::KeepVerticesInsideImage(double padding) {
  for (int i = 0; i < vertices_.rows(); i++) {
    for (int j = 0; j < dimension_; j++) {
      double bound = image_interp_->GetImageExtent(j);
      if (vertices_(i, j) < padding) {
        vertices_(i, j) = padding;
      } else if (vertices_(i, j) > bound - padding) {
        vertices_(i, j) = bound - padding;
      }
    }
  }
}

void Snake::AddExternalForce(MatrixXd *rhs) {
  for (int i = 0; i < vertices_.rows(); i++) {
    VectorXd gvec(dimension_);
    if (gradient_interp_->Interpolate(vertices_.row(i), &gvec)) {
      rhs->row(i) += parameters_->external_factor() * gvec;
    }
  }
}

void Snake::AddStretchingForce(MatrixXd *rhs) {
  if (!HeadIsFixed()) {
    double local_stretch = ComputeLocalStretch(true);
    VectorXd head_tangent = GetHeadTangent(parameters_->delta());
    double stretch = parameters_->stretch_factor() * local_stretch;

    if (parameters_->damp_z() && dimension_ == 3)
      stretch *= exp(-fabs(head_tangent(2)));

    rhs->row(0) += stretch * head_tangent;
  }

  if (!TailIsFixed()) {
    double local_stretch = ComputeLocalStretch(false);
    VectorXd tail_tangent = GetTailTangent(parameters_->delta());
    double stretch = parameters_->stretch_factor() * local_stretch;

    if (parameters_->damp_z() && dimension_ == 3)
      stretch *= exp(-fabs(tail_tangent(2)));

    rhs->row(rhs->rows() - 1) += stretch * tail_tangent;
  }
}

bool Snake::HeadIsFixed() {
  return fixed_head_.size() != 0;
}

bool Snake::TailIsFixed() {
  return fixed_tail_.size() != 0;
}

VectorXd Snake::GetTipTangent(bool is_head, int delta) const {
  if (is_head)
    return GetHeadTangent(delta);
  else
    return GetTailTangent(delta);
}

VectorXd Snake::GetHeadTangent(int delta) const {
  if (vertices_.rows() > delta)
    return (vertices_.row(0) - vertices_.row(delta)).normalized();
  else
    return (vertices_.row(0) -
            vertices_.row(vertices_.rows() - 1)).normalized();
}

VectorXd Snake::GetTailTangent(int delta) const {
  if (vertices_.rows() > delta)
    return (vertices_.row(vertices_.rows() - 1) -
            vertices_.row(vertices_.rows() -1 - delta)).normalized();
  else
    return (vertices_.row(vertices_.rows() - 1) -
            vertices_.row(0)).normalized();
}

VectorXd Snake::GetVertexTangent(int index, int delta) const {
  if (index < delta / 2) {
    return (vertices_.row(0) -
            vertices_.row(index + (delta + 1) / 2)).normalized();
  } else if (index + (delta + 1) /2 >= vertices_.rows()) {
    return (vertices_.row(index - delta / 2) -
            vertices_.row(vertices_.rows() - 1)).normalized();
  } else {
    return (vertices_.row(index - delta / 2) -
            vertices_.row(index + (delta + 1) / 2)).normalized();
  }
}

double Snake::ComputeLocalStretch(bool is_head) {
  double p[3] = {0.0, 0.0, 0.0};
  if (is_head) {
    GetVertex(0, p);
  } else {
    GetVertex(vertices_.rows() - 1, p);
  }

  double fg = .0;
  if (image_interp_->Interpolate(p, &fg)) {
    if (fg < parameters_->minimum_foreground() ||
        fg > parameters_->maximum_foreground())
      return 0.0;

    double bg = 0.0;
    if (dimension_ == 2) {
      if (!EstimateLocalBackgroundIntensity2D(is_head, &bg))
        return 0.0;
    } else if (dimension_ == 3) {
      if (!EstimateLocalBackgroundIntensity3D(is_head, &bg))
        return 0.0;
    }

    if (fg > bg)
      return 1.0 - bg / fg;
    else
      return 0.0;
  }
  return 0.0;
}

bool Snake::EstimateLocalBackgroundIntensity2D(bool is_head, double *bg) {
  Vector2d normal = GetTipTangent(is_head, parameters_->delta());
  Vector2d vertex = is_head ? vertices_.row(0) :
      vertices_.row(vertices_.rows() - 1);
  std::vector<double> bgs;

  double d = parameters_->radial_near();
  const double radial_step = 1.0;
  while (d < parameters_->radial_far()) {
    double pod[3] = {0.0, 0.0, 0.0};
    pod[0] = ComputePodX(vertex(0), normal, d, true);
    pod[1] = ComputePodY(vertex(1), normal, d, false);

    double intensity = 0.0;
    if (image_interp_->Interpolate(pod, &intensity)) {
      bgs.push_back(intensity);
    }

    pod[0] = ComputePodX(vertex(0), normal, d, false);
    pod[1] = ComputePodY(vertex(1), normal, d, true);

    if (image_interp_->Interpolate(pod, &intensity)) {
      bgs.push_back(intensity);
    }

    d += radial_step;
  }

  if (bgs.empty()) {
    return false;
  } else {
    *bg = Mean(bgs);
    return true;
  }
}

double Snake::ComputePodX(double x, const Vector2d &tvec,
                          double dist, bool plus_root) const {
  if (std::abs(tvec[0]) < kEpsilon) {
    if (plus_root)
      return x + dist;
    else
      return x - dist;
  } else {
    double frac = tvec[1] / tvec[0];
    if (plus_root)
      return x + frac * dist / std::sqrt(1 + frac * frac);
    else
      return x - frac * dist / std::sqrt(1 + frac * frac);
  }
}

double Snake::ComputePodY(double y, const Vector2d &tvec,
                          double dist, bool plus_root) const {
  if (std::abs(tvec[0]) < kEpsilon) {
    return y;
  } else {
    double frac = tvec[1] / tvec[0];
    if (plus_root)
      return y + dist / std::sqrt(1 + frac * frac);
    else
      return y - dist / std::sqrt(1 + frac * frac);
  }
}

bool Snake::EstimateLocalBackgroundIntensity3D(bool is_head, double *bg) {
  Vector3d normal = GetTipTangent(is_head, parameters_->delta());
  Vector3d vertex = is_head ? vertices_.row(0) :
      vertices_.row(vertices_.rows() - 1);
  Vector3d projection = Vector3d::UnitZ() - normal(2) * normal;
  Vector3d long_axis = Vector3d::UnitX();
  Vector3d short_axis = Vector3d::UnitY();
  if (projection.norm() > kEpsilon) {
    long_axis = projection.normalized();
    short_axis = long_axis.cross(normal);
    short_axis.normalize();
  }
  std::vector<double> bgs;
  const double angle_step = 2 * kPi / parameters_->number_of_sectors();
  int d = parameters_->radial_near();
  while (d < parameters_->radial_far()) {
    for (int s = 0; s < parameters_->number_of_sectors(); s++) {
      double angle = s * angle_step;
      Vector3d v = d * (std::cos(angle) * long_axis +
                        std::sin(angle) * short_axis);
      v(2) *= parameters_->zspacing();
      Vector3d p = vertex + v;
      double intensity = 0.0;
      if (image_interp_->Interpolate(p, &intensity)) {
        if (intensity > parameters_->minimum_foreground())
          bgs.push_back(intensity);
      }
    }
    d++;
  }

  if (bgs.empty()) {
    return false;
  } else {
    *bg = Mean(bgs);
    return true;
  }
}

void Snake::FillEntriesOpen(std::vector<SpEntry> *entries) {
  const double spacing_squared = spacing_ * spacing_;
  const double alpha = parameters_->alpha() / spacing_squared;
  const double beta = parameters_->beta() /
                      (spacing_squared * spacing_squared);
  // const double alpha = parameters_->alpha();
  // const double beta = parameters_->beta();
  const double gamma = parameters_->gamma();
  const double diag0 = 2 * alpha + 6 * beta + gamma;
  const double diag1 = -alpha - 4 * beta;
  const int order = vertices_.rows();
  // main diagonal
  entries->push_back(SpEntry(0, 0, alpha + beta + gamma));
  entries->push_back(SpEntry(1, 1, 2 * alpha + 5 * beta + gamma));
  for (int i = 2; i < order - 2; i++)
    entries->push_back(SpEntry(i, i, diag0));
  entries->push_back(SpEntry(order - 2, order - 2,
                            2 * alpha + 5 * beta + gamma));
  entries->push_back(SpEntry(order - 1, order - 1, alpha + beta + gamma));

  // +1/-1 diagonal
  entries->push_back(SpEntry(0, 1, -alpha - 2 * beta));
  entries->push_back(SpEntry(1, 0, -alpha - 2 * beta));
  for (int i = 1; i < order - 2; i++) {
    entries->push_back(SpEntry(i, i+1, diag1));
    entries->push_back(SpEntry(i+1, i, diag1));
  }
  entries->push_back(SpEntry(order - 2, order - 1, -alpha - 2 * beta));
  entries->push_back(SpEntry(order - 1, order - 2, -alpha - 2 * beta));

  // +2/-2 diagonal
  for (int i = 2; i < order; ++i) {
    entries->push_back(SpEntry(i, i-2, beta));
    entries->push_back(SpEntry(i-2, i, beta));
  }
}

void Snake::FillEntriesClosed(std::vector<SpEntry> *entries) {
  const double spacing_squared = spacing_ * spacing_;
  const double alpha = parameters_->alpha() / spacing_squared;
  const double beta = parameters_->beta() /
                      (spacing_squared * spacing_squared);
  // const double alpha = parameters_->alpha();
  // const double beta = parameters_->beta();
  const double gamma = parameters_->gamma();
  const double diag0 = 2 * alpha + 6 * beta + gamma;
  const double diag1 = -alpha - 4 * beta;
  const int order = vertices_.rows();
  for (int i = 0; i < order; ++i) {
    entries->push_back(SpEntry(i, (i + order - 2) % order, beta));
    entries->push_back(SpEntry(i, (i + order - 1) % order, diag1));
    entries->push_back(SpEntry(i, i, diag0));
    entries->push_back(SpEntry(i, (i + order + 1) % order, diag1));
    entries->push_back(SpEntry(i, (i + order + 2) % order, beta));
  }
}

bool Snake::Resample(double min_length) {
  if (vertices_.rows() < 2)
    return false;

  double length = GetLength();
  if (length < min_length)
    return false;

  int num_vertices = ComputeSize(length, parameters_->spacing());
  if (num_vertices < kMinimumEvolvingSize) {
    if (iterations_ <= parameters_->check_period())
      num_vertices = kMinimumEvolvingSize;
    else
      return false;
  }

  // double spacing = parameters_->spacing();
  // if (iterations_ <= parameters_->check_period())
  //   spacing = 0.25;
  // if (length < spacing) return false;
  // int num_vertices = ComputeSize(length, spacing);
  // if (num_vertices < kMinimumEvolvingSize)
  //   return false;

  spacing_ = length / (num_vertices - 1);
  InterpolateVertices(num_vertices);
  return true;
}

double Snake::GetLength() const {
  double length = 0.0;
  for (int i = 0; i < vertices_.rows() - 1; i++) {
    length += (vertices_.row(i + 1) - vertices_.row(i)).norm();
  }
  return length;
  // return spacing_ * (vertices_.rows() - 1);
}

double Snake::DistanceTo(ConstRowVec p) const {
  assert(vertices_.rows() > 0);
  double min_d = (p - vertices_.row(0)).norm();
  for (int i = 1; i < vertices_.rows(); i++) {
    double d = (p - vertices_.row(i)).norm();
    if (d < min_d)
      min_d = d;
  }
  return min_d;
}

double Snake::TranslatedDistanceTo(const RowVectorXd &p) const {
  assert(vertices_.rows() > 0);
  assert(centroid_.size() > 0);
  double min_d = (p - (vertices_.row(0) - centroid_)).norm();
  for (int i = 1; i < vertices_.rows(); i++) {
    double d = (p - (vertices_.row(i) - centroid_)).norm();
    if (d < min_d)
      min_d = d;
  }
  return min_d;
}


double Snake::EuclideanAndAngleDistanceTo(ConstRowVec p,
                                          VectorXd tangent) const {
  assert(vertices_.rows() > 0);
  int min_index = 0;
  double min_d = (p - vertices_.row(0)).norm();
  for (int i = 1; i < vertices_.rows(); i++) {
    double d = (p - vertices_.row(i)).norm();
    if (d < min_d) {
      min_d = d;
      min_index = i;
    }
  }

  VectorXd v = GetVertexTangent(min_index, 2);
  return min_d * exp(-fabs(tangent.dot(v)));
}

int Snake::ComputeSize(double length, double spacing) const {
  double ncells = length / spacing;
  double residue = length - std::floor(ncells) * spacing;
  if (residue > spacing / 2)
    return static_cast<int>(std::ceil(ncells)) + 1;
  else
    return static_cast<int>(std::floor(ncells)) + 1;
}

void Snake::InterpolateVertices(int size) {
  MatrixXd new_vertices(size, dimension_);
  new_vertices.row(0) = vertices_.row(0);
  new_vertices.row(size - 1) = vertices_.row(vertices_.rows() - 1);

  for (int j = 0; j < dimension_; j++) {
    PairContainer partial_sum;
    double length = 0.0;
    int index = 0;
    while (true) {
      partial_sum.push_back(std::make_pair(length, vertices_(index, j)));
      if (index == vertices_.rows() - 1)
        break;
      length += (vertices_.row(index + 1) - vertices_.row(index)).norm();
      index++;
    }

    auto upper = partial_sum.begin();
    for (int i = 1; i < size - 1; i++) {
      upper = std::lower_bound(upper, partial_sum.end(),
                               std::make_pair(spacing_ * i, 0.0));
      auto lower = upper - 1;
      new_vertices(i, j) = upper->second + (lower->second - upper->second) *
                           (spacing_ * i - upper->first) /
                           (lower->first - upper->first);
    }
  }

  vertices_ = new_vertices;
}

void Snake::CopyCoordinatesFromBlocks(const std::deque<MatrixXd> &blocks) {
  int nrows = GetNumberOfRows(blocks);
  vertices_.resize(nrows, dimension_);
  int row = 0;
  for (auto it = blocks.begin(); it != blocks.end(); ++it) {
    vertices_.block(row, 0, it->rows(), it->cols()) = *it;
    row += it->rows();
  }
}

int Snake::GetNumberOfRows(const std::deque<MatrixXd> &blocks) const {
  int nrows = 0;
  for (auto it = blocks.begin(); it != blocks.end(); ++it) {
    nrows += it->rows();
  }
  return nrows;
}


bool IsShorter(const Snake *s1, const Snake *s2) {
  return s1->GetLength() < s2->GetLength();
}

bool IsLonger(const Snake *s1, const Snake *s2) {
  return s1->GetLength() > s2->GetLength();
}

/**
 * Assume the centroid of snake has been updated.
 */
double ComputeCurveDistance(const Snake *s1, const Snake *s2) {
  std::vector<double> distances;
  distances.reserve(s1->GetSize() + s2->GetSize());

  for (size_t i = 0; i < s1->GetSize(); i++) {
    distances.push_back(s2->DistanceTo(s1->GetVertex(static_cast<int>(i))));
    // distances.push_back(s2->TranslatedDistanceTo(
    //     s1->GetVertex(i) - s1->centroid()));
  }

  for (size_t i = 0; i < s2->GetSize(); i++) {
    distances.push_back(s1->DistanceTo(s2->GetVertex(static_cast<int>(i))));
    // distances.push_back(s1->TranslatedDistanceTo(
    //     s2->GetVertex(i) - s2->centroid()));
  }
  return Mean(distances);
}

double ComputeOneWayCurveDistance(const Snake *snake, const Snake *target) {
  std::vector<double> distances;
  distances.reserve(snake->GetSize());
  for (size_t i = 0; i < snake->GetSize(); i++) {
    VectorXd tangent = snake->GetVertexTangent(static_cast<int>(i), 2);
    distances.push_back(target->EuclideanAndAngleDistanceTo(
        snake->GetVertex(static_cast<int>(i)), tangent));
    // distances.push_back(target->DistanceTo(
    //     snake->GetVertex(i)));
  }

  return Mean(distances);
}

Snake * GetClosestSnake(const Snake *snake, const SnakeContainer &snakes,
                        double &min_dist) {
  min_dist = kPlusInfinity;
  Snake * close = nullptr;
  for (Snake * s : snakes) {
    double d = ComputeOneWayCurveDistance(snake, s);
    if (d < min_dist) {
      min_dist = d;
      close = s;
    }
  }
  return close;
}

}  // namespace soax
