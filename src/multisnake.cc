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
 */

#include "include/multisnake.h"
#include <iostream>
#include <limits>
#include <QApplication>
#include "vtkImageData.h"
#include "include/gradient_calculator.h"
#include "include/snake.h"
#include "include/interpolator.h"
#include "include/image.h"
#include "include/snake_parameters.h"
#include "include/snake_tip.h"


namespace soax {

Multisnake::Multisnake() : image_(nullptr),
                           gradient_calc_(new GradientCalculator),
                           image_interp_(nullptr),
                           gradient_interp_(nullptr),
                           snake_parameters_(new SnakeParameters) {}

Multisnake::~Multisnake() {
  Reset();
  delete gradient_calc_;
  delete snake_parameters_;
}

void Multisnake::set_image(vtkImageData *image) {
  image_ = image;
  DeleteImageInterpolator();
  image_interp_ = new Interpolator(image);
}

bool Multisnake::SaveInitialSnakes(const QString &filename,
                                   const QString &image_path) {
  std::ofstream outfile;
  outfile.open(filename.toStdString().c_str());
  if (!outfile.is_open()) {
    std::cerr << "Couldn't open file: " << filename.toStdString()
              << std::endl;
    return false;
  }

  outfile << "image\t" << image_path.toStdString() << std::endl;

  SaveSnakeCoordinates(initial_snakes_, outfile);
  outfile.close();
  path_ = filename;
  return true;
}

bool Multisnake::SaveConvergedSnakes(const QString &filename,
                                     const QString &image_path) {
  std::ofstream outfile;
  outfile.open(filename.toStdString().c_str());
  if (!outfile.is_open()) {
    std::cerr << "Couldn't open file: " << filename.toStdString()
              << std::endl;
    return false;
  }

  outfile << "image\t" << image_path.toStdString() << std::endl;
  outfile << snake_parameters_->ToString();
  int dim = image_->GetDataDimension();
  outfile << "dimension\t" << dim << std::endl;
  for (size_t i = 0; i < converged_snake_sequence_.size(); i++) {
    SaveSnakeCoordinates(converged_snake_sequence_[i], outfile);
    if (i < junction_sequence_.size())
      PrintPointContainer(junction_sequence_[i], outfile, dim);
    outfile << "$frame " << i << std::endl;
  }
  SaveConvergedSnakeTrack(outfile);
  outfile.close();
  path_ = filename;
  return true;
}

void Multisnake::SaveConvergedSnakes(size_t index, std::ostream &os,
                                     int dim) {
  SaveSnakeCoordinates(converged_snake_sequence_[index], os);
  if (index < junction_sequence_.size())
    PrintPointContainer(junction_sequence_[index], os, dim);
  os << "$frame " << index << std::endl;
}

void Multisnake::SaveConvergedSnakeTrack(std::ostream &os) const {
  os << "Tracks" << std::endl;
  for (auto t : converged_snake_track_) {
    os << t << std::endl;
  }
  // for (auto it = converged_snake_track_.begin();
  //      it != converged_snake_track_.end(); it++) {
  //   for (size_t i = 0; i < it->size(); i++) {
  //     os << (*it)[i] << " ";
  //   }
  //   os << std::endl;
  // }
}

bool Multisnake::LoadConvergedSnakes(const QString &filename) {
  junction_sequence_.clear();
  converged_snake_track_.clear();
  return LoadSnakes(filename, &converged_snake_sequence_,
                    &junction_sequence_, true);
}

bool Multisnake::LoadComparingSnakes(const QString &filename) {
  comparing_junction_sequence_.clear();
  return LoadSnakes(filename, &comparing_snake_sequence_,
                    &comparing_junction_sequence_, false);
}

bool Multisnake::LoadTamaraSnakes(const QString &filename) {
  std::ifstream infile(filename.toStdString().c_str());
  if (!infile.is_open()) {
    std::cerr << "Couldn't open snake file: " << filename.toStdString()
              << std::endl;
    return false;
  }
  double offset[3] = {40, 70, 40};
  std::string line;
  unsigned new_curve_id = 0;
  unsigned old_curve_id = 0;
  SnakeContainer snakes;
  std::vector<double> coordinates;
  unsigned point_id = 0;

  while (std::getline(infile, line)) {
    std::istringstream buffer(line);
    double x, y, z;
    buffer >> new_curve_id >> point_id >> x >> y >> z;
    if (new_curve_id != old_curve_id) {
      if (coordinates.size() > 2) {
        Snake *s = new Snake(coordinates, 3, image_interp_, gradient_interp_,
                             snake_parameters_, true);
        if (s->Resample())
          snakes.push_back(s);

        coordinates.clear();
      }
      old_curve_id = new_curve_id;
    }

    coordinates.push_back(x + offset[0]);
    coordinates.push_back(y + offset[1]);
    coordinates.push_back(z + offset[2]);
  }
  infile.close();
  if (coordinates.size() > 2) {
    Snake *s = new Snake(coordinates, 3, image_interp_, gradient_interp_,
                         snake_parameters_, true);
    if (s->Resample())
      snakes.push_back(s);

    coordinates.clear();
  }
  converged_snake_sequence_.push_back(snakes);
  return true;
}

void Multisnake::Initialize() {
  DeleteSnakes(&initial_snakes_);
  ComputeGradient();
  if (snake_parameters_->Valid()) {
    int dim = image_->GetDataDimension();
    vtkImageData *ridge_image = InitializeFlagImage(dim);
    vtkImageData *candidate_image = InitializeFlagImage(dim);
    ScanGradient(ridge_image, dim);
    GenerateCandidates(ridge_image, candidate_image, dim);
    LinkCandidates(candidate_image, dim);
    ridge_image->Delete();
    candidate_image->Delete();
    std::sort(initial_snakes_.begin(), initial_snakes_.end(),
              IsShorter);
  }
}

void Multisnake::AddLastConvergedToInitialSnakes(size_t index) {
  if (!index) return;
  for (auto it = converged_snake_sequence_[index - 1].begin();
       it != converged_snake_sequence_[index - 1].end(); it++) {
    Snake * s = new Snake((*it)->vertices(), (*it)->dimension(),
                          image_interp_, gradient_interp_,
                          snake_parameters_, (*it)->open(),
                          snake_parameters_->GetSnakeId());
    initial_snakes_.push_back(s);
  }
  // std::reverse(initial_snakes_.begin(), initial_snakes_.end());
}

void Multisnake::CopyConvergedToInitial(size_t index) {
  assert(initial_snakes_.empty());
  ComputeGradient();
  CopyConvergedSnakes(converged_snake_sequence_[index], &initial_snakes_);
  std::sort(initial_snakes_.begin(), initial_snakes_.end(),
            IsShorter);
}

void Multisnake::CopyConvergedSnakes(const SnakeContainer &src,
                                     SnakeContainer *dest) {
  for (auto it = src.begin(); it != src.end(); it++) {
    Snake * s = new Snake((*it)->vertices(), (*it)->dimension(),
                          image_interp_, gradient_interp_,
                          snake_parameters_, (*it)->open(),
                          snake_parameters_->GetSnakeId());
    s->set_previous_snake(*it);
    dest->push_back(s);
  }
}

//const Snake * Multisnake::GetInitialSnake(unsigned index) const {
//  if (index > initial_snakes_.size())
//    return nullptr;
//  else
//    return initial_snakes_[index];
//}

void Multisnake::PrintInitialSnakes(std::ostream &os) const {
  PrintSnakeContainer(initial_snakes_, os);
}

const SnakeContainer & Multisnake::GetConvergedSnakes(int index) const {
  return converged_snake_sequence_[index];
}

const SnakeContainer & Multisnake::GetComparingSnakes(int index) const {
  return comparing_snake_sequence_[index];
}

const PointContainer & Multisnake::GetJunctions(int index) const {
  return junction_sequence_[index];
}

const PointContainer & Multisnake::GetComparingJunctions(int index) const {
  return comparing_junction_sequence_[index];
}

void Multisnake::Evolve() {
  std::size_t number_of_initial_snakes = initial_snakes_.size();
  std::cout << "# initial snakes: " << number_of_initial_snakes
            << std::endl;
  SnakeContainer converged;

  while (!initial_snakes_.empty()) {
    Snake *snake = initial_snakes_.back();
    initial_snakes_.pop_back();
    if (snake->Evolve(converged)) {
      converged.push_back(snake);
    } else {
      initial_snakes_.insert(initial_snakes_.end(),
                             snake->subsnakes().begin(),
                             snake->subsnakes().end());
      delete snake;
    }

    int ncompleted = static_cast<int>(number_of_initial_snakes - initial_snakes_.size());
    if (ncompleted < 0)
      ncompleted = 0;
    emit ExtractionProgressed(ncompleted);
    qApp->processEvents();
    std::cout << "\rRemaining: " << std::setw(6)
              << initial_snakes_.size() << std::flush;
  }

  std::cout << "\n# converged snakes: " << converged.size()
            << std::endl;
  converged_snake_sequence_.push_back(converged);
}

void Multisnake::Reconfigure(size_t index) {
  SnakeContainer segments;
  // SnakeContainer long_snakes;
  // for (auto s : converged_snake_sequence_[index]) {
  //   if (s->GetLength() >= snake_parameters_->minimum_length())
  //     long_snakes.push_back(s);
  // }
  CutSnakesAtTJunctions(converged_snake_sequence_[index], &segments);
  // CutSnakesAtTJunctions(long_snakes, &segments);

  SnakeContainer grouped;
  SnakeContainer previous_snakes;
  if (index) {
    previous_snakes = converged_snake_sequence_[index - 1];
  }
  GroupSnakeSegments(previous_snakes, &segments, &grouped);

  DeleteSnakes(&(converged_snake_sequence_[index]));
  converged_snake_sequence_[index] = grouped;
  // converged_snake_sequence_[index] = segments;
}

void Multisnake::ReconfigureWithoutGrouping(size_t index) {
  SnakeContainer long_snakes;
  for (auto s : converged_snake_sequence_[index]) {
    if (s->GetLength() >= snake_parameters_->minimum_length())
      long_snakes.push_back(s);
  }
  SnakeContainer segments;
  CutSnakesAtTJunctions(long_snakes, &segments);
  DeleteSnakes(&(converged_snake_sequence_[index]));
  // auto it = segments.begin();
  // while (it != segments.end()) {
  //   if ((*it)->GetLength() < snake_parameters_->minimum_length()) {
  //     delete *it;
  //     it = segments.erase(it);
  //   } else {
  //     it++;
  //   }
  // }
  converged_snake_sequence_[index] = segments;
}

// void Multisnake::EvolveNetwork(size_t index) {
//   SnakeContainer snakes;
//   ComputeGradient();
//   CopyConvergedSnakes(converged_snake_sequence_[index - 1], &snakes);
//   EvolveFinal(&snakes, snake_parameters_->change_threshold());
//   converged_snake_sequence_.push_back(snakes);
//   UpdateConvergedTrack(snakes, index);
// }

void Multisnake::DeleteConvergedSnakeSequence() {
  DeleteSnakeSequence(&converged_snake_sequence_);
}

void Multisnake::Reset() {
  image_ = nullptr;
  gradient_calc_->Reset();
  DeleteImageInterpolator();
  DeleteGradientInterpolator();
  DeleteSnakes(&initial_snakes_);
  DeleteSnakeSequence(&converged_snake_sequence_);
  DeleteSnakeSequence(&comparing_snake_sequence_);
  junction_sequence_.clear();
  comparing_junction_sequence_.clear();
  converged_snake_track_.clear();
}

Snake * Multisnake::GetCorrespondingSnake(Snake *s,
                                          int frame_index) const {
  if (!s) return nullptr;
  int track_id = -1;
  for (size_t i = 0; i < converged_snake_track_.size(); i++) {
    if (converged_snake_track_[i].Contains(s)) {
      track_id = static_cast<int>(i);
      break;
    }
  }

  if (track_id >= 0) {
    return converged_snake_track_[track_id].GetSnake(frame_index);
  }
  return nullptr;
}

void Multisnake::SolveCorrespondence(size_t nframes) {
  if (nframes < 2) return;
  converged_snake_track_.clear();
  const size_t n = GetNumberOfConvergedSnakes();
  const double threshold = snake_parameters_->association_threshold();
  Matrix<double> distance(n, n, threshold);
  ComputeCurveDistanceMatrix(&distance, threshold);

  Matrix<double> original_distance = distance;

  Munkres solver;
  solver.solve(distance);
  SnakeContainer snakes;
  // Array maps a row/column of distance/solved matrix to frame index
  std::vector<size_t> frame_indices;
  frame_indices.reserve(n);

  // Compute snake ids and frame indices
  for (size_t i = 0; i < converged_snake_sequence_.size(); i++) {
    for (size_t j= 0; j < converged_snake_sequence_[i].size(); j++) {
      snakes.push_back(converged_snake_sequence_[i][j]);
      frame_indices.push_back(i);
    }
  }

  /* An example of distance matrix of a sequence of 3 frames, each of which
   * contains 2 snakes. We only need to consider the upper right submatrix.
   * Here "x" and "o" represent an invalid and valid distance value,
   * respectively.
   *
   * x x o o o o
   * x x o o o o
   * x x x x o o
   * x x x x o o
   * x x x x x x
   * x x x x x x
   *
   */

  for (size_t i = 0;
       i < distance.rows() - converged_snake_sequence_.back().size(); i++) {
    for (size_t j = converged_snake_sequence_.front().size();
         j < distance.columns(); j++) {
      if (distance(i, j) == 0 && original_distance(i, j) < threshold) {
        int track_id = FindTrack(snakes[i]);
        if (track_id < 0) { // new track
          SnakeTrack track(nframes);
          track.SetSnake(frame_indices[i], snakes[i]);
          converged_snake_track_.push_back(track);
          track_id = static_cast<int>(converged_snake_track_.size() - 1);
        }
        // put snake j into the track of snake i
        converged_snake_track_[track_id].SetSnake(frame_indices[j],
                                                  snakes[j]);
      }
    }
  }
  std::sort(converged_snake_track_.begin(), converged_snake_track_.end());
}

size_t Multisnake::GetNumberOfConvergedSnakes() const {
  size_t count = 0;
  for (size_t i = 0; i < converged_snake_sequence_.size(); i++) {
    count += converged_snake_sequence_[i].size();
  }
  return count;
}

std::vector<size_t> Multisnake::GetNumberOfConvergedSnakesInSequence()
    const {
  std::vector<size_t> result;
  for (const auto &c : converged_snake_sequence_)
    result.emplace_back(c.size());
  return result;
}

int Multisnake::GetFrameOfFirstAppearance(size_t track_index) const {
  return converged_snake_track_[track_index].GetFirstFrame();
}

Snake *Multisnake::GetFirstSnake(size_t track_index) const {
  return converged_snake_track_[track_index].GetFirstSnake();
}

/************************ Private Methods ************************/

void Multisnake::ComputeGradient() {
  gradient_calc_->DeleteGradient();
  gradient_calc_->set_image(image_);
  gradient_calc_->Compute(snake_parameters_->intensity_scaling(),
                          snake_parameters_->sigma());
  DeleteGradientInterpolator();
  gradient_interp_ = new Interpolator(gradient_calc_->gradient());
}

vtkImageData * Multisnake::InitializeFlagImage(int ncomponents) const {
  vtkImageData *img = vtkImageData::New();
  img->SetDimensions(image_->GetDimensions());
  img->AllocateScalars(VTK_UNSIGNED_CHAR, ncomponents);
  int *dim = img->GetDimensions();
  for (int z = 0; z < dim[2]; z++) {
    for (int y = 0; y < dim[1]; y++) {
      for (int x = 0; x < dim[0]; x++) {
        for (int j = 0; j < ncomponents; j++) {
          GetFlag(img, x, y, z)[j] = false;
        }
      }
    }
  }
  return img;
}

void Multisnake::ScanGradient(vtkImageData *ridge_image, int dim) const {
  int *dims = ridge_image->GetDimensions();

  for (int z = 0; z < dims[2]; z++) {
    for (int y = 0; y < dims[1]; y++) {
      for (int x = 0; x < dims[0]; x++) {
        double *gplus = GetImageGradient(gradient_calc_->gradient(), x, y, z);
        for (int j = 0; j < dim; j++) {
          if (gplus[j] < snake_parameters_->ridge_threshold()) continue;
          int index[3] = {x, y, z};
          unsigned cnt = 0;

          while (true) {
            index[j]++;
            cnt++;
            if (!IsInside(gradient_calc_->gradient(), index)) break;
            double *gminus = GetImageGradient(gradient_calc_->gradient(),
                                              index);
            if (gminus[j] > snake_parameters_->ridge_threshold()) {
              break;
            } else if (gminus[j] < -snake_parameters_->ridge_threshold()) {
              index[j] -= cnt / 2;
              GetFlag(ridge_image, index)[j] = true;
              break;
            }
          }
        }
      }
    }
  }
}

void Multisnake::GenerateCandidates(vtkImageData *ridge_image,
                                    vtkImageData *candidate_image,
                                    int dim) const {
  int *dims = ridge_image->GetDimensions();

  for (int z = 0; z < dims[2]; z++) {
    for (int y = 0; y < dims[1]; y++) {
      for (int x = 0; x < dims[0]; x++) {
        double intensity = GetImageIntensity(image_, x, y, z);
        if (intensity > snake_parameters_->maximum_foreground() ||
            intensity < snake_parameters_->minimum_foreground())
          continue;
        bool *ridge_flag = GetFlag(ridge_image, x, y, z);
        bool *candidate_flag = GetFlag(candidate_image, x, y, z);

        for (int j = 0; j < dim; j++) {
          if (dim == 2) {
            candidate_flag[j] = ridge_flag[(j + 1) % dim];
          } else if (dim == 3) {
            candidate_flag[j] = ridge_flag[(j + 1) % dim] &&
                                ridge_flag[(j + 2) % dim];
          }
        }
      }
    }
  }
}

void Multisnake::LinkCandidates(vtkImageData *candidate_image, int dim) {
  int *dims = candidate_image->GetDimensions();
  for (int j = 0; j < dim; j++) {
    if (!snake_parameters_->init_directions()[j]) continue;
    if (dim == 3) {
      for (int z = 0; z < dims[j]; z++) {
        for (int y = 0; y < dims[(j + 1) % 3]; y++) {
          for (int x = 0; x < dims[(j + 2) % 3]; x++) {
            int index[3];
            index[j] = z;
            index[(j + 1) % 3] = y;
            index[(j + 2) % 3] = x;

            if (GetFlag(candidate_image, index)[j])
              LinkFromIndex(candidate_image, index, dim, j);
          }
        }
      }
    } else if (dim == 2) {
      for (int z = 0; z < dims[j]; z++) {
        for (int y = 0; y < dims[(j + 1) % 2]; y++) {
          int index[3];
          index[j] = z;
          index[(j + 1) % 2] = y;
          index[2] = 0;

          if (GetFlag(candidate_image, index)[j])
            LinkFromIndex(candidate_image, index, dim, j);
        }
      }
    }
  }
}

void Multisnake::LinkFromIndex(vtkImageData *candidate_image,
                               int *index, int dim, int dim_j) {
  std::vector<double> coordinates;
  while (true) {
    for (int j = 0; j < dim; j++) {
      coordinates.push_back(index[j]);
    }

    bool *candidate_flag = GetFlag(candidate_image, index);
    candidate_flag[dim_j] = false;
    // for (int j = 0; j < dim; j++) {
    //   candidate_flag[j] = false;
    // }

    if (!FindNextCandidate(candidate_image, index, dim, dim_j)) break;
  }
  if (coordinates.size() >= 2 * static_cast<unsigned>(dim)) {
    Snake *s = new Snake(coordinates, dim, image_interp_, gradient_interp_,
                         snake_parameters_, true,
                         snake_parameters_->GetSnakeId());
    initial_snakes_.push_back(s);
  }
}

bool Multisnake::FindNextCandidate(vtkImageData *candidate_image,
                                   int *index, int dim, int dim_j) const {
  int current_index[3];
  current_index[0] = index[0];
  current_index[1] = index[1];
  current_index[2] = index[2];

  index[dim_j]++;
  if (!IsInside(candidate_image, index))
    return false;

  if (GetFlag(candidate_image, index)[dim_j])
    return true;

  if (dim == 2) {
    int d1 = (dim_j + 1) % 2;
    for (int c1 = current_index[d1] - 1;
         c1 <= current_index[d1] + 1; ++c1) {
      index[d1] = c1;
      if (IsInside(candidate_image, index)
          && GetFlag(candidate_image, index)[dim_j])
        return true;
    }
  } else if (dim == 3) {
    int d1 = (dim_j + 1) % 3;
    int d2 = (dim_j + 2) % 3;

    for (int c1 = current_index[d1] - 1;
         c1 <= current_index[d1] + 1; ++c1) {
      for (int c2 = current_index[d2] - 1;
           c2 <= current_index[d2] + 1; ++c2) {
        index[d1] = c1;
        index[d2] = c2;
        if (IsInside(candidate_image, index)
            && GetFlag(candidate_image, index)[dim_j])
          return true;
      }
    }
  }
  return false;
}

void Multisnake::DeleteSnakes(SnakeContainer *snakes) {
  for (auto s : *snakes) {
    delete s;
  }
  snakes->clear();
}

void Multisnake::DeleteSnakeSequence(std::vector<SnakeContainer> *seq) {
  for (size_t i = 0; i < seq->size(); i++) {
    DeleteSnakes(&seq->at(i));
  }
  seq->clear();
}

void Multisnake::DeleteImageInterpolator() {
  if (image_interp_) {
    delete image_interp_;
    image_interp_ = nullptr;
  }
}

void Multisnake::DeleteGradientInterpolator() {
  if (gradient_interp_) {
    delete gradient_interp_;
    gradient_interp_ = nullptr;
  }
}

void Multisnake::PrintSnakeContainer(const SnakeContainer &snakes,
                                     std::ostream &os) const {
  // std::cout << "# of snakes: " << snakes.size() << std::endl;
  for (auto it = snakes.begin(); it != snakes.end(); it++) {
    (*it)->Print(os);
    os << std::endl;
  }
}

void Multisnake::CutSnakesAtTJunctions(const SnakeContainer &converged,
                                       SnakeContainer *segments) const {
  if (converged.empty()) return;
  for (auto it = converged.begin(); it != converged.end(); it++) {
    (*it)->UpdateHookedIndices();
  }
  for (auto it = converged.begin(); it != converged.end(); it++) {
    (*it)->CopySegments(segments);
  }
}

void Multisnake::GroupSnakeSegments(const SnakeContainer &previous_snakes,
                                    SnakeContainer *segments,
                                    SnakeContainer *grouped) {
  if (segments->empty()) return;
  Junctions junctions;
  junctions.set_distance_threshold(
      snake_parameters_->grouping_distance_threshold());
  junctions.set_direction_threshold(snake_parameters_->direction_threshold());
  junctions.set_delta(snake_parameters_->grouping_delta());
  junctions.Union(segments);
  junctions.Configure(previous_snakes);

  LinkSegments(junctions, segments, grouped);
  EvolveFinal(grouped, snake_parameters_->change_threshold());
  junctions.UpdateLocation(*grouped, snake_parameters_->overlap_threshold());
  junction_sequence_.push_back(junctions.locations());
}

void Multisnake::LinkSegments(const Junctions &junctions,
                              SnakeContainer *segments,
                              SnakeContainer *grouped) {
  while (!segments->empty()) {
    Snake *s = segments->back();
    segments->pop_back();
    std::deque<MatrixXd> blocks;
    s->CopyCoordinatesOut(&blocks);
    bool is_open = true;
    std::vector<Snake *> log;
    log.push_back(s);

    // Link from head
    SnakeTip * t = junctions.GetTip(s, true);
    LinkFromSegmentTip(junctions, t->neighbor(), true, segments,
                       &blocks, &is_open, &log);
    // Link from tail
    t = junctions.GetTip(s, false);
    LinkFromSegmentTip(junctions, t->neighbor(), false, segments,
                       &blocks, &is_open, &log);
    delete s;
    int dim = image_->GetDataDimension();
    Snake *snake = new Snake(blocks, dim, image_interp_, gradient_interp_,
                             snake_parameters_, is_open,
                             snake_parameters_->GetSnakeId());
    grouped->push_back(snake);
  }
}

void Multisnake::LinkFromSegmentTip(const Junctions &junctions,
                                    SnakeTip *neighbor,
                                    bool from_head,
                                    SnakeContainer *segments,
                                    std::deque<MatrixXd> *blocks,
                                    bool *is_open,
                                    SnakeContainer *log) {
  if (!neighbor) {
    return;
  } else if (std::find(log->begin(), log->end(), neighbor->snake()) !=
             log->end()) {
    *is_open = false;
    return;
  } else {
    neighbor->snake()->CopyCoordinatesOut(blocks, from_head,
                                          neighbor->is_head() == from_head);
    auto it = std::find(segments->begin(), segments->end(),
                        neighbor->snake());
    if (it != segments->end()) {
      segments->erase(it);
    }
    log->push_back(neighbor->snake());
    delete neighbor->snake();
    SnakeTip *t = junctions.GetTip(neighbor->snake(), !neighbor->is_head());
    // Link to the next segment
    LinkFromSegmentTip(junctions, t->neighbor(), from_head, segments,
                       blocks, is_open, log);
  }
}

bool Multisnake::LoadSnakes(const QString &filename,
                            std::vector<SnakeContainer> *snake_seq,
                            std::vector<PointContainer> *junction_seq,
                            bool load_parameters) {
  std::ifstream infile(filename.toStdString().c_str());
  if (!infile.is_open()) {
    std::cerr << "Couldn't open snake file: " << filename.toStdString()
              << std::endl;
    return false;
  }

  DeleteSnakeSequence(snake_seq);
  std::string line;
  std::vector<double> coordinates;
  bool is_open = true;
  int dim = 3;
  SnakeContainer snakes;
  PointContainer junction_points;
  int id = 0;
  int max_id = 0;
  std::map<unsigned, Snake *> id_snakes;

  while (std::getline(infile, line)) {
    if (line == "Tracks") {
      break;  // read tracks next
    } else if (isalpha(line[0])) {
      std::stringstream converter;
      converter << line;
      std::string name, value;
      converter >> name >> value;
      if (load_parameters) {
        snake_parameters_->AssignParameters(name, value);
      }
      if (name == "dimension")
        dim = std::stoi(value);
    } else if (line[0] == '$') {
      if (!coordinates.empty()) {
        Snake *s = new Snake(coordinates, dim, image_interp_,
                             gradient_interp_, snake_parameters_,
                             is_open, id);
        snakes.push_back(s);
        coordinates.clear();
      }
      snake_seq->push_back(snakes);
      snakes.clear();
      junction_seq->push_back(junction_points);
      junction_points.clear();
    } else if (line[0] == '#') {
      if (!coordinates.empty()) {
        Snake *s = new Snake(coordinates, dim, image_interp_,
                             gradient_interp_, snake_parameters_,
                             is_open, id);
        snakes.push_back(s);
        id_snakes[id] = s;
        coordinates.clear();
      }
      is_open = (line[1] != '0');
    } else if (line[0] == '[') {
      LoadPoint(line, &junction_points, dim);
    } else if (isdigit(line[0])) {
      std::istringstream stream(line);
      std::string item;
      const int offset = 2;
      int count = 0;
      while (stream >> item) {
        if (count == 0) {
          id = std::stoi(item);
          if (id > max_id)
            max_id = id;
        }
        if (count >= offset && count < offset + dim) {
          coordinates.push_back(std::stod(item));
        }
        count++;
      }
    }
  }

  while (std::getline(infile, line)) {
    std::istringstream buffer(line);
    // std::vector<int> track;
    SnakeTrack track;
    while (buffer >> id) {
      // track.push_back(id);
      track.Append(id_snakes[id]);
      // std::cout << id << std::endl;
    }
    converged_snake_track_.push_back(track);
  }

  // Set the new start for snake id so that no duplicate id is possible.
  snake_parameters_->set_snake_id_counter(max_id);
  infile.close();
  path_ = filename;
  std::sort(converged_snake_track_.begin(), converged_snake_track_.end());
  return true;
}

void Multisnake::SaveSnakeCoordinates(const SnakeContainer &snakes,
                                      std::ostream &os) const {
  const unsigned column_width = 16;
  for (auto it = snakes.begin(); it != snakes.end(); ++it) {
    os << "#" << (*it)->open() << std::endl;
    for (size_t j = 0; j != (*it)->GetSize(); ++j) {
      double vertex[3] = {0.0, 0.0, 0.0};
      (*it)->GetVertex(static_cast<int>(j), vertex);
      double intensity = -1.0;
      image_interp_->Interpolate(vertex, &intensity);

      os << (*it)->id() << std::setw(column_width) << j
         << std::setw(column_width) << vertex[0]
         << std::setw(column_width) << vertex[1]
         << std::setw(column_width) << vertex[2]
         << std::setw(column_width) << intensity << std::endl;
    }
  }
}

void Multisnake::EvolveFinal(SnakeContainer *snakes,
                             double threshold) {
  std::sort(snakes->begin(), snakes->end(), IsShorter);
  std::vector<bool> changed(snakes->size(), true);
  int count = 0;

  while (!snakes->empty() && !AllFalse(changed)) {
    Snake *s = snakes->front();
    s->SaveVertexState();
    snakes->erase(snakes->begin());
    if (s->EvolveFinal(*snakes, snake_parameters_->minimum_length())) {
      snakes->push_back(s);
      changed[count % changed.size()] = s->VertexChanged(threshold);
    }
    count++;
  }
}

void Multisnake::PrintPointContainer(const PointContainer &points,
                                     std::ostream &os, int dim) const {
  for (auto it = points.begin(); it != points.end(); ++it) {
    os << "[";
    for (int j = 0; j < dim; j++) {
      if (j == dim - 1)
        os << (*it)(j);
      else
        os << (*it)(j) << ", ";
    }
    os << "]"<< std::endl;
  }
}

void Multisnake::LoadPoint(const std::string &coordinates,
                           PointContainer *points, int dim) const {
  std::istringstream buffer(coordinates);
  char padding;
  VectorXd p(dim);
  for (int j = 0; j < dim; j++) {
    buffer >> padding;
    buffer >> p(j);
  }
  points->push_back(p);
}


int Multisnake::FindTrack(Snake *s) const {
  for (size_t i = 0; i < converged_snake_track_.size(); i++) {
    if (converged_snake_track_[i].Contains(s)) {
      // if (TrackHasId(converged_snake_track_[i], id)) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

void Multisnake::ComputeCurveDistanceMatrix(Matrix<double> *distance,
                                            double threshold) {
  // UpdateSnakeCentroid();
  // const double c = 0.2;
  const double c = snake_parameters_->c();
  for (size_t i = 0; i < converged_snake_sequence_.size() - 1; i++) {
    for (size_t j = i + 1; j < converged_snake_sequence_.size(); j++) {
      for (size_t k = 0; k < converged_snake_sequence_[i].size(); k++) {
        for (size_t l = 0; l < converged_snake_sequence_[j].size(); l++) {
          double d = ComputeCurveDistance(converged_snake_sequence_[i][k],
                                          converged_snake_sequence_[j][l]);

          d *= std::exp(c * (j - i - 1));
          if (d < threshold) {
            size_t row = ComputeSnakeIndex(i, k);
            size_t col = ComputeSnakeIndex(j, l);
            // std::cout << "row: " << row << "\tcol: " << col << std::endl;
            (*distance)(row, col) = d;
          }
        }
      }
    }
  }
}

void Multisnake::UpdateSnakeCentroid() {
  for (auto snakes : converged_snake_sequence_) {
    for (auto s : snakes) {
      s->ComputeCentroid();
    }
  }
}

size_t Multisnake::ComputeSnakeIndex(size_t frame_index, size_t index) const {
  if (frame_index == 0)
    return index;

  size_t count = 0;
  for (size_t i = 0; i < frame_index; i++) {
    count += GetNumberOfConvergedSnakes(i);
  }
  return count + index;
}

}  // namespace soax
