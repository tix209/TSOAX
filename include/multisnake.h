/**
 * Copyright (C) 2017 Lehigh University.
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
 * Multisnake extracts the centerlines of 2D/3D curvilinear networks.
 */


#ifndef MULTISNAKE_H_
#define MULTISNAKE_H_

#include <QObject>
#include <vector>
#include <deque>
#include <QString>
#include <omp.h>
#include "include/util.h"
#include "include/junctions.h"
#include "include/munkres.h"
#include "include/snake_track.h"
#include "include/grid.h"
#include "include/lapjv.h"

class vtkImageData;
class QProgressBar;

namespace soax {

class GradientCalculator;
class Interpolator;
class Snake;
class SnakeParameters;

class Multisnake : public QObject {
  Q_OBJECT

 public:
  friend class MultisnakeTest;

  Multisnake();

  ~Multisnake();

  /**
   * Returns the snake file path.
   */
  QString path() const {return path_;}
  void set_path(const QString &path) {path_ = path;}

  vtkImageData *image() const {return image_;}
  void set_image(vtkImageData *image);

  /**
   * Save initial_snake_ to a file. Returns true if it is saved
   * successfully. The file also records the IMAGE_PATH for the results.
   */
  bool SaveInitialSnakes(const QString &filename,
                         const QString &image_path);


  bool SaveConvergedSnakes(const QString &filename,
                           const QString &image_path);
  void SaveConvergedSnakeTrack(std::ostream &os) const;

  void SaveConvergedSnakes(size_t index, std::ostream &os, int dim);
  bool LoadConvergedSnakes(const QString &filename);
  bool LoadComparingSnakes(const QString &filename);

  bool LoadTamaraSnakes(const QString &filename);

  /*
   * Initialize snakes.
   */
  void Initialize();
  void AddLastConvergedToInitialSnakes(size_t index);
  void CopyConvergedToInitial(size_t index);

  size_t GetNumberOfInitialSnakes() const {
    return initial_snakes_.size();
  }

  // const Snake *GetInitialSnake(unsigned index) const;

  const SnakeContainer &initial_snakes() const {return initial_snakes_;}
  const SnakeContainer &GetConvergedSnakes(int index = 0) const;
  const SnakeContainer &GetComparingSnakes(int index = 0) const;
  bool ConvergedSnakesAreEmpty() const {
    return converged_snake_sequence_.empty();
  }
  bool ComparingSnakesAreEmpty() const {
    return comparing_snake_sequence_.empty();
  }

  size_t GetNumberOfSetsOfConvergedSnakes() const {
    return converged_snake_sequence_.size();
  }

  size_t GetNumberOfSetsOfComparingSnakes() const {
    return comparing_snake_sequence_.size();
  }

  size_t GetNumberOfConvergedSnakes() const;
  size_t GetNumberOfConvergedSnakes(size_t index) const {
    return converged_snake_sequence_[index].size();
  }

  std::vector<size_t> GetNumberOfConvergedSnakesInSequence() const;

  const PointContainer &GetJunctions(int index = 0) const;
  const PointContainer &GetComparingJunctions(int index = 0) const;

  bool JunctionsAreEmpty() const {return junction_sequence_.empty();}
  bool ComparingJunctionsAreEmpty() const {
    return comparing_junction_sequence_.empty();
  }

  size_t GetNumberOfSetsOfJunctions() const {
    return junction_sequence_.size();
  }

  size_t GetNumberOfSetsOfComparingJunctions() const {
    return comparing_junction_sequence_.size();
  }

  void PrintInitialSnakes(std::ostream &os) const;  // NOLINT

  void Evolve();
  void Reconfigure(size_t index);
  void ReconfigureWithoutGrouping(size_t index);

  // void EvolveNetwork(size_t index);

  SnakeParameters * snake_parameters() const {return snake_parameters_;}

  void PrintParameters(std::ostream &os) const;  // NOLINT

  void DeleteConvergedSnakeSequence();
  void Reset();

  Snake * GetCorrespondingSnake(Snake *s, int frame_index) const;

  void SolveCorrespondence(size_t nframes);

  size_t GetNumberOfTracks() const {
    return converged_snake_track_.size();
  }

  int GetFrameOfFirstAppearance(size_t track_index) const;

  Snake *GetFirstSnake(size_t track_index) const;

  void ComputeSphericalOrientation(vtkImageData *image, const PointType &center,
                                   double max_r, double padding,
                                   size_t i, std::ostream &os) const;
                                   
  const Grid &converged_snakes_grid() const {
    return converged_snakes_grid_two_;
  }

 signals:
  void ExtractionProgressed(int value);

 private:
  /**
   * Compute the image gradient.
   */
  void ComputeGradient();
  /*
   * Allocate a multi-component flag image. User needs to free the returned
   * image.
   */
  vtkImageData * InitializeFlagImage(int ncomponents) const;

  void ScanGradient(vtkImageData *ridge_image, int dim) const;

  void GenerateCandidates(vtkImageData *ridge_image,
                          vtkImageData *candidate_image, int dim) const;

  void LinkCandidates(vtkImageData *candidate_image, int dim);

  void LinkFromIndex(vtkImageData *candidate_image,
                     int *index, int dim, int dim_j);

  bool FindNextCandidate(vtkImageData *candidate_image,
                         int *index, int dim, int dim_j) const;

  void DeleteSnakes(SnakeContainer *snakes);
  void DeleteSnakeSequence(std::vector<SnakeContainer> *seq);
  void DeleteImageInterpolator();
  void DeleteGradientInterpolator();

  void PrintSnakeContainer(const SnakeContainer &snakes,
                           std::ostream &os) const;  // NOLINT

  void CutSnakesAtTJunctions(const SnakeContainer &converged,
                             SnakeContainer *segments) const;

  void GroupSnakeSegments(const SnakeContainer &previous_snakes,
                          SnakeContainer *segments,
                          SnakeContainer *grouped);

  void LinkSegments(const Junctions &junctions,
                    SnakeContainer *segments,
                    SnakeContainer *grouped);

  void LinkFromSegmentTip(const Junctions &junctions,
                          SnakeTip *neighbor,
                          bool from_head,
                          SnakeContainer *segments,
                          std::deque<MatrixXd> *blocks,
                          bool *is_open,
                          SnakeContainer *log);

  //unsigned GetNumberOfSnakesCloseToPoint(const VectorXd &p,
  //                                       const SnakeContainer *grouped);


  bool LoadSnakes(const QString &filename,
                  std::vector<SnakeContainer> *snake_seq,
                  std::vector<PointContainer> *junction_seq,
                  bool load_parameters = false);
  void SaveSnakeCoordinates(const SnakeContainer &snakes,
                            std::ostream &os) const;

  void EvolveFinal(SnakeContainer *snakes, double threshold);

  void PrintPointContainer(const PointContainer &points,
                           std::ostream &os, int dim) const;  // NOLINT
  void LoadPoint(const std::string &coordinates,
                 PointContainer *points, int dim) const;

  void CopyConvergedSnakes(const SnakeContainer &src, SnakeContainer *dest);

  int FindTrack(Snake *s) const;

  /**
   * Compute the matrix where each entry at (i, j) is the distance between
   * snake i and snake j. Returns the maximum allowable distance (less than
   * threshold);
   */
  //void ComputeCurveDistanceMatrix(Matrix<double> *distance, double threshold);
  void ComputeCurveDistanceMatrix(double** distance_matrix, double threshold);
  
  /**
   * Returns the index of snakes as a whole.
   */
  size_t ComputeSnakeIndex(size_t frame_index, size_t index) const;

  void UpdateSnakeCentroid();
  bool IsInsideSphere(const PointType &center, const PointType &p,
                      double r) const;

  void ComputeThetaPhi(VectorXd v, double &theta, double &phi) const;



  QString path_ = "..";
  vtkImageData *image_;

  GradientCalculator *gradient_calc_;
  Interpolator *image_interp_;
  Interpolator *gradient_interp_;
  SnakeParameters *snake_parameters_;
  // grid with pair comprised of (converged_snakes index, index of vertex of converged_snakes)
  //std::vector<std::vector<IndexPairContainer> > converged_snakes_grid_;
  
  Grid converged_snakes_grid_two_;

  SnakeContainer initial_snakes_;

  // vector of SnakeContiners where the index is the frame count
  std::vector<SnakeContainer> converged_snake_sequence_;
  std::vector<SnakeContainer> comparing_snake_sequence_;
  std::vector<SnakeTrack> converged_snake_track_;
  
  std::vector<Grid> converged_snake_grid_sequence_;

  std::vector<PointContainer> junction_sequence_;
  std::vector<PointContainer> comparing_junction_sequence_;

  // add converged snake to grid
  //void AddConvergedSnakeIndexesToGrid(int org_x_grid, int org_y_grid, int converged_snake_index, int vertex_index);
  
  // clear converged snakes
  //void ClearConvergedSnakesGrid();
  
  Multisnake(const Multisnake &);
  void operator=(const Multisnake &);
};

}  // namespace soax
#endif  // MULTISNAKE_H_
