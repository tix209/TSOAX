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
 * This class defines parametric Active Contour (Snake). It can be a
 * traditional Snake or a Stretching Open Active Contour (SOAC).
 */

#ifndef SNAKE_H_
#define SNAKE_H_

#include <vector>
#include <deque>
#include <utility>
#include "eigen3/Eigen/SparseCore"
#include "eigen3/Eigen/SparseCholesky"
#include "./util.h"


namespace soax {

class Interpolator;
class SnakeParameters;

class Snake {
 public:
  Snake();

  /**
   * Construct using a set of coordinates. COORDINATES contains the data as
   * {x0, y0, z0, x1, y1, z1, ... } if DIM is 3 and {x0, y0, x1, y1, ... }
   * if DIM is 2.
   */
  Snake(const std::vector<double> &coordinates, int dim,
        const Interpolator *image_interp,
        const Interpolator *gradient_interp,
        SnakeParameters *p, bool open, int id = 0);

  Snake(const MatrixXd &coordinates, int dim,
        const Interpolator *image_interp,
        const Interpolator *gradient_interp,
        SnakeParameters *p, bool open, int id = 0);

  Snake(const std::deque<MatrixXd> &blocks, int dim,
        const Interpolator *image_interp,
        const Interpolator *gradient_interp,
        SnakeParameters *p, bool open, int id = 0);

  int id() const {return id_;}
  void set_id(int id) {id_ = id;}

  /**
   * Returns/sets the number of components each snake point has.
   */
  int dimension() const {return dimension_;}
  void set_dimension(int dim) {dimension_ = dim;}

  const MatrixXd &vertices() const {return vertices_;}
  const RowVectorXd &centroid() const {return centroid_;}
  size_t GetSize() const {return vertices_.rows();}
  void ComputeCentroid();

  void GetVertex(int index, double *vertex) const;
  ConstRowVec GetVertex(int index) const;
  ConstRowVec GetTip(bool is_head) const;
  VectorXd GetTipTangent(bool is_head, int delta) const;
  double GetLength() const;

  /**
   * Returns distance to a target point. The distance is the minimum Euclidean
   * distance from one of the points in this snake to the target point.
   */
  double DistanceTo(ConstRowVec p) const;
  double TranslatedDistanceTo(const RowVectorXd &p) const;

  /**
   * Returns the above Euclidean distance multiplied by exp(-|cos(theta)|).
   */
  double EuclideanAndAngleDistanceTo(ConstRowVec p, VectorXd tangent) const;

  /**
   * Uniformly sample the snake so that the distance between consecutive
   * snake points is as specified in SnakeParameters. Returns false if the
   * number of points of resampled snake is less than kMinimumEvolvingSize
   * or the length is less than MIN_LENGTH.
   */
  bool Resample(double min_length = 1.0);

  /**
   * Returns true if the snake survive after evolution with overlap
   * checking with SNAKES.
   */
  bool Evolve(const SnakeContainer &snakes);
  bool EvolveWithTipFixed(int max_iter);
  bool EvolveFinal(const SnakeContainer &snakes, double length);

  const SnakeContainer &subsnakes() const {return subsnakes_;}

  bool open() const {return open_;}

  unsigned iterations() const {return iterations_;}

  /**
   * Set snake parameters. Must be called after construction.
   */
  void set_parameters(SnakeParameters *parameters) {
    parameters_ = parameters;
  }

  void Print(std::ostream &os) const;  // NOLINT

  void UpdateHookedIndices();
  void CopySegments(SnakeContainer *segments);

  void CopyCoordinatesOut(std::deque<MatrixXd> *blocks,
                          bool prepend = false,
                          bool reverse = false) const;

  bool PassThrough(const VectorXd &point, double threshold) const;
  bool PassThrough(const RowVec &point, double threshold) const;

  /**
   * Save the VERTICES_ to ORIGINAL_VERTICES_.
   */
  void SaveVertexState();

  /**
   * Returns the maximum difference (Euclidean distance) between VERTICES_
   * and ORIGINAL_VERTICES_.
   */
  double ComputeVertexChange();

  /**
   * Returns true if the distance between VERTICES_ and ORIGINAL_VERTICES_
   * is greater than THRESHOLD.
   */
  bool VertexChanged(double threshold);

  const Snake * previous_snake() const {return previous_snake_;}
  void set_previous_snake(Snake *s) {previous_snake_ = s;}

  VectorXd GetVertexTangent(int index, int delta) const;

 private:
  typedef Eigen::SparseMatrix<double> SpMatrix;
  typedef Eigen::Triplet<double> SpEntry;
  typedef std::vector<std::pair<double, double> > PairContainer;

  /**
   * Copy the coordinates to vertices_.
   */
  void CopyCoordinates(const std::vector<double> &coordinates);

  bool IsConverged();

  /**
   * Returns true if the snake intersects with itself. This function
   * assumes there is no self-intersection beforehand.
   */
  bool SelfIntersect();
  bool CheckHeadOverlap(const SnakeContainer &snakes);
  bool CheckTailOverlap(const SnakeContainer &snakes);
  bool TipsStopAtSameLocation() const;
  void InitializeFromPart(int start, int end, bool is_open,
                          SnakeContainer *snakes);


  int FindFirstDetachFromHead(int start, const SnakeContainer &snakes);
  int FindFirstDetachFromTail(int start, const SnakeContainer &snakes);
  bool VertexOverlap(int index, const SnakeContainer &snakes);

  void FindHookedSnake(int last_touch, const SnakeContainer &snakes,
                       Snake **s, int *index);
  double FindClosestIndexTo(const RowVec &point, int *index) const;
  /**
   * Returns true if the snake has no body overlap.
   */
  bool CheckBodyOverlap(const SnakeContainer &snakes);

  void IterateOnce();
  void AddExternalForce(MatrixXd *rhs);
  void AddStretchingForce(MatrixXd *rhs);
  void SolveLinearSystem(const MatrixXd &rhs);
  void KeepVerticesInsideImage(double padding);

  bool HeadIsFixed();
  bool TailIsFixed();

  /**
   * Returns the tangent vector at snake head/tail.
   */
  VectorXd GetHeadTangent(int delta) const;
  VectorXd GetTailTangent(int delta) const;

  double ComputeLocalStretch(bool is_head);

  bool EstimateLocalBackgroundIntensity2D(bool is_head, double *bg);
  bool EstimateLocalBackgroundIntensity3D(bool is_head, double *bg);

  double ComputePodX(double x, const Vector2d &tvec,
                     double dist, bool plus_root) const;

  double ComputePodY(double y, const Vector2d &tvec,
                     double dist, bool plus_root) const;

  void FillEntriesOpen(std::vector<SpEntry> *entries);
  void FillEntriesClosed(std::vector<SpEntry> *entries);



  /**
   * Returns current length of the snake and compute the partial sum of
   * lengths with respect to the coordinate of each dimension. SUMS is an
   * array of vectors of pairs. Each pair is <partial length, coordinate
   * x/y/z>. Used for resampling the snake.
   */
  // double ComputeLengthPartialSum(PairContainer *sums) const;

  /**
   * Returns the size of the snake given the LENGTH and SPACING.
   */
  int ComputeSize(double length, double spacing) const;

  /**
   * Linearly interpolate VERTICES_ to be uniformly spaced.
   */
  void InterpolateVertices(int size);


  void CopyCoordinatesFromBlocks(const std::deque<MatrixXd> &blocks);
  int GetNumberOfRows(const std::deque<MatrixXd> &blocks) const;

  /**
   * Returns true if any coordinates is a NaN.
   */
  bool CheckNaN();


  /**
   * Snake vertices stored in a N by DIMENSION matrix, where N is the
   * number of vertices and DIMENSION is the number of components per
   * vertex.
   */
  MatrixXd vertices_;

  /**
   * Cached vertices for determining convergence. Vertices are saved after
   * PERIOD number of iterations.
   */
  MatrixXd last_vertices_;
  MatrixXd original_vertices_;

  /**
   * Number of components per vertex.
   */
  int dimension_ = 3;

  const Interpolator *image_interp_ = nullptr;
  const Interpolator *gradient_interp_ = nullptr;
  SnakeParameters *parameters_ = nullptr;
  /**
   * True if the snake is open curve.
   */
  bool open_ = true;

  /**
   * An id number to distinguish the snake. Valid when > 0.
   */
  int id_ = 0;

  /***
   * Distance (in pixels) between consecutive snake points.
   */
  double spacing_ = 1.0;

  /**
   * Current number of iterations of this snake.
   */
  int iterations_ = 0;

  VectorXd fixed_head_;
  VectorXd fixed_tail_;

  Snake *head_hooked_snake_ = nullptr;
  Snake *tail_hooked_snake_ = nullptr;
  int head_hooked_index_ = 0;
  int tail_hooked_index_ = 0;

  /**
   * Snake can be devided into several subsnakes during its evolution.
   */
  SnakeContainer subsnakes_;

  /**
   * Indices where T-junctions are located on this snake.
   */
  std::vector<int> junction_indices_;

  /**
   * Corresponding snake from previous frame.
   */
  const Snake * previous_snake_ = nullptr;

  RowVectorXd centroid_;
};

bool IsLonger(const Snake *s1, const Snake *s2);
bool IsShorter(const Snake *s1, const Snake *s2);

double ComputeCurveDistance(const Snake *s1, const Snake *s2);
double ComputeOneWayCurveDistance(const Snake *snake, const Snake *target);

Snake * GetClosestSnake(const Snake *s, const SnakeContainer &snakes,
                        double &min_dist);


}  // namespace soax

#endif  // SNAKE_H_
