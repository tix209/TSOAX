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
 * This class is an assemble of all parameters for a SOAC.
 */

#ifndef SNAKE_PARAMETERS_H_
#define SNAKE_PARAMETERS_H_

#include <string>
#include <QString>

namespace soax {

class SnakeParameters {
 public:
  SnakeParameters();

  QString path() const {return path_;}

  double intensity_scaling() const {return intensity_scaling_;}
  void set_intensity_scaling(double scale) {intensity_scaling_ = scale;}

  double sigma() const {return sigma_;}
  void set_sigma(double sigma) {sigma_ = sigma;}

  double ridge_threshold() const {return ridge_threshold_;}
  void set_ridge_threshold(double threshold) {ridge_threshold_ = threshold;}

  int maximum_foreground() const {return maximum_foreground_;}
  void set_maximum_foreground(int max) {maximum_foreground_ = max;}

  int minimum_foreground() const {return minimum_foreground_;}
  void set_minimum_foreground(int min) {minimum_foreground_ = min;}

  const bool * init_directions() const {return init_directions_;}
  void set_init_direction(int index, bool value) {
    init_directions_[index] = value;
  }

  double spacing() const {return spacing_;}
  void set_spacing(double spacing) {spacing_ = spacing;}

  double minimum_length() const {return minimum_length_;}
  void set_minimum_length(double length) {minimum_length_ = length;}

  int maximum_iterations() const {return maximum_iterations_;}
  void set_maximum_iterations(int max_iter) {
    maximum_iterations_ = max_iter;
  }

  double change_threshold() const {return change_threshold_;}
  void set_change_threshold(double change) {change_threshold_ = change;}

  int check_period() const {return check_period_;}
  void set_check_period(int check_period) {check_period_ = check_period;}

  double alpha() const {return alpha_;}
  void set_alpha(double alpha) {alpha_ = alpha; }

  double beta() const {return beta_;}
  void set_beta(double beta) {beta_ = beta;}

  double gamma() const {return gamma_;}
  void set_gamma(double gamma) {gamma_ = gamma;}

  double external_factor() const {return external_factor_;}
  void set_external_factor(double factor) {external_factor_ = factor;}

  double stretch_factor() const {return stretch_factor_;}
  void set_stretch_factor(double factor) {stretch_factor_ = factor;}

  int number_of_sectors() const {return number_of_sectors_;}
  void set_number_of_sectors(int nsectors) {number_of_sectors_ = nsectors;}

  double zspacing() const {return zspacing_;}
  void set_zspacing(double zspacing) {zspacing_ = zspacing;}

  int radial_near() const {return radial_near_;}
  void set_radial_near(int rnear) {radial_near_ = rnear;}

  int radial_far() const {return radial_far_;}
  void set_radial_far(int rfar) {radial_far_ = rfar;}

  int delta() const {return delta_;}
  void set_delta(int delta) {delta_ = delta;}

  double overlap_threshold() const {return overlap_threshold_;}
  void set_overlap_threshold(double t) {overlap_threshold_ = t;}

  double grouping_distance_threshold() const {
    return grouping_distance_threshold_;
  }
  void set_grouping_distance_threshold(double threshold) {
    grouping_distance_threshold_ = threshold;
  }

  int grouping_delta() const {return grouping_delta_;}
  void set_grouping_delta(int delta) {grouping_delta_ = delta;}

  double direction_threshold() const {return direction_threshold_;}
  void set_direction_threshold(double threshold) {
    direction_threshold_ = threshold;
  }

  bool damp_z() const {return damp_z_;}
  void set_damp_z(bool damp_z) {damp_z_ = damp_z;}

  double association_threshold() const {return association_threshold_;}
  void set_association_threshold(double threshold) {
    association_threshold_ = threshold;
  }

  std::string ToString() const;

  bool Valid() const;

  /**
   * Load parameters from a file.
   */
  bool Load(const QString &filename);

  /**
   * Save parameters to a file.
   */
  bool Save(const QString &filename);

  void AssignParameters(const std::string &name,
                        const std::string &value);

  int GetSnakeId();
  int snake_id_counter() const {return snake_id_counter_;}
  void set_snake_id_counter(int counter) {snake_id_counter_ = counter;}

 private:
  QString path_ = "..";

  /**
   * Factor to scale the intensity into the range of [0, 1]. If zero, the
   * actual factor is set to 1 / maximum intensity of the image.
   */
  double intensity_scaling_ = 0.0;

  /**
   * Standard deviation of Gaussian smoothing for computing the gradient.
   */
  double sigma_ = 0.0;

  /**
   * Threshold of the significance of a ridge point.
   */
  double ridge_threshold_ = 0.01;

  /**
   * Allowed intensity range for a snake.
   */
  int maximum_foreground_ = 65535;
  int minimum_foreground_ = 0;

  /**
   * True if snakes are initialized along that axis direction.
   */
  bool init_directions_[3];

  /**
   * Spacing bewtween vertices.
   */
  double spacing_ = 1.0;

  /**
   * Minimum length allowed (in pixels).
   */
  double minimum_length_ = 10.0;

  /**
   * Maximum number of iterations allowed for a snake.
   */
  int maximum_iterations_ = 10000;

  /**
   * Change threshold for determining snake convergence. If every
   * snake point move a distance less than this threshold, the
   * snake is converged.
   */
  double change_threshold_ = 0.1;

  /**
   * Number of iterations for checking convergence.
   */
  int check_period_ = 100;

  /**
   * Weight for first-order curve continuity.
   */
  double alpha_ = 0.01;
  double beta_ = 0.1;
  double gamma_ = 2;
  double external_factor_ = 1;
  double stretch_factor_ = 0.2;

  /**
   * These four parameters determines the sample locations for estimating
   * the local background intensity around snake tips.
   */
  int number_of_sectors_ = 8;
  double zspacing_ = 1.0;   // Voxel size along z axis relative to x or y.
  int radial_near_ = 4;
  int radial_far_ = 8;

  /**
   * Number of vertices apart for estimating tangent vector at snake
   * points.
   */
  int delta_ = 4;

  /**
   * Distance threshold to determine if two vertices overlap.
   */
  double overlap_threshold_ = 1.0;

  /**
   * Distance threshold for grouping junctions.
   */
  double grouping_distance_threshold_ = 4.0;

  /**
   * Number of vertices apart for estimating snake tip directions during
   * network reconfiguration.
   */
  int grouping_delta_ = 8;

  /**
   * Angle threshold (in radians) to determine if two snake segments are
   * smooth enough so that they should be linked up.
   */
  double direction_threshold_ = 2.1;

  /**
   * Whether the stretching force is reduced when it aligns with z-axis.
   */
  bool damp_z_ = false;

  /**
   * Maximum distance between two curves that are associated in time for
   * tracking.
   */
  double association_threshold_ = 10.0;

  unsigned snake_id_counter_ = 0;
};

std::ostream & operator<<(std::ostream &os, const SnakeParameters &sp);

}  // namespace soax

#endif  // SNAKE_PARAMETERS_H_
