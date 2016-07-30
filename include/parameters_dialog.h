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
 * This class is the dialog for viewing and setting SOAX extraction
 * parameters.
 */

#ifndef PARAMETERS_DIALOG_H_
#define PARAMETERS_DIALOG_H_

#include <QDialog>

class QLineEdit;
class QDialogButtonBox;
class QGroupBox;
class QCheckBox;

namespace soax {
class SnakeParameters;

class ParametersDialog : public QDialog {
  Q_OBJECT

 public:
  explicit ParametersDialog(QWidget *parent = NULL);

  double GetIntensityScaling() const;
  double GetSigma() const;
  double GetRidgeThreshold() const;
  int GetMaximumForeground() const;
  int GetMinimumForeground() const;
  bool InitXChecked() const;
  bool InitYChecked() const;
  bool InitZChecked() const;
  double GetSpacing() const;
  double GetMinimumLength() const;
  int GetMaximumIterations() const;
  double GetChangeThreshold() const;
  int GetCheckPeriod() const;
  double GetAlpha() const;
  double GetBeta() const;
  double GetGamma() const;
  double GetExternalFactor() const;
  double GetStretchFactor() const;
  int GetNumberOfSectors() const;
  double GetZSpacing() const;
  int GetRadialNear() const;
  int GetRadialFar() const;
  int GetDelta() const;
  double GetOverlapThreshold() const;
  double GetGroupingDistanceThreshold() const;
  int GetGroupingDelta() const;
  double GetDirectionThreshold() const;
  bool DampZ() const;
  double GetAssociationThreshold() const;


  /*
   * Set the LineEdit values from the initial default.
   */
  void SetParameters(const SnakeParameters *sp);

 private:
  QGroupBox * CreateSettings();

  QLineEdit *intensity_scaling_edit_;
  QLineEdit *sigma_edit_;
  QLineEdit *ridge_threshold_edit_;
  QLineEdit *maximum_foreground_edit_;
  QLineEdit *minimum_foreground_edit_;
  QCheckBox *init_x_check_;
  QCheckBox *init_y_check_;
  QCheckBox *init_z_check_;
  QLineEdit *spacing_edit_;
  QLineEdit *minimum_length_edit_;
  QLineEdit *maximum_iterations_edit_;
  QLineEdit *change_threshold_edit_;
  QLineEdit *check_period_edit_;
  // QLineEdit *iterations_per_press_edit_;
  QLineEdit *alpha_edit_;
  QLineEdit *beta_edit_;
  QLineEdit *gamma_edit_;
  QLineEdit *external_factor_edit_;
  QLineEdit *stretch_factor_edit_;
  QLineEdit *number_of_sectors_edit_;
  QLineEdit *z_spacing_edit_;
  QLineEdit *radial_near_edit_;
  QLineEdit *radial_far_edit_;
  QLineEdit *delta_edit_;
  QLineEdit *overlap_threshold_edit_;
  QLineEdit *grouping_distance_threshold_edit_;
  QLineEdit *grouping_delta_edit_;
  QLineEdit *direction_threshold_edit_;
  QCheckBox *damp_z_check_;
  QLineEdit *association_threshold_edit_;

  QDialogButtonBox *ok_cancel_;

  ParametersDialog(const ParametersDialog &);
  void operator=(const ParametersDialog &);
};

}  // namespace soax
#endif  // PARAMETERS_DIALOG_H_
