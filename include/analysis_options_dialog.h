/**
 * Copyright (C) 2017 Lehigh University.
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
 * This class is the options dialog for analyzing extraction results.
 */

#ifndef ANALYSIS_OPTIONS_DIALOG_H_
#define ANALYSIS_OPTIONS_DIALOG_H_

#include <QDialog>
#include "include/util.h"

class QLineEdit;
class QGroupBox;
class QCheckBox;
class QDialogButtonBox;

namespace soax {

class AnalysisOptionsDialog : public QDialog {
  Q_OBJECT

 public:
  explicit AnalysisOptionsDialog(QWidget * parent = NULL);

  bool GetPixelSize(double *pixel_size) const;
  bool GetCoarseGraining(int *coarse_graining) const;

  bool GetImageCenter(PointType *center) const;
  void SetImageCenter(const PointType &center);

  bool GetRadius(double *radius) const;
  void SetRadius(double r);

  bool GetInsideRatio(double *ratio) const;

  bool ExcludeBoundaryChecked() const;

 public slots:  // NOLINT(whitespace/indent)
  void EnableOKButton();
  void DisableOKButton();

 private:
  static constexpr unsigned kDimension = 3;
  QGroupBox *CreateGeneralGroup();
  QGroupBox *CreateCurvatureGroup();
  QGroupBox *CreateSphericalConfinementGroup();

  QLineEdit *coarse_graining_edit_;
  QLineEdit *center_edit_[kDimension];
  QLineEdit *radius_edit_;
  QLineEdit *pixel_size_edit_;
  QLineEdit *inside_ratio_edit_;
  QDialogButtonBox *button_box_;
  QCheckBox *exclude_boundary_check_;

  AnalysisOptionsDialog(const AnalysisOptionsDialog &);
  void operator=(const AnalysisOptionsDialog &);
};

}  // namespace soax

#endif  // ANALYSIS_OPTIONS_DIALOG_H_
