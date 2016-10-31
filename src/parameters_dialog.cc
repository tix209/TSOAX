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
 * This file implements the dialog for viewing and setting SOAX extraction
 * parameters.
 */

#include "include/parameters_dialog.h"
#include <QLineEdit>
#include <QDialogButtonBox>
#include <QGroupBox>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QFormLayout>
#include "include/snake_parameters.h"

namespace soax {

ParametersDialog::ParametersDialog(QWidget *parent) : QDialog(parent) {
  QVBoxLayout *vbox = new QVBoxLayout;
  vbox->addWidget(CreateSettings());
  ok_cancel_ = new QDialogButtonBox(QDialogButtonBox::Ok |
                                    QDialogButtonBox::Cancel);

  vbox->addWidget(ok_cancel_);
  setLayout(vbox);
  setWindowTitle(tr("Parameter Settings"));

  connect(ok_cancel_, SIGNAL(accepted()), this, SLOT(accept()));
  connect(ok_cancel_, SIGNAL(rejected()), this, SLOT(reject()));
}

void ParametersDialog::SetParameters(const SnakeParameters *sp) {
  intensity_scaling_edit_->setText(QString::number(sp->intensity_scaling()));
  sigma_edit_->setText(QString::number(sp->sigma()));
  ridge_threshold_edit_->setText(QString::number(sp->ridge_threshold()));

  minimum_foreground_edit_->setText(
      QString::number(sp->minimum_foreground()));
  maximum_foreground_edit_->setText(
      QString::number(sp->maximum_foreground()));

  init_x_check_->setChecked(sp->init_directions()[0]);
  init_y_check_->setChecked(sp->init_directions()[1]);
  init_z_check_->setChecked(sp->init_directions()[2]);

  spacing_edit_->setText(QString::number(sp->spacing()));
  minimum_length_edit_->setText(QString::number(sp->minimum_length()));
  maximum_iterations_edit_->setText(
      QString::number(sp->maximum_iterations()));
  change_threshold_edit_->setText(QString::number(sp->change_threshold()));
  check_period_edit_->setText(QString::number(sp->check_period()));
  alpha_edit_->setText(QString::number(sp->alpha()));
  beta_edit_->setText(QString::number(sp->beta()));
  gamma_edit_->setText(QString::number(sp->gamma()));
  external_factor_edit_->setText(QString::number(sp->external_factor()));
  stretch_factor_edit_->setText(QString::number(sp->stretch_factor()));
  number_of_sectors_edit_->setText(QString::number(sp->number_of_sectors()));
  z_spacing_edit_->setText(QString::number(sp->zspacing()));
  radial_near_edit_->setText(QString::number(sp->radial_near()));
  radial_far_edit_->setText(QString::number(sp->radial_far()));
  delta_edit_->setText(QString::number(sp->delta()));
  overlap_threshold_edit_->setText(QString::number(sp->overlap_threshold()));
  grouping_distance_threshold_edit_->setText(
      QString::number(sp->grouping_distance_threshold()));
  grouping_delta_edit_->setText(QString::number(sp->grouping_delta()));
  direction_threshold_edit_->setText(
      QString::number(sp->direction_threshold()));
  damp_z_check_->setChecked(sp->damp_z());
  association_threshold_edit_->setText(
      QString::number(sp->association_threshold()));
}

double ParametersDialog::GetIntensityScaling() const {
  return intensity_scaling_edit_->text().toDouble();
}

double ParametersDialog::GetSigma() const {
  return sigma_edit_->text().toDouble();
}

double ParametersDialog::GetRidgeThreshold() const {
  return ridge_threshold_edit_->text().toDouble();
}

int ParametersDialog::GetMaximumForeground() const {
  return maximum_foreground_edit_->text().toInt();
}

int ParametersDialog::GetMinimumForeground() const {
  return minimum_foreground_edit_->text().toInt();
}

bool ParametersDialog::InitXChecked() const {
  return init_x_check_->isChecked();
}

bool ParametersDialog::InitYChecked() const {
  return init_y_check_->isChecked();
}

bool ParametersDialog::InitZChecked() const {
  return init_z_check_->isChecked();
}

double ParametersDialog::GetSpacing() const {
  return spacing_edit_->text().toDouble();
}

double ParametersDialog::GetMinimumLength() const {
  return minimum_length_edit_->text().toDouble();
}

int ParametersDialog::GetMaximumIterations() const {
  return maximum_iterations_edit_->text().toInt();
}

double ParametersDialog::GetChangeThreshold() const {
  return change_threshold_edit_->text().toDouble();
}

int ParametersDialog::GetCheckPeriod() const {
  return check_period_edit_->text().toInt();
}

double ParametersDialog::GetAlpha() const {
  return alpha_edit_->text().toDouble();
}

double ParametersDialog::GetBeta() const {
  return beta_edit_->text().toDouble();
}

double ParametersDialog::GetGamma() const {
  return gamma_edit_->text().toDouble();
}

double ParametersDialog::GetExternalFactor() const {
  return external_factor_edit_->text().toDouble();
}

double ParametersDialog::GetStretchFactor() const {
  return stretch_factor_edit_->text().toDouble();
}

int ParametersDialog::GetNumberOfSectors() const {
  return number_of_sectors_edit_->text().toInt();
}

double ParametersDialog::GetZSpacing() const {
  return z_spacing_edit_->text().toDouble();
}

int ParametersDialog::GetRadialNear() const {
  return radial_near_edit_->text().toInt();
}

int ParametersDialog::GetRadialFar() const {
  return radial_far_edit_->text().toInt();
}

int ParametersDialog::GetDelta() const {
  return delta_edit_->text().toInt();
}

double ParametersDialog::GetOverlapThreshold() const {
  return overlap_threshold_edit_->text().toDouble();
}

double ParametersDialog::GetGroupingDistanceThreshold() const {
  return grouping_distance_threshold_edit_->text().toDouble();
}

int ParametersDialog::GetGroupingDelta() const {
  return grouping_delta_edit_->text().toInt();
}

double ParametersDialog::GetDirectionThreshold() const {
  return direction_threshold_edit_->text().toDouble();
}

bool ParametersDialog::DampZ() const {
  return damp_z_check_->isChecked();
}

double ParametersDialog::GetAssociationThreshold() const {
  return association_threshold_edit_->text().toDouble();
}

/****************** Private Methods ********************/

QGroupBox *ParametersDialog::CreateSettings() {
  QGroupBox *gp = new QGroupBox("");
  intensity_scaling_edit_ = new QLineEdit("0.0");
  sigma_edit_ = new QLineEdit("0.0");
  ridge_threshold_edit_ = new QLineEdit("0.0");
  maximum_foreground_edit_ = new QLineEdit("0");
  minimum_foreground_edit_ = new QLineEdit("0");

  init_x_check_ = new QCheckBox(tr("X"));
  init_y_check_ = new QCheckBox(tr("Y"));
  init_z_check_ = new QCheckBox(tr("Z"));
  QHBoxLayout *hbox1 = new QHBoxLayout;
  hbox1->addWidget(init_x_check_);
  hbox1->addWidget(init_y_check_);
  hbox1->addWidget(init_z_check_);
  hbox1->addStretch();

  spacing_edit_ = new QLineEdit("0.0");
  minimum_length_edit_ = new QLineEdit("0.0");
  maximum_iterations_edit_ = new QLineEdit("0.0");
  change_threshold_edit_ = new QLineEdit("0.0");
  check_period_edit_ = new QLineEdit("0");
  alpha_edit_ = new QLineEdit("0.0");
  beta_edit_ = new QLineEdit("0.0");
  gamma_edit_ = new QLineEdit("0.0");
  external_factor_edit_ = new QLineEdit("0.0");
  stretch_factor_edit_ = new QLineEdit("0.0");
  number_of_sectors_edit_ = new QLineEdit("0");
  z_spacing_edit_ = new QLineEdit("0");
  radial_near_edit_ = new QLineEdit("0");
  radial_far_edit_ = new QLineEdit("0");
  delta_edit_ = new QLineEdit("0");
  overlap_threshold_edit_ = new QLineEdit("0.0");
  grouping_distance_threshold_edit_ = new QLineEdit("0.0");
  grouping_delta_edit_ = new QLineEdit("0");
  direction_threshold_edit_ = new QLineEdit("0.0");
  association_threshold_edit_ = new QLineEdit("0.0");

  damp_z_check_ = new QCheckBox(tr(""));
  damp_z_check_->setChecked(false);
  QHBoxLayout *hbox2 = new QHBoxLayout;
  hbox2->addWidget(damp_z_check_);
  hbox2->addStretch();

  QFormLayout *left_form  = new QFormLayout;
  left_form->addRow(tr("Intensity Scaling (0 for automatic)"),
                    intensity_scaling_edit_);
  left_form->addRow(tr("Gaussian Std (pixels)"), sigma_edit_);
  left_form->addRow(tr("Ridge Threshold (tau)"), ridge_threshold_edit_);
  left_form->addRow(tr("Maximum foreground"), maximum_foreground_edit_);
  left_form->addRow(tr("Minimum foreground"), minimum_foreground_edit_);
  left_form->addRow(tr("Snake Point Spacing (pixels)"), spacing_edit_);
  left_form->addRow(tr("Minimum SOAC Length (pixels)"), minimum_length_edit_);
  left_form->addRow(tr("Maximum Iterations"), maximum_iterations_edit_);
  left_form->addRow(tr("Change Threshold (pixels)"), change_threshold_edit_);
  left_form->addRow(tr("Check Period"), check_period_edit_);
  left_form->addRow(tr("Delta (SOAC points)"), delta_edit_);
  left_form->addRow(tr("Initialization Directions"), hbox1);
  left_form->addRow(tr("Damp Z"), hbox2);

  QFormLayout *right_form  = new QFormLayout;
  right_form->addRow(tr("Alpha"), alpha_edit_);
  right_form->addRow(tr("Beta"), beta_edit_);
  right_form->addRow(tr("Gamma"), gamma_edit_);
  right_form->addRow(tr("External Factor (k_img)"), external_factor_edit_);
  right_form->addRow(tr("Stretch Factor (k_str)"), stretch_factor_edit_);
  right_form->addRow(tr("Number of Background Radial Sectors"),
                     number_of_sectors_edit_);
  right_form->addRow(tr("Radial Near (pixels)"), radial_near_edit_);
  right_form->addRow(tr("Radial Far (pixels)"), radial_far_edit_);
  right_form->addRow(tr("Background Z/XY Ratio (pixels)"), z_spacing_edit_);
  right_form->addRow(tr("Overlap Threshold (pixels)"),
                     overlap_threshold_edit_);
  right_form->addRow(tr("Grouping Distance Threshold (pixels)"),
                     grouping_distance_threshold_edit_);
  right_form->addRow(tr("Grouping Delta (SOAC points)"),
                     grouping_delta_edit_);
  right_form->addRow(tr("Minimum Angle for SOAC Linking (radians)"),
                     direction_threshold_edit_);
  right_form->addRow(tr("Association Threshold (pixels)"),
                     association_threshold_edit_);

  QHBoxLayout *hbox = new QHBoxLayout;
  hbox->addLayout(left_form);
  hbox->addLayout(right_form);
  gp->setLayout(hbox);
  return gp;
}

}  // namespace soax

