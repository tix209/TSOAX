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
 * This class implements the dialog for setting visualization parameters.
 */

#include "include/view_options_dialog.h"
#include <QLineEdit>
#include <QGroupBox>
#include <QDialogButtonBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QLabel>

namespace soax {

ViewOptionsDialog::ViewOptionsDialog(QWidget *parent) : QDialog(parent) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(this->CreateSlicePlanesGroup());
  layout->addWidget(this->CreateMIPGroup());
  layout->addWidget(this->CreateClipGroup());
  layout->addWidget(this->CreateColorOrientationGroup());
  button_box_ = new QDialogButtonBox(QDialogButtonBox::Ok |
                                     QDialogButtonBox::Cancel);

  layout->addWidget(button_box_);
  setLayout(layout);
  setWindowTitle(tr("View Options"));
  connect(button_box_, SIGNAL(accepted()), this, SLOT(accept()));
  connect(button_box_, SIGNAL(rejected()), this, SLOT(reject()));
}


double ViewOptionsDialog::GetWindow() const {
  return window_edit_->text().toDouble();
}

double ViewOptionsDialog::GetLevel() const {
  return level_edit_->text().toDouble();
}

void ViewOptionsDialog::SetWindow(double window) {
  QString s;
  s.setNum(window);
  window_edit_->setText(s);
}

void ViewOptionsDialog::SetLevel(double level) {
  QString s;
  s.setNum(level);
  level_edit_->setText(s);
}

double ViewOptionsDialog::GetMinIntensity() const {
  return min_intensity_edit_->text().toDouble();
}

double ViewOptionsDialog::GetMaxIntensity() const {
  return max_intensity_edit_->text().toDouble();
}

void ViewOptionsDialog::SetMinIntensity(double min) {
  QString s;
  s.setNum(min);
  min_intensity_edit_->setText(s);
}

void ViewOptionsDialog::SetMaxIntensity(double max) {
  QString s;
  s.setNum(max);
  max_intensity_edit_->setText(s);
}

double ViewOptionsDialog::GetClipSpan() const {
  return clip_span_edit_->text().toDouble();
}

void ViewOptionsDialog::SetClipSpan(double span) {
  QString s;
  s.setNum(span);
  clip_span_edit_->setText(s);
}

unsigned ViewOptionsDialog::GetColorSegmentStep() const {
  return color_segment_step_edit_->text().toUInt();
}

void ViewOptionsDialog::SetColorSegmentStep(unsigned step) {
  QString s;
  s.setNum(step);
  color_segment_step_edit_->setText(s);
}

/****************** Private Methods ********************/

QGroupBox * ViewOptionsDialog::CreateSlicePlanesGroup() {
  QGroupBox *gb = new QGroupBox(tr("Window/Level for Slice Planes"));
  window_edit_ = new QLineEdit("0");
  level_edit_ = new QLineEdit("0");

  connect(window_edit_, SIGNAL(textChanged(const QString &)),
          this, SLOT(EnableOKButton()));
  connect(level_edit_, SIGNAL(textChanged(const QString &)),
          this, SLOT(EnableOKButton()));

  QLabel *window_label = new QLabel(tr("Win"));
  QLabel *level_label = new QLabel(tr("Lev"));

  QHBoxLayout *layout = new QHBoxLayout;
  layout->addWidget(window_label);
  layout->addWidget(window_edit_);
  layout->addWidget(level_label);
  layout->addWidget(level_edit_);

  gb->setLayout(layout);
  return gb;
}

QGroupBox * ViewOptionsDialog::CreateMIPGroup() {
  QGroupBox *gb = new QGroupBox(tr("Intensity Range for MIP Rendering"));
  min_intensity_edit_ = new QLineEdit("0");
  max_intensity_edit_ = new QLineEdit("0");

  connect(min_intensity_edit_, SIGNAL(textChanged(const QString &)),
          this, SLOT(EnableOKButton()));
  connect(max_intensity_edit_, SIGNAL(textChanged(const QString &)),
          this, SLOT(EnableOKButton()));

  QLabel *min_intensity_label = new QLabel(tr("Min"));
  QLabel *max_intensity_label = new QLabel(tr("Max"));

  QHBoxLayout *layout = new QHBoxLayout;
  layout->addWidget(min_intensity_label);
  layout->addWidget(min_intensity_edit_);
  layout->addWidget(max_intensity_label);
  layout->addWidget(max_intensity_edit_);
  gb->setLayout(layout);
  return gb;
}

QGroupBox * ViewOptionsDialog::CreateClipGroup() {
  QGroupBox *gb = new QGroupBox(tr("Show Snake Locally"));
  clip_span_edit_ = new QLineEdit("0.0");
  connect(clip_span_edit_, SIGNAL(textChanged(const QString &)),
          this, SLOT(EnableOKButton()));
  QLabel *label = new QLabel(tr("Interval"));
  QHBoxLayout *layout = new QHBoxLayout;
  layout->addWidget(label);
  layout->addWidget(clip_span_edit_);
  layout->addStretch();
  gb->setLayout(layout);
  return gb;
}

QGroupBox *ViewOptionsDialog::CreateColorOrientationGroup() {
  QGroupBox *gb = new QGroupBox(tr("Color Snake Orientation"));
  color_segment_step_edit_ = new QLineEdit("0");
  connect(color_segment_step_edit_, SIGNAL(textChanged(const QString &)),
          this, SLOT(EnableOKButton()));
  QLabel *label = new QLabel(tr("Segment size"));
  QHBoxLayout *layout = new QHBoxLayout;
  layout->addWidget(label);
  layout->addWidget(color_segment_step_edit_);
  layout->addStretch();
  gb->setLayout(layout);
  return gb;
}

void ViewOptionsDialog::EnableOKButton() {
  button_box_->button(QDialogButtonBox::Ok)->setEnabled(true);
}

void ViewOptionsDialog::DisableOKButton() {
  button_box_->button(QDialogButtonBox::Ok)->setEnabled(false);
}


}  // namespace soax
