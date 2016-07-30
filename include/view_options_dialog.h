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
 * This class is the dialog for setting visulization parameters.
 */

#ifndef VIEW_OPTIONS_DIALOG_H_
#define VIEW_OPTIONS_DIALOG_H_

#include <QDialog>

class QLineEdit;
class QGroupBox;
class QDialogButtonBox;

namespace soax {

class ViewOptionsDialog : public QDialog {
  Q_OBJECT

 public:
  explicit ViewOptionsDialog(QWidget * parent = NULL);

  double GetWindow() const;
  double GetLevel() const;

  void SetWindow(double window);
  void SetLevel(double level);

  double GetMinIntensity() const;
  double GetMaxIntensity() const;

  void SetMinIntensity(double min);
  void SetMaxIntensity(double max);

  double GetClipSpan() const;
  void SetClipSpan(double span);

  unsigned GetColorSegmentStep() const;
  void SetColorSegmentStep(unsigned step);

 public slots:  // NOLINT(whitespace/indent)
  void EnableOKButton();
  void DisableOKButton();

 private:
  QGroupBox *CreateSlicePlanesGroup();
  QGroupBox *CreateMIPGroup();
  QGroupBox *CreateClipGroup();
  QGroupBox *CreateColorOrientationGroup();

  QLineEdit *window_edit_;
  QLineEdit *level_edit_;
  QLineEdit *min_intensity_edit_;
  QLineEdit *max_intensity_edit_;
  QLineEdit *clip_span_edit_;
  QLineEdit *color_segment_step_edit_;

  QDialogButtonBox *button_box_;

  ViewOptionsDialog(const ViewOptionsDialog &);
  void operator=(const ViewOptionsDialog &);
};

}  // namespace soax

#endif  // VIEW_OPTIONS_DIALOG_H_
