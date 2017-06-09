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
 * This class is the main window of the Troax program.
 */


#ifndef MAIN_WINDOW_H_
#define MAIN_WINDOW_H_

#include <QMainWindow>

class QProgressBar;
class QScrollBar;
class QActionGroup;
class QVTKWidget;
class QVTKOpenGLWidget;

namespace soax {
class ImageReader;
class Viewer;
class Multisnake;
class ParametersDialog;
class ViewOptionsDialog;
class AnalysisOptionsDialog;


class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow();
  ~MainWindow();

  /**
   * Disable right click on the toolbar.
   */
  virtual QMenu *createPopupMenu() {return NULL;}

 private slots:  // NOLINT(whitespace/indent)
  void OpenImageFile();
  void OpenImageDir();
  void SaveIsotropicImage();
  void LoadParameters();
  void SaveParameters();
  void LoadSnakes();
  void SaveSnakes();
  void CompareSnakes();
  // void LoadTamaraSnakes();
  void CloseSession();

  void ShowTracks();
  void ShowViewOptions();

  void InitializeSnakes();
  void DeformSnakes();
  // void TrackSnakes();
  // void TrackSnakesFinal();
  void SolveCorrespondence();
  void ShowParametersDialog();

  /**
   * Compute snake local oriention for the image or frames.
   */
  void ComputeSphericalOrientation();
  // void ComputeRadialOrientation();
  // void ComputePointDensity();
  // void ComputeCurvature();
  void ShowAnalysisOptions();

  void LoadViewpoint();
  void SaveViewpoint();
  void SaveSnapshot();

  void AboutTroax();

  void TogglePlanes(bool state);
  void ToggleVolumeRendering(bool state);
  void ToggleCornerText(bool state);
  void ToggleBoundingBox(bool state);

  void UpdateImageView(int index = 0);
  void UpdateSnakeView(int index = 0);
  void UpdateComparingSnakeView(int index = 0);
  void UpdateJunctionView(int index = 0);
  void UpdateComparingJunctionView(int index);
  void UpdateFrameNumber(int index = 0);
  void UpdateWindowTitle(int index = 0);

 private:
  void CreateMenus();
  void CreateActions();
  void CreateFileMenuActions();
  void CreateEditMenuActions();
  void CreateViewMenuActions();
  void CreateProcessMenuActions();
  void CreateAnalysisMenuActions();
  void CreateToolsMenuActions();
  void CreateHelpMenuActions();
  void CreateToolBar();
  void CreateScrollBar();
  void CreateProgressBar();
  void ResetActions();

  /**
   * Common procedure for OpenImageFile() and OpenImageDir().
   */
  void ShowImage();

  ImageReader *reader_;
  Viewer *viewer_;
  Multisnake *multisnake_;
  QWidget *central_widget_;
  ParametersDialog *parameters_dialog_;
  ViewOptionsDialog *view_options_dialog_;
  AnalysisOptionsDialog *analysis_options_dialog_;

  // Menus
  QMenu *file_;
  QMenu *edit_;
  QMenu *view_;
  QMenu *process_;
  QMenu *analysis_;
  QMenu *tools_;
  QMenu *help_;

  // Actions in file menu
  QAction *open_image_file_;
  QAction *open_image_dir_;
  QAction *save_isotropic_image_;
  QAction *load_parameters_;
  QAction *save_parameters_;
  QAction *load_snakes_;
  QAction *compare_snakes_;
  // QAction *load_tamara_snakes_;
  QAction *save_snakes_;
  QAction *close_session_;
  QAction *exit_;


  // Actions in Edit menu
  QAction *toggle_normal_;
  // QAction *toggle_delete_snake_;
  // QAction *toggle_trim_tip_;
  // QAction *toggle_extend_tip_;
  // QAction *toggle_trim_body_;
  // QAction *toggle_delete_junction_;
  // QAction *edit_snake_;
  QActionGroup *snake_edit_group_;

  // Actions in View menu
  QAction *toggle_slice_planes_;
  QAction *toggle_volume_rendering_;
  QAction *toggle_orientation_marker_;
  QAction *toggle_corner_text_;
  QAction *toggle_bounding_box_;
  QAction *toggle_snakes_;
  QAction *show_tracks_;
  QAction *toggle_comparing_snakes_;
  QAction *toggle_junctions_;
  QAction *toggle_comparing_junctions_;
  QAction *toggle_clip_;
  QAction *toggle_azimuthal_;
  QAction *toggle_polar_;
  QAction *load_viewpoint_;
  QAction *save_viewpoint_;
  QAction *reset_viewpoint_;
  QAction *save_snapshot_;
  QAction *show_view_options_;

  // Actions in Process menu
  QAction *initialize_snakes_;
  QAction *deform_snakes_;
  // QAction *track_snakes_;
  // QAction *track_snakes_final_;
  QAction *solve_correspondence_;
  // QAction *cut_snakes_;
  // QAction *group_snakes_;
  QAction *show_parameters_;

  // Actions in Analysis menu
  QAction *compute_spherical_orientation_;
  // QAction *compute_radial_orientation_;
  // QAction *compute_point_density_;
  // QAction *compute_curvature_;
  QAction *show_analysis_options_;

  // Actions in Help menu
  QAction *about_troax_;
  QAction *about_qt_;

  QToolBar *toolbar_;
  QScrollBar *scroll_bar_;
  QProgressBar *progress_bar_;

  QVTKOpenGLWidget *qvtk_ = nullptr;

  size_t track_examined_ = 0;
  size_t stepsize_ = 0;
  size_t track_number_ = 0;

  QString analysis_dir_ = "..";

  MainWindow(const MainWindow &);
  void operator=(const MainWindow &);
};

}  // namespace soax

#endif  // MAIN_WINDOW_H_
