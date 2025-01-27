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
 * This file implements the main window of the TSOAX program.
 */


#include "include/main_window.h"
#include <QMenuBar>
#include <QProgressBar>
#include <QScrollBar>
#include <QToolBar>
#include <QActionGroup>
#include <QApplication>
#include <QStatusBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QInputDialog>
#include "QVTKOpenGLWidget.h"

#include "include/util.h"
#include "include/image_reader.h"
#include "include/viewer.h"
#include "include/viewpoint.h"
#include "include/multisnake.h"
#include "include/snake_parameters.h"
#include "include/snake_actor.h"
#include "include/image.h"
#include "include/image_resampler.h"
#include "include/parameters_dialog.h"
#include "include/view_options_dialog.h"
#include "include/image_plane.h"
#include "include/slice_planes.h"
#include "include/volume_rendering.h"
#include "include/analysis_options_dialog.h"
// #ifdef __APPLE__
// #include "include/osx_helper.h"
// #endif

namespace soax {

MainWindow::MainWindow() {
  qvtk_ = new QVTKOpenGLWidget(this);

  reader_ = new ImageReader;
  viewer_ = new Viewer(qvtk_);
  multisnake_ = new Multisnake;
  central_widget_ = new QWidget(this);
  parameters_dialog_ = new ParametersDialog(this);
  view_options_dialog_ = new ViewOptionsDialog(this);
  analysis_options_dialog_ = new AnalysisOptionsDialog(this);

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(qvtk_);
  CreateScrollBar();
  layout->addWidget(scroll_bar_);
  central_widget_->setLayout(layout);
  setCentralWidget(central_widget_);

// #ifdef __APPLE__
//   disableGLHiDPI(qvtk_->winId());
// #endif

  // These have to come after setCentralWidget otherwise runtime error will
  // be emitted.
  CreateMenus();
  CreateActions();
  CreateToolBar();
  CreateProgressBar();

  setWindowIcon(QIcon(":/icon/Letter-T.png"));
  setWindowTitle("TSOAX");
  // setUnifiedTitleAndToolBarOnMac(true);

  ResetActions();
}

MainWindow::~MainWindow() {
  delete multisnake_;
  delete viewer_;
  delete reader_;
}

/********************** Private Slots **********************/
void MainWindow::OpenImageFile() {
  QString filename = QFileDialog::getOpenFileName(
      this, tr("Open an Image File"), reader_->path(),
      reader_->GetAllowedFormatAsString());
  if (filename.isEmpty())  return;

  QMessageBox msg;
  msg.setText(tr("A single image file has been selected."));
  msg.setInformativeText(tr("Does it have multiple time frames?"));
  msg.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
  msg.setIcon(QMessageBox::Question);
  msg.setDefaultButton(QMessageBox::No);

  if (msg.exec() == QMessageBox::Yes) {
    bool ok;
    int nslices_per_frame = QInputDialog::getInt(
        this, tr("TSOAX"), tr("How many z-slices does each frame have?"),
        1, 1, 2147483647, 1, &ok);
    if (ok) {
      reader_->set_nslices_per_frame(nslices_per_frame);
      connect(scroll_bar_, SIGNAL(valueChanged(int)),
              this, SLOT(UpdateFrameNumber(int)));
    }
  }

  reader_->ReadFile(filename);
  // reader_->ReverseImageSequence();
  ShowImage();
}

void MainWindow::OpenImageDir() {
  QString dir = QFileDialog::getExistingDirectory(
      this, tr("Open an Image Folder"), reader_->path(),
      QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
  if (dir.isEmpty())  return;

  reader_->ReadDir(dir);
  // reader_->ReverseImageSequence();

  connect(scroll_bar_, SIGNAL(valueChanged(int)),
          this, SLOT(UpdateFrameNumber(int)));
  connect(scroll_bar_, SIGNAL(valueChanged(int)),
          this, SLOT(UpdateWindowTitle(int)));

  ShowImage();
}

void MainWindow::SaveIsotropicImage() {
  QString dir = QFileDialog::getExistingDirectory(
      this, tr("Open Folder for Saving"), reader_->path(),
      QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
  if (dir.isEmpty())  return;
  // A dialog asking for the image z spacing relative to x/y
  bool ok;
  double z_spacing = QInputDialog::getDouble(
      this, tr("Set Z spacing"), tr("Z Spacing (relative to X/Y)"),
      1.0, 0.1, 10, 4, &ok);
  if (!ok) return;

  ImageResampler resampler;
  for (std::size_t i = 0; i < reader_->GetNumberOfImages(); ++i) {
    QString path = QDir::toNativeSeparators(
        dir + "/" + reader_->GetFileNameWithoutSuffix(i) + "_iso_"
        + QString::number(i) + ".tif");
    resampler.Resize(reader_->GetImage(i), z_spacing);
    resampler.WriteImage(path.toStdString());
  }
}

void MainWindow::LoadParameters() {
  QString filename = QFileDialog::getOpenFileName(
      this, tr("Open Parameter File"),
      multisnake_->snake_parameters()->path(), tr("Text Files (*.txt)"));
  if (filename.isEmpty()) return;
  if (multisnake_->snake_parameters()->Load(filename))
    statusBar()->showMessage("Parameters are successfully loaded.");
  else
    statusBar()->showMessage("Problem occurred loading parameters!");
}

void MainWindow::SaveParameters() {
  QString filename = QFileDialog::getSaveFileName(
      this, tr("Save Parameters"), multisnake_->snake_parameters()->path(),
      tr("Text Files (*.txt)"));
  if (filename.isEmpty()) return;
  if (multisnake_->snake_parameters()->Save(filename))
    statusBar()->showMessage("Parameters are successfully saved.");
  else
    statusBar()->showMessage("Problem occurred saving parameters!");
}

void MainWindow::LoadSnakes() {
  QString filename = QFileDialog::getOpenFileName(
      this, tr("Load Snakes"), multisnake_->path(), tr("Text Files (*.txt)"));
  if (filename.isEmpty()) return;

  if (multisnake_->LoadConvergedSnakes(filename))
    statusBar()->showMessage("Snakes are successfully loaded.");
  else
    statusBar()->showMessage("Problem occurred loading snakes!");

  save_snakes_->setEnabled(true);
  toggle_snakes_->setEnabled(true);
  toggle_snakes_->setChecked(true);
  compute_spherical_orientation_->setEnabled(true);
  show_analysis_options_->setEnabled(true);

  UpdateSnakeView(scroll_bar_->value());
  connect(scroll_bar_, SIGNAL(valueChanged(int)),
          this, SLOT(UpdateSnakeView(int)));

  toggle_junctions_->setEnabled(true);
  toggle_junctions_->setChecked(true);
  UpdateJunctionView(scroll_bar_->value());
  connect(scroll_bar_, SIGNAL(valueChanged(int)),
          this, SLOT(UpdateJunctionView(int)));
  viewer_->UpdateCornerTextSnake(filename);
  viewer_->Render();

  solve_correspondence_->setEnabled(true);
  show_tracks_->setEnabled(true);
}

void MainWindow::SaveSnakes() {
  QString filename = QFileDialog::getSaveFileName(
      this, tr("Save Snakes"), multisnake_->path(),
      tr("Text Files (*.txt)"));
  if (filename.isEmpty()) return;

  std::ofstream outfile(filename.toStdString().c_str());
  if (!outfile) {
    QString msg = QString("Couldn't open file: ") + filename;
    statusBar()->showMessage(msg);
    return;
  }

  outfile << "image\t" << reader_->GetFilePath().toStdString() << std::endl;
  outfile << multisnake_->snake_parameters()->ToString();
  int dim = GetImageDimension(reader_->GetImage());
  outfile << "dimension\t" << dim << std::endl;

  for (size_t i = 0; i < reader_->GetNumberOfImages(); i++) {
    multisnake_->set_image(reader_->GetImage(i));
    multisnake_->SaveConvergedSnakes(i, outfile, dim);
  }

  multisnake_->SaveConvergedSnakeTrack(outfile);
  outfile.close();
  multisnake_->set_path(filename);

  statusBar()->showMessage(QString("Snakes are saved in ") + filename);
}

void MainWindow::CompareSnakes() {
  QString filename = QFileDialog::getOpenFileName(
      this, tr("Compare Snakes"), multisnake_->path(),
      tr("Text Files (*.txt)"));
  if (filename.isEmpty()) return;

  if (multisnake_->LoadComparingSnakes(filename))
    statusBar()->showMessage("Comparing snakes are successfully loaded.");
  else
    statusBar()->showMessage("Problem occurred loading comparing snakes!");

  toggle_comparing_snakes_->setEnabled(true);
  toggle_comparing_snakes_->setChecked(true);
  UpdateComparingSnakeView(scroll_bar_->value());
  connect(scroll_bar_, SIGNAL(valueChanged(int)),
          this, SLOT(UpdateComparingSnakeView(int)));

  toggle_comparing_junctions_->setEnabled(true);
  toggle_comparing_junctions_->setChecked(true);
  UpdateComparingJunctionView(scroll_bar_->value());
  connect(scroll_bar_, SIGNAL(valueChanged(int)),
          this, SLOT(UpdateComparingJunctionView(int)));

  viewer_->UpdateCornerTextComparingSnake(filename);
  viewer_->Render();
}

void MainWindow::CloseSession() {
  toggle_slice_planes_->setChecked(false);
  toggle_volume_rendering_->setChecked(false);
  toggle_orientation_marker_->setChecked(false);
  toggle_corner_text_->setChecked(false);
  toggle_bounding_box_->setChecked(false);
  toggle_snakes_->setChecked(false);
  toggle_comparing_snakes_->setChecked(false);
  toggle_junctions_->setChecked(false);
  toggle_comparing_junctions_->setChecked(false);

  multisnake_->Reset();
  viewer_->Reset();
  reader_->Reset();

  disconnect(scroll_bar_, SIGNAL(valueChanged(int)),
             this, SLOT(UpdateWindowTitle(int)));
  disconnect(scroll_bar_, SIGNAL(valueChanged(int)),
             this, SLOT(UpdateFrameNumber(int)));
  disconnect(scroll_bar_, SIGNAL(valueChanged(int)),
             this, SLOT(UpdateImageView(int)));
  disconnect(scroll_bar_, SIGNAL(valueChanged(int)),
             this, SLOT(UpdateSnakeView(int)));
  disconnect(scroll_bar_, SIGNAL(valueChanged(int)),
             this, SLOT(UpdateComparingSnakeView(int)));
  disconnect(scroll_bar_, SIGNAL(valueChanged(int)),
             this, SLOT(UpdateJunctionView(int)));
  disconnect(scroll_bar_, SIGNAL(valueChanged(int)),
             this, SLOT(UpdateComparingJunctionView(int)));

  scroll_bar_->setMaximum(0);
  statusBar()->showMessage("");
  ResetActions();

  open_image_file_->setEnabled(true);
  open_image_dir_->setEnabled(true);
  setWindowTitle("TSOAX");

  track_examined_ = 0;
  stepsize_ = 0;
  track_number_ = 0;
}

void MainWindow::ShowTracks() {
  const size_t max_tracks = multisnake_->GetNumberOfTracks();
  if (!max_tracks) return;
  if (track_number_ >= max_tracks) {
    QMessageBox msg;
    msg.setText(tr("All tracks have been examined."));
    msg.setInformativeText(tr("Do you want to start over?"));
    msg.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
    msg.setIcon(QMessageBox::Question);
    msg.setDefaultButton(QMessageBox::Yes);
    if (msg.exec() == QMessageBox::Yes) {
      track_examined_ = 0;
      track_number_ = 0;
    } else {
      return;
    }
  }

  if (!track_examined_) {
    bool ok;
    QString question = QString("# of tracks to check: (max: ") +
                       QString::number(max_tracks) + ")";
    int num_tracks = QInputDialog::getInt(
        this, tr("TSOAX"), question, 2, 2, static_cast<int>(max_tracks), 1, &ok);
    stepsize_ = max_tracks / num_tracks;
  }

  // display next track and set the frame number to 1
  int start = multisnake_->GetFrameOfFirstAppearance(track_number_);

  if (start >= 0) {
    Snake *s = multisnake_->GetFirstSnake(track_number_);
    viewer_->snake_actor()->Highlight(s);
    viewer_->snake_actor()->set_selected_snake(s);
    scroll_bar_->setValue(start);
    viewer_->Render();
  }

  std::ostringstream buffer;
  buffer << "Track " << track_examined_ << "/" << max_tracks
         << " (#" << track_number_ << ") starts from frame "
         << start + 1 << "." << std::endl;

  QString msg = buffer.str().c_str();
  statusBar()->showMessage(msg);

  track_number_ += stepsize_;
  ++track_examined_;
}

void MainWindow::ShowViewOptions() {
  if (view_options_dialog_->exec()) {
    double window = view_options_dialog_->GetWindow();
    double level = view_options_dialog_->GetLevel();
    double min_intensity = view_options_dialog_->GetMinIntensity();
    double max_intensity = view_options_dialog_->GetMaxIntensity();

    if (GetImageDimension(reader_->GetImage(scroll_bar_->value())) == 3) {
      viewer_->slice_planes()->SetWindowLevel(window, level);
      viewer_->volume()->SetDisplayRange(min_intensity, max_intensity);
    } else {
      viewer_->image_plane()->SetWindowLevel(window, level);
    }
    view_options_dialog_->DisableOKButton();
  }
  viewer_->Render();
}

void MainWindow::InitializeSnakes() {
  multisnake_->set_image(reader_->GetImage(scroll_bar_->value()));
  multisnake_->Initialize();
  QString msg = QString::number(multisnake_->GetNumberOfInitialSnakes()) +
                " snakes initialized.";
  statusBar()->showMessage(msg);

  viewer_->RemoveSnakes();
  viewer_->SetupSnakes(multisnake_->initial_snakes());

  toggle_snakes_->setEnabled(true);
  toggle_snakes_->setChecked(true);
  viewer_->Render();

  toggle_normal_->setEnabled(true);
}

void MainWindow::DeformSnakes() {
  multisnake_->DeleteConvergedSnakeSequence();
  std::cout << "============ Parameters ============" << std::endl;
  std::cout << multisnake_->snake_parameters()->ToString() << std::endl;
  std::cout << "====================================" << std::endl;

  connect(multisnake_, SIGNAL(ExtractionProgressed(int)),
          progress_bar_, SLOT(setValue(int)));

  time_t start, end;
  time(&start);
  for (size_t i = 0; i < reader_->GetNumberOfImages(); i++) {
    multisnake_->set_image(reader_->GetImage(i));
    multisnake_->Initialize();
    // multisnake_->AddLastConvergedToInitialSnakes(i);

    // the following is to reuse converged snake from previous frame so
    // that the topology stays stable
    // if (i == 0)
    //   multisnake_->Initialize();
    // else
    //   multisnake_->CopyConvergedToInitial(i - 1);

    std::cout << "Extracting frame " << i << "..." << std::endl;
    progress_bar_->setMaximum(static_cast<int>(multisnake_->GetNumberOfInitialSnakes()));
    multisnake_->Evolve();
    if (multisnake_->snake_parameters()->grouping()) {
      multisnake_->Reconfigure(i);
    }
    else {
      multisnake_->ReconfigureWithoutGrouping(i);
    }
  }
  time(&end);
  double time_elasped = difftime(end, start);
  QString msg = QString("Extraction completed in ")
                + QString::number(time_elasped) + "s.";
  statusBar()->showMessage(msg);

  // statusBar()->showMessage("Solving correspondence...");
  // multisnake_->SolveCorrespondence(reader_->GetNumberOfImages());
  // statusBar()->showMessage("Correspondence solved.");

  toggle_snakes_->setEnabled(true);
  toggle_snakes_->setChecked(true);
  UpdateSnakeView(scroll_bar_->value());
  connect(scroll_bar_, SIGNAL(valueChanged(int)),
          this, SLOT(UpdateSnakeView(int)));

  toggle_junctions_->setEnabled(true);
  toggle_junctions_->setChecked(true);
  UpdateJunctionView(scroll_bar_->value());
  connect(scroll_bar_, SIGNAL(valueChanged(int)),
          this, SLOT(UpdateJunctionView(int)));

  save_snakes_->setEnabled(true);
  solve_correspondence_->setEnabled(true);
  show_tracks_->setEnabled(false);
  compute_spherical_orientation_->setEnabled(true);
  show_analysis_options_->setEnabled(true);
}

void MainWindow::SolveCorrespondence() {
  time_t start, end;
  time(&start);
  
  statusBar()->showMessage("Solving correspondence...");
  multisnake_->SolveCorrespondence(reader_->GetNumberOfImages());
  statusBar()->showMessage("Correspondence solved.");
  
  
  time(&end);
  double time_elasped = difftime(end, start);
  QString msg = QString("Correspondence completed in ")
                + QString::number(time_elasped) + "s.";
  statusBar()->showMessage(msg);
  
  show_tracks_->setEnabled(true);
}

void MainWindow::ComputeSphericalOrientation() {
  // TODO: save the orientation on a frame basis
  QString dir = QFileDialog::getExistingDirectory(
      this, tr("Open folder for spherical orientation "), analysis_dir_,
      QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
  if (dir.isEmpty())  return;
  analysis_dir_ = dir;

  // Assume all the frames have the same dimension
  int dim = GetImageDimension(reader_->GetImage());
  PointType center(dim);
  if (!analysis_options_dialog_->GetImageCenter(center)) {
    statusBar()->showMessage("Image center is invalid!");
    return;
  }
  std::cout << center << std::endl;
  double inside_percentage = 1.0;
  if (!analysis_options_dialog_->GetInsideRatio(&inside_percentage)) {
    statusBar()->showMessage("Inside ratio is invalid!");
    return;
  }

  double radius = 0.0;
  if (!analysis_options_dialog_->GetRadius(&radius)) {
    statusBar()->showMessage("Radius is invalid!");
    return;
  }

  double padding = 0.0;
  if (analysis_options_dialog_->ExcludeBoundaryChecked()) {
    padding = 2.0;
  }

  double max_r = inside_percentage * radius;
  for (std::size_t i = 0; i < reader_->GetNumberOfImages(); ++i) {
    QString path = QDir::toNativeSeparators(
        dir + "/" + reader_->GetFileNameWithoutSuffix(i) + "_spherical_"
        + QString::number(i) + ".csv");
    std::ofstream outfile;
    outfile.open(path.toStdString().c_str());
    if (!outfile.is_open()) {
      statusBar()->showMessage("Open file failed!");
      return;
    }

    multisnake_->ComputeSphericalOrientation(reader_->GetImage(i),
                                             center, max_r, padding, i, outfile);
    outfile.close();
  }
  statusBar()->showMessage(tr("Spherical orientation files are saved."));
}

void MainWindow::ShowParametersDialog() {
  parameters_dialog_->SetParameters(multisnake_->snake_parameters());
  if (parameters_dialog_->exec()) {
    multisnake_->snake_parameters()->set_intensity_scaling(
        parameters_dialog_->GetIntensityScaling());
    multisnake_->snake_parameters()->set_sigma(
        parameters_dialog_->GetSigma());
    multisnake_->snake_parameters()->set_ridge_threshold(
        parameters_dialog_->GetRidgeThreshold());
    multisnake_->snake_parameters()->set_maximum_foreground(
        parameters_dialog_->GetMaximumForeground());
    multisnake_->snake_parameters()->set_minimum_foreground(
        parameters_dialog_->GetMinimumForeground());
    multisnake_->snake_parameters()->set_init_direction(
        0, parameters_dialog_->InitXChecked());
    multisnake_->snake_parameters()->set_init_direction(
        1, parameters_dialog_->InitYChecked());
    multisnake_->snake_parameters()->set_init_direction(
        2, parameters_dialog_->InitZChecked());
    multisnake_->snake_parameters()->set_spacing(
        parameters_dialog_->GetSpacing());
    multisnake_->snake_parameters()->set_minimum_length(
        parameters_dialog_->GetMinimumLength());
    multisnake_->snake_parameters()->set_maximum_iterations(
        parameters_dialog_->GetMaximumIterations());
    multisnake_->snake_parameters()->set_change_threshold(
        parameters_dialog_->GetChangeThreshold());
    multisnake_->snake_parameters()->set_check_period(
        parameters_dialog_->GetCheckPeriod());
    multisnake_->snake_parameters()->set_alpha(
        parameters_dialog_->GetAlpha());
    multisnake_->snake_parameters()->set_beta(
        parameters_dialog_->GetBeta());
    multisnake_->snake_parameters()->set_gamma(
        parameters_dialog_->GetGamma());
    multisnake_->snake_parameters()->set_external_factor(
        parameters_dialog_->GetExternalFactor());
    multisnake_->snake_parameters()->set_stretch_factor(
        parameters_dialog_->GetStretchFactor());
    multisnake_->snake_parameters()->set_number_of_sectors(
        parameters_dialog_->GetNumberOfSectors());
    multisnake_->snake_parameters()->set_zspacing(
        parameters_dialog_->GetZSpacing());
    multisnake_->snake_parameters()->set_radial_near(
        parameters_dialog_->GetRadialNear());
    multisnake_->snake_parameters()->set_radial_far(
        parameters_dialog_->GetRadialFar());
    multisnake_->snake_parameters()->set_delta(
        parameters_dialog_->GetDelta());
    multisnake_->snake_parameters()->set_overlap_threshold(
        parameters_dialog_->GetOverlapThreshold());
    multisnake_->snake_parameters()->set_grouping_distance_threshold(
        parameters_dialog_->GetGroupingDistanceThreshold());
    multisnake_->snake_parameters()->set_grouping_delta(
        parameters_dialog_->GetGroupingDelta());
    multisnake_->snake_parameters()->set_direction_threshold(
        parameters_dialog_->GetDirectionThreshold());
    multisnake_->snake_parameters()->set_damp_z(
        parameters_dialog_->DampZ());
    multisnake_->snake_parameters()->set_association_threshold(
        parameters_dialog_->GetAssociationThreshold());
    multisnake_->snake_parameters()->set_c(
        parameters_dialog_->GetC());
    multisnake_->snake_parameters()->set_grouping(
        parameters_dialog_->Grouping());
  }
}

void MainWindow::LoadViewpoint() {
  QString filename = QFileDialog::getOpenFileName(
      this, tr("Load Viewpoint"), viewer_->GetViewpointPath(),
      tr("Text Files (*.txt *.cam)"));
  if (filename.isEmpty())  return;
  viewer_->LoadViewpoint(filename);
  QString msg = QFileInfo(filename).fileName() + " loaded.";
  statusBar()->showMessage(msg);
}

void MainWindow::SaveViewpoint() {
  QString filename = QFileDialog::getSaveFileName(
      this, tr("Save Viewpoint"), viewer_->GetViewpointPath(),
      tr("Text Files (*.cam *.txt)"));
  if (filename.isEmpty()) return;
  viewer_->SaveViewpoint(filename);
  QString msg = QFileInfo(filename).fileName() + " saved.";
  statusBar()->showMessage(msg);
}

void MainWindow::SaveSnapshot() {
  QString filename = QFileDialog::getSaveFileName(
      this, tr("Save Snapshot"), viewer_->snapshot_path(),
      tr("Image Files (*.png)"));
  if (filename.isEmpty()) return;
  viewer_->SaveWindowImage(filename);
}

void MainWindow::ShowAnalysisOptions() {
  if (analysis_options_dialog_->exec()) {
    analysis_options_dialog_->DisableOKButton();
  }
}

void MainWindow::AboutTSOAX() {
  QMessageBox::about(
      this, tr("About TSOAX"),
      tr("<center>TSOAX v0.2.1</center>"
         "<p style=\"font-size:11px;font-weight:normal\">"
         "TSOAX extract and track the growth and deformation of biopolymer networks from 2D and 3D time-lapse sequences. This work was supported by NIH grants R35GM136372, R01GM114201, R01GM098430.</p>"
         "<center><p><a href=\"https://github.com/tix209/tsoax\">TSOAX on GitHub</a></p>"
         "<p style=\"font-size:11px; font-weight:normal\">"
         "&copy; 2018 Lehigh University.</p></center>" ));
}

void MainWindow::TogglePlanes(bool state) {
  if (GetImageDimension(reader_->GetImage(scroll_bar_->value())) == 3)
    viewer_->ToggleSlicePlanes(
        state, reader_->GetImage(scroll_bar_->value()));
  else
    viewer_->ToggleImagePlane(state, reader_->GetImage(scroll_bar_->value()));
}

void MainWindow::ToggleVolumeRendering(bool state) {
  viewer_->ToggleVolumeRendering(
      state, reader_->GetImage(scroll_bar_->value()));
}

void MainWindow::ToggleCornerText(bool state) {
  viewer_->ToggleCornerText(state, reader_->GetImage(scroll_bar_->value()));
}

void MainWindow::ToggleBoundingBox(bool state) {
  viewer_->ToggleBoundingBox(state, reader_->GetImage(scroll_bar_->value()));
}

void MainWindow::UpdateWindowTitle(int index) {
  setWindowTitle(QString("TSOAX - ") + reader_->GetFilePath(index));
}

void MainWindow::UpdateFrameNumber(int index) {
  QString msg = QString::number(index + 1) + "/"
                + QString::number(reader_->GetNumberOfImages());
  statusBar()->showMessage(msg);
}

void MainWindow::UpdateImageView(int index) {
  if (GetImageDimension(reader_->GetImage(index)) == 3) {
    toggle_volume_rendering_->setEnabled(true);
    if (toggle_slice_planes_->isChecked()) {
      viewer_->UpdateSlicePlanes(reader_->GetImage(index));
    }
    if (toggle_volume_rendering_->isChecked()) {
      viewer_->UpdateVolumeRendering(reader_->GetImage(index));
    }
  } else {
    toggle_volume_rendering_->setEnabled(false);
    if (toggle_slice_planes_->isChecked()) {
      viewer_->UpdateImagePlane(reader_->GetImage(index));
    }
  }
  viewer_->UpdateCornerTextImageInfo(reader_->GetImage(index));
}

void MainWindow::UpdateSnakeView(int index) {
  if (!toggle_snakes_->isChecked()) return;
  viewer_->RemoveSnakes();
  if (static_cast<unsigned>(index) <
      multisnake_->GetNumberOfSetsOfConvergedSnakes()) {
    viewer_->SetupSnakes(multisnake_->GetConvergedSnakes(index));

    viewer_->snake_actor()->Highlight((multisnake_->GetCorrespondingSnake(
        viewer_->snake_actor()->selected_snake(), index)));

    toggle_snakes_->setEnabled(true);
    toggle_snakes_->setChecked(true);
    viewer_->Render();
  }
}

void MainWindow::UpdateComparingSnakeView(int index) {
  if (!toggle_comparing_snakes_->isChecked()) return;
  viewer_->RemoveComparingSnakes();
  if (static_cast<unsigned>(index) <
      multisnake_->GetNumberOfSetsOfComparingSnakes()) {
    viewer_->SetupComparingSnakes(multisnake_->GetComparingSnakes(index));
    toggle_comparing_snakes_->setEnabled(true);
    toggle_comparing_snakes_->setChecked(true);
    viewer_->Render();
  }
}

void MainWindow::UpdateJunctionView(int index) {
  if (!toggle_junctions_->isChecked()) return;
  viewer_->RemoveJunctions();
  if (static_cast<unsigned>(index) <
      multisnake_->GetNumberOfSetsOfJunctions()) {
    viewer_->SetupJunctions(multisnake_->GetJunctions(index));
    toggle_junctions_->setEnabled(true);
    toggle_junctions_->setChecked(true);
    viewer_->Render();
  }
}

void MainWindow::UpdateComparingJunctionView(int index) {
  if (!toggle_comparing_junctions_->isChecked()) return;
  viewer_->RemoveComparingJunctions();
  if (static_cast<unsigned>(index) <
      multisnake_->GetNumberOfSetsOfComparingJunctions()) {
    viewer_->SetupComparingJunctions(
        multisnake_->GetComparingJunctions(index));
    toggle_comparing_junctions_->setEnabled(true);
    toggle_comparing_junctions_->setChecked(true);
    viewer_->Render();
  }
}

/********************** Private Methods **********************/
void MainWindow::CreateMenus() {
  file_ = menuBar()->addMenu(tr("&File"));
  edit_ = menuBar()->addMenu(tr("&Edit"));
  view_ = menuBar()->addMenu(tr("&View"));
  process_ = menuBar()->addMenu(tr("&Process"));
  analysis_ = menuBar()->addMenu(tr("&Analysis"));
  tools_ = menuBar()->addMenu(tr("&Tools"));
  help_ = menuBar()->addMenu(tr("&Help"));
}

void MainWindow::CreateActions() {
  this->CreateFileMenuActions();
  this->CreateEditMenuActions();
  this->CreateViewMenuActions();
  this->CreateProcessMenuActions();
  this->CreateAnalysisMenuActions();
  this->CreateToolsMenuActions();
  this->CreateHelpMenuActions();
}

void MainWindow::CreateFileMenuActions() {
  open_image_file_ = file_->addAction(
      QIcon(":/icon/Open.png"), tr("Open File"),
      this, SLOT(OpenImageFile()), QKeySequence::Open);
  addAction(open_image_file_);

  open_image_dir_ = file_->addAction(
      tr("Open Folder"), this, SLOT(OpenImageDir()),
      QKeySequence(Qt::CTRL + Qt::SHIFT + Qt::Key_O));
  addAction(open_image_dir_);

  save_isotropic_image_ = file_->addAction(
      tr("Save Isotropic Image"), this, SLOT(SaveIsotropicImage()));
  addAction(save_isotropic_image_);

  load_parameters_ = file_->addAction(
      QIcon(":/icon/Properties.png"), tr("Load Parameters"),
      this, SLOT(LoadParameters()), QKeySequence(Qt::CTRL + Qt::Key_R));
  addAction(load_parameters_);

  save_parameters_ = file_->addAction(
      tr("Save Parameters"), this, SLOT(SaveParameters()));
  addAction(save_parameters_);

  load_snakes_ = file_->addAction(
      QIcon(":/icon/Upload.png"), tr("Load Snakes"),
      this, SLOT(LoadSnakes()), QKeySequence(Qt::CTRL + Qt::Key_L));
  addAction(load_snakes_);

  save_snakes_ = file_->addAction(
      QIcon(":/icon/Save.png"), tr("Save Snakes"),
      this, SLOT(SaveSnakes()), QKeySequence::Save);
  addAction(save_snakes_);

  compare_snakes_ = file_->addAction(
      QIcon(":/icon/Copy.png"), tr("Compare Snakes"),
      this, SLOT(CompareSnakes()), QKeySequence(Qt::CTRL + Qt::Key_C));
  addAction(compare_snakes_);

  close_session_ = file_->addAction(
      QIcon(":/icon/Logout.png"), tr("Close Session"),
      this, SLOT(CloseSession()), QKeySequence::Close);
  addAction(close_session_);

  exit_ = file_->addAction(
      tr("Exit"), this, SLOT(close()), QKeySequence::Quit);
  addAction(exit_);
}

void MainWindow::CreateEditMenuActions() {
  toggle_normal_ = edit_->addAction(QIcon(":/icon/Cancel.png"),
                                    tr("Normal Mode"));
  toggle_normal_->setCheckable(true);
  connect(toggle_normal_, SIGNAL(toggled(bool)),
          viewer_, SLOT(ToggleNormal(bool)));
  addAction(toggle_normal_);

  snake_edit_group_ = new QActionGroup(this);
  snake_edit_group_->addAction(toggle_normal_);
  snake_edit_group_->setExclusive(true);
  toggle_normal_->setChecked(true);
}

void MainWindow::CreateViewMenuActions() {
  toggle_slice_planes_ = view_->addAction(
      QIcon(":/icon/Picture.png"), tr("Slice Planes"));
  toggle_slice_planes_->setCheckable(true);
  connect(toggle_slice_planes_, SIGNAL(toggled(bool)),
          this, SLOT(TogglePlanes(bool)));
  addAction(toggle_slice_planes_);

  toggle_volume_rendering_ = view_->addAction(
      QIcon(":/icon/Globe.png"), tr("Volume Rendering"));
  toggle_volume_rendering_->setCheckable(true);
  connect(toggle_volume_rendering_, SIGNAL(toggled(bool)),
          this, SLOT(ToggleVolumeRendering(bool)));
  addAction(toggle_volume_rendering_);

  toggle_orientation_marker_ = view_->addAction(tr("Orientation Marker"));
  toggle_orientation_marker_->setCheckable(true);
  connect(toggle_orientation_marker_, SIGNAL(toggled(bool)),
          viewer_, SLOT(ToggleOrientationMarker(bool)));
  addAction(toggle_orientation_marker_);

  toggle_corner_text_ = view_->addAction(tr("Corner Texts"));
  toggle_corner_text_->setCheckable(true);
  connect(toggle_corner_text_, SIGNAL(toggled(bool)),
          this, SLOT(ToggleCornerText(bool)));
  addAction(toggle_corner_text_);

  toggle_bounding_box_ = view_->addAction(tr("Bounding Box"));
  toggle_bounding_box_->setCheckable(true);
  connect(toggle_bounding_box_, SIGNAL(toggled(bool)),
          this, SLOT(ToggleBoundingBox(bool)));
  addAction(toggle_bounding_box_);

  toggle_snakes_ = view_->addAction(
      QIcon(":/icon/Synchronize.png"), tr("Snakes"));
  toggle_snakes_->setCheckable(true);
  connect(toggle_snakes_, SIGNAL(toggled(bool)),
          viewer_, SLOT(ToggleSnakes(bool)));
  addAction(toggle_snakes_);

  show_tracks_ = view_->addAction(tr("Show Snake Tracks"), this,
                                  SLOT(ShowTracks()),
                                  QKeySequence((Qt::CTRL + Qt::Key_T)));
  addAction(show_tracks_);

  toggle_comparing_snakes_ = view_->addAction(tr("Comparing Snakes"));
  toggle_comparing_snakes_->setCheckable(true);
  connect(toggle_comparing_snakes_, SIGNAL(toggled(bool)),
          viewer_, SLOT(ToggleComparingSnakes(bool)));
  addAction(toggle_comparing_snakes_);

  toggle_junctions_ = view_->addAction(
      QIcon(":/icon/Positive.png"), tr("Junctions"));
  toggle_junctions_->setCheckable(true);
  connect(toggle_junctions_, SIGNAL(toggled(bool)),
          viewer_, SLOT(ToggleJunctions(bool)));
  addAction(toggle_junctions_);

  toggle_comparing_junctions_ = view_->addAction(tr("Comparing Junctions"));
  toggle_comparing_junctions_->setCheckable(true);
  connect(toggle_comparing_junctions_, SIGNAL(toggled(bool)),
          viewer_, SLOT(ToggleComparingJunctions(bool)));
  addAction(toggle_comparing_junctions_);

  toggle_clip_ = view_->addAction(
      QIcon(":/icon/Search.png"), tr("Show Snakes Locally"));
  toggle_clip_->setCheckable(true);
  connect(toggle_clip_, SIGNAL(toggled(bool)),
          viewer_, SLOT(ToggleClipSnakes(bool)));
  addAction(toggle_clip_);

  toggle_azimuthal_ = view_->addAction(tr("Color by Azimuthal Angle"));
  toggle_azimuthal_->setCheckable(true);
  connect(toggle_azimuthal_, SIGNAL(toggled(bool)),
          viewer_, SLOT(ColorByAzimuthalAngle(bool)));
  addAction(toggle_azimuthal_);

  toggle_polar_ = view_->addAction(tr("Color by Polar Angle"));
  toggle_polar_->setCheckable(true);
  connect(toggle_polar_, SIGNAL(toggled(bool)),
          viewer_, SLOT(ColorByPolarAngle(bool)));
  addAction(toggle_polar_);

  reset_viewpoint_ = view_->addAction(
      tr("Reset Viewpoint"), viewer_, SLOT(ResetViewpoint()));
  addAction(reset_viewpoint_);

  show_view_options_ = view_->addAction(
      tr("Options"), this, SLOT(ShowViewOptions()));
  addAction(show_view_options_);
}

void MainWindow::CreateProcessMenuActions() {
  initialize_snakes_ = process_->addAction(
      QIcon(":/icon/Add.png"), tr("Initialize Snakes"), this,
      SLOT(InitializeSnakes()), QKeySequence((Qt::CTRL + Qt::Key_I)));
  addAction(initialize_snakes_);

  deform_snakes_ = process_->addAction(
      QIcon(":/icon/Play.png"), tr("Deform Snakes"), this,
      SLOT(DeformSnakes()), QKeySequence((Qt::CTRL + Qt::Key_D)));
  addAction(deform_snakes_);

  solve_correspondence_ = process_->addAction(tr("Track Snakes"), this,
                                              SLOT(SolveCorrespondence()));
  addAction(solve_correspondence_);
}

void MainWindow::CreateAnalysisMenuActions() {
  compute_spherical_orientation_ = analysis_->addAction(
      tr("Compute Spherical Orientation"), this,
      SLOT(ComputeSphericalOrientation()));
  addAction(compute_spherical_orientation_);

  show_analysis_options_ = analysis_->addAction(tr("Analysis Options"),
                                                this, SLOT(ShowAnalysisOptions()));
  addAction(show_analysis_options_);
}

void MainWindow::CreateToolsMenuActions() {
  show_parameters_ = tools_->addAction(
      QIcon(":/icon/Settings.png"), tr("Parameters"), this,
      SLOT(ShowParametersDialog()), QKeySequence(Qt::CTRL + Qt::Key_P));
  addAction(show_parameters_);

  load_viewpoint_ = tools_->addAction(
      tr("Load Viewpoint"), this, SLOT(LoadViewpoint()));
  addAction(load_viewpoint_);

  save_viewpoint_ = tools_->addAction(
      tr("Save Viewpoint"), this, SLOT(SaveViewpoint()));
  addAction(save_viewpoint_);

  save_snapshot_ = tools_->addAction(
      tr("Save Snapshot"), this, SLOT(SaveSnapshot()));
  addAction(save_snapshot_);
}

void MainWindow::CreateHelpMenuActions() {
  about_tsoax_ = help_->addAction(
      tr("About TSOAX"), this, SLOT(AboutTSOAX()));
  addAction(about_tsoax_);

  about_qt_ = help_->addAction(tr("About Qt"), qApp, SLOT(aboutQt()));
  addAction(about_qt_);
}

void MainWindow::CreateToolBar() {
  toolbar_ = addToolBar(tr("TSOAX Toolbar"));
  toolbar_->addAction(open_image_file_);
  toolbar_->addAction(load_parameters_);
  toolbar_->addAction(save_snakes_);
  toolbar_->addAction(load_snakes_);

  toolbar_->addSeparator();
  toolbar_->addAction(toggle_slice_planes_);
  toolbar_->addAction(toggle_volume_rendering_);
  toolbar_->addAction(toggle_snakes_);
  toolbar_->addAction(toggle_junctions_);

  toolbar_->addSeparator();
  toolbar_->addAction(initialize_snakes_);
  toolbar_->addAction(deform_snakes_);
  toolbar_->addAction(show_parameters_);

  toolbar_->addSeparator();
  toolbar_->addAction(toggle_normal_);

  toolbar_->addSeparator();
  toolbar_->addAction(close_session_);
}

void MainWindow::CreateScrollBar() {
  scroll_bar_ = new QScrollBar(Qt::Horizontal, this);
  scroll_bar_->setFocusPolicy(Qt::StrongFocus);
  scroll_bar_->setMinimum(0);
  scroll_bar_->setMaximum(0);
}

void MainWindow::CreateProgressBar() {
  progress_bar_ = new QProgressBar(this);
  statusBar()->addPermanentWidget(progress_bar_);
}

void MainWindow::ResetActions() {
  save_snakes_->setEnabled(false);
  load_snakes_->setEnabled(false);
  save_isotropic_image_->setEnabled(false);
  compare_snakes_->setEnabled(false);
  close_session_->setEnabled(false);

  toggle_normal_->setEnabled(false);

  toggle_slice_planes_->setEnabled(false);
  toggle_volume_rendering_->setEnabled(false);
  toggle_orientation_marker_->setEnabled(false);
  toggle_corner_text_->setEnabled(false);
  toggle_bounding_box_->setEnabled(false);
  toggle_snakes_->setEnabled(false);
  show_tracks_->setEnabled(false);
  toggle_comparing_snakes_->setEnabled(false);
  toggle_junctions_->setEnabled(false);
  toggle_comparing_junctions_->setEnabled(false);
  toggle_clip_->setEnabled(false);
  toggle_azimuthal_->setEnabled(false);
  toggle_polar_->setEnabled(false);
  load_viewpoint_->setEnabled(false);
  save_viewpoint_->setEnabled(false);
  reset_viewpoint_->setEnabled(false);
  save_snapshot_->setEnabled(false);
  show_view_options_->setEnabled(false);

  initialize_snakes_->setEnabled(false);
  deform_snakes_->setEnabled(false);
  solve_correspondence_->setEnabled(false);
  compute_spherical_orientation_->setEnabled(false);
  show_analysis_options_->setEnabled(false);
  show_parameters_->setEnabled(false);
}

void MainWindow::ShowImage() {
  analysis_options_dialog_->SetImageCenter(GetImageCenter(reader_->GetImage()));
  double radius = GetImageDiagonal(reader_->GetImage()) / 2.0;
  analysis_options_dialog_->SetRadius(radius);

  scroll_bar_->setMaximum(static_cast<int>(reader_->GetNumberOfImages() - 1));

  if (GetImageDimension(reader_->GetImage(scroll_bar_->value())) == 3) {
    toggle_volume_rendering_->setEnabled(true);
    toggle_volume_rendering_->setChecked(true);
    toggle_orientation_marker_->setEnabled(true);
    toggle_orientation_marker_->setChecked(true);
    toggle_bounding_box_->setEnabled(true);
  }
  toggle_slice_planes_->setEnabled(true);
  toggle_slice_planes_->setChecked(true);
  toggle_corner_text_->setEnabled(true);
  toggle_corner_text_->setChecked(true);

  viewer_->SaveViewpoint();

  if (scroll_bar_->maximum() > 0) {
    connect(scroll_bar_, SIGNAL(valueChanged(int)),
            this, SLOT(UpdateImageView(int)));
  }

  // Update window/level and mip viewing
  if (GetImageDimension(reader_->GetImage(scroll_bar_->value())) == 3) {
    view_options_dialog_->SetWindow(viewer_->slice_planes()->GetWindow());
    view_options_dialog_->SetLevel(viewer_->slice_planes()->GetLevel());
    view_options_dialog_->SetMinIntensity(viewer_->volume()->min_intensity());
    view_options_dialog_->SetMaxIntensity(viewer_->volume()->max_intensity());
  } else {
    view_options_dialog_->SetWindow(viewer_->image_plane()->GetWindow());
    view_options_dialog_->SetLevel(viewer_->image_plane()->GetLevel());
  }

  UpdateWindowTitle();
  UpdateFrameNumber();

  open_image_file_->setEnabled(false);
  open_image_dir_->setEnabled(false);
  load_snakes_->setEnabled(true);
  save_isotropic_image_->setEnabled(true);
  compare_snakes_->setEnabled(true);
  close_session_->setEnabled(true);

  reset_viewpoint_->setEnabled(true);
  show_view_options_->setEnabled(true);

  initialize_snakes_->setEnabled(true);
  deform_snakes_->setEnabled(true);

  load_viewpoint_->setEnabled(true);
  save_viewpoint_->setEnabled(true);
  save_snapshot_->setEnabled(true);
  show_parameters_->setEnabled(true);
}



}  // namespace soax
