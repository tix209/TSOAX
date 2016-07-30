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
 * Viewer is the render window of SOAX program. It performs all the 2D/3D
 * display of images, SOACs, and other information.
 */


#ifndef VIEWER_H_
#define VIEWER_H_

#include <vector>
#include <string>
#include <QObject>  // NOLINT
#include "./viewpoint.h"
#include "./util.h"

class QVTKWidget;
class vtkRenderer;
class vtkOrientationMarkerWidget;
class vtkCornerAnnotation;
class vtkImageData;
class vtkEventQtSlotConnect;
class vtkPointPicker;


namespace soax {
class ImagePlane;
class SlicePlanes;
class VolumeRendering;
class SnakeActor;
class JunctionActor;

class Viewer : public QObject {
  Q_OBJECT

 public:
  explicit Viewer(QVTKWidget *qvtk);

  ~Viewer();

  ImagePlane *image_plane() const {return image_plane_;}

  SlicePlanes *slice_planes() const {return slice_planes_;}

  VolumeRendering *volume() const {return volume_;}

  /**
   * Show/hide the image plane on a 2D IMAGE.
   */
  void ToggleImagePlane(bool state, vtkImageData *image);

  /**
   * Show/hide the slice planes on a 3D IMAGE.
   */
  void ToggleSlicePlanes(bool state, vtkImageData *image);

  /**
   * Show/hide the volume rendering on a 3D IMAGE.
   */
  void ToggleVolumeRendering(bool state, vtkImageData *image);

  /**
   * Show/hide the bounding box for a 3D IMAGE .
   */
  void ToggleBoundingBox(bool state, vtkImageData *image);

  /**
   * Show/hide the text at corners of rendering screen for IMAGE.
   */
  void ToggleCornerText(bool state, vtkImageData *image);

  void RemoveSnakes();
  void SetupSnakes(const SnakeContainer &snakes);

  void RemoveComparingSnakes();
  void SetupComparingSnakes(const SnakeContainer &snakes);

  void RemoveJunctions();
  void SetupJunctions(const PointContainer &points);

  void RemoveComparingJunctions();
  void SetupComparingJunctions(const PointContainer &points);

  /**
   * Update the rendering of image plane for a new IMAGE.
   */
  void UpdateImagePlane(vtkImageData *image);

  /**
   * Update the rendering of slice planes for a new IMAGE.
   */
  void UpdateSlicePlanes(vtkImageData *image);

  /**
   * Update the volume rendering for a new IMAGE.
   */
  void UpdateVolumeRendering(vtkImageData *image);

  /**
   * Update the image info text at corners of rendering screen for a new
   * IMAGE.
   */
  void UpdateCornerTextImageInfo(vtkImageData *image);

  /**
   * Update the text of snake file path at corners of rendering screen.
   */
  void UpdateCornerTextSnake(const QString &filename);
  void UpdateCornerTextComparingSnake(const QString &filename);

  /**
   * Save the snapshot of current render window in PNG format.
   */
  void SaveWindowImage(const QString &filename);

  /**
   * Returns the path for snapshot.
   */
  QString snapshot_path() const {return snapshot_path_;}

  /**
   * Returns the path of current viewpoint.
   */
  QString GetViewpointPath() const;

  /**
   * Load viewpoint from file.
   */
  void LoadViewpoint(const QString &filename);

  /**
   * Save viewpoint from file.
   */
  void SaveViewpoint(const QString &filename);

  /**
   * Save viewpoint as default, for future resetting of viewpoint.
   */
  void SaveViewpoint();

  SnakeActor *snake_actor() const {return snake_actor_;}

  /**
   * Reset the state of members.
   */
  void Reset();

  /**
   * Render the scene.
   */
  void Render();


 private slots:  // NOLINT(whitespace/indent)
  /**
   * Toggle the snake edit normal mode.
   */
  void ToggleNormal(bool state);

  /**
   * Select/deselect snake for viewing only.
   */
  void SelectSnakeForView();
  void DeselectSnakeForView();

  /**
   * Show/hide snakes.
   */
  void ToggleSnakes(bool state);
  void ToggleComparingSnakes(bool state);

  /**
   * Set the viewpoint as default.
   */
  void ResetViewpoint();

  /**
   * Show/hide the orientation marker.
   */
  void ToggleOrientationMarker(bool state);

  /**
   * Show/hide network junctions.
   */
  void ToggleJunctions(bool state);
  void ToggleComparingJunctions(bool state);

  void ToggleClipSnakes(bool state);
  void ColorByAzimuthalAngle(bool state);
  void ColorByPolarAngle(bool state);

 private:
  void SetupImagePlane(vtkImageData *image);
  void SetupSlicePlanes(vtkImageData *image);
  void SetupVolumeRendering(vtkImageData *image);
  void SetupOrientationMarker();

  void ToggleSnakeActor(bool state, SnakeActor *actor);
  void RemoveSnakeActor(SnakeActor *actor);
  void SetupSnakeActor(SnakeActor **actor, const SnakeContainer &snakes);

  void TogglePointActor(bool state, JunctionActor *actor);
  void RemovePointActor(JunctionActor *actor);
  void SetupPointActor(JunctionActor **actor, const PointContainer &points);

  QVTKWidget *qvtk_ = nullptr;
  vtkRenderer *renderer_;

  ImagePlane *image_plane_ = nullptr;
  SlicePlanes *slice_planes_ = nullptr;
  VolumeRendering *volume_ = nullptr;
  vtkOrientationMarkerWidget *orientation_marker_ = nullptr;
  vtkCornerAnnotation *corner_text_ = nullptr;
  Viewpoint viewpoint_;

  SnakeActor *snake_actor_ = nullptr;
  SnakeActor *comparing_snake_actor_ = nullptr;
  JunctionActor *junction_actor_ = nullptr;
  JunctionActor *comparing_junction_actor_ = nullptr;

  vtkEventQtSlotConnect* slot_connector_;
  vtkPointPicker* picker_;

  QString snapshot_path_;
};

}  // namespace soax

#endif  // VIEWER_H_
