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
 * ImagePlane shows a 2D image in a 3D scene.
 */


#ifndef IMAGE_PLANE_H_
#define IMAGE_PLANE_H_

class vtkImageActor;
class vtkImageData;
class vtkRenderer;

namespace soax {

class ImagePlane {
 public:
  ImagePlane();
  explicit ImagePlane(vtkImageData *image);

  ~ImagePlane();

  /**
   * Update actor_ for a new IMAGE.
   */
  void Update(vtkImageData *image);

  /**
   * Show/hide the plane for RENDERER.
   */
  void Show(vtkRenderer *renderer);
  void Hide(vtkRenderer *renderer);

  /**
   * Get/Set window/level.
   */
  double GetWindow() const;
  double GetLevel() const;
  void SetWindowLevel(double window, double level);

 private:
  /**
   * Update the window/level for IMAGE.
   */
  void UpdateWindowLevel(vtkImageData *image);


  vtkImageActor *actor_ = nullptr;
  bool window_level_manually_set_ = false;

  ImagePlane(const ImagePlane &);
  void operator=(const ImagePlane &);
};

}  // namespace soax

#endif  // IMAGE_PLANE_H_
