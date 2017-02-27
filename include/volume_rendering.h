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
 * VolumeRendering (Maximum Intensity Projection) visualizes 3D image
 * data. It also has a bounding box that can show or hide.
 */


#ifndef VOLUME_RENDERING_H_
#define VOLUME_RENDERING_H_

class vtkImageData;
class vtkVolume;
class vtkActor;
class vtkRenderer;

namespace soax {

class VolumeRendering {
 public:
  VolumeRendering();
  explicit VolumeRendering(vtkImageData *image);

  ~VolumeRendering();

  void Update(vtkImageData *image);

  void Show(vtkRenderer *renderer);
  void Hide(vtkRenderer *renderer);

  double min_intensity() const {return min_intensity_;}
  double max_intensity() const {return max_intensity_;}
  void SetDisplayRange(double min, double max);

  void ShowBoundingBox(vtkRenderer *renderer);
  void HideBoundingBox(vtkRenderer *renderer);

 private:
  /**
   * Update the display range based on the image intensity.
   */
  void UpdateDisplayRange(vtkImageData *image);

  void SetupBoundingBox();


  vtkVolume * volume_ = nullptr;
  vtkActor *bounding_box_ = nullptr;

  /**
   * The range of displayed intensity.
   */
  double min_intensity_ = 0.0;
  double max_intensity_ = 0.0;

  /**
   * Transparency ratio.
   */
  double ratio_ = 0.05;

  VolumeRendering(const VolumeRendering &);
  void operator=(const VolumeRendering &);
};

}  // namespace soax

#endif  // VOLUME_RENDERING_H_
