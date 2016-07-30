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
 * SlicePlanes visualize volumetric image data using three orthogonal
 * planes that moving along the image axes.
 */


#ifndef SLICE_PLANES_H_
#define SLICE_PLANES_H_

class QVTKInteractor;
class vtkImageData;
class vtkImagePlaneWidget;

namespace soax {

class SlicePlanes {
 public:
  SlicePlanes();
  SlicePlanes(vtkImageData *image, QVTKInteractor *interactor);

  ~SlicePlanes();

  /**
   * Update planes_ for a new IMAGE.
   */
  void Update(vtkImageData *image);

  /**
   * Show/hide the planes.
   */
  void Show() const;
  void Hide() const;

  double GetWindow() const;
  double GetLevel() const;
  void SetWindowLevel(double window, double level);

 private:
  static const int kNPlanes = 3;
  vtkImagePlaneWidget *planes_[kNPlanes];

  SlicePlanes(const SlicePlanes &);
  void operator=(const SlicePlanes &);
};

}  // namespace soax

#endif  // SLICE_PLANES_H_
