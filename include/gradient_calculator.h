/**
 * Copyright (C) 2017 Lehigh University.
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
 * GradientCalculator calculates image gradient. The input image can be
 * first smoothed by a Gaussian filter.
 */


#ifndef GRADIENT_CALCULATOR_H_
#define GRADIENT_CALCULATOR_H_


class vtkImageData;

namespace soax {

class GradientCalculator {
 public:
  GradientCalculator();
  explicit GradientCalculator(vtkImageData *image);

  ~GradientCalculator();

  void set_image(vtkImageData *image) {image_ = image;}

  vtkImageData * gradient() const {return gradient_;}

  /*
   * Compute the image gradient.
   */
  void Compute(double scale = 0.0, double sigma = 0.0);

  /**
   * Release the memory of gradient_.
   */
  void DeleteGradient();

  void Reset();

 private:
  /**
   * Returns a scaled image of which intensity is between [0, 1].
   */
  vtkImageData * ScaleImage(vtkImageData *image, double scale) const;

  /**
   * Returns the actual scaling factor used.
   */
  double GetIntensityScaling(double scale) const;

  /*
   * Returns a grayscale image of a color image. User needs to free the
   * memory of returned image.
   */
  vtkImageData * CondenseToSingleChannel() const;

  /**
   * Returns a Gaussian smoothed image.
   */
  vtkImageData * SmoothImage(vtkImageData *image, double sigma) const;

  vtkImageData *image_ = nullptr;
  vtkImageData *gradient_ = nullptr;


  GradientCalculator(const GradientCalculator &);
  void operator=(const GradientCalculator &);
};

}  // namespace soax


#endif  // GRADIENT_CALCULATOR_H_
