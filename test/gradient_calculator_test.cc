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
 */


#include "../gradient_calculator.h"
#include "vtkImageData.h"
#include "vtkImageExtractComponents.h"
#include "gtest/gtest.h"
#include "../image_reader.h"
#include "../image.h"


namespace soax {

class GradientCalculatorTest : public ::testing::Test {
 protected:
  GradientCalculatorTest() {}

  void WriteComponentImage(vtkImageData *image, int dim,
                             const std::string &filename) {
    vtkImageExtractComponents *extractor =
        vtkImageExtractComponents::New();
    extractor->SetInputData(image);
    extractor->SetComponents(dim);
    extractor->Update();
    std::string final_name = filename;
    final_name += std::to_string(dim) + ".mha";
    std::cout << "Writing " << final_name << std::endl;
    WriteMetaImage(extractor->GetOutput(), final_name);
    extractor->Delete();
  }

  ImageReader reader;
  GradientCalculator gc;
};

TEST_F(GradientCalculatorTest, Constructor) {
  EXPECT_EQ(nullptr, gc.gradient());
}

TEST_F(GradientCalculatorTest, ComputeGradientFor2D) {
  reader.ReadFile("../../rel4.0/test/data/2d_set/asian-water1.jpg");
  GradientCalculator gc1(reader.GetImage());

  gc1.Compute(1.0, 0.0);
  vtkImageData *grad1 = gc1.gradient();

  EXPECT_EQ(2, GetImageDimension(grad1));
  EXPECT_EQ(2, grad1->GetNumberOfScalarComponents());
  double *g0 = static_cast<double*>(grad1->GetScalarPointer(20, 20, 0));
  EXPECT_EQ(-1.5, g0[0]);
  EXPECT_EQ(3.5, g0[1]);
  // WriteComponentImage(grad1, 0, "asian-water1-grad");
  // WriteComponentImage(grad1, 1, "asian-water1-grad");

  gc.set_image(reader.GetImage());
  gc.Compute(0.0, 1.0);
  vtkImageData *grad = gc.gradient();

  EXPECT_EQ(2, GetImageDimension(grad));
  EXPECT_EQ(2, grad->GetNumberOfScalarComponents());
  // WriteComponentImage(grad, 0, "asian-water1-grad-smoothed");
  // WriteComponentImage(grad, 1, "asian-water1-grad-smoothed");
}

TEST_F(GradientCalculatorTest, ComputeGradientFor3D) {
  reader.ReadFile("../../rel4.0/test/data/simple.mha");
  gc.set_image(reader.GetImage());

  gc.Compute();
  vtkImageData *grad = gc.gradient();

  double *g0 = static_cast<double*>(grad->GetScalarPointer(15, 5, 5));
  EXPECT_DOUBLE_EQ(0, g0[0]);
  EXPECT_DOUBLE_EQ(0, g0[1]);
  EXPECT_DOUBLE_EQ(0, g0[2]);

  double *g1 = static_cast<double*>(grad->GetScalarPointer(15, 5, 4));
  EXPECT_DOUBLE_EQ(0, g1[0]);
  EXPECT_DOUBLE_EQ(0, g1[1]);
  EXPECT_DOUBLE_EQ(1 / 3.0, g1[2]);

  double *g2 = static_cast<double*>(grad->GetScalarPointer(10, 5, 5));
  EXPECT_DOUBLE_EQ(0.5, g2[0]);
  EXPECT_DOUBLE_EQ(0, g2[1]);
  EXPECT_DOUBLE_EQ(0, g2[2]);

  double *g3 = static_cast<double*>(grad->GetScalarPointer(15, 6, 4));
  EXPECT_DOUBLE_EQ(0, g3[0]);
  EXPECT_DOUBLE_EQ(-0.5 / 3.0, g3[1]);
  EXPECT_DOUBLE_EQ(0.5 / 3.0, g3[2]);

  double *g4 = GetImageGradient(grad, 8, 3, 3);
  EXPECT_DOUBLE_EQ(0, g4[0]);
  EXPECT_DOUBLE_EQ(0, g4[1]);
  EXPECT_DOUBLE_EQ(0, g4[2]);

  double *g5 = GetImageGradient(grad, 9, 3, 5);
  EXPECT_DOUBLE_EQ(0.5 / 3.0, g5[0]);
  EXPECT_DOUBLE_EQ(0, g5[1]);
  EXPECT_DOUBLE_EQ(0, g5[2]);

  int *image_dim = reader.GetImage()->GetDimensions();
  int *grad_dim = grad->GetDimensions();
  EXPECT_EQ(std::vector<int>(image_dim, image_dim + 3),
            std::vector<int>(grad_dim, grad_dim + 3));
}

TEST_F(GradientCalculatorTest, ComputeGradientFor3DSmoothed) {
  reader.ReadFile("../../rel4.0/test/data/simple.mha");
  gc.set_image(reader.GetImage());
  gc.Compute(0.0, 1.0);
  vtkImageData *grad = gc.gradient();

  EXPECT_EQ(3, GetImageDimension(grad));
  EXPECT_EQ(3, grad->GetNumberOfScalarComponents());
  // WriteComponentImage(grad, 0, "simple-grad-smoothed");
  // WriteComponentImage(grad, 1, "simple-grad-smoothed");
  // WriteComponentImage(grad, 2, "simple-grad-smoothed");
}

}  // namespace soax
