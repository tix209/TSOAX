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
 */

#include "../interpolator.h"
#include <eigen3/Eigen/Dense>
#include "gtest/gtest.h"
#include "../image_reader.h"
#include "../gradient_calculator.h"
#include "../image.h"

namespace soax {
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;

class InterpolatorTest : public ::testing::Test {
 protected:
  InterpolatorTest() : kEps(1e-5) {}

  ImageReader reader;
  GradientCalculator gc;
  Interpolator interp;
  const double kEps;
};

TEST_F(InterpolatorTest, InterpolateIntensity2D) {
  reader.ReadFile("../../rel4.0/test/data/2d_set/asian-water1.jpg");
  interp.Initialize(reader.GetImage());
  EXPECT_EQ(3, interp.GetNumberOfComponents());
  int size[3];
  interp.GetImageSize(size);
  EXPECT_EQ(1920, size[0]);
  EXPECT_EQ(1200, size[1]);
  EXPECT_EQ(1, size[2]);

  double intensity1[3];
  Vector2d p1(10, 1190);  // Coordinates of ImageJ: x' = x, y' = y_max - y
  EXPECT_TRUE(interp.Interpolate(p1, intensity1));
  EXPECT_EQ(std::vector<double>({0, 67, 110}),
            std::vector<double>(intensity1, intensity1 + 3));

  VectorXd p2(2);
  p2 << 10, 1190;
  int ncomponents = 3;
  VectorXd voxel2(ncomponents);
  EXPECT_TRUE(interp.Interpolate(p2, &voxel2));
  VectorXd expected_voxel2(ncomponents);
  expected_voxel2 << 0, 67, 110;
  EXPECT_EQ(expected_voxel2, voxel2);
}

TEST_F(InterpolatorTest, InterpolateIntensity3D) {
  reader.ReadFile("../../rel4.0/test/data/simple.mha");
  interp.Initialize(reader.GetImage());
  EXPECT_EQ(1, interp.GetNumberOfComponents());
  int size[3];
  interp.GetImageSize(size);
  EXPECT_EQ(30, size[0]);
  EXPECT_EQ(10, size[1]);
  EXPECT_EQ(10, size[2]);

  double intensity1 = 0.0;
  Vector3d p1(10, 5, 5);
  EXPECT_TRUE(interp.Interpolate(p1, &intensity1));
  EXPECT_EQ(3, intensity1);

  double intensity2;
  Vector3d p2(15, 100, 100);
  EXPECT_FALSE(interp.Interpolate(p2, &intensity2));
  EXPECT_EQ(0, intensity2);

  double intensity3;
  Vector3d p3(9.8, 4.2, 4.6);
  EXPECT_TRUE(interp.Interpolate(p3, &intensity3));
  EXPECT_NEAR(1.696, intensity3, kEps);

  double intensity4;
  Vector3d p4(10, 4.2, 4.6);
  EXPECT_TRUE(interp.Interpolate(p4, &intensity4));
  EXPECT_NEAR(2.12, intensity4, kEps);
}

TEST_F(InterpolatorTest, InterpolateGradient2D) {
  reader.ReadFile("../../rel4.0/test/data/2d_set/asian-water1.jpg");
  gc.set_image(reader.GetImage());
  gc.Compute(1.0, 0.0);
  EXPECT_EQ(2, GetImageDimension(gc.gradient()));
  interp.Initialize(gc.gradient());
  EXPECT_EQ(2, interp.GetNumberOfComponents());

  const double p1[3] = {20, 20, 0};
  double g1[2];
  EXPECT_TRUE(interp.Interpolate(p1, g1));
  EXPECT_EQ(-1.5, g1[0]);
  EXPECT_EQ(3.5, g1[1]);

  const double p2[3] = {20.5, 20.5, 0.0};
  double g2[3] = {0.0, 0.0, 0.0};  // intentionally allocate one more spot
  EXPECT_TRUE(interp.Interpolate(p2, g2));
  EXPECT_EQ(-2, g2[0]);
  EXPECT_EQ(1.375, g2[1]);
  EXPECT_EQ(0, g2[2]);

  Vector2d p3(20.5, 20.5);
  Vector2d g3;
  EXPECT_TRUE(interp.Interpolate(p3, g3));
  EXPECT_EQ(-2, g3(0));
  EXPECT_EQ(1.375, g3(1));
  Vector2d g3_expect(-2, 1.375);
  EXPECT_EQ(g3_expect, g3);

  Vector3d p4(20, 20, 0);
  Vector3d g4;
  EXPECT_TRUE(interp.Interpolate(p4, g4));
  Vector3d g4_expect(-1.5, 3.5, 0);
  EXPECT_EQ(g4_expect, g4);
}

TEST_F(InterpolatorTest, InterpolateGradient3D) {
  reader.ReadFile("../../rel4.0/test/data/simple.mha");
  gc.set_image(reader.GetImage());
  gc.Compute();
  interp.Initialize(gc.gradient());
  EXPECT_EQ(3, interp.GetNumberOfComponents());

  Vector3d p1(15, 5, 5);
  Vector3d g1;
  interp.Interpolate(p1, g1);
  EXPECT_EQ(Vector3d(0, 0, 0), g1);

  Vector3d p2(15, 6, 4);
  Vector3d g2;
  interp.Interpolate(p2, g2);
  EXPECT_EQ(Vector3d(0, -0.5 / 3, 0.5 / 3), g2);
}

}  // namespace soax
