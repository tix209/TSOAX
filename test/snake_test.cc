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

#include "../snake.h"
#include <vector>
#include "gtest/gtest.h"
#include <eigen3/Eigen/Dense>
#include "../image_reader.h"
#include "../gradient_calculator.h"
#include "../interpolator.h"
#include "../snake_parameters.h"

namespace soax {

class SnakeTest : public ::testing::Test {
 protected:
  // typedef Snake::PairContainer PairContainer;
  // SnakeTest() {
  //   s1.set_parameters(&sp);
  //   s2.set_parameters(&sp);
  // }

  // bool Resample(Snake *s) {return s->Resample();}

  // double ComputeLengthPartialSum(Snake *s, PairContainer *sums) {
  //   return s->ComputeLengthPartialSum(sums);
  // }

  // int ComputeSize(Snake *s, double length, double spacing) {
  //   return s->ComputeSize(length, spacing);
  // }

  // void InterpolateVertices(Snake *s, const PairContainer *sums,
  //                          double length, int size) {
  //   s->InterpolateVertices(sums, length, size);
  // }

  ImageReader reader;
  GradientCalculator gc;
  Interpolator ii;
  Interpolator gi;
  // Snake s1;
  // Snake s2;
  SnakeParameters sp;
};

TEST_F(SnakeTest, Constructor) {
  Snake s1;
  EXPECT_EQ(3, s1.dimension());
  EXPECT_TRUE(s1.open());
  EXPECT_EQ(0, s1.GetSize());
  EXPECT_FALSE(s1.Resample());
}

// TEST_F(SnakeTest, ComputeLengthPartialSum) {
//   std::vector<double> pts1 = {10, 4, 4,
//                               12, 4, 4,
//                               14, 4, 4,
//                               16, 4, 4,
//                               18, 4, 4};
//   Snake s1(pts1, 3, &ii, &gi, &sp);
//   double spacing = 2.0;
//   sp.set_spacing(spacing);

//   EXPECT_EQ(3, s1.dimension());
//   EXPECT_EQ(5, s1.GetSize());
//   PairContainer sums1[3];
//   EXPECT_EQ(8, ComputeLengthPartialSum(&s1, sums1));

//   PairContainer expected_sums1[3];
//   expected_sums1[0].push_back(std::make_pair(0, 10));
//   expected_sums1[0].push_back(std::make_pair(2, 12));
//   expected_sums1[0].push_back(std::make_pair(4, 14));
//   expected_sums1[0].push_back(std::make_pair(6, 16));
//   expected_sums1[0].push_back(std::make_pair(8, 18));
//   expected_sums1[1].push_back(std::make_pair(0, 4));
//   expected_sums1[1].push_back(std::make_pair(2, 4));
//   expected_sums1[1].push_back(std::make_pair(4, 4));
//   expected_sums1[1].push_back(std::make_pair(6, 4));
//   expected_sums1[1].push_back(std::make_pair(8, 4));
//   expected_sums1[2].push_back(std::make_pair(0, 4));
//   expected_sums1[2].push_back(std::make_pair(2, 4));
//   expected_sums1[2].push_back(std::make_pair(4, 4));
//   expected_sums1[2].push_back(std::make_pair(6, 4));
//   expected_sums1[2].push_back(std::make_pair(8, 4));
//   for (int j = 0; j < 3; j++) {
//     EXPECT_EQ(expected_sums1[j], sums1[j]);
//   }

//   std::vector<double> pts2 = {10, 4, 4,
//                               12, 4, 4,
//                               15, 4, 4,
//                               18, 4, 4};
//   s2.set_vertices(pts2);
//   sp.set_spacing(spacing);
//   EXPECT_EQ(3, s2.dimension());
//   EXPECT_EQ(4, s2.GetSize());
//   PairContainer sums2[3];
//   EXPECT_EQ(8, ComputeLengthPartialSum(s2, sums2));

//   PairContainer expected_sums2[3];
//   expected_sums2[0].push_back(std::make_pair(0, 10));
//   expected_sums2[0].push_back(std::make_pair(2, 12));
//   expected_sums2[0].push_back(std::make_pair(5, 15));
//   expected_sums2[0].push_back(std::make_pair(8, 18));
//   expected_sums2[1].push_back(std::make_pair(0, 4));
//   expected_sums2[1].push_back(std::make_pair(2, 4));
//   expected_sums2[1].push_back(std::make_pair(5, 4));
//   expected_sums2[1].push_back(std::make_pair(8, 4));
//   expected_sums2[2].push_back(std::make_pair(0, 4));
//   expected_sums2[2].push_back(std::make_pair(2, 4));
//   expected_sums2[2].push_back(std::make_pair(5, 4));
//   expected_sums2[2].push_back(std::make_pair(8, 4));
//   for (int j = 0; j < 3; j++) {
//     EXPECT_EQ(expected_sums2[j], sums2[j]);
//   }
// }

// TEST_F(SnakeTest, ComputeSize) {
//   Snake s1;
//   s1.set_parameters(&sp);
//   sp.set_spacing(2.0);
//   EXPECT_EQ(5, ComputeSize(&s1, 8, sp.spacing()));
//   sp.set_spacing(1.0);
//   EXPECT_EQ(9, ComputeSize(&s1, 8, sp.spacing()));
// }

// TEST_F(SnakeTest, InterpolateVertices) {
//   std::vector<double> pts1 = {4, 10, 4,
//                               4, 12, 4,
//                               4, 15, 4,
//                               4, 18, 4};
//   Snake s1(pts1, 3, &ii, &gi, &sp);
//   // s1.set_vertices(pts1);
//   // sp.set_spacing(2.0);
//   EXPECT_EQ(3, s1.dimension());
//   EXPECT_EQ(4, s1.GetSize());

//   // PairContainer sums1[3];
//   // EXPECT_EQ(8, ComputeLengthPartialSum(s1, sums1));

//   InterpolateVertices(s1, sums1, 8, 5);
//   MatrixXd expected_vertices1(5, 3);
//   expected_vertices1 <<
//       4, 10, 4,
//       4, 12, 4,
//       4, 14, 4,
//       4, 16, 4,
//       4, 18, 4;
//   EXPECT_EQ(expected_vertices1, s1.vertices());

//   std::vector<double> pts2 = {10, 4, 4,
//                               12, 4, 4,
//                               14, 4, 4,
//                               16, 4, 4,
//                               18, 4, 4};
//   s2.set_vertices(pts2);
//   sp.set_spacing(2.0);
//   PairContainer sums2[3];
//   EXPECT_EQ(8, ComputeLengthPartialSum(s2, sums2));
//   InterpolateVertices(s2, sums2, 8, 5);
//   MatrixXd expected_vertices2(5, 3);
//   expected_vertices2 <<
//       10, 4, 4,
//       12, 4, 4,
//       14, 4, 4,
//       16, 4, 4,
//       18, 4, 4;
//   EXPECT_EQ(expected_vertices2, s2.vertices());
// }

TEST_F(SnakeTest, Resample) {
  std::vector<double> pts1 = {4, 10, 4,
                              4, 12, 4,
                              4, 15, 4,
                              4, 18, 4};
  // s1.set_vertices(pts1);
  Snake s1(pts1, 3, &ii, &gi, &sp, true);
  sp.set_spacing(2.0);
  EXPECT_TRUE(s1.Resample());
  MatrixXd expected_vertices1(5, 3);
  expected_vertices1 <<
      4, 10, 4,
      4, 12, 4,
      4, 14, 4,
      4, 16, 4,
      4, 18, 4;
  EXPECT_EQ(expected_vertices1, s1.vertices());

  std::vector<double> pts2 = {10, 4, 4,
                              12, 4, 4,
                              14, 4, 4,
                              16, 4, 4,
                              18, 4, 4};
  // s2.set_vertices(pts2);
  Snake s2(pts2, 3, &ii, &gi, &sp, true);
  sp.set_spacing(1.0);
  EXPECT_TRUE(s2.Resample());
  MatrixXd expected_vertices2(9, 3);
  expected_vertices2 <<
      10, 4, 4,
      11, 4, 4,
      12, 4, 4,
      13, 4, 4,
      14, 4, 4,
      15, 4, 4,
      16, 4, 4,
      17, 4, 4,
      18, 4, 4;
  EXPECT_EQ(expected_vertices2, s2.vertices());
}


TEST_F(SnakeTest, GetLength) {
  std::vector<double> pts1 = {4, 10, 4,
                              4, 12, 4,
                              4, 15, 4,
                              4, 18, 4};
  // s1.set_vertices(pts1);
  Snake s1(pts1, 3, &ii, &gi, &sp, true);
  EXPECT_EQ(8, s1.GetLength());
}

TEST_F(SnakeTest, Evolve) {
  reader.ReadFile("../../rel4.0/test/data/simple.mha");
  gc.set_image(reader.GetImage());
  // gc.set_intensity_scaling();
  gc.Compute();
  ii.Initialize(reader.GetImage());
  gi.Initialize(gc.gradient());

  // s1.set_image_interp(&ii);
  // s1.set_gradient_interp(&gi);
  std::vector<double> pts1 = {0, 4, 4, 12, 4, 4, 15, 5, 5, 31, 5, 5};
  Snake s1(pts1, 3, &ii, &gi, &sp, true);
  // s1.set_vertices(pts1);
  sp.set_stretch_factor(0.1);
  // s1.Evolve();
  // std::cout << s1.vertices() << std::endl;
  // std::cout << s1.iterations() << std::endl;
}

TEST_F(SnakeTest, ComputeDistance) {
  std::vector<double> pts1 = {4, 10, 4,
                              4, 12, 4,
                              4, 15, 4,
                              4, 18, 4};
  Snake s1(pts1, 3, &ii, &gi, &sp, true);
  std::vector<double> pts2 = {4, 10, 4,
                              4, 12, 4,
                              4, 15, 4};
  Snake s2(pts2, 3, &ii, &gi, &sp, true);

  EXPECT_EQ(3.0/7.0, ComputeCurveDistance(&s1, &s2));
}



}  // namespace soax
