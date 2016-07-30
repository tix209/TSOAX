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

#include "../snake_tip.h"
#include <vector>
#include <iostream>
#include "gtest/gtest.h"
#include "../snake.h"
#include "../snake_parameters.h"
#include "../util.h"

namespace soax {

class SnakeTipTest : public ::testing::Test {
 protected:
  SnakeTip *t1;
  SnakeTip *t2;
  Snake *s1;
  Snake *s2;
  SnakeParameters sp;
};

TEST_F(SnakeTipTest, Constructor) {
  std::vector<double> pts1 = {10, 4, 4,
                              12, 4, 4,
                              14, 4, 4,
                              16, 4, 4,
                              18, 4, 4};
  s1 = new Snake(pts1, 3, nullptr, nullptr, &sp, true);
  t1 = new SnakeTip(s1, false);
  EXPECT_FALSE(t1->is_head());
  EXPECT_EQ(nullptr, t1->neighbor());
  EXPECT_EQ(Vector3d(10, 4, 4), Vector3d(t1->snake()->GetTip(true)));
  EXPECT_EQ(Vector3d(18, 4, 4), t1->GetLocation());
  EXPECT_EQ(Vector3d(1, 0, 0), t1->GetDirection(4));
  delete s1;
  delete t1;
}

TEST_F(SnakeTipTest, Link) {
  std::vector<double> pts1 = {1, 4, 4,
                              12, 4, 4,
                              14, 4, 4,
                              16, 4, 4,
                              18, 4, 4};
  s1 = new Snake(pts1, 3, nullptr, nullptr, &sp, true);
  t1 = new SnakeTip(s1, true);
  std::vector<double> pts2 = {1, 4, 4,
                              1, 5, 4,
                              1, 6, 4,
                              1, 7, 4,
                              1, 8, 4};
  s2 = new Snake(pts2, 3, nullptr, nullptr, &sp, true);
  t2 = new SnakeTip(s2, false);

  Link(t1, t2);
  EXPECT_EQ(t2, t1->neighbor());
  EXPECT_EQ(t1, t2->neighbor());

  EXPECT_EQ(4, ComputeDistance(t1, t2));
  double half_pi = 3.1415926 / 2.0;
  EXPECT_NEAR(half_pi, ComputeAngle(t1, t2, 2), 1e-6);
  t2->Print(std::cout);
  delete s1;
  delete s2;
  delete t1;
  delete t2;
}

}  // namespace soax

