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

#include "../junctions.h"
#include <vector>
#include <iostream>
#include "gtest/gtest.h"
#include "../snake_tip_set.h"
#include "../snake_tip.h"
#include "../snake.h"
#include "../snake_parameters.h"
#include "../util.h"

namespace soax {

class JunctionsTest : public ::testing::Test {
 protected:
  typedef std::vector<double> DataContainer;
  typedef SnakeTipSet::TipContainer TipContainer;

  ~JunctionsTest() {
    for (auto it = snakes.begin(); it != snakes.end(); it++) {
      delete *it;
    }
    snakes.clear();
  }

  Snake * ConstructSnake(const DataContainer &pts, int dim = 3) {
    Snake *s = new Snake(pts, dim, nullptr, nullptr, &sp, true);
    snakes.push_back(s);
    return s;
  }

  SnakeContainer snakes;
  Junctions junc;
  SnakeParameters sp;
};

TEST_F(JunctionsTest, Union1) {
  DataContainer pts = {0, 1,
                       0, 2};
  Snake *s0 = ConstructSnake(pts, 2);

  pts = {-1, 0,
         -3, 0};
  Snake *s1 = ConstructSnake(pts, 2);

  pts = {0, -1,
         0, -3};
  Snake *s2 = ConstructSnake(pts, 2);

  pts = {1, 1,
         3, 3};
  Snake *s3 = ConstructSnake(pts, 2);

  junc.set_distance_threshold(2);
  junc.set_direction_threshold(3.14159 * 2 / 3);
  junc.set_delta(1);
  junc.Union(&snakes);

  SnakeTipSet *ts = junc.GetTipSet(s0, true);
  EXPECT_TRUE(ts->Has(junc.GetTip(s0, false)));
  EXPECT_TRUE(ts->Has(junc.GetTip(s1, true)));
  EXPECT_FALSE(ts->Has(junc.GetTip(s1, false)));
  EXPECT_TRUE(ts->Has(junc.GetTip(s2, true)));
  EXPECT_FALSE(ts->Has(junc.GetTip(s2, false)));
  EXPECT_TRUE(ts->Has(junc.GetTip(s3, true)));
  EXPECT_FALSE(ts->Has(junc.GetTip(s3, false)));
}

/**
 * Reverse one snake.
 */
TEST_F(JunctionsTest, Union2) {
  DataContainer pts = {0, 1,
                       0, 3};
  Snake *s0 = ConstructSnake(pts, 2);

  pts = {0, -1,
         0, -3};
  Snake *s1 = ConstructSnake(pts, 2);

  pts = {1, 0,
         0, 0};
  Snake *s2 = ConstructSnake(pts, 2);

  junc.set_distance_threshold(1.5);
  junc.set_direction_threshold(3.14159 * 2 / 3);
  junc.set_delta(1);
  junc.Union(&snakes);

  SnakeTipSet *ts = junc.GetTipSet(s0, true);
  EXPECT_FALSE(ts->Has(junc.GetTip(s0, false)));
  EXPECT_TRUE(ts->Has(junc.GetTip(s1, true)));
  EXPECT_FALSE(ts->Has(junc.GetTip(s1, false)));
  EXPECT_TRUE(ts->Has(junc.GetTip(s2, true)));
  EXPECT_TRUE(ts->Has(junc.GetTip(s2, false)));
}

}  // namespace soax
