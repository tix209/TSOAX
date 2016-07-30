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

#include "../snake_tip_set.h"
#include <vector>
#include <iostream>
#include "gtest/gtest.h"
#include "../snake_tip.h"
#include "../snake.h"
#include "../snake_parameters.h"
#include "../util.h"

namespace soax {

class SnakeTipSetTest : public ::testing::Test {
 protected:
  typedef std::vector<double> DataContainer;
  typedef SnakeTipSet::TipContainer TipContainer;

  ~SnakeTipSetTest() {
    for (auto it = snakes.begin(); it != snakes.end(); it++) {
      delete *it;
    }
    snakes.clear();

    for (auto it = tips.begin(); it != tips.end(); it++) {
      delete *it;
    }
    tips.clear();
  }

  /**
   * Construct head and tail tips of a snake of which points are specified
   * by PTS.
   */
  void ConstructTips(const DataContainer &pts, SnakeTip **t1,
                    SnakeTip **t2, int dim = 3) {
    Snake *s = new Snake(pts, dim, nullptr, nullptr, &sp, true);
    snakes.push_back(s);
    *t1 = new SnakeTip(s, true);
    *t2 = new SnakeTip(s, false);
    tips.push_back(*t1);
    tips.push_back(*t2);
  }

  /**
   * Return the head or tail tip of a snake of which points are specified by
   * PTS.
   */
  SnakeTip * ConstructTip(const DataContainer &pts, bool is_head,
                          int dim = 3) {
    Snake *s = new Snake(pts, dim, nullptr, nullptr, &sp, true);
    snakes.push_back(s);
    SnakeTip *t = new SnakeTip(s, is_head);
    tips.push_back(t);
    return t;
  }

  TipContainer tips;
  SnakeContainer snakes;
  SnakeParameters sp;
};

TEST_F(SnakeTipSetTest, Constructor) {
  std::vector<double> pts = {1, 0, 0,
                             2, 0, 0};
  Snake *s = new Snake(pts, 3, nullptr, nullptr, &sp, true);
  snakes.push_back(s);
  SnakeTip *t = new SnakeTip(s, true);
  tips.push_back(t);
  SnakeTipSet *ts = new SnakeTipSet(t);
  EXPECT_EQ(0, ts->ComputeDistanceTo(t));
  pts = {0, 0, 0,
         -1, 0, 0};
  ts->Add(ConstructTip(pts, true));

  ts->UpdateCentroid();
  EXPECT_EQ(Vector3d(0.5, 0, 0), ts->centroid());
  // ts->Print(std::cout);
  delete ts;
}

/**
 * Test this case:
 *             __|/
 *               |
 */
TEST_F(SnakeTipSetTest, Configure) {
  DataContainer pts = {0, 0,
                       0, 2};
  SnakeTip * t1 = ConstructTip(pts, true, 2);
  SnakeTipSet *ts = new SnakeTipSet(t1);

  pts = {0, 0,
         -1, 0};
  SnakeTip * t2 = ConstructTip(pts, true, 2);
  ts->Add(t2);

  pts = {0, 0,
         0, -2};
  SnakeTip * t3 = ConstructTip(pts, true, 2);
  ts->Add(t3);

  pts = {0, 0,
         2, 2};
  SnakeTip * t4 = ConstructTip(pts, true, 2);
  ts->Add(t4);

  EXPECT_TRUE(ts->Has(t1));
  EXPECT_TRUE(ts->Has(t2));
  EXPECT_TRUE(ts->Has(t3));
  EXPECT_TRUE(ts->Has(t4));
  SnakeContainer previous_snakes;
  ts->Configure(previous_snakes, 1, 3.1415926 * 2 / 3, 1);
  // EXPECT_FALSE(ts->Has(t1));
  // EXPECT_FALSE(ts->Has(t2));
  // EXPECT_FALSE(ts->Has(t3));
  // EXPECT_FALSE(ts->Has(t4));

  EXPECT_TRUE(AreLinked(t1, t3));
  EXPECT_TRUE(AreLinked(t2, t4));
  delete ts;
}

/**
 * Test this case:
 *               |
 *               |
 */
TEST_F(SnakeTipSetTest, Configure2) {
  DataContainer pts = {0, 0,
                       0, 2};
  SnakeTip * t0h = nullptr;
  SnakeTip * t0t = nullptr;
  ConstructTips(pts, &t0h, &t0t, 2);
  SnakeTipSet *ts = new SnakeTipSet(t0h);
  ts->Add(t0t);

  pts = {0, -1,
         0, -2};
  SnakeTip * t1h = nullptr;
  SnakeTip * t1t = nullptr;
  ConstructTips(pts, &t1h, &t1t, 2);
  ts->Add(t1h);
  ts->Add(t1t);
  SnakeContainer previous_snakes;
  ts->Configure(previous_snakes, 1, 3.1415926 * 2 / 3, 10);

  EXPECT_FALSE(AreLinked(t0h, t0t));
  EXPECT_TRUE(AreLinked(t0h, t1h));
  EXPECT_FALSE(AreLinked(t0h, t1t));

  EXPECT_FALSE(AreLinked(t0t, t1h));
  EXPECT_FALSE(AreLinked(t0t, t1t));

  EXPECT_FALSE(AreLinked(t1h, t1t));

  delete ts;
}

/**
 * Test this case:
 *              __/__
 */
TEST_F(SnakeTipSetTest, Configure3) {
  DataContainer pts = {0, 0,
                       1, 4};
  SnakeTip * t0h = nullptr;
  SnakeTip * t0t = nullptr;
  ConstructTips(pts, &t0h, &t0t, 2);
  SnakeTipSet *ts = new SnakeTipSet(t0h);
  ts->Add(t0t);

  pts = {-1, 0,
         -3, 0};
  SnakeTip * t1h = nullptr;
  SnakeTip * t1t = nullptr;
  ConstructTips(pts, &t1h, &t1t, 2);
  ts->Add(t1h);
  ts->Add(t1t);

  pts = {1, 0,
         3, 0};
  SnakeTip * t2h = nullptr;
  SnakeTip * t2t = nullptr;
  ConstructTips(pts, &t2h, &t2t, 2);
  ts->Add(t2h);
  ts->Add(t2t);
  SnakeContainer previous_snakes;
  ts->Configure(previous_snakes, 1, 3.1415926 / 2, 10);

  EXPECT_FALSE(AreLinked(t0h, t0t));
  EXPECT_FALSE(AreLinked(t0h, t1h));
  EXPECT_FALSE(AreLinked(t0h, t1t));
  EXPECT_FALSE(AreLinked(t0h, t2h));
  EXPECT_FALSE(AreLinked(t0h, t2t));

  EXPECT_FALSE(AreLinked(t0t, t1h));
  EXPECT_FALSE(AreLinked(t0t, t1t));
  EXPECT_FALSE(AreLinked(t0t, t2h));
  EXPECT_FALSE(AreLinked(t0t, t2t));

  EXPECT_FALSE(AreLinked(t1h, t1t));
  EXPECT_TRUE(AreLinked(t1h, t2h));
  EXPECT_FALSE(AreLinked(t1h, t2t));

  EXPECT_FALSE(AreLinked(t1t, t2h));
  EXPECT_FALSE(AreLinked(t1t, t2t));

  EXPECT_FALSE(AreLinked(t2h, t2t));

  delete ts;
}

/**
 * Test this case:
 *              __\/__
 */
TEST_F(SnakeTipSetTest, Configure4) {
  DataContainer pts = {0, 0,
                       1, 4};
  SnakeTip * t0h = nullptr;
  SnakeTip * t0t = nullptr;
  ConstructTips(pts, &t0h, &t0t, 2);
  SnakeTipSet *ts = new SnakeTipSet(t0h);
  ts->Add(t0t);

  pts = {-1, 0,
         -3, 0};
  SnakeTip * t1h = nullptr;
  SnakeTip * t1t = nullptr;
  ConstructTips(pts, &t1h, &t1t, 2);
  ts->Add(t1h);
  ts->Add(t1t);

  pts = {1, 0,
         3, 0};
  SnakeTip * t2h = nullptr;
  SnakeTip * t2t = nullptr;
  ConstructTips(pts, &t2h, &t2t, 2);
  ts->Add(t2h);
  ts->Add(t2t);

  pts = {0, 0,
         -1, 4};
  SnakeTip * t3h = nullptr;
  SnakeTip * t3t = nullptr;
  ConstructTips(pts, &t3h, &t3t, 2);
  ts->Add(t3h);
  ts->Add(t3t);
  SnakeContainer previous_snakes;
  ts->Configure(previous_snakes, 1, 3.1415926 / 2, 10);


  EXPECT_FALSE(AreLinked(t0h, t0t));
  EXPECT_FALSE(AreLinked(t0h, t1h));
  EXPECT_FALSE(AreLinked(t0h, t1t));
  EXPECT_FALSE(AreLinked(t0h, t2h));
  EXPECT_FALSE(AreLinked(t0h, t2t));
  EXPECT_FALSE(AreLinked(t0h, t3h));
  EXPECT_FALSE(AreLinked(t0h, t3t));

  EXPECT_FALSE(AreLinked(t0t, t1h));
  EXPECT_FALSE(AreLinked(t0t, t1t));
  EXPECT_FALSE(AreLinked(t0t, t2h));
  EXPECT_FALSE(AreLinked(t0t, t2t));
  EXPECT_FALSE(AreLinked(t0t, t3h));
  EXPECT_FALSE(AreLinked(t0t, t3t));

  EXPECT_FALSE(AreLinked(t1h, t1t));
  EXPECT_TRUE(AreLinked(t1h, t2h));
  EXPECT_FALSE(AreLinked(t1h, t2t));
  EXPECT_FALSE(AreLinked(t1h, t3h));
  EXPECT_FALSE(AreLinked(t1h, t3t));

  EXPECT_FALSE(AreLinked(t1t, t2h));
  EXPECT_FALSE(AreLinked(t1t, t2t));
  EXPECT_FALSE(AreLinked(t1t, t3h));
  EXPECT_FALSE(AreLinked(t1t, t3t));

  EXPECT_FALSE(AreLinked(t2h, t2t));
  EXPECT_FALSE(AreLinked(t2h, t3h));
  EXPECT_FALSE(AreLinked(t2h, t3t));

  EXPECT_FALSE(AreLinked(t2t, t3h));
  EXPECT_FALSE(AreLinked(t2t, t3t));

  EXPECT_FALSE(AreLinked(t3h, t3t));

  delete ts;
}

/**
 * Test this case:
 *              \__/
 *              /  \
 */
TEST_F(SnakeTipSetTest, Configure5) {
  DataContainer pts = {3, 0,
                       4, 4};
  SnakeTip * t0h = nullptr;
  SnakeTip * t0t = nullptr;
  ConstructTips(pts, &t0h, &t0t, 2);
  SnakeTipSet *ts = new SnakeTipSet(t0h);
  ts->Add(t0t);

  pts = {-2, 0,
         2, 0};
  SnakeTip * t1h = nullptr;
  SnakeTip * t1t = nullptr;
  ConstructTips(pts, &t1h, &t1t, 2);
  ts->Add(t1h);
  ts->Add(t1t);

  pts = {-3, 0,
         -4, 4};
  SnakeTip * t2h = nullptr;
  SnakeTip * t2t = nullptr;
  ConstructTips(pts, &t2h, &t2t, 2);
  ts->Add(t2h);
  ts->Add(t2t);

  pts = {-3, 0,
         -4, -4};
  SnakeTip * t3h = nullptr;
  SnakeTip * t3t = nullptr;
  ConstructTips(pts, &t3h, &t3t, 2);
  ts->Add(t3h);
  ts->Add(t3t);

  pts = {3, 0,
         4, -4};
  SnakeTip * t4h = nullptr;
  SnakeTip * t4t = nullptr;
  ConstructTips(pts, &t4h, &t4t, 2);
  ts->Add(t4h);
  ts->Add(t4t);
  SnakeContainer previous_snakes;
  ts->Configure(previous_snakes, 1, 3.1415926 / 2, 10);


  EXPECT_FALSE(AreLinked(t0h, t0t));
  EXPECT_FALSE(AreLinked(t0h, t1h));
  EXPECT_FALSE(AreLinked(t0h, t1t));
  EXPECT_FALSE(AreLinked(t0h, t2h));
  EXPECT_FALSE(AreLinked(t0h, t2t));
  EXPECT_TRUE(AreLinked(t0h, t3h));
  EXPECT_FALSE(AreLinked(t0h, t3t));
  EXPECT_FALSE(AreLinked(t0h, t4h));
  EXPECT_FALSE(AreLinked(t0h, t4t));

  EXPECT_FALSE(AreLinked(t0t, t1h));
  EXPECT_FALSE(AreLinked(t0t, t1t));
  EXPECT_FALSE(AreLinked(t0t, t2h));
  EXPECT_FALSE(AreLinked(t0t, t2t));
  EXPECT_FALSE(AreLinked(t0t, t3h));
  EXPECT_FALSE(AreLinked(t0t, t3t));
  EXPECT_FALSE(AreLinked(t0t, t4h));
  EXPECT_FALSE(AreLinked(t0t, t4t));

  EXPECT_FALSE(AreLinked(t1h, t1t));
  EXPECT_FALSE(AreLinked(t1h, t2h));
  EXPECT_FALSE(AreLinked(t1h, t2t));
  EXPECT_FALSE(AreLinked(t1h, t3h));
  EXPECT_FALSE(AreLinked(t1h, t3t));
  EXPECT_FALSE(AreLinked(t1h, t4h));
  EXPECT_FALSE(AreLinked(t1h, t4t));

  EXPECT_FALSE(AreLinked(t1t, t2h));
  EXPECT_FALSE(AreLinked(t1t, t2t));
  EXPECT_FALSE(AreLinked(t1t, t3h));
  EXPECT_FALSE(AreLinked(t1t, t3t));
  EXPECT_FALSE(AreLinked(t1t, t4h));
  EXPECT_FALSE(AreLinked(t1t, t4t));

  EXPECT_FALSE(AreLinked(t2h, t2t));
  EXPECT_FALSE(AreLinked(t2h, t3h));
  EXPECT_FALSE(AreLinked(t2h, t3t));
  EXPECT_TRUE(AreLinked(t2h, t4h));
  EXPECT_FALSE(AreLinked(t2h, t4t));

  EXPECT_FALSE(AreLinked(t2t, t3h));
  EXPECT_FALSE(AreLinked(t2t, t3t));
  EXPECT_FALSE(AreLinked(t2t, t4h));
  EXPECT_FALSE(AreLinked(t2t, t4t));

  EXPECT_FALSE(AreLinked(t3h, t3t));
  EXPECT_FALSE(AreLinked(t3h, t4h));
  EXPECT_FALSE(AreLinked(t3h, t4t));

  EXPECT_FALSE(AreLinked(t3t, t4h));
  EXPECT_FALSE(AreLinked(t3t, t4t));

  EXPECT_FALSE(AreLinked(t4h, t4t));

  delete ts;
}

/**
 * Test this case:
 *             __  __
 *              /  \
 *              \__/
 */
TEST_F(SnakeTipSetTest, Configure6) {
  DataContainer pts = {-1, 0,
                       -3, 0};
  SnakeTip * t0h = nullptr;
  SnakeTip * t0t = nullptr;
  ConstructTips(pts, &t0h, &t0t, 2);
  SnakeTipSet *ts = new SnakeTipSet(t0h);
  ts->Add(t0t);

  pts = {0, 0,
         -3, -1,
         -3, -3,
         3, -3,
         3, -1,
         0, 0};
  SnakeTip * t1h = nullptr;
  SnakeTip * t1t = nullptr;
  ConstructTips(pts, &t1h, &t1t, 2);
  ts->Add(t1h);
  ts->Add(t1t);

  pts = {1, 0,
         3, 0};
  SnakeTip * t2h = nullptr;
  SnakeTip * t2t = nullptr;
  ConstructTips(pts, &t2h, &t2t, 2);
  ts->Add(t2h);
  ts->Add(t2t);

  const double direction_threshold = 2 * atan(3.0);
  std::cout << "threshold: " << direction_threshold << std::endl;
  SnakeContainer previous_snakes;
  ts->Configure(previous_snakes, 1, direction_threshold, 5);

  EXPECT_FALSE(AreLinked(t0h, t0t));
  EXPECT_FALSE(AreLinked(t0h, t1h));
  EXPECT_FALSE(AreLinked(t0h, t1t));
  EXPECT_TRUE(AreLinked(t0h, t2h));
  EXPECT_FALSE(AreLinked(t0h, t2t));

  EXPECT_FALSE(AreLinked(t0t, t1h));
  EXPECT_FALSE(AreLinked(t0t, t1t));
  EXPECT_FALSE(AreLinked(t0t, t2h));
  EXPECT_FALSE(AreLinked(t0t, t2t));

  EXPECT_TRUE(AreLinked(t1h, t1t));
  EXPECT_FALSE(AreLinked(t1h, t2h));
  EXPECT_FALSE(AreLinked(t1h, t2t));

  EXPECT_FALSE(AreLinked(t1t, t2h));
  EXPECT_FALSE(AreLinked(t1t, t2t));

  EXPECT_FALSE(AreLinked(t2h, t2t));

  delete ts;
}

/**
 * Test this case:
 *               |/
 */
TEST_F(SnakeTipSetTest, Configure7) {
  DataContainer pts = {0, 0,
                       0, 2};
  SnakeTip * t0h = nullptr;
  SnakeTip * t0t = nullptr;
  ConstructTips(pts, &t0h, &t0t, 2);
  SnakeTipSet *ts = new SnakeTipSet(t0h);
  ts->Add(t0t);

  pts = {0, 0,
         1, 4};
  SnakeTip * t1h = nullptr;
  SnakeTip * t1t = nullptr;
  ConstructTips(pts, &t1h, &t1t, 2);
  ts->Add(t1h);
  ts->Add(t1t);
  SnakeContainer previous_snakes;
  ts->Configure(previous_snakes, 1, 3.1415926 * 2 / 3, 10);

  EXPECT_FALSE(AreLinked(t0h, t0t));
  EXPECT_FALSE(AreLinked(t0h, t1h));
  EXPECT_FALSE(AreLinked(t0h, t1t));

  EXPECT_FALSE(AreLinked(t0t, t1h));
  EXPECT_FALSE(AreLinked(t0t, t1t));

  EXPECT_FALSE(AreLinked(t1h, t1t));

  delete ts;
}

/**
 * Test this case:
 *              /  \
 *              |__|
 */
TEST_F(SnakeTipSetTest, Configure8) {
  DataContainer   pts = {0, 0,
                         -3, -1,
                         -3, -3,
                         3, -3,
                         3, -1,
                         0, 0};
  SnakeTip * t0h = nullptr;
  SnakeTip * t0t = nullptr;
  ConstructTips(pts, &t0h, &t0t, 2);
  SnakeTipSet *ts = new SnakeTipSet(t0h);
  ts->Add(t0t);

  const double direction_threshold = 2 * atan(3.0);
  std::cout << "threshold: " << direction_threshold << std::endl;
  SnakeContainer previous_snakes;
  ts->Configure(previous_snakes, 1, direction_threshold, 5);

  EXPECT_FALSE(AreLinked(t0h, t0t));
  delete ts;
}


}  // namespace soax
