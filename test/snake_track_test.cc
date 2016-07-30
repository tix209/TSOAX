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


#include "../snake_track.h"
#include <vector>
#include <algorithm>
#include "gtest/gtest.h"


namespace soax {

class SnakeTrackTest : public ::testing::Test {
 protected:
  // SnakeTrackTest() {
  //   t1_ = {1, 2, 3, 0, 4};
  //   t2_ = {1, 0, 3, 0, 4};
  //   t3_ = {0, 0, 0, 0, 0};
  //   tracks_ = {t1_, t2_, t3_};
  // }

  // void PrintTracks() {
  //   for (auto t : tracks_)
  //     std::cout << t << std::endl;
  // }

  // SnakeTrack t1_;  // = {1, 2, 3, 0, 4};
  // SnakeTrack t2_;  // = {1, 0, 3, 0, 4};
  // SnakeTrack t3_;  // = {0, 0, 0, 0, 0};
  // std::vector<SnakeTrack> tracks_;
};

TEST_F(SnakeTrackTest, Constructor) {
  SnakeTrack t(5);
  for (size_t i = 0; i < 5; i++)
    EXPECT_EQ(nullptr, t.GetSnake(i));
}

// TEST_F(SnakeTrackTest, LessThanOp) {
//   PrintTracks();
//   std::sort(tracks_.begin(), tracks_.end());
//   PrintTracks();
//   EXPECT_EQ(t3_, tracks_[0]);
//   EXPECT_EQ(t2_, tracks_[1]);
//   EXPECT_EQ(t1_, tracks_[2]);
// }

// TEST_F(SnakeTrackTest, Contains) {
//   EXPECT_TRUE(t1_.Contains(4));
//   EXPECT_FALSE(t2_.Contains(24));
//   t2_.Append(24);
//   EXPECT_TRUE(t2_.Contains(24));
// }

}  // namespace soax
