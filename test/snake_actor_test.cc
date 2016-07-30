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


#include "../snake_actor.h"
#include "gtest/gtest.h"
#include "vtkPolyData.h"
#include "../snake.h"

namespace soax {
class SnakeActorTest : public ::testing::Test {
 protected:
  SnakeActorTest() {}

  // vtkPolyData * MakePolyData(const SnakeContainer &snakes) const {
  //   return sa.MakePolyData(snakes);
  // }

  Snake s1;
  Snake s2;
  Snake s3;
  SnakeActor sa;
};

TEST_F(SnakeActorTest, Constructor) {
  // EXPECT_EQ(nullptr, sa.single_actor());
}

// TEST_F(SnakeActorTest, MakePolyData) {
//   std::vector<double> pts1 = {4, 10, 4,
//                               4, 12, 4,
//                               4, 15, 4,
//                               4, 18, 4};
//   s1.set_vertices(pts1);

//   std::vector<double> pts2 = {10, 4, 4,
//                               12, 4, 4,
//                               14, 4, 4,
//                               16, 4, 4,
//                               18, 4, 4};
//   s2.set_vertices(pts2);

//   std::vector<double> pts3 = {4, 10, 4,
//                               4, 12, 4,
//                               4, 15, 4,
//                               4, 18, 4};
//   s3.set_vertices(pts3);

//   std::vector<double> total_pts(pts1);
//   total_pts.insert(total_pts.end(), pts2.begin(), pts2.end());
//   total_pts.insert(total_pts.end(), pts3.begin(), pts3.end());
//   SnakeContainer sc;
//   sc.push_back(&s1);
//   sc.push_back(&s2);
//   sc.push_back(&s3);

//   vtkPolyData *curve = MakePolyData(sc);
//   EXPECT_EQ(3 + 4 + 3, curve->GetNumberOfCells());
//   EXPECT_EQ(13, curve->GetNumberOfPoints());

//   for(vtkIdType i = 0; i < curve->GetNumberOfPoints(); i++) {
//     double p[3];
//     curve->GetPoint(i,p);
//     EXPECT_EQ(total_pts[3 * i], p[0]);
//     EXPECT_EQ(total_pts[3 * i + 1], p[1]);
//     EXPECT_EQ(total_pts[3 * i + 2], p[2]);
//   }
//   curve->Delete();
// }

}  // namespace soax
