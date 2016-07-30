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
 * This file test the Viewpoint class.
 */


#include "../viewpoint.h"
#include <vector>
#include <fstream>
#include <sstream>
#include "gtest/gtest.h"


namespace soax {

class ViewpointTest : public ::testing::Test {
 protected:
  ViewpointTest() {
  }

  Viewpoint vp;
};

TEST_F(ViewpointTest, Constructor) {
  EXPECT_EQ(vp.path().toStdString(), std::string(".."));
  EXPECT_EQ(vp.camera()->GetClippingRange()[0], 0.01);
  EXPECT_EQ(vp.camera()->GetClippingRange()[1], 1000.01);
  EXPECT_EQ(std::vector<double>(vp.camera()->GetPosition(),
                                vp.camera()->GetPosition() + 3),
            std::vector<double>({0, 0, 1}));
  EXPECT_EQ(std::vector<double>(vp.camera()->GetFocalPoint(),
                                vp.camera()->GetFocalPoint() + 3),
            std::vector<double>({0, 0, 0}));
  EXPECT_EQ(std::vector<double>(vp.camera()->GetViewUp(),
                                vp.camera()->GetViewUp() + 3),
            std::vector<double>({0, 1, 0}));
  EXPECT_TRUE(vp.default_viewpoint_str().empty());
}

TEST_F(ViewpointTest, ToString) {
  std::string vpstr = "ClippingRange\n0.01 1000.01\n"
      "CameraPosition\n0 0 1\n"
      "CameraFocalPoint\n0 0 0\n"
      "CameraViewUp\n0 1 0\n";
  ASSERT_EQ(vpstr, vp.ToString());
}

TEST_F(ViewpointTest, SaveDefault) {
  vp.SaveDefault();
  std::string vpstr = "ClippingRange\n0.01 1000.01\n"
      "CameraPosition\n0 0 1\n"
      "CameraFocalPoint\n0 0 0\n"
      "CameraViewUp\n0 1 0\n";
  EXPECT_EQ(vp.default_viewpoint_str(), vpstr);
}

TEST_F(ViewpointTest, SaveToFile) {
  vp.camera()->SetClippingRange(440.812, 892.834);
  vp.camera()->SetPosition(94.5, 94.5, 736.745);
  vp.camera()->SetFocalPoint(94.5, 94.5, 96.5);
  vp.camera()->SetViewUp(0, 1, 0);
  vp.SaveToFile("output_viewpoint.cam");
  std::ifstream infile("output_viewpoint.cam");
  ASSERT_TRUE(infile.is_open());
  std::stringstream buffer;
  std::string line;
  while (std::getline(infile, line)) {
    buffer << line << "\n";
  }
  EXPECT_EQ(buffer.str(), vp.ToString());
  EXPECT_EQ(vp.path().toStdString(),
            std::string("/home/ting/exp/soax/debug/test"));
}

TEST_F(ViewpointTest, LoadFromFile) {
  vp.LoadFromFile("../../rel4.0/test/data/myview.txt");
  std::string vpstr = "ClippingRange\n440.812 892.834\n"
      "CameraPosition\n94.5 94.5 736.745\n"
      "CameraFocalPoint\n94.5 94.5 96.5\n"
      "CameraViewUp\n0 1 0\n";
  EXPECT_EQ(vpstr, vp.ToString());
  EXPECT_EQ(vp.path().toStdString(),
            std::string("/home/ting/exp/soax/rel4.0/test/data"));
}

TEST_F(ViewpointTest, LoadDefault) {
  vp.LoadDefault();
  std::string vpstr = "ClippingRange\n0.01 1000.01\n"
      "CameraPosition\n0 0 1\n"
      "CameraFocalPoint\n0 0 0\n"
      "CameraViewUp\n0 1 0\n";
  EXPECT_EQ(vp.ToString(), vpstr);
}

}  // namespace soax
