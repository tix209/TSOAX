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
 * This file test the ParametersDialog.
 */


#include "../parameters_dialog.h"
#include <QApplication>
#include <QtTest/QtTest>
#include "gtest/gtest.h"
#include "../snake_parameters.h"

namespace soax {

class ParametersDialogTest : public ::testing::Test {
 protected:
  ParametersDialogTest() {}

  ParametersDialog pd;
  SnakeParameters sp;
};

TEST_F(ParametersDialogTest, Constructor) {
  // pd.SetParameters(&sp);
  EXPECT_EQ(0, pd.GetIntensityScaling());
  EXPECT_EQ(0, pd.GetSigma());
  EXPECT_EQ(0, pd.GetRidgeThreshold());
  EXPECT_EQ(0, pd.GetMaximumForeground());
  EXPECT_EQ(0, pd.GetMinimumForeground());
  EXPECT_FALSE(pd.InitXChecked());
  EXPECT_FALSE(pd.InitYChecked());
  EXPECT_FALSE(pd.InitZChecked());
  EXPECT_EQ(0, pd.GetSpacing());
  EXPECT_EQ(0, pd.GetMinimumLength());
  EXPECT_EQ(0, pd.GetMaximumIterations());
  EXPECT_EQ(0, pd.GetChangeThreshold());
  EXPECT_EQ(0, pd.GetCheckPeriod());
  EXPECT_EQ(0, pd.GetBeta());
  EXPECT_EQ(0, pd.GetGamma());
  EXPECT_EQ(0, pd.GetExternalFactor());
  EXPECT_EQ(0, pd.GetStretchFactor());
  EXPECT_EQ(0, pd.GetNumberOfSectors());
  EXPECT_EQ(0, pd.GetZSpacing());
  EXPECT_EQ(0, pd.GetRadialNear());
  EXPECT_EQ(0, pd.GetRadialFar());
  EXPECT_EQ(0, pd.GetDelta());
  EXPECT_EQ(0, pd.GetOverlapThreshold());
  EXPECT_EQ(0, pd.GetGroupingDistanceThreshold());
  EXPECT_EQ(0, pd.GetGroupingDelta());
  EXPECT_EQ(0, pd.GetDirectionThreshold());
  EXPECT_FALSE(pd.DampZ());
}

TEST_F(ParametersDialogTest, DefaultParameters) {
  pd.SetParameters(&sp);
  EXPECT_EQ(0, pd.GetIntensityScaling());
  EXPECT_EQ(0, pd.GetSigma());
  EXPECT_EQ(0.01, pd.GetRidgeThreshold());
  EXPECT_EQ(65535, pd.GetMaximumForeground());
  EXPECT_EQ(0, pd.GetMinimumForeground());
  EXPECT_TRUE(pd.InitXChecked());
  EXPECT_TRUE(pd.InitYChecked());
  EXPECT_TRUE(pd.InitZChecked());
  EXPECT_EQ(1, pd.GetSpacing());
  EXPECT_EQ(10, pd.GetMinimumLength());
  EXPECT_EQ(10000, pd.GetMaximumIterations());
  EXPECT_EQ(0.1, pd.GetChangeThreshold());
  EXPECT_EQ(100, pd.GetCheckPeriod());
  EXPECT_EQ(0.01, pd.GetAlpha());
  EXPECT_EQ(0.1, pd.GetBeta());
  EXPECT_EQ(2, pd.GetGamma());
  EXPECT_EQ(1, pd.GetExternalFactor());
  EXPECT_EQ(0.2, pd.GetStretchFactor());
  EXPECT_EQ(8, pd.GetNumberOfSectors());
  EXPECT_EQ(1.0, pd.GetZSpacing());
  EXPECT_EQ(4, pd.GetRadialNear());
  EXPECT_EQ(8, pd.GetRadialFar());
  EXPECT_EQ(4, pd.GetDelta());
  EXPECT_EQ(1, pd.GetOverlapThreshold());
  EXPECT_EQ(4, pd.GetGroupingDistanceThreshold());
  EXPECT_EQ(8, pd.GetGroupingDelta());
  EXPECT_EQ(2.1, pd.GetDirectionThreshold());
  EXPECT_FALSE(pd.DampZ());
}

TEST_F(ParametersDialogTest, ChangeValues) {
  if (pd.exec()) {
    EXPECT_EQ(65534, pd.GetMaximumForeground());
  }
}

}  // namespace soax

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  QApplication app(argc, argv);
  return RUN_ALL_TESTS();
}
