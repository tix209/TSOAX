/**
 * Copyright (C) 2016 Lehigh University.
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


#include "../image_resampler.h"
#include "vtkImageData.h"
#include "gtest/gtest.h"
#include "../image_reader.h"
#include "../image.h"

namespace soax {

class ImageResamplerTest : public ::testing::Test {
 protected:
  ImageResamplerTest() {}

  ImageResampler resampler_;
};

TEST_F(ImageResamplerTest, SimpleImage) {
  ImageReader reader;
  /**
   * The image dimension is 30x10x10. When x is in [10, 20), the center of its
   * YZ cross-section looks like the following:
   *
   *                  0 0 0 0 0 0 0
   *                  0 1 1 1 1 1 0
   *                  0 1 2 2 2 1 0
   *                  0 1 2 3 2 1 0
   *                  0 1 2 2 2 1 0
   *                  0 1 1 1 1 1 0
   *                  0 0 0 0 0 0 0
   *
   * The center (value = 3) is (x, 5, 5). The rest of the image are all zeros.
   */
  reader.ReadFile("../../rel4.0/test/data/simple.mha");
  int *dim = reader.GetImage()->GetDimensions();
  ASSERT_EQ(30, dim[0]);
  ASSERT_EQ(10, dim[1]);
  ASSERT_EQ(10, dim[2]);

  reader.GetImage()->SetSpacing(1.0, 1.0, 2.0);
  double *spacing = reader.GetImage()->GetSpacing();
  ASSERT_EQ(1, spacing[0]);
  ASSERT_EQ(1, spacing[1]);
  ASSERT_EQ(2, spacing[2]);

  resampler_.Resize(reader.GetImage());
  // Sanity chack of the resultant image size
  EXPECT_EQ(30, resampler_.GetImage()->GetDimensions()[0]);
  EXPECT_EQ(10, resampler_.GetImage()->GetDimensions()[1]);
  EXPECT_EQ(20, resampler_.GetImage()->GetDimensions()[2]);

  spacing = resampler_.GetImage()->GetSpacing();
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(1.0, spacing[i]) << i;
  }

  EXPECT_EQ(0, GetImageIntensity(resampler_.GetImage(), 9, 5, 5));
  EXPECT_EQ(3, GetImageIntensity(resampler_.GetImage(), 10, 5, 10));
  resampler_.WriteImage("../test_resampled_image.tif");
}

}  // namespace soax
