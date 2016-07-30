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


#include "../multisnake.h"
#include <vector>
#include "vtkImageData.h"
#include "vtkImageExtractComponents.h"
#include "../image_reader.h"
#include "gtest/gtest.h"
#include "../snake.h"
#include "../snake_parameters.h"
#include "../image.h"
#include "../interpolator.h"

namespace soax {

class MultisnakeTest : public ::testing::Test {
 protected:
  MultisnakeTest() {}

  void ReadImage(Multisnake &ms) {
    reader.ReadFile("../../rel4.0/test/data/simple.mha");
    ms.set_image(reader.GetImage());
  }

  vtkImageData * InitializeFlagImage(Multisnake &ms, int ncomponents) {
    return ms.InitializeFlagImage(ncomponents);
  }

  void ScanGradient(Multisnake &ms, vtkImageData *ridge_image, int dim) {
    ms.ComputeGradient();
    ms.ScanGradient(ridge_image, dim);
  }

  void GenerateCandidates(Multisnake &ms, vtkImageData *ridge_image,
                          vtkImageData *candidate_image, int dim) {
    ms.GenerateCandidates(ridge_image, candidate_image, dim);
  }

  void WriteFlagImages(vtkImageData *image, int dim,
                       const std::string &filename) {
    for (int j = 0; j < dim; j++) {
      vtkImageExtractComponents *extractor =
          vtkImageExtractComponents::New();
      extractor->SetInputData(image);
      extractor->SetComponents(j);
      extractor->Update();
      std::string final_name = filename;
      final_name += std::to_string(j) + ".mhd";
      WriteMetaImage(extractor->GetOutput(), final_name);
      extractor->Delete();
    }
  }

  ImageReader reader;
  Multisnake ms1;
};


TEST_F(MultisnakeTest, InitializeFlagImage) {
  ReadImage(ms1);
  vtkImageData * flag_image = InitializeFlagImage(ms1, 3);

  EXPECT_EQ(3, flag_image->GetNumberOfScalarComponents());
  EXPECT_EQ(3, flag_image->GetDataDimension());

  int total_n_pts = 1;
  int total_n_cells = 1;
  for (int j = 0; j < 3; j++) {
    std::cout << "Dimensions along axis " << j << ": "
              << flag_image->GetDimensions()[j] << std::endl;
    total_n_pts *= flag_image->GetDimensions()[j];
    total_n_cells *= flag_image->GetDimensions()[j] - 1;
    // EXPECT_EQ(gc.gradient()->GetDimensions()[j],
    //           flag_image->GetDimensions()[j]);
    EXPECT_EQ(0, flag_image->GetOrigin()[j]);
    EXPECT_EQ(1, flag_image->GetSpacing()[j]);
  }

  EXPECT_EQ(total_n_pts, flag_image->GetNumberOfPoints());
  EXPECT_EQ(total_n_cells, flag_image->GetNumberOfCells());
  // flag_image->Initialize();  // this changes scalar type to double, why?
  int *dim = flag_image->GetDimensions();
  for (int z = 0; z < dim[2]; z++) {
    for (int y = 0; y < dim[1]; y++) {
      for (int x = 0; x < dim[0]; x++) {
        for (int j = 0; j < 3; j++) {
          EXPECT_EQ(0, GetFlag(flag_image, x, y, z)[j]);
        }
      }
    }
  }
  EXPECT_STREQ("unsigned char", flag_image->GetScalarTypeAsString());

  WriteFlagImages(flag_image, 3, "flag_image");
  flag_image->Delete();
}

TEST_F(MultisnakeTest, ScanGradient) {
  ReadImage(ms1);
  ms1.snake_parameters()->set_ridge_threshold(0.1);
  vtkImageData *ridge_image = InitializeFlagImage(ms1, 3);
  ScanGradient(ms1, ridge_image, 3);
  int *dim = ridge_image->GetDimensions();
  for (int z = 0; z < dim[2]; z++) {
    for (int y = 0; y < dim[1]; y++) {
      for (int x = 0; x < dim[0]; x++) {
        if (x == 15 && y > 2 && y < 8 && z > 2 && z < 8) { // x
          EXPECT_TRUE(GetFlag(ridge_image, x, y, z)[0]);
        } else {
          EXPECT_FALSE(GetFlag(ridge_image, x, y, z)[0]);
        }

        if (y == 5 && x > 9 && x < 20 && z > 2 && z < 8) { // y
          EXPECT_TRUE(GetFlag(ridge_image, x, y, z)[1]);
        } else {
          EXPECT_FALSE(GetFlag(ridge_image, x, y, z)[1]);
        }

        if (z == 5 && x > 9 && x < 20 && y > 2 && y < 8) { // z
          EXPECT_TRUE(GetFlag(ridge_image, x, y, z)[2]);
        } else {
          EXPECT_FALSE(GetFlag(ridge_image, x, y, z)[2]);
        }
      }
    }
  }

  WriteFlagImages(ridge_image, 3, "ridge");
  ridge_image->Delete();
}

TEST_F(MultisnakeTest, GenerateCandidates) {
  ReadImage(ms1);
  ms1.snake_parameters()->set_ridge_threshold(0.1);
  ms1.snake_parameters()->set_maximum_foreground(65535);
  EXPECT_EQ(65535, ms1.snake_parameters()->maximum_foreground());
  vtkImageData *ridge_image = InitializeFlagImage(ms1, 3);
  vtkImageData *candidate_image = InitializeFlagImage(ms1, 3);
  ScanGradient(ms1, ridge_image, 3);
  GenerateCandidates(ms1, ridge_image, candidate_image, 3);

  int *dim = candidate_image->GetDimensions();
  for (int z = 0; z < dim[2]; z++) {
    for (int y = 0; y < dim[1]; y++) {
      for (int x = 0; x < dim[0]; x++) {
        if (x > 9 && x < 20 && y == 5 && z == 5) { // x
          EXPECT_TRUE(GetFlag(candidate_image, x, y, z)[0]);
        } else {
          EXPECT_FALSE(GetFlag(candidate_image, x, y, z)[0]);
        }

        if (x == 15 && z == 5 && y > 2 && y < 8) { // y
          EXPECT_TRUE(GetFlag(candidate_image, x, y, z)[1]);
        } else {
          EXPECT_FALSE(GetFlag(candidate_image, x, y, z)[1]);
        }

        if (x == 15 && y == 5 && z > 2 && z < 8) { // z
          EXPECT_TRUE(GetFlag(candidate_image, x, y, z)[2]);
        } else {
          EXPECT_FALSE(GetFlag(candidate_image, x, y, z)[2]);
        }
      }
    }
  }
  WriteFlagImages(candidate_image, 3, "candidate");
  ridge_image->Delete();
  candidate_image->Delete();
}

TEST_F(MultisnakeTest, InitializeAndDeleteSnakes) {
  ReadImage(ms1);
  ms1.snake_parameters()->set_ridge_threshold(0.01);
  ms1.snake_parameters()->set_maximum_foreground(65535);
  ms1.Initialize();

  EXPECT_EQ(5, ms1.GetNumberOfInitialSnakes());

  // for (unsigned i = 0; i < ms1.GetNumberOfInitialSnakes(); i++) {
  //   std::cout << ms1.GetInitialSnake(i)->GetLength() << std::endl;
  // }
}




}  // namespace soax
