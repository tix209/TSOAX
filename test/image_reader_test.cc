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


#include "../image_reader.h"
#include <string>
#include <vector>
#include <QDebug>
#include <QString>  // NOLINT(build/include_order)
#include "vtkImageData.h"
#include "gtest/gtest.h"
#include "../image.h"

namespace soax {

class ImageReaderTest : public ::testing::Test {
 protected:
  ImageReaderTest() {}

  ImageReader reader;
};

TEST_F(ImageReaderTest, Constructor) {
  EXPECT_EQ(0, reader.nslices_per_frame());
  EXPECT_EQ(nullptr, reader.GetImage());
  EXPECT_EQ(0, reader.GetNumberOfImages());
  EXPECT_EQ(QString(".."), reader.path());
  EXPECT_EQ(QString(), reader.GetFilePath());
  qDebug() << reader.GetAllowedFormatAsString();
}

TEST_F(ImageReaderTest, Read2DImageAndReset) {
  QString path = "../../rel4.0/test/data/last_frame.tif";
  reader.ReadFile(path);

  EXPECT_EQ(0, reader.nslices_per_frame());
  ASSERT_EQ(1, reader.GetNumberOfImages());
  EXPECT_EQ(QString("/home/ting/exp/soax/rel4.0/test/data"),
            reader.path());
  EXPECT_EQ(QString("/home/ting/exp/soax/rel4.0/test/data/last_frame.tif"),
            reader.GetFilePath());

  int *dim = reader.GetImage()->GetDimensions();
  EXPECT_EQ(std::vector<int>({672, 512, 1}),
            std::vector<int>(dim, dim + 3));
  EXPECT_EQ(288, GetMinimumIntensity(reader.GetImage()));
  EXPECT_EQ(2038, GetMaximumIntensity(reader.GetImage()));
  EXPECT_EQ(2, GetImageDimension(reader.GetImage()));

  reader.Reset();
  EXPECT_EQ(0, reader.nslices_per_frame());
  EXPECT_EQ(nullptr, reader.GetImage());
  ASSERT_EQ(0, reader.GetNumberOfImages());
  EXPECT_EQ(QString("/home/ting/exp/soax/rel4.0/test/data"),
            reader.path());
  EXPECT_EQ(QString(), reader.GetFilePath());
}

TEST_F(ImageReaderTest, ReadFileAs3DImage) {
  QString path = "../../rel4.0/test/data/fz.mha";
  reader.ReadFile(path);
  EXPECT_EQ(0, reader.nslices_per_frame());
  EXPECT_EQ(1, reader.GetNumberOfImages());
  EXPECT_EQ(QString("/home/ting/exp/soax/rel4.0/test/data"),
            reader.path());
  EXPECT_EQ(QString("/home/ting/exp/soax/rel4.0/test/data/fz.mha"),
            reader.GetFilePath());

  int *dim = reader.GetImage()->GetDimensions();
  EXPECT_EQ(std::vector<int>({65, 66, 28}),
            std::vector<int>(dim, dim + 3));
  EXPECT_EQ(0, GetMinimumIntensity(reader.GetImage()));
  EXPECT_EQ(255, GetMaximumIntensity(reader.GetImage()));
  EXPECT_EQ(3, GetImageDimension(reader.GetImage()));
}

TEST_F(ImageReaderTest, ReadFileAs2DSequence) {
  QString path = "../../rel4.0/test/data/actin-em.tif";
  reader.set_nslices_per_frame(1);
  reader.ReadFile(path);

  EXPECT_EQ(1, reader.nslices_per_frame());
  EXPECT_EQ(50, reader.GetNumberOfImages());
  EXPECT_EQ(QString("/home/ting/exp/soax/rel4.0/test/data"),
            reader.path());
  EXPECT_EQ(QString("/home/ting/exp/soax/rel4.0/test/data/actin-em.tif"),
            reader.GetFilePath());

  int *dim = reader.GetImage()->GetDimensions();
  EXPECT_EQ(std::vector<int>({134, 135, 1}),
            std::vector<int>(dim, dim + 3));
  EXPECT_EQ(76, GetMinimumIntensity(reader.GetImage()));
  EXPECT_EQ(184, GetMaximumIntensity(reader.GetImage()));
  EXPECT_EQ(2, GetImageDimension(reader.GetImage()));
}

TEST_F(ImageReaderTest, ReadFileAs3DSequence) {
  QString path = "../../rel4.0/test/data/seq_iso_4-1.mha";
  reader.set_nslices_per_frame(74);
  reader.ReadFile(path);

  EXPECT_EQ(74, reader.nslices_per_frame());
  EXPECT_EQ(30, reader.GetNumberOfImages());
  EXPECT_EQ(QString("/home/ting/exp/soax/rel4.0/test/data"),
            reader.path());
  EXPECT_EQ(QString("/home/ting/exp/soax/rel4.0/test/data/seq_iso_4-1.mha"),
            reader.GetFilePath());

  int *dim = reader.GetLastImage()->GetDimensions();
  EXPECT_EQ(std::vector<int>({190, 76, 74}),
            std::vector<int>(dim, dim + 3));
  EXPECT_EQ(189, GetMinimumIntensity(reader.GetImage()));
  EXPECT_EQ(527, GetMaximumIntensity(reader.GetImage()));
  for (unsigned i = 0; i < 30; i++) {
    EXPECT_EQ(0, reader.GetImage(i)->GetExtent()[4]);
    EXPECT_EQ(73, reader.GetImage(i)->GetExtent()[5]);
  }

  EXPECT_EQ(3, GetImageDimension(reader.GetImage()));
}

TEST_F(ImageReaderTest, ReadDirAs2DSequence) {
  reader.ReadDir("../../rel4.0/test/data/2d_seq");
  EXPECT_EQ(0, reader.nslices_per_frame());
  EXPECT_EQ(12, reader.GetNumberOfImages());
  EXPECT_EQ(QString("/home/ting/exp/soax/rel4.0/test/data/2d_seq"),
            reader.path());
  EXPECT_EQ(QString("/home/ting/exp/soax/rel4.0/test/data/2d_seq/"
                    "ftfm3 timelapse_w1Brightfield_s7_t12.TIF"),
            reader.GetFilePath(reader.GetNumberOfImages() - 1));

  vtkImageData *image = reader.GetLastImage();
  int *dim = image->GetDimensions();
  EXPECT_EQ(std::vector<int>({1280, 1024, 1}),
            std::vector<int>(dim, dim + 3));
  EXPECT_EQ(std::vector<double>({880, 4095}),
            std::vector<double>(image->GetScalarRange(),
                                image->GetScalarRange() + 2));
  for (unsigned i = 0; i < 12; i++) {
    EXPECT_EQ(0, reader.GetImage(i)->GetExtent()[4]);
    EXPECT_EQ(0, reader.GetImage(i)->GetExtent()[5]);
  }

  EXPECT_EQ(2, GetImageDimension(reader.GetImage()));
}

TEST_F(ImageReaderTest, ReadDirAs2DSet) {
  reader.ReadDir("../../rel4.0/test/data/2d_set");

  EXPECT_EQ(0, reader.nslices_per_frame());
  EXPECT_EQ(8, reader.GetNumberOfImages());
  EXPECT_EQ(QString("/home/ting/exp/soax/rel4.0/test/data/2d_set"),
            reader.path());
  EXPECT_EQ(QString("/home/ting/exp/soax/rel4.0/test/data/2d_set/"
                    "lake-mountains5.jpg"),
            reader.GetFilePath(reader.GetNumberOfImages() - 1));

  EXPECT_EQ(2, GetImageDimension(reader.GetImage()));
}

TEST_F(ImageReaderTest, ReadDirAs3DSequence) {
  reader.ReadDir("../../rel4.0/test/data/3d_seq/");

  EXPECT_EQ(0, reader.nslices_per_frame());
  EXPECT_EQ(5, reader.GetNumberOfImages());
  EXPECT_EQ(QString("/home/ting/exp/soax/rel4.0/test/data/3d_seq"),
            reader.path());
  EXPECT_EQ(QString("/home/ting/exp/soax/rel4.0/test/data/3d_seq/10s.mha"),
            reader.GetFilePath());

  int *dim = reader.GetImage(1)->GetDimensions();
  EXPECT_EQ(std::vector<int>({80, 150, 80}),
            std::vector<int>(dim, dim + 3));
  EXPECT_EQ(std::vector<double>({224, 270}),
            std::vector<double>(reader.GetImage()->GetScalarRange(),
                                reader.GetImage()->GetScalarRange() + 2));
  for (unsigned i = 0; i < 5; i++) {
    EXPECT_EQ(0, reader.GetImage(i)->GetExtent()[4]);
    EXPECT_EQ(79, reader.GetImage(i)->GetExtent()[5]);
  }

  EXPECT_EQ(3, GetImageDimension(reader.GetImage()));
}


}  // namespace soax
