/**
 * Copyright (C) 2017 Lehigh University.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see http://www.gnu.org/licenses/.
 *
 * Author: Ting Xu (xuting.bme@gmail.com)
 *
 * ImageReader reads 2D/3D/4D images in TIFF, Meta, PNG, JPEG, and BMP
 * format. The input can be a single file (READFILE()) or a directory
 * containing multiple files (READDIR()). For a single file, it can be a
 * 2D/3D image, or it can be a sequence of images stacked together as a
 * single file. In the latter case, one needs to call
 * SET_NSLICE_PER_FRAME() before calling READFILE().
 */


#ifndef IMAGE_READER_H_
#define IMAGE_READER_H_

#include <vector>
#include <QString>  // NOLINT
#include <QStringList>  // NOLINT
#include <QSet>  // NOLINT
#include "./util.h"

class vtkImageData;

namespace soax {

class ImageReader {
 public:
  ImageReader();
  ~ImageReader();

  /**
   * Read a single file containing one or more images.
   */
  void ReadFile(const QString &filename);

  /**
   * Read a DIRECTORY that typically contains a time-lapse sequence of
   * images.
   */
  void ReadDir(const QString &directory);

  /**
   * Returns the number of slices per frame in the image sequence. The
   * value is greater than 0 only if the input file contains multiple
   * images. Default is 0.
   */
  int nslices_per_frame() const {return nslices_per_frame_;}

  /**
   * Set the number of slices per frame. It needs to be set before loading
   * a single file containing multiple images.
   */
  void set_nslices_per_frame(int nslices) {nslices_per_frame_ = nslices;}

  /**
   * Returns the pointer to the image indexed by INDEX. Default is the
   * first image.
   */
  vtkImageData * GetImage(size_t index = 0) const;

  /**
   * Returns the pointer to the last image.
   */
  vtkImageData * GetLastImage() const;

  /**
   * Returns the index of IMAGE. If not found, returns -1.
   */
  int FindImage(vtkImageData *image) const;

  /**
   * Returns the number of images read.
   */
  size_t GetNumberOfImages() const {return images_.size();}

  /**
   * Returns the absolute path of the input file (without
   * filename). Default is "..".
   */
  QString path() const {return path_;}

  /**
   * Returns the path of the image with filename indexed by INDEX.
   */
  QString GetFilePath(size_t index = 0) const;

  /**
   * Returns the frame name without the suffix.
   */
  QString GetFileNameWithoutSuffix(size_t index = 0) const;

  /**
   * Returns a string representation of allowed format. This is used as the
   * format filter argument in the QFileDialog.
   */
  QString GetAllowedFormatAsString() const;

  /**
   * Reset the state of this class. The path_ is kept until a new file is read.
   */
  void Reset();

  /**
   * Reverse the sequence.
   */
  void ReverseImageSequence();

 private:
  /**
   * Release the memory of members. Called by the destructor.
   */
  void DeleteImages();

  /**
   * Divide and load multiple images from a single IMAGE.
   */
  void LoadImages(vtkImageData *image);

  /**
   * Returns a sorted list of filenames (natural sorting). Files without
   * indices in its name are put before the ones with indices. If multiple
   * files have the same index, those collided names are put before the
   * ones with indices but after files without indices.
   */
  QStringList SortFilenames(const QStringList &filenames) const;

  /**
   * Returns the sequence index from the filename. The index is assumed to
   * be the last numeric value appeared in the filename.
   */
  int ExtractIndex(const QString &filename) const;

  /**
   * The path of input directory.
   */
  QString path_ = "..";
  /**
   * A list of paths of input images.
   */
  QStringList paths_;
  int nslices_per_frame_ = 0;

  std::vector<vtkImageData *> images_;
  QSet<QString> allowed_format_;

  ImageReader(const ImageReader &);
  void operator=(const ImageReader &);
};

}  // namespace soax

#endif  // IMAGE_READER_H_
