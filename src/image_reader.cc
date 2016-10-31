/**
 * Copyright (C) 2016 Lehigh University.
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
 */


#include "include/image_reader.h"
#include <algorithm>
#include <QFileInfo>
#include <QMap>
#include <QDir>
#include "vtkImageData.h"
#include "vtkImageReader2Factory.h"
#include "vtkImageReader2.h"
#include "vtkExtractVOI.h"

namespace soax {

ImageReader::ImageReader() {
  allowed_format_ << "tif" << "tiff" << "mhd" << "mha" << "png" << "jpg"
                  << "jpeg" << "bmp";
}

ImageReader::~ImageReader() {
  DeleteImages();
}

void ImageReader::ReadFile(const QString &filename) {
  vtkImageReader2 *reader = vtkImageReader2Factory::CreateImageReader2(
      filename.toStdString().c_str());

  reader->SetFileName(filename.toStdString().c_str());
  reader->Update();

  if (nslices_per_frame_ > 0) { // contain multiple frames
    LoadImages(reader->GetOutput());
  } else { // single frame
    vtkImageData *image = vtkImageData::New();
    image->ShallowCopy(reader->GetOutput());
    images_.push_back(image);
  }
  reader->Delete();
  path_ = QFileInfo(filename).absolutePath();
  paths_ << QFileInfo(filename).absoluteFilePath();
}

void ImageReader::ReadDir(const QString &directory) {
  QDir dir(directory);
  dir.setFilter(QDir::Files | QDir::NoDotAndDotDot);
  dir.setSorting(QDir::Name);
  QStringList filenames = SortFilenames(dir.entryList());

  QString name;
  foreach(name, filenames) {
    QString path = dir.absoluteFilePath(name);
    vtkImageReader2 *reader = vtkImageReader2Factory::CreateImageReader2(
        path.toStdString().c_str());
    reader->SetFileName(path.toStdString().c_str());
    reader->Update();
    vtkImageData *image = vtkImageData::New();
    image->ShallowCopy(reader->GetOutput());
    images_.push_back(image);
    reader->Delete();
    paths_ << path;
  }
  path_ = dir.absolutePath();
}

vtkImageData * ImageReader::GetImage(size_t index) const {
  if (images_.empty())
    return nullptr;
  return images_[index];
}

vtkImageData * ImageReader::GetLastImage() const {
  if (images_.empty())
    return nullptr;
  return images_.back();
}

int ImageReader::FindImage(vtkImageData *image) const {
  for (size_t i = 0; i < images_.size(); i++) {
    if (images_[i] == image)
      return static_cast<int>(i);
  }
  return -1;
}

QString ImageReader::GetFilePath(int index) const {
  if (paths_.empty())
    return "";
  return paths_[index];
}

QString ImageReader::GetFileNameWithoutSuffix(int index) const {
  QString result;
  return result;
}

QString ImageReader::GetAllowedFormatAsString() const {
  QString filter_str = "Image Files (";
  for (QSet<QString>::const_iterator it = allowed_format_.begin();
       it != allowed_format_.end(); it++) {
    filter_str.append(" *.");
    filter_str.append(*it);
  }
  filter_str += ")";
  return filter_str;
}

void ImageReader::Reset() {
  paths_.clear();
  DeleteImages();
  images_.clear();
  nslices_per_frame_ = 0;
}

void ImageReader::ReverseImageSequence() {
  std::reverse(images_.begin(), images_.end());
}

/******************* Private Methods *******************/

void ImageReader::DeleteImages() {
  for (unsigned i = 0; i < images_.size(); i++) {
    images_[i]->Delete();
  }
}

void ImageReader::LoadImages(vtkImageData *image) {
  int *dimensions = image->GetDimensions();
  if (nslices_per_frame_ > dimensions[2])
    nslices_per_frame_ = dimensions[2];
  for (int i = nslices_per_frame_; i <= dimensions[2];
       i += nslices_per_frame_) {
    vtkExtractVOI * extractor = vtkExtractVOI::New();
    extractor->SetInputData(image);
    extractor->SetVOI(0, dimensions[0] - 1,
                      0, dimensions[1] - 1,
                      i - nslices_per_frame_, i - 1);
    extractor->Update();
    vtkImageData *subimage = vtkImageData::New();
    subimage->ShallowCopy(extractor->GetOutput());
    int *extent = subimage->GetExtent();
    extent[4] = 0;
    extent[5] = nslices_per_frame_ - 1;
    subimage->SetExtent(extent);
    images_.push_back(subimage);
    extractor->Delete();
  }
}

QStringList ImageReader::SortFilenames(const QStringList &filenames)
    const {
  QMap<int, QString> map;
  QStringList no_index_names;
  QStringList collision_names;
  foreach(const QString &name, filenames) {
    if (allowed_format_.contains(QFileInfo(name).suffix().toLower())) {
      int index = ExtractIndex(name);
      if (index == -1) {
        no_index_names << name;
      } else {
        if (map.contains(index))
          collision_names << name;
        else
          map.insert(index, name);
      }
    }
  }
  return no_index_names << collision_names << map.values();
}

int ImageReader::ExtractIndex(const QString &filename) const {
  std::string s = filename.toStdString();

  int start = -1;
  int n = 0;
  bool start_found = false;

  for (int i = static_cast<int>(s.length() - 1); i >= 0; i--) {
    if (isdigit(s[i])) {
      if (!start_found) {
        start = i;
        start_found = true;
      }
      n++;
    } else {
      if (start_found)
        break;
    }
  }

  if (start_found)
    return std::stoi(s.substr(start - n + 1, n));
  else
    return -1;
}

}  // namespace soax
