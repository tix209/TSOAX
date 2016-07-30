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
 * This class is the visualization viewpoint of the SOAX program.
 */


#ifndef VIEWPOINT_H_
#define VIEWPOINT_H_

#include <string>
#include <iostream>
#include <QString>  // NOLINT(build/include_order)
#include "vtkCamera.h"

namespace soax {

class Viewpoint {
 public:
  Viewpoint();

  ~Viewpoint();

  /**
   * Returns the path of the saved viewpoint file.
   */
  QString path() const {return path_;}

  vtkCamera *camera() const {return camera_;}

  std::string default_viewpoint_str() const {
    return default_viewpoint_str_;
  }

  std::string ToString() const;

  /**
   * Save/Load camera parameters to file.
   */
  void SaveToFile(const QString &filename);
  void LoadFromFile(const QString &filename);

  void SaveDefault();
  void LoadDefault();

 private:
  void Load(std::istream &is);  // NOLINT(runtime/references)

  QString path_;
  vtkCamera *camera_;
  // cache for default camera values
  std::string default_viewpoint_str_;
};

std::ostream & operator<<(std::ostream &os, const Viewpoint &vp);

}  // namespace soax

#endif  // VIEWPOINT_H_
