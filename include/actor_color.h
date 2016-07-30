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
 * This class defines the color of an vtkActor.
 */

#ifndef ACTOR_COLOR_H_
#define ACTOR_COLOR_H_

namespace soax {

class ActorColor {
 public:
  static double kWhite[3];
  static double kGray[3];
  static double kRed[3];
  static double kMagenta[3];
  static double kYellow[3];
  static double kGreen[3];
  static double kCyan[3];
  static double kBlue[3];
};

}  // namespace soax

#endif  // ACTOR_COLOR_H_
