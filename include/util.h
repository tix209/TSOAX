/**
 * Copyright (C) 2017 Lehigh University.
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
 * This file defines common constants and functions.
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <limits>
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>  // NOLINT(build/include_order)
#include <deque>


namespace soax {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::RowVectorXd;

class Snake;
typedef std::vector<Snake *> SnakeContainer;
typedef VectorXd PointType;
typedef std::vector<PointType> PointContainer;
typedef MatrixXd::RowXpr RowVec;
typedef MatrixXd::ConstRowXpr ConstRowVec;

typedef std::deque<std::pair<int, int>> IndexPairContainer;

// Constants
const double kPi = 3.14159265358979323846264338;
const double kEpsilon = std::numeric_limits<double>::epsilon();
const double kPlusInfinity = std::numeric_limits<double>::max();
const double kMinusInfinity = std::numeric_limits<double>::min();

const int kMinimumEvolvingSize = 5;

// Utility functions
double Mean(const std::vector<double> &vec);
double Maximum(const std::vector<double> &vec);
bool AllFalse(const std::vector<bool> &vec);

template<typename T>
void PrintVector(const std::vector<T> &vec, std::ostream &os) {
  for (auto x : vec) {
    os << x << ", ";
  }
  os << std::endl;
}

}  // namespace soax

#endif  // UTIL_H_
