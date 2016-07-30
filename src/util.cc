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
 * This file implements common functions.
 */


#include "./util.h"
#include <numeric>


namespace soax {

double Mean(const std::vector<double> &vec) {
  return std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();
}

double Maximum(const std::vector<double> &vec) {
  return *std::max_element(vec.begin(), vec.end());
}

bool AllFalse(const std::vector<bool> &vec) {
  for (size_t i = 0; i < vec.size(); i++) {
    if (vec[i]) return false;
  }
  return true;
}

}  // namespace soax
