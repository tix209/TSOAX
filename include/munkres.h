/*
 *   Copyright (c) 2007 John Weaver
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
 
// THIS FILE WILL EVENTUALLY BE REMOVED AND REPLACED WITH MORE EFFICIENT lapjv SOLVER

#if !defined(_MUNKRES_H_)
#define _MUNKRES_H_

#include "include/matrix.h"

#include <list>
#include <utility>


class Munkres {
public:
  void solve(Matrix<double> &m);
private:
  static constexpr int NORMAL = 0;
  static constexpr int STAR   = 1;
  static constexpr int PRIME  = 2;
  inline bool find_uncovered_in_matrix(const double, size_t&, size_t&) const;
  inline bool pair_in_list(const std::pair<size_t,size_t> &, const std::list<std::pair<size_t,size_t> > &);
  int step1();
  int step2();
  int step3();
  int step4();
  int step5();
  int step6();

  Matrix<int> mask_matrix;
  Matrix<double> matrix;
  bool *row_mask;
  bool *col_mask;
  size_t saverow, savecol;
};

#endif /* !defined(_MUNKRES_H_) */
