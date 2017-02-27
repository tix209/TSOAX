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
 */

#ifndef snake_track_h_
#define snake_track_h_

#include <vector>
#include <initializer_list>
#include <iostream>

namespace soax {
class Snake;

class SnakeTrack {
 public:
  SnakeTrack();
  SnakeTrack(std::size_t nframes);

  std::size_t GetTrackLength() const;

  double GetTotalCurveLength() const;

  bool operator<(const SnakeTrack &other) const;

  bool operator==(const SnakeTrack &other) const;

  friend std::ostream& operator<<(std::ostream &os, const SnakeTrack &t);

  bool Contains(Snake *s) const;

  Snake * GetSnake(size_t frame) const;
  void SetSnake(size_t frame, Snake *s);

  void Append(Snake *s);

  /**
   * Returns the starting frame number. Return -1 for empty track.
   */
  int GetFirstFrame() const;

  Snake * GetFirstSnake() const;

 private:
  std::vector<Snake *> track_;
};

std::ostream& operator<<(std::ostream &os, const SnakeTrack &t);

}  // namespace soax

#endif  // snake_track_h_
