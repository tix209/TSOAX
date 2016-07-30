/**
 * Copyright (C) 2015 Lehigh University.
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
 */

#include "./snake_track.h"
#include <iostream>
#include "./snake.h"

namespace soax {

SnakeTrack::SnakeTrack() {}

SnakeTrack::SnakeTrack(std::size_t nframes) : track_(nframes, nullptr) {}

std::size_t SnakeTrack::GetTrackLength() const {
  unsigned cnt = 0;
  for (auto s : track_) {
    if (s) ++cnt;
  }
  return cnt;
}

double SnakeTrack::GetTotalCurveLength() const {
  double length = 0.0;
  for (auto s : track_)
    if (s)
      length += s->GetLength();

  return length;
}

bool SnakeTrack::operator < (const SnakeTrack &other) const {
  return GetTotalCurveLength() > other.GetTotalCurveLength();
}

bool SnakeTrack::operator == (const SnakeTrack &other) const {
  return track_ == other.track_;
}

bool SnakeTrack::Contains(Snake *snake) const {
  for (auto s : track_) {
    if (s == snake)
      return true;
  }
  return false;
}

Snake * SnakeTrack::GetSnake(size_t frame) const {
  return track_[frame];
}

void SnakeTrack::SetSnake(size_t frame, Snake *s) {
  if (frame < track_.size()) {
    track_[frame] = s;
  } else {
    std::cerr << "SnakeTrack::SetSnakeId: invalid frame number!"
              << std::endl;
  }
}

void SnakeTrack::Append(Snake *s) {
  track_.push_back(s);
}

int SnakeTrack::GetFirstFrame() const {
  int frame = -1;
  for (size_t i = 0; i < track_.size(); ++i) {
    if (track_[i]) {
      frame = static_cast<int>(i);
      break;
    }
  }
  return frame;
}

Snake * SnakeTrack::GetFirstSnake() const {
  for (size_t i = 0; i < track_.size(); ++i) {
    if (track_[i]) {
      return track_[i];
    }
  }
  return nullptr;
}

std::ostream& operator<<(std::ostream &os, const SnakeTrack &t) {
  for (auto s : t.track_) {
    if (s)
      os << s->id() << " ";
    else
      os << 0 << " ";
  }
  return os;
}

}  // namespace soax
