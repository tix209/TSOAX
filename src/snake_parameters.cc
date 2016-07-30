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
 *
 * This file implements the assemble of all parameters for a SOAC.
 */

#include "./snake_parameters.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include "./util.h"

namespace soax {

SnakeParameters::SnakeParameters() {
  init_directions_[0] = true;
  init_directions_[1] = true;
  init_directions_[2] = true;
}

std::string SnakeParameters::ToString() const {
  std::ostringstream os;
  os << std::boolalpha;
  os << "intensity-scaling\t" << intensity_scaling_ << std::endl;
  os << "gaussian-std\t" << sigma_ << std::endl;
  os << "ridge-threshold\t" << ridge_threshold_ << std::endl;
  os << "maximum-foreground\t" << maximum_foreground_ << std::endl;
  os << "minimum-foreground\t" << minimum_foreground_ << std::endl;
  os << "init-z\t" << init_directions_[2] << std::endl;
  os << "snake-point-spacing\t" << spacing_ << std::endl;
  os << "minimum-snake-length\t" << minimum_length_ << std::endl;
  os << "maximum-iterations\t" << maximum_iterations_ << std::endl;
  os << "change-threshold\t" << change_threshold_ << std::endl;
  os << "check-period\t" << check_period_ << std::endl;
  os << "alpha\t" << alpha_ << std::endl;
  os << "beta\t" << beta_ << std::endl;
  os << "gamma\t" << gamma_ << std::endl;
  os << "external-factor\t" << external_factor_ << std::endl;
  os << "stretch-factor\t" << stretch_factor_ << std::endl;
  os << "number-of-background-radial-sectors\t" << number_of_sectors_
     << std::endl;
  os << "background-z-xy-ratio\t" << zspacing_ << std::endl;
  os << "radial-near\t" << radial_near_ << std::endl;
  os << "radial-far\t" << radial_far_ << std::endl;
  os << "delta\t" << delta_ << std::endl;
  os << "overlap-threshold\t" << overlap_threshold_ << std::endl;
  os << "grouping-distance-threshold\t" << grouping_distance_threshold_
     << std::endl;
  os << "grouping-delta\t" << grouping_delta_ << std::endl;
  os << "minimum-angle-for-soac-linking\t" << direction_threshold_
     << std::endl;
  os << "damp-z\t" << damp_z_ << std::endl;
  os << "association-threshold\t" << association_threshold_ << std::endl;
  os << std::noboolalpha;
  return os.str();
}

bool SnakeParameters::Valid() const {  // todo: complete parameter constraints
  if (maximum_foreground_ < minimum_foreground_) {
    std::cerr << "Minimum foreground is greater than maximum foreground!"
              << std::endl;
    return false;
  }

  if (spacing_ < kEpsilon) {
    std::cerr << "Spacing is too small!" << std::endl;
    return false;
  }

  if (change_threshold_ < kEpsilon) {
    std::cerr << "Change threshold is too small!" << std::endl;
    return false;
  }

  if (minimum_length_ < kEpsilon) {
    std::cerr << "Minimum length is too small!" << std::endl;
    return false;
  }

  if (alpha_ < 0.0) {
    std::cerr << "Alpha cannot be negative!" << std::endl;
    return false;
  }

  if (beta_ < 0.0) {
    std::cerr << "Beta cannot be negative!" << std::endl;
    return false;
  }

  if (gamma_ < 0.0) {
    std::cerr << "Gamma cannot be negative" << std::endl;
    return false;
  }

  if (radial_near_ >= radial_far_) {
    std::cerr << "Radial far must be greater than radial near!"
              << std::endl;
    return false;
  }

  if (grouping_distance_threshold_ < 0.0) {
    std::cerr << "Grouping distance threshold cannot be negative!"
              << std::endl;
    return false;
  }

  if (grouping_delta_ <= 0) {
    std::cerr << "Grouping delta must be positive!" << std::endl;
    return false;
  }

  if (direction_threshold_ <= 0.0) {
    std::cerr << "Minimum angle for SOAC linking must be positive!"
              << std::endl;
    return false;
  }

  return true;
}

bool SnakeParameters::Load(const QString &filename) {
  std::ifstream infile(filename.toStdString().c_str());
  if (!infile.is_open()) {
    std::cerr << "Couldn't open parameter file: "
              << filename.toStdString() << std::endl;
    return false;
  }

  std::string line;
  while (std::getline(infile, line)) {
    std::stringstream converter;
    converter << line;
    std::string name, value;
    converter >> name >> value;
    AssignParameters(name, value);
  }
  path_ = filename;
  return true;
}

bool SnakeParameters::Save(const QString &filename) {
  std::ofstream outfile;
  outfile.open(filename.toStdString().c_str());
  if (!outfile.is_open()) {
    std::cerr << "Couldn't open file:  " << filename.toStdString()
              << std::endl;
    return false;
  }
  outfile << this->ToString();
  outfile.close();
  path_ = filename;
  return true;
}

void SnakeParameters::AssignParameters(const std::string &name,
                                       const std::string &value) {
  if (name == "intensity-scaling") {
    intensity_scaling_ = std::stod(value);
  } else if (name == "smoothing" || name == "gaussian-std") {
    sigma_ = std::stod(value);
  } else if (name == "grad-diff" || name == "ridge-threshold") {
    ridge_threshold_ = std::stod(value);
  } else if (name == "foreground" || name == "maximum-foreground") {
    maximum_foreground_ = std::stoi(value);
  } else if (name == "background" || name == "minimum-foreground") {
    minimum_foreground_ = std::stoi(value);
  } else if (name == "spacing" || name == "snake-point-spacing") {
    spacing_ = std::stod(value);
  } else if (name == "init-z") {
    init_directions_[2] = (value == "true");
  } else if (name == "minimum-size" || name == "minimum-snake-length") {
    minimum_length_ = std::stod(value);
  } else if (name == "max-iterations" || name == "maximum-iterations") {
    maximum_iterations_ = std::stoi(value);
  } else if (name == "change-threshold") {
    change_threshold_ = std::stod(value);
  } else if (name == "check-period") {
    check_period_ = std::stoi(value);
  } else if (name == "alpha") {
    alpha_ = std::stod(value);
  } else if (name == "beta") {
    beta_ = std::stod(value);
  } else if (name == "gamma") {
    gamma_ = std::stod(value);
  } else if (name == "weight" || name == "external-factor") {
    external_factor_ = std::stod(value);
  } else if (name == "stretch" || name == "stretch-factor") {
    stretch_factor_ = std::stod(value);
  } else if (name == "nsector" ||
             name == "number-of-background-radial-sectors") {
    number_of_sectors_ = std::stoi(value);
  } else if (name == "radial-near") {
    radial_near_ = std::stoi(value);
  } else if (name == "radial-far") {
    radial_far_ = std::stoi(value);
  } else if (name == "background-z-xy-ratio") {
    zspacing_ = std::stod(value);
  } else if (name == "delta") {
    delta_ = std::stoi(value);
  } else if (name == "overlap-threshold") {
    overlap_threshold_ = std::stod(value);
  } else if (name == "grouping-distance-threshold") {
    grouping_distance_threshold_ = std::stod(value);
  } else if (name == "grouping-delta") {
    grouping_delta_ = std::stoi(value);
  } else if (name == "direction-threshold" ||
             name == "minimum-angle-for-soac-linking") {
    direction_threshold_ = std::stod(value);
  } else if (name == "damp-z") {
    damp_z_ = (value == "true");
  } else if (name == "association-threshold") {
    association_threshold_ = std::stod(value);
  }
}

int SnakeParameters::GetSnakeId() {
  snake_id_counter_++;
  return snake_id_counter_;
}

}  // namespace soax
