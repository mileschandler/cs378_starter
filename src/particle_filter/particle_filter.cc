//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"


using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");

// Fill in the body of these functions and create helpers as needed
// in order to implement localization using a particle filter.

// Milestone 2 will be implemented here.

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {

}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
}

void ParticleFilter::Resample() {
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
}

void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {

  //cout << "Observe : " << endl;
  const float k = 0.1;
  float delta_x = (odom_loc - prev_odom_loc_).norm();
  float std_dev = k * delta_x;
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
  //particles_.clear();
  for (Particle p : particles_)
  {
    Vector2f error(rng_.Gaussian(0, std_dev), rng_.Gaussian(0, std_dev));
    //cout << "ERROR: " << std_dev << endl;
    p.loc += (odom_loc - prev_odom_loc_) + error;     //add noise here
    p.angle = odom_angle + rng_.Gaussian(0, std_dev); //add noise here
    // particles_.push_back(p);
  }

}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  cout << "init?" << endl;
  cout << loc << endl;
  cout << angle << endl;
  // std::vector<Particle> init_particles_;
  // Particle p;
  prev_odom_angle_ = angle;
  prev_odom_loc_ = loc;
  for (int i = 0; i < FLAGS_num_particles; i++)
  {
    Particle p;
    Vector2f error(rng_.Gaussian(0, 1), rng_.Gaussian(0, 1));
    p.loc = loc + error; //add noise here
    p.angle = angle + rng_.Gaussian(0, 1); //add noise here
    particles_.push_back(p);
  }
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc, float* angle) const {
}


}  // namespace particle_filter
