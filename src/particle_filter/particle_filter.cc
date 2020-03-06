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
using Eigen::Rotation2Df;
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

// Vector2f ParticleFilter::GetLocation(Vector2f p0, Vector2f p1) {
//   //make a line from p0 to p1
//   for (auto line : map_.lines) {
//     //if line collides with our line
//     if (!Intersection(reference)) {
//       //find new loc
//     //else
//       //keep
//   }
// }
void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
  cout << "Observe prev: " << prev_odom_loc_ << "Observe current: " << odom_loc << endl;
  const float k = 0.4;
  if (!odom_initialized_) {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
    return;
  }
  if (odom_initialized_) {
    Rotation2Df rotation(-prev_odom_angle_);
    Vector2f delta_x = rotation * (odom_loc - prev_odom_loc_);
    float delta_x_magnitude = delta_x.norm();

    float std_dev = k * delta_x_magnitude;

    //i think we need to translate this theta to the map reference
    //check page 6 of slideset 11. We just need to update the theta.
    float delta_theta = odom_angle - prev_odom_angle_;
    cout << "delta theta: " << delta_theta << endl;
    //cout << std_dev << endl;
    //particles_.clear();
    for (Particle &p : particles_)
    {
      Rotation2Df rotation2(p.angle);
      Vector2f error(rng_.Gaussian(0, std_dev), rng_.Gaussian(0, std_dev));
      //cout << "ERROR: " << std_dev << endl;
      // p.loc = GetLocation(p.loc, p.loc + odom_loc);
      p.loc += rotation2 * (delta_x + error);                              //add noise here
      p.angle = p.angle + delta_theta + rng_.Gaussian(0, k * delta_theta); //add noise here
      // particles_.push_back(p);
    }
    if (particles_.size() > 0)
      cout << "Sample Particle: " << particles_[0].loc << particles_[0].angle << endl;
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
  }
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  cout << "init?" << endl;
  cout << loc << endl;
  cout << angle << endl;
  map_.Load("maps/GDC1.txt");
 
  particles_.clear();
  
  for (int i = 0; i < FLAGS_num_particles; i++)
  {
    Particle p;
    // Vector2f error(rng_.Gaussian(0, 0.07), rng_.Gaussian(0, 0.07));
    p.loc = loc;// + error; //add noise here
    p.angle = angle;//  +rng_.Gaussian(0, 0.07); //add noise here
    particles_.push_back(p);
  }
  odom_initialized_ = false;
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc, float* angle) const {
}


}  // namespace particle_filter
