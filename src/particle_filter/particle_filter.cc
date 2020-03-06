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


void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
  cout << "Observe prev: " << prev_odom_loc_ << "Observe current: " << odom_loc << endl;
  const float k = 0.3;
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

    float delta_theta = odom_angle - prev_odom_angle_;
    cout << "delta theta: " << delta_theta << endl;
   
    Vector2f avg_loc(0, 0);
    float avg_theta = 0;
    int count = 0;
    Vector2f car_length(0.42, 0);

    for (Particle &p : particles_)
    {
      Rotation2Df rotation2(p.angle);
      Vector2f error(rng_.Gaussian(0, std_dev / 2 ), rng_.Gaussian(0, std_dev));
      Vector2f pcopy(p.loc.x(), p.loc.y());
      p.loc += rotation2 * (delta_x + error);
      p.angle = p.angle + delta_theta + rng_.Gaussian(0, k * delta_theta); //add noise here
      if (map_.Intersects(pcopy, p.loc + (rotation2 * car_length)) && count != 0) {
        p.loc = avg_loc / count;
        p.angle = avg_theta / count;
      }

      avg_loc += p.loc;
      avg_theta += p.angle;
      ++count;
    }
    
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
    p.loc = loc;// + error; //add noise here
    p.angle = angle;//  +rng_.Gaussian(0, 0.07); //add noise here
    particles_.push_back(p);
  }
  odom_initialized_ = false;
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc, float* angle) const {
}


}  // namespace particle_filter
