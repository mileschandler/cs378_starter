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

#include <math.h>


using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using Eigen::Rotation2Df;
using vector_map::VectorMap;
using std::max;
using std::min;

DEFINE_double(num_particles, 50, "Number of particles");

// Fill in the body of these functions and create helpers as needed
// in order to implement localization using a particle filter.

// Milestone 2 will be implemented here.

namespace particle_filter {

vector<Vector2f> best_pred;

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
  // IM THINKING I CHANGE ALL OF THIS FOR CP6 -> map_.GetPredictedScan
	//TODO Based on the car location and angle we look for intersections in the map
	// x inc = rnage_maxcostheta
	// y inc = range_maxsintheta
  bool cp_6 = true;
  if (cp_6) {
    *scan_ptr = best_pred;
  } else {
    vector<Vector2f> our_pred;
    // cout << "angle: " << angle << endl;
    const float bound_angle = 2.35619;
    float min_angle = math_util::AngleMod(angle - bound_angle); // math_util::AngleDiff(angle, bound_angle);
    // float max_angle = math_util::AngleMod(angle + bound_angle); //math_util::AngleMod(angle + bound_angle);
    // cout << "angle_min: " << min_angle << endl;
    // cout << "angle_max: " << max_angle << endl;
    const float increment = 0.05;
    int num_viz = int((2 * bound_angle) / increment);
    float iter_angle = min_angle;
    cout << "num_viz: " << num_viz << endl;
    for (int i = 0; i < num_viz; i += 1) {
      iter_angle = math_util::AngleMod(iter_angle + increment);
      //cout << "iter_angle: " << iter_angle << endl;
      float x_inc = range_max * cos(iter_angle);
      float y_inc = range_max * sin(iter_angle);
      Vector2f point(loc.x() + x_inc, loc.y() + y_inc);
      Vector2f min_point(0.0, 0.0);
      for (auto line : map_.lines) {
        Vector2f inter(0.0, 0.0);
        if (line.Intersection(loc, point, &inter)) {
          //our_pred.push_back(inter);
          if ((inter - loc).norm() < (min_point - loc).norm()) {
            min_point = inter;
          }
        }
      }
      if (map_.Intersects(loc, min_point)) {
        our_pred.push_back(min_point);
      }
    }
    *scan_ptr = our_pred;
  }

}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  //ranges holds s values
  //calculate the weight here
  int num_rays = 1081;
  vector<float> scan_ptr;
  float gamma = 1.0 / (num_rays / 2);
  float sigma = 0.05;
  map_.GetPredictedScan(p_ptr->loc, range_min, range_max, angle_min, angle_max, num_rays, &scan_ptr);

  float weight = 0;
  for (int i = 0; i < num_rays; i+=10) {
    // cout << "diff: " << ranges[i] - scan_ptr[i] << endl;
    weight += (pow(ranges[i] - scan_ptr[i], 2) / pow(sigma, 2));
  }
  weight *= -gamma;
  // cout << "weight: " << weight << endl;
  p_ptr -> weight = weight;

}

float ParticleFilter::ConfigWeights () {
  float max_weight = 0;
  float weight_sum = 0;
  for (Particle p : particles_) {
    if (p.weight > max_weight) {
      max_weight = p.weight;
    }
  }

  for (Particle p : particles_) {
    // float w_prime = p.weight / max_weight;
    p.weight = log(p.weight) - log(max_weight);
    weight_sum += p.weight;
  }

  return weight_sum;
}

void ParticleFilter::Resample() {

  vector<Particle> new_particles;
  int sum_W = ConfigWeights(); // Let W = sum of all weights wi
  
  for (int i = 0; i < FLAGS_num_particles; i++) { // Repeat N times
    // Draw a random number between 0 and W
    float x = rand() % (sum_W + 1);
    float w_prime = 0; // w’ = 0
    for (Particle p: particles_) { // for each particle weight wi
      w_prime += p.weight;
      if (w_prime > x) {
          Particle r;
          r.loc = p.loc;// + error; // add noise here
          r.angle = p.angle;//  +rng_.Gaussian(0, 0.07); // add noise here
          //r weight we need to calculate
          // possible convert back to regular weight?
          new_particles.push_back(r);
      }
    }
  }

  particles_ = new_particles;

  /*
    for each particle weight wi
      w’ = w’ + wi
      if w’ > x:
        Replicate particle i
          Particle p;
          p.loc = loc;// + error; //add noise here
          p.angle = angle;//  +rng_.Gaussian(0, 0.07); //add noise here
          new_particles.push_back(p);
        break for
  */
  
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  /*
  CP6:
  Use only every 10th ray (108 rays total per scan)
  First implement the simplest model (pure Gaussian, with stddev~0.05m)
  Test in areas where the map is accurate (no unexpected observations)
  Tune gamma to prevent overconfidence
  idea for pseudocode:
  Iterate through each particle:
    grab the predicted lidarscan from vectormap's GetPredictedScan which returns ranges? from the point
    ^^ this means I also need the base truth of ranges for the true particle location
    log[(p(st|xt))] ~ -gamma summed from 1 to n over (si -shati)^2 / sigma^2 
      -- gamma has a range from 1/n (where n is the number of rays you are considering) to 1

  */
  for (Particle &p : particles_) {
    Update(ranges, range_min, range_max, angle_min, angle_max, &p);
  }

  // find the best particle
  if (particles_.size() > 0) {
    Particle max_p = particles_[0];

    for (Particle p : particles_) {
      if (p.weight > max_p.weight) {
        max_p = p;
      }
    }

    
    //
    best_pred.clear();
    // cout << "angle: " << max_p.angle << endl;
    const float bound_angle = 2.35619;
    float min_angle = math_util::AngleMod(max_p.angle - bound_angle); // math_util::AngleDiff(angle, bound_angle);
    // float max_angle = math_util::AngleMod(max_p.angle + bound_angle); //math_util::AngleMod(angle + bound_angle);
    // cout << "angle_min: " << min_angle << endl;
    // cout << "angle_max: " << max_angle << endl;
    const float increment = (angle_max - angle_min) / 1081;
    float iter_angle = min_angle;
    const float lidar_offset = 0.0;
    //change this loop
    for (unsigned int i = 0; i < ranges.size(); i += 1) {
      iter_angle = math_util::AngleMod(iter_angle + increment);
      if (ranges[i] < range_max) {
        //cout << "iter_angle: " << iter_angle << endl;
        float x_inc = ranges[i] * cos(iter_angle);
        float y_inc = ranges[i] * sin(iter_angle);
        Vector2f point(max_p.loc.x() + lidar_offset + x_inc, max_p.loc.y() + y_inc);
        best_pred.push_back(point);
      } 
    }
  }




}


void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
  // cout << "Observe prev: " << prev_odom_loc_ << "Observe current: " << odom_loc << endl;
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

    float delta_theta = math_util::AngleMod(odom_angle - prev_odom_angle_);
    // cout << "delta theta: " << delta_theta << endl;
   
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
      p.angle = math_util::AngleMod(p.angle + delta_theta + rng_.Gaussian(0, k * delta_theta)); //add noise here
      // if the particle intersects with the map set it to the average of the points that dont intersect
      if (map_.Intersects(pcopy, p.loc + (rotation2 * car_length)) && count != 0) {
        p.loc = avg_loc / count;
        p.angle = avg_theta / count;
      }

      avg_loc += p.loc;
      avg_theta += p.angle;
      ++count;
    }
    if (particles_.size() > 0) {
      //cout << "Sample Particle: " << particles_[0].loc << particles_[0].angle << endl
    	prev_odom_loc_ = odom_loc;
   	prev_odom_angle_ = odom_angle;
    }
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
	//TODO here we just need to compute the mean location, angle and then set
	// float x = 0.0;
	// float y = 0.0;
	// float ang = 0.0;
	// int count = 0;


	// for (Particle p : particles_) {
  //   // cout << "curious???????? " << p.weight << endl;
	// 	x += p.loc.x();
	// 	y += p.loc.y();
	// 	ang += p.angle;
	// 	count += 1;
	// }
	// x /= count;
	// y /= count;
	// Vector2f avg(x, y);
	// *loc = avg;
	// *angle = ang / count;

  if (particles_.size() > 0) {
    Particle max_p = particles_[0];

    for (Particle p : particles_) {
      if (p.weight > max_p.weight) {
        max_p = p;
      }
    }
    *loc = max_p.loc;
    *angle = max_p.angle;

  }
}

}  // namespace particle_filter
