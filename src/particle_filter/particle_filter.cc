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
float distance_traveled;

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
  float gamma = -1.0 / (num_rays / 3);
  float sigma = 0.15;
  map_.GetPredictedScan(p_ptr->loc, range_min, range_max, angle_min, angle_max, num_rays, &scan_ptr);
  float smin = 0.5;
  float smax = 15;
  float dshort = 0.03;
  float dlong = 0.15;

  float weight = 0;
  for (int i = 0; i < num_rays; i+=10) {
    // cout << "diff: " << ranges[i] - scan_ptr[i] << endl;
    float s_i = ranges[i];  //true
    float s_hat = scan_ptr[i]; //predicted
    if (ranges[i] >= smin && ranges[i] <= smax) {
      // weight += (pow(ranges[i] - scan_ptr[i], 2) / pow(sigma, 2));
    //change to robust
      if (s_i < (s_hat - dshort)) {
        weight += (pow(dshort, 2) / pow(sigma, 2));
      } else if (s_i > (s_hat + dlong)) {
        weight += (pow(dlong, 2) / pow(sigma, 2));
      } else {
        weight += (pow(s_i - s_hat, 2) / pow(sigma, 2));
      }
    }
  }
  weight *= gamma;
  // cout << "weight: " << weight << endl;
  p_ptr -> weight = weight;

}

float ParticleFilter::ConfigWeights () {
  float max_weight = INT_MIN;
  float weight_sum = 0;
  for (Particle p : particles_) {
    if (p.weight > max_weight) {
      max_weight = p.weight;
    }
  }

  for (Particle p : particles_) {
    // float w_prime = p.weight / max_weight;
    cout << p.weight << endl;
    p.weight = p.weight - max_weight;
    cout << p.weight << endl;
    weight_sum += p.weight;
  }

  return weight_sum;
}

void ParticleFilter::Resample() {

  vector<Particle> new_particles;
  float sum_W = ConfigWeights(); // Let W = sum of all weights wi
  // cout <<"sumW: " << sum_W << endl;
  int count = 0;
  
  for (int i = 0; i < FLAGS_num_particles; i++) { // Repeat N times
    // cout << "hi" << endl;
    // Draw a random number between 0 and W
    float x = abs(float(rand()) / float((RAND_MAX)) * sum_W);
    // cout << "x: " << x << endl;
    float w_prime = 0; // w’ = 0
    for (Particle p: particles_) { // for each particle weight wi
      w_prime += abs(p.weight);
      // cout <<"w_prime: " << w_prime << endl;
      if (w_prime > x) {
          Particle r;
          r.loc = p.loc;// + error; // add noise here
          r.angle = p.angle;//  +rng_.Gaussian(0, 0.07); // add noise here
          //r weight we need to calculate
          // possible convert back to regular weight?
          new_particles.push_back(r);
          break;
      }
    }
    count += 1;
  }
  cout << particles_.size() << endl;
  if (new_particles.size() != 0) {
    particles_ = new_particles;
  } else {
    cout << "newp empty" << endl;
  }

  cout << particles_.size() << endl;

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
  // cout << "angle_min: " << angle_min << " angle_max: " << angle_max << endl;
  for (Particle &p : particles_) {
    // float my_angle_min = (p.angle + angle_min);
    // float my_angle_max = (p.angle + angle_max);
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
    cout << "angle: " << max_p.angle << endl;
    // const float bound_angle = 2.35619;
    float min_angle = math_util::AngleMod(max_p.angle + angle_min); // math_util::AngleDiff(angle, bound_angle);
    // float max_angle = math_util::AngleMod(max_p.angle + angle_max); //math_util::AngleMod(angle + bound_angle);
    cout << "angle_min: " << angle_min << endl;
    cout << "angle_max: " << angle_max << endl;

    //This loop creates a vector for the point cloud so that we can publish the "best" point's lidar scan 
    const float increment = (angle_max - angle_min) / ranges.size();
    float iter_angle = min_angle; //math_util::AngleMod(min_angle + max_p.angle);

    Rotation2Df lidar_rot(max_p.angle);
    // cout << "maxp_angle: " << max_p.angle << endl;
    // Vector2f lidar_base(0.20, 0);
    // Vector2f lidar_offset = lidar_rot * lidar_base;
    float lidar_offset = 0.2;
    //change this loop
    for (unsigned int i = 0; i < ranges.size(); i += 1) {
      if (ranges[i] < range_max) {
        //cout << "iter_angle: " << iter_angle << endl;
        Rotation2Df rotato(iter_angle);
        float x_inc = (ranges[i] + lidar_offset) * cos(iter_angle);
        cout << "x_inc: " << x_inc << endl;
        float y_inc = ranges[i] * sin(iter_angle);
        Vector2f point(max_p.loc.x() + x_inc, max_p.loc.y() + y_inc);
        // Vector2f real_point = point + lidar_offset;
        // Vector2f real_real_point = rotato * real_point;
        best_pred.push_back(point);
      } 
      iter_angle = math_util::AngleMod(iter_angle + increment);
    }
    // cout << "calling resample" << endl;
    if (distance_traveled >= 0.5) {
      cout << "resample" << endl;
      // Resample();
      distance_traveled = 0.0;
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
    distance_traveled += delta_x_magnitude;

    float std_dev = k * delta_x_magnitude;
    // cout << "odom_angle: " << odom_angle << endl;
    // cout << "prev_odom_angle: " << prev_odom_angle_ << endl;
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
      float test_error = 0; //(0.1 * k * delta_x_magnitude);
      p.angle = math_util::AngleMod(p.angle + delta_theta + rng_.Gaussian(0, test_error + (k * delta_theta))); //add noise here
      // if the particle intersects with the map set it to the average of the points that dont intersect
      if (map_.Intersects(pcopy, p.loc + (rotation2 * car_length)) && count != 0) {
        p.loc = avg_loc / count;
        p.angle = avg_theta / count;
        // p.weight = INT_MIN;
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
    // Vector2f error(rng_.Gaussian(0, 0.03), rng_.Gaussian(0, 0.03));
    p.loc = loc; // + error; //add noise here
    p.angle = angle; // + rng_.Gaussian(0, 0.03); //add noise here
    particles_.push_back(p);
  }
  odom_initialized_ = false;
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc, float* angle) const {
	//TODO here we just need to compute the mean location, angle and then set
	float x = 0.0;
	float y = 0.0;
	float ang = 0.0;
	int count = 0;


	for (Particle p : particles_) {
    // cout << "curious???????? " << p.weight << endl;
		x += p.loc.x();
		y += p.loc.y();
		ang += p.angle;
		count += 1;
	}
	x /= count;
	y /= count;
	Vector2f avg(x, y);
	*loc = avg;
	*angle = ang / count;

  // if (particles_.size() > 0) {
  //   Particle max_p = particles_[0];
  //   // cout << "beginning" << endl;
  //   for (Particle p : particles_) {
  //     // cout << p.weight << endl;
  //     if (p.weight > max_p.weight) {
  //       max_p = p;
  //     }
  //   }
  //   // cout << "end" << endl;
  //   *loc = max_p.loc;
  //   *angle = max_p.angle;

  // }
}

}  // namespace particle_filter
