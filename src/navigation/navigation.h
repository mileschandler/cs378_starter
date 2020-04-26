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
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"
#include "vector_map/vector_map.h"
#include <unordered_map>
#include "shared/math/line2d.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class Navigation {
 public:

// this is a hash function for the Vector2f. Here is the source we found this from.
//https://wjngkoh.wordpress.com/2015/03/04/c-hash-function-for-eigen-matrix-and-vector/
 template<typename T>
  struct matrix_hash : std::unary_function<T, size_t> {
    std::size_t operator()(T const& matrix) const {
      // Note that it is oblivious to the storage order of Eigen matrix (column- or
      // row-major). It will give you the same hash value for two different matrices if they
      // are the transpose of each other in different storage order.
      int seed = 0;
      for (int i = 0; i < (int) matrix.size(); ++i) {
        auto elem = *(matrix.data() + i);
        seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      }
      return seed;
    }
  };

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);

  float FutureVelocity();

  std::vector<float> GetScores(float delta_x);

  float GetClearance(float delta_x, float theta);
  
  std::pair<float, float> GetBestPath(float old_delta, Eigen::Vector2f& carrot);

  // Gets the velocity to publish to the driver for the next time step
  float GetVelocity(float delta_x);

  // Returns the free distance left based on current path and lidar obstacles
  std::pair<float, float> UpdateFreeDistance(float curvature);

  // Main function called continously from main
  void Run(float delta_x, float theta);
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

  void PlanPath(std::unordered_map<Eigen::Vector2f, Eigen::Vector2f, matrix_hash<Eigen::Vector2f>>& came_from);

  std::vector<Eigen::Vector2f> FindNeighbors (Eigen::Vector2f& loc);

  double FindPathWeight(Eigen::Vector2f& current, Eigen::Vector2f& next);

  double heuristic (Eigen::Vector2f& loc, Eigen::Vector2f& goal);

  Eigen::Vector2f GetCarrot();
  float GetDistanceRemaining(float phi, float curvature, Eigen::Vector2f& carrot);

 private:

  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;

  //tracks the distance travelled
  float robot_dist_traveled_;

  //computed deceleration needed to stop
  float robot_decc_;

  // the distance to travel freely
  float robot_free_dist_;

  // -1 for none, 0 for accelerating, 1 for cruising, 2 for deccelerating
  int robot_prev_state;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;

  vector_map::VectorMap map_;

  bool path_set;

  Eigen::Vector2f nav_goal_loc_print_;

  std::vector<geometry::line2f> path_lines;
};

}  // namespace navigation

#endif  // NAVIGATION_H
