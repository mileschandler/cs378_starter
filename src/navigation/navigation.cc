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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "f1tenth_course/AckermannCurvatureDriveMsg.h"
#include "f1tenth_course/Pose2Df.h"
#include "f1tenth_course/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using f1tenth_course::AckermannCurvatureDriveMsg;
using f1tenth_course::VisualizationMsg;
using std::string;
using std::vector;
using std::cout;
using std::endl;
using std::sqrt;
using std::pow;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    robot_dist_traveled_(0.0),
    robot_decc_(0.0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {

}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
   
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
    
    if (robot_loc_ == Vector2f(0,0)){
        robot_loc_ = loc;
    }
    robot_dist_traveled_ += sqrt(pow(loc.x() - robot_loc_.x(), 2) + pow(loc.y() - robot_loc_.y(), 2));
    robot_vel_ = vel;
    robot_angle_ = angle;
    robot_loc_ = loc;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
}

void Navigation::Run(float delta_x) {
    const float max_vel = 1.0;
    const float max_acc = 3.0;
    const float max_decc = 3.0;
    const float min_decc_distance = pow(max_vel, 2) / ( 2 * max_decc);
    float vel = robot_vel_.x();//sqrt(pow(robot_vel_.x(), 2) + pow(robot_vel_.y(), 2));


    float time_step = 1.0/20;
    float future_vel = vel + max_acc * time_step;//vt = v0 + aT
    float dist_acc_step = (pow(future_vel, 2) - pow(robot_vel_.x(), 2)) / (2 * max_acc);// v^2t = v^20 + 2as -> (v^2t - v^20) / 2a = s
    float poss_acc_distance = robot_dist_traveled_ + dist_acc_step;
    bool can_accelerate = delta_x - (poss_acc_distance + min_decc_distance) >= 0;

    bool at_max_speed = vel >= max_vel;

    float dist_cruise_step =  max_vel * time_step;
    float poss_cruise_dist = robot_dist_traveled_ + dist_cruise_step; // xt = x0 + voT + (1/2)aT^2
    bool cruise = delta_x - (poss_cruise_dist + min_decc_distance) >= 0;
      // Accelerate if not at max speed and there is distance left
    if (can_accelerate && !at_max_speed) {
        //accelerate
        cout << "accelerate" << endl;
        vel += max_acc * (time_step);
    } else if (cruise) { // If car travels at max speed for one time-step, and then decelerates to a stop, there should be >= 0 distance left after that
        //cruise
        cout << "cruise" << endl;
        vel = max_vel;
    } else {
        //slow down, mayday
        cout << "mayday" << endl;
        float rem_dist = delta_x - robot_dist_traveled_;
        if (robot_decc_ == 0.0) {
            robot_decc_ = vel / (2.0 * rem_dist);
        }

        vel -= robot_decc_ * (time_step);
        if (vel <= 0) {
            cout << "vel < 0" << endl;
            vel = 0;
        }
    }
    // Cruise if at max speed, and there is distance left
    // Decelerate if not enough distance left
    cout << "x_vel: " << robot_vel_.x() << endl;
    cout << "vel: " << vel << " dist: " << robot_dist_traveled_ << endl;

    AckermannCurvatureDriveMsg msg;
    msg.velocity = vel;
    msg.curvature = 0;
    drive_pub_.publish(msg);
    
// Milestone 3 will complete the rest of navigation.


}

}  // namespace navigation
