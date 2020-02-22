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
#include <utility>  
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Rotation2Df;
using Eigen::Vector2f;
using f1tenth_course::AckermannCurvatureDriveMsg;
using f1tenth_course::VisualizationMsg;
using std::abs;
using std::cout;
using std::endl;
using std::max;
using std::min;
using std::pow;
using std::sqrt;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;
using namespace visualization;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
const float max_vel = 1.0;
const float max_acc = 3.0;
const float max_decc = 3.0;
const float time_step = (1.0 / 20);
bool first_odom = true;
const float margin = 0.06;
const float car_half_width = 0.1405;
const float w = car_half_width + margin;
const float base_to_tip = .42;
const float h = base_to_tip + margin;
const float curve_epsilon = 1e-3;
const float free_dist_cutoff = 0.01;
const float curve_delta = 0.25;
const float c_max = 0.1;
const float w1 = 0.05;
const float w2 = -0.05;

std::vector<Eigen::Vector2f> point_cloud;

const float latency = time_step * 6; // this is approximate, could actually be closer to 0.15

} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    robot_dist_traveled_(0.0),
    robot_decc_(0.0),
    robot_free_dist_(0),
    robot_prev_state(-1),
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

    // robot_dist_traveled_ += vel.x() * time_step;
    // need to use the location to be updating our distance
    
    if (first_odom) {
        robot_loc_ = loc;
        robot_angle_ = angle;
        first_odom = false;
    }
    //cout << "angle " << (angle - robot_angle_) << endl;
    //checking
    // Rotation2Df delta_theta(angle - robot_angle_);
    Rotation2Df delta_theta(-robot_angle_);
    // Vector2f delta_loc = (delta_theta * loc) - robot_loc_;
    Vector2f delta_loc = delta_theta * (loc - robot_loc_);
    // Vector2f delta_loc = loc - robot_loc_;

    robot_dist_traveled_ += sqrt(Sq(delta_loc.x()) + Sq(delta_loc.y()));
    robot_loc_ = loc;
    robot_angle_ = angle;
    //cout << "delta: " << delta_loc << endl;
}

//calculates distance from car to obstacle
float GetFreeDistance(Vector2f& point, float curvature) {
    //cout << "curve   :" <<  curvature << endl;
    if (abs(curvature) < curve_epsilon && abs(curvature) >= 0) {
            //practically 0 curvature
            //cout << "FOOOO" << endl;
            return point.x() - h;
    } else {
        // there is curvature to be accounted for
        // cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
        float r = 1.0 / abs(curvature);
        float theta = atan2(point.x(), r - point.y());
        //cout << "theta " << theta << " curve " << curvature << endl;
        float omega = atan2(h, r - w);
        float phi = theta - omega;
        //cout << "R*PHI >> " << r * phi << endl;
        return r * phi;
    }

}

//determines if a point is in the car's immediate path trajectory
bool isObstacle(Vector2f& point, float curvature) {
    if (curvature < 0 && point.y() > 0 )
        return false;


    curvature = abs(curvature);
    if (curvature < curve_epsilon && curvature >= 0)
    {
        //practically 0 curvature
        // cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
        return abs(point.y()) <= w;
    } else {
       
        // there is curvature
        float r = 1.0 / curvature;
        float r1 = r - w;
        float r2 = sqrt(Sq(r + w) + Sq(h));
        //signs
        //r1 = r > 0 ? r1 : -1.0 * r1;
        //r2 = r > 0 ? r2 : -1.0 * r2;
        Vector2f c(0, r);
        float theta = atan2(point.x(), r - point.y());
        Vector2f pc_diff = point - c;
        float norm_pc = sqrt(Sq(pc_diff.x()) + Sq(pc_diff.y()));

        return norm_pc >= r1 && norm_pc <= r2 && theta > 0;
    }
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
    point_cloud = cloud;
}

float Navigation::FutureVelocity() {
    float old = robot_vel_.x();
    old += max_acc * time_step;
    return min(old, max_vel);
}


float Navigation::UpdateFreeDistance(float curvature) {
    float min_free_dist = 2;
    for (Vector2f point : point_cloud) {
        //determine if point is obstacle
        if (curvature < 0) 
            point.y() = abs(point.y());
        if (isObstacle(point, curvature)) {
            
            //DrawCross(robot_loc_ + point, 0.5, 0xFF0000, local_viz_msg_);
            //cout << "OBSTACLE" << endl;
            //if so find free distance to point
            float free_dist = GetFreeDistance(point, abs(curvature));
            //cout << free_dist << endl;
            if (free_dist < min_free_dist && free_dist >= 0) {
                min_free_dist = free_dist;
            }
            //keep track of this min distance
        } 
    }
    //min_free_dist = min_free_dist == MAXFLOAT ? 2 : min_free_dist;
    // robot_free_dist_ = min_free_dist;
    //set delta_x to the min distance
    //cutoff for min_free_dist
    min_free_dist = min_free_dist < free_dist_cutoff ? 0 : min_free_dist;
    //cout << "FREE DIST " << min_free_dist << endl;
    return min_free_dist;
}

float Navigation::GetVelocity(float delta_x) {
    // delta_x = robot_free_dist_;
    const float old_vel = robot_vel_.x();
    //robot_dist_traveled_ += robot_vel_.x() * time_step;
    // get the possible new velocity assuming we wont deccelerate
    const float poss_new_vel = FutureVelocity();
    // given the next velocity, this is how much distance it would take to slow down to 0.
    const float min_decc_dist = pow(poss_new_vel, 2) / (2 * max_decc);
    // if the dist so far + the min decc dist given a new vel is less than delta_x
    float lat_dist = robot_vel_.x() * latency;
    if (lat_dist + min_decc_dist < delta_x) {
        // we can continue with the new vel
        robot_vel_.x() = poss_new_vel;
    } else {
        // we need to deccelerate
        robot_vel_.x() -= max_decc * time_step;
        if (robot_vel_.x() < 0){
            robot_vel_.x() = 0;
        }
    }

    cout << old_vel << " ==> " << robot_vel_.x() << " dist: " << robot_dist_traveled_ << endl;
    return robot_vel_.x();
}

float Navigation::GetClearance(float delta_x, float theta) {
    //cout << "in get clearance " << point_cloud.size()<< endl;
    float r = 1.0 / theta; //maybe
    float r1 = abs(r) - w;
    float r2 = sqrt(Sq(abs(r) + w) + Sq(h));
    Vector2f c(0.0, r);
    float min_clearance = c_max;
    for (Vector2f point : point_cloud) {
        Vector2f pc_diff = c - point;
        float norm_pc = sqrt(Sq(pc_diff.x()) + Sq(pc_diff.y()));
        float theta = atan2(point.x(), abs(r - point.y()));  //maybe
        
        if ((abs((abs(norm_pc) - r)) < c_max) && (theta > 0) && (theta < 2)) { //TODO idk what theta_max should be so i just set it to 2 lol
        //cout << "THETA " << theta << " NORM " << norm_pc << endl;
        //cout << "r " << r <<" r1 " << r1 << " r2 " << r2 << endl;
            float l = abs(norm_pc) > r ? (abs(norm_pc) - r2) : (r1 - abs(norm_pc));//double check for negative in second case
            if (l < min_clearance) {
                //cout << " >>>>>>>>>>>>>>>>>>>>>>>>>>>> new min " << l << endl;
                // gettings cases where the point is between R and R2 (or R1)
                // this shouldnt really happen outside of sim..
                min_clearance = l < 0 ? 0 : l; 
            }
        } 
    }
    return min_clearance;
}

std::pair<float, float> Navigation::GetBestPath(float old_delta) {
    // for each possible path
    float max_score = 0;
    std::pair<float, float> best_path;
    for (float curve = -1; curve <= 1; curve += curve_delta) {
        float delta_x = UpdateFreeDistance(curve);
        float clearance = GetClearance(delta_x, curve);
        //cout << "Free Distance: " << delta_x << endl;
        //cout << "Clearance: " << clearance << endl;
        //cout << "theta index " << theta << endl;
        float score = delta_x; //+ (w1 * clearance) + (w2 * old_delta);
        cout << "SCORE " << score << " curve " << curve << endl;
        if (score >= max_score) {
            //cout << "MAX SCORE " << max_score << endl;
            max_score = score;
            best_path.first = delta_x;
            best_path.second = clearance = c_max ? curve : curve;
            DrawPathOption(curve, delta_x, 1, local_viz_msg_);
        }

    }
    return best_path;
}

void Navigation::Run(float delta_x, float theta) {

   // Vector2f p(5,0);
    ClearVisualizationMsg(local_viz_msg_);
    ClearVisualizationMsg(global_viz_msg_);
    

    std::pair<float, float> best_path = GetBestPath(delta_x);
    cout << ">>>>>> PATH : " << best_path.first << " " << best_path.second << endl; 
    const float new_vel = GetVelocity(best_path.first);
    AckermannCurvatureDriveMsg msg;
    msg.velocity = new_vel;
    msg.curvature = best_path.second;
    drive_pub_.publish(msg);
    viz_pub_.publish(local_viz_msg_);
}

}  // namespace navigation
