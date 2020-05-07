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
#include "vector_map/vector_map.h"
#include "simple_queue.h"
#include <unordered_map>
#include "shared/math/line2d.h"

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
using namespace geometry;


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
const float margin = 0.1;
const float car_half_width = 0.1405;
const float w = car_half_width + margin;
const float base_to_tip = .42;
const float h = base_to_tip + margin;
const float curve_epsilon = 1e-3;
const float free_dist_cutoff = 0.01;
const float curve_delta = 0.1;
const float c_max = 5.0;
const float done_dist = 1.0;


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
  //load map object here somewhere
  map_.Load("maps/GDC1.txt"); //eventually can change to map_file
  path_set = true;
}


bool Navigation::CheckNode(Vector2f& node) {
    const float grid_dist = 0.35;
    vector<Vector2f> neighbors;
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            if (!(i == 0 && j == 0)) {
                Vector2f n(node.x() + (i * grid_dist), node.y() + (j * grid_dist));
                //check if this neighbor n intercects with a map line and check that its not too close to wall
                if (map_.Intersects(node, n) ) {
                    return false;
                }
            }
        }
    }
    return true;
}

vector<Vector2f> Navigation::FindNeighbors(Vector2f& loc) {
    const float grid_dist = 0.125;
    vector<Vector2f> neighbors;
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            if (!(i == 0 && j == 0)) {
                Vector2f n(loc.x() + (i * grid_dist), loc.y() + (j * grid_dist));
                //check if this neighbor n intercects with a map line and check that its not too close to wall
                if (!map_.Intersects(loc, n) && CheckNode(n)) {
                    neighbors.push_back(n);
                }
            }
        }
    }
    return neighbors;

}

double Navigation::FindPathWeight(Vector2f& current, Vector2f& next) {
    //if xs are the same or ys are the same
    double weight = 0;
    double epsilon = 0.1;
    if (abs(current.x() - next.x()) < epsilon || abs(current.y() - next.y()) < epsilon ){
        weight = 1;
    } else {
        //they are diagonals
        weight = sqrt(2);
    }
    return weight;
}

double Navigation::heuristic(Vector2f& loc, Vector2f& goal) {
    return (goal - loc).norm();
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
    // cout << "nav_goal: " << loc << endl;
    nav_goal_loc_ = loc;
    path_set = false;

}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
    // cout << "loc: " << loc << endl;
    robot_loc_ = loc;
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
        nav_goal_loc_ = robot_loc_;
        first_odom = false;
        //hi
    }
    Rotation2Df delta_theta(-robot_angle_);
    // Vector2f delta_loc = (delta_theta * loc) - robot_loc_;
    Vector2f delta_loc = delta_theta * (loc - robot_loc_);
    // Vector2f delta_loc = loc - robot_loc_;

    robot_dist_traveled_ += sqrt(Sq(delta_loc.x()) + Sq(delta_loc.y()));
    robot_loc_ = loc;
    robot_angle_ = angle;
    //cout << "delta: " << delta_loc << endl;
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

float Navigation::GetVelocity(float delta_x) {
    // delta_x = robot_free_dist_;
    // const float old_vel = robot_vel_.x();
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

    // cout << old_vel << " ==> " << robot_vel_.x() << " dist: " << robot_dist_traveled_ << endl;
    return robot_vel_.x();
}

float Navigation::GetClearance(float delta_x, float curve) {
    //cout << "in get clearance " << point_cloud.size()<< endl;
    curve = abs(curve);
    assert(curve >= 0);
    float r = 1.0 / curve; //maybe
    float r1 = r - w;
    float r2 = sqrt(Sq(r + w) + Sq(h));
    Vector2f c(0.0, r);
    float min_clearance = c_max;
    for (Vector2f point : point_cloud) {
        // if (curve < 0)
        //     point.y() = abs(point.y());
        Vector2f pc_diff = c - point;
        float norm_pc = pc_diff.norm();
        float theta = atan2(point.x(), r - point.y());  //maybe
        
        if ((abs(norm_pc - r) < c_max) && (theta > 0) && (theta < curve)) { //TODO idk what theta_max should be so i just set it to 2 lol
        //cout << "THETA " << theta << " NORM " << norm_pc << endl;
        //cout << "r " << r <<" r1 " << r1 << " r2 " << r2 << endl;
            float l = norm_pc > r ? (norm_pc - r2) : (r1 - norm_pc);//double check for negative in second case
            if (l < min_clearance) {
                // cout << " >>>>>>>>>>>>>>>>>>>>>>>>>>>> new min " << l << endl;
                // gettings cases where the point is between R and R2 (or R1)
                // this shouldnt really happen outside of sim..
                min_clearance = l < 0 ? 0 : l; 
            }
        } 
    }
    return min_clearance;
}

float Navigation::GetDistanceRemaining(float free_distance, float curvature, Vector2f& carrot) {
    
    Vector2f goal = carrot;
    //cout << "goal: " << carrot << endl;
    //cout << "phi: " << phi << endl;
    Rotation2Df rot(robot_angle_);
   
    if (abs(curvature) <= curve_epsilon) {

        //cout << "distance remaining straight: " << 5 << endl;
        Vector2f straight(robot_loc_.x() + free_distance, robot_loc_.y());
        Vector2f straight_(robot_loc_.x() + (free_distance * cos(robot_angle_)), robot_loc_.y() + (free_distance * sin(robot_angle_))); //need free distance
        // Vector2f straight_ = rot * straight;
        //DrawCross(straight_, 0.5, 0x00ff00, local_viz_msg_);
        Vector2f straight_diff = goal - straight_;
        //assert(false);
        return straight_diff.norm();
    }
    
    float r = 1.0 / curvature; //was abs(curvature)
    float theta = free_distance / r;
   
    //cout << "THETA: " << theta << "ROBOT_ANGLE: " << robot_angle_<< endl;
    float y = r - (cos(theta) * r);
    assert(y >= 0);
    // float x = r * cos(phi);
    float x = r * sin(theta);
    assert(x >= 0);
    Vector2f end_pos(x, y);
    end_pos = rot * end_pos;
    Vector2f vis_point = end_pos + robot_loc_;
    //vis_point = rot * vis_point;
    // Vector2f vis_point_(vis_point.x() + (5.0 * cos(robot_angle_)), vis_point.y() + (5.0 * sin(robot_angle_)));
    //DrawCross(vis_point, 0.5, 0x32a8a8, local_viz_msg_);

    // DrawCross(vis_point, 0.5, 0x00ff00, local_viz_msg_);
    //cout << "end_pos: " << end_pos << endl;
    //cout << "robot loc: " << robot_loc_ << endl;
    //cout << "Where it thinks it is: " << end_pos + robot_loc_ << endl;
    Vector2f diff = goal - vis_point;
    //cout << "distance remaining curve: " << diff.norm() << endl;
    return diff.norm(); //- (delta * curvature);
}


//calculates distance from car to obstacle
std::pair<float, float> GetFreeDistance(Vector2f& point, float curvature) {
    //cout << "curve   :" <<  curvature << endl;
    std::pair<float, float> delta_x_phi;
    float free_distance;
    assert(curvature >= 0);
    assert(point.y() >= 0);
    if (curvature < curve_epsilon) {
            //practically 0 curvature
            //slideset 6 slide 20
            free_distance = point.x() - h;
            delta_x_phi.first = free_distance;
            delta_x_phi.second = 100;
    }
    else
    {
        // there is curvature to be accounted for
        //slideset 6 slide 35
        float r = 1.0 / curvature;
        float theta = atan2(point.x(), r - point.y());
        //cout << "theta " << theta << " curve " << curvature << endl;
        float omega = atan2(h, r - w);
        float phi = theta - omega;
        assert(phi > 0);
        //cout << "R*PHI >> " << r * phi << endl;
        free_distance = r * phi;
        delta_x_phi.first = free_distance;
        delta_x_phi.second = phi;
    }
    return delta_x_phi;
}


//determines if a point is in the car's immediate path trajectory
bool isObstacle(Vector2f& point, float curvature) {
    if (curvature < 0 && point.y() > w )
        return false;

    if (curvature > 0 && point.y() < -w )
        return false;


    curvature = abs(curvature);
    float y = abs(point.y());
    assert(curvature >= 0);
    assert(y >= 0);

    if (curvature < curve_epsilon)
    {
        //practically 0 curvature
        // cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
        //slideset 6 slide 18
        return y <= w;
    } else {
       
        // there is curvature
        float r = 1.0 / curvature;
        float r1 = r - w;
        float r2 = sqrt(Sq(r + w) + Sq(h));
        //signs
        //r1 = r > 0 ? r1 : -1.0 * r1;
        //r2 = r > 0 ? r2 : -1.0 * r2;
        Vector2f c(0, r);
        float theta = atan2(point.x(), r - y);
        Vector2f Bug(point.x(), y);
        Vector2f pc_diff = Bug - c;
        float norm_pc = pc_diff.norm(); //sqrt(Sq(pc_diff.x()) + Sq(pc_diff.y()));

        return norm_pc >= r1 && norm_pc <= r2 && theta > 0;
    }
}

std::pair<float, float> Navigation::UpdateFreeDistance(float curvature) {
    // float min_free_dist = 3;
    std::pair<float, float> delta_x_phi;
    delta_x_phi.first = 5.0;
    delta_x_phi.second = 5.0 / (1 / abs(curvature));
    for (Vector2f point : point_cloud)
    {
        //determine if point is obstacle

        if (isObstacle(point, curvature)) {
            point.y() = abs(point.y());
            assert(point.y() >= 0);
            //cout << "OBSTACLE" << endl;
            //if so find free path length to the obstacle point
            std::pair<float, float> trial_delta_x_phi = GetFreeDistance(point, abs(curvature));
            assert(free_dist >= 0);
            // free_dist = max(free_dist, (float)0);
            //cout << free_dist << endl;
            if (trial_delta_x_phi.first < delta_x_phi.first && trial_delta_x_phi.first >= 0) {
                delta_x_phi.first = trial_delta_x_phi.first;
                delta_x_phi.second = trial_delta_x_phi.second;
            }
            //keep track of this min distance
        } 
    }
    //min_free_dist = min_free_dist == MAXFLOAT ? 2 : min_free_dist;
    // robot_free_dist_ = min_free_dist;
    //set delta_x to the min distance
    //cutoff for min_free_dist
    delta_x_phi.first = delta_x_phi.first < free_dist_cutoff ? 0 : delta_x_phi.first;
    //cout << "FREE DIST " << min_free_dist << endl;
    return delta_x_phi;
}


std::pair<float, float> Navigation::GetBestPath(float old_delta, Vector2f& carrot) {
    // for each possible path
    // const float w1 = 0.4;
    const float w2 = -2; //-0.4;
    float max_score = -MAXFLOAT;
    std::pair<float, float> best_path;

    //TODO CHANGE BACK TO -1
    float right = 0;
    float center = 0; 
    float left = 0;
    for (float curve = -1; curve <= 1; curve += curve_delta) {
        std::pair<float, float> delta_x_phi = UpdateFreeDistance(curve);
        // float clearance = GetClearance(delta_x_phi.first, curve);
       // cout << "clearance: " << clearance << endl;
        float distance_to_goal = GetDistanceRemaining(delta_x_phi.first, curve, carrot);
        //float clearance = 0;
        if (curve == -0.5)
            right = distance_to_goal;
        if (curve == 0)
            center = distance_to_goal;
        if (curve == 0.5)
            left = distance_to_goal;

        float score = delta_x_phi.first + w2 * distance_to_goal;
        //this shows me that left is always getting a lower distance remaining. 
        //cout << "SCORE " << score << " curve " << curve << " clearance " << clearance << endl;
        //DrawPathOption(curve, delta_x_phi.first, clearance, local_viz_msg_);
        if (score >= max_score) {
            //cout << "MAX SCORE " << max_score << endl;
            max_score = score;
            best_path.first = delta_x_phi.first;
            best_path.second = curve;
            
        }
    }
    //DrawPathOption(best_path.second, best_path.first, 0, local_viz_msg_);
    cout << "LEFT:  " << left << " Center: " << center << " RIGHT: " << right << endl; 
    return best_path;
}



void Navigation::PlanPath(std::unordered_map<Vector2f, Vector2f, matrix_hash<Vector2f>>& came_from) {
    // A* from https://www.redblobgames.com/pathfinding/a-star/implementation.html
    //https://www.geeksforgeeks.org/unordered_map-in-cpp-stl/
    came_from.clear();
    std::unordered_map<Vector2f, double, matrix_hash<Vector2f>> cost_so_far;
    SimpleQueue<Vector2f, double> frontier; //probably have to change this
    Vector2f start = robot_loc_; 

    frontier.Push(start, 0);
    came_from[start] = start;
    cost_so_far[start] = 0;

    while (!frontier.Empty()) {
        // cout << "top of loop" << endl;
        Vector2f current = frontier.Pop();

        // cout << "curr - nav: " << (current - nav_goal_loc_).norm() << endl;
        if ((current - nav_goal_loc_).norm() < 0.25) {
            nav_goal_loc_print_ = current;
            break;
        }

        for (Vector2f next : FindNeighbors(current)) {//change to FindNeighbors
            double new_cost = cost_so_far[current] + FindPathWeight(current, next); //find way to get cost
            if (cost_so_far.find(next) == cost_so_far.end()
                || new_cost < cost_so_far[next]) {
                cost_so_far[next] = new_cost;
                double priority = new_cost + heuristic(next, nav_goal_loc_);
                frontier.Push(next, priority);
                came_from[next] = current;
            }
        }
    }   
}

Vector2f Navigation::GetCarrot() {
    Vector2f carrot(69, 69);
    float radius = 4.0;
    float epsilon = 0.2;
    float min_pos = path_points.size() - 1;
    for (int j = 0; j < (int) path_points.size(); j++) {
        Vector2f point = path_points[j];
        if ((robot_loc_ - point).norm() - radius <= epsilon && j < min_pos){
            //double check that this carrot isnt gonna run you through a wall...
            //if (!map_.Intersects(robot_loc_, point)){
                min_pos = j;
                carrot = path_points[j];
            //}
        }
    }
    //cout << "CARROT: " << carrot.x() << " " << carrot.y() << endl;
    return carrot;
}


void Navigation::Run(float delta_x, float theta) {

   // Vector2f p(5,0);
    ClearVisualizationMsg(local_viz_msg_);
    ClearVisualizationMsg(global_viz_msg_);
    AckermannCurvatureDriveMsg msg;

    //check if we are at the goal location
    
    
    //******CP7*******
    //getCarrot()
    if ((nav_goal_loc_ - robot_loc_).norm() <= done_dist || nav_goal_loc_ == Vector2f(0,0) ){
        msg.velocity = 0;
        msg.curvature = 0;
        // msg.curvature = theta;
        drive_pub_.publish(msg);
        viz_pub_.publish(local_viz_msg_);
        return;
    } 
    Vector2f carrot;
    if (path_set) {
        carrot = GetCarrot();
    } else {
        came_from.clear();
        path_points.clear();
        //cout << "PLANNING: " << endl;
        
        PlanPath(came_from);
        //cout << "Path set" << endl;
        path_set = true;
        /*
        void DrawLine(const Eigen::Vector2f& p0,
              const Eigen::Vector2f& p1,
              uint32_t color,
              f1tenth_course::VisualizationMsg& msg);
        */
        //cout <<"plan_path nav_goal: " << nav_goal_loc_print_ << endl;
        Vector2f end = nav_goal_loc_print_;
        Vector2f start = came_from[nav_goal_loc_print_];
        path_points.push_back(end);
       
        while (start != end) {
            //build line list here
          
            DrawLine(end, start, 0xFF0000, local_viz_msg_);
            end = start;
            start = came_from[start];
            path_points.push_back(end);
          
        }
        carrot = GetCarrot(); //hopefully wont fail
    } 

    Vector2f end = nav_goal_loc_print_;
    Vector2f start = came_from[nav_goal_loc_print_];
    while (start != end) {
            //build line list here
          
            DrawLine(end, start, 0xFF0000, local_viz_msg_);
            end = start;
            start = came_from[start];
    
          
        }
    
    DrawCross(carrot, 0.5, 0x050505, local_viz_msg_);

    
    std::pair<float, float> best_path = GetBestPath(delta_x, carrot);
    //  cout << ">>>>>> PATH : " << best_path.first << " " << best_path.second << endl; 
    
    // const std::pair<float,float> x = UpdateFreeDistance(theta);
    const float new_vel = GetVelocity(best_path.first);
    // cout << "dist left: " << x.first << endl;
    // const float new_vel = GetVelocity(x.first);
    
    msg.velocity = new_vel;
    msg.curvature = best_path.second;
    // msg.curvature = theta;
    drive_pub_.publish(msg);
    viz_pub_.publish(local_viz_msg_);
}

}  // namespace navigation
