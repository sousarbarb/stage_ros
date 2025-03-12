/*
 *  stageros
 *  Copyright (c) 2008, Willow Garage, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  >>>>> Adaptation by Ricardo B. Sousa to make stage_ros compatible with ROS 2
 */

/**

@mainpage

@htmlinclude manifest.html
**/

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

// libstage
#include <stage.hh>

// ROS
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>

// boost
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#define USAGE "stageros <worldfile>"
#define IMAGE "image"
#define DEPTH "depth"
#define CAMERA_INFO "camera_info"
#define ODOM "odom"
#define BASE_SCAN "base_scan"
#define BASE_POSE_GROUND_TRUTH "base_pose_ground_truth"
#define CMD_VEL "cmd_vel"

// Our node
class StageROS2Node : public rclcpp::Node
{
 private:

  boost::mutex msg_lock;  //!< mutex lock access to fields used in msg callbacks

  std::vector<Stg::ModelCamera*> cameramodels;      //!< Stage camera model
  std::vector<Stg::ModelRanger*> lasermodels;       //!< Stage laser model
  std::vector<Stg::ModelPosition*> positionmodels;  //!< Stage position model

  /**
   * @brief structure representing a robot in the simulator
   *        (interface with Stage's backend)
   */
  struct StageRobot
  {
    // stage related models
    Stg::ModelPosition* positionmodel;            //!< one position / robot
    std::vector<Stg::ModelCamera*> cameramodels;  //!< multiple cameras / robot
    std::vector<Stg::ModelRanger*> lasermodels;   //!< multiple lasers / robot

    // ros publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
        odom_pub;  //!< odometry publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
        ground_truth_pub;  //!< ground-truth pose publisher

    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>
        image_pubs;  //!< multiple images publishers
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>
        depth_pubs;  //!< multiple depths publishers
    std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr>
        camera_pubs;  //!< multiple cameras publishers
    std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr>
        laser_pubs;  //!< multiple lasers publishers

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        cmdvel_sub;  //!< one cmd_vel subscriber
  };

  std::vector<StageRobot const*> robotmodels_;
  std::vector<Stg::Pose> initial_poses;  //!< remember init poses for soft rst

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;  //!< reset stage

  rclcpp::Publisher<rosgraph_msgs::msg::Clock> clock_pub_;  //!< sim clock

  bool isDepthCanonical;
  bool use_model_names;

  tf2_ros::TransformBroadcaster tf;               //!< TF broadcaster
  tf2_ros::StaticTransformBroadcaster tf_static;  //!< TF static broadcaster

  bool is_tf_static_init = false;   //!< check if TF static initialized
  bool param_pub_gt_tf = false;     //!< enable TF ground-truth publication
  bool param_pub_static_tf = true;  //!< enable TF static frames publication
  uint32_t msgs_seq = 0;            //!< ROS messages header.seq counter

  std::string param_frame_gt;              //!< ground-truth TF frame ID
  std::string param_frame_odom;            //!< odometry TF frame ID
  std::string param_frame_base_footprint;  //!< base footprint TF frame ID
  std::string param_frame_base_link;       //!< base link TF frame ID
  std::string param_frame_laser;           //!< 2D laser scan TF frame ID
  std::string param_frame_camera;          //!< camera TF frame ID

  bool param_use_odom_model;  //!< odometry model vs ground-truth in odom_msg

  rclcpp::Time base_last_cmd;              //!< last time received velocity cmd
  rclcpp::Time base_last_globalpos_time;   //!< last time global pose saved
                                           //!< (vel compute)
  rclcpp::Time sim_time;                   //!< current simulation time
  rclcpp::Duration base_watchdog_timeout;  //!< base watchdog timeout

  std::vector<Stg::Pose> base_last_globalpos;  //!< last published robots pose

 public:

  Stg::World* world;  //!< main simulator object
};

int main(int argc, char** argv)
{
  std::cout << "Hello Brave New World!" << std::endl;
  return 0;
}
