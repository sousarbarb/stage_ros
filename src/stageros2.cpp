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

#include <exception>
#include <mutex>
#include <thread>

// rclcpp (you need to ensure that this header is included before all others...)
#include <rclcpp/rclcpp.hpp>

// libstage
#include <stage.hh>

// ROS
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>

#ifdef ROS_DISTRO_FOXY
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#define USAGE "stageros2 [-g] [-u] [-p] <worldfile>"
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

  std::mutex msg_lock;  //!< mutex lock access to fields used in msg callbacks

  std::vector<Stg::ModelCamera*> cameramodels;      //!< Stage camera model
  std::vector<Stg::ModelRanger*> lasermodels;       //!< Stage laser model
  std::vector<Stg::ModelPosition*> positionmodels;  //!< Stage position model

  /**
   * @brief structure representing a robot in the simulator
   *        (interface with Stage's backend)
   */
  struct StageRobot
  {
    int robot_id;  //!< robot ID (required for the cmd vel callback)

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

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr
      clock_pub_;  //!< sim clock

  bool isDepthCanonical;
  bool use_model_names;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf;  //!< TF broadcaster
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster>
      tf_static;  //!< TF static broadcaster

  bool is_tf_static_init = false;   //!< check if TF static initialized
  bool param_pub_gt_tf = false;     //!< enable TF ground-truth publication
  bool param_pub_static_tf = true;  //!< enable TF static frames publication

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

 public:

  /**
   * @brief Construct a new StageROS2Node object
   * @param[in] argc number of arguments entered on the command line
   * @param[in] argv string pointers array for each argument
   * @param[in] gui enable Graphical User Interface (GUI)
   * @param[in] fname world filename
   * @param[in] use_model_names use model names from the .world configuration
   *                            file
   */
  StageROS2Node(int argc, char** argv, bool gui, const char* /* fname */,
                bool use_model_names)
      : rclcpp::Node("stageros2"),
        base_watchdog_timeout(std::chrono::nanoseconds(0))
  {
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    this->use_model_names = use_model_names;
    this->sim_time = rclcpp::Time(0.0);
    this->base_last_cmd = rclcpp::Time(0.0);

    // Declare ROS parameters
    this->declare_parameter<std::string>("world", "");
    this->declare_parameter<double>("base_watchdog_timeout", 0.2);
    this->declare_parameter<bool>("is_depth_canonical", true);
    this->declare_parameter<bool>("pub_gt_tf", false);
    this->declare_parameter<bool>("pub_static_tf", true);
    this->declare_parameter<std::string>("gt_frame", "gt");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_footprint_frame",
                                         "base_footprint");
    this->declare_parameter<std::string>("base_link_frame", "base_link");
    this->declare_parameter<std::string>("laser_frame", "base_laser_link");
    this->declare_parameter<std::string>("camera_frame", "camera");
    this->declare_parameter<bool>("use_odom_model", true);

    std::string world = this->get_parameter("world").as_string();

    this->base_watchdog_timeout.from_seconds(
        this->get_parameter("base_watchdog_timeout").as_double());

    isDepthCanonical = this->get_parameter("is_depth_canonical").as_bool();
    param_pub_gt_tf = this->get_parameter("pub_gt_tf").as_bool();
    param_pub_static_tf = this->get_parameter("pub_static_tf").as_bool();
    param_frame_gt = this->get_parameter("gt_frame").as_string();
    param_frame_odom = this->get_parameter("odom_frame").as_string();
    param_frame_base_footprint =
        this->get_parameter("base_footprint_frame").as_string();
    param_frame_base_link = this->get_parameter("base_link_frame").as_string();
    param_frame_laser = this->get_parameter("laser_frame").as_string();
    param_frame_camera = this->get_parameter("camera_frame").as_string();
    param_use_odom_model = this->get_parameter("use_odom_model").as_bool();

    // TF initialization
    tf = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_static = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    // We'll check the existence of the world file, because libstage doesn't
    // expose its failure to open it.  Could go further with checks (e.g., is
    // it readable by this user).
    struct stat s;
    if (stat(world.c_str(), &s) != 0)
    {
      RCLCPP_FATAL(this->get_logger(), "The world file %s does not exist.",
                   world.c_str());
      rclcpp::shutdown();
      throw std::runtime_error(
          "StageROS2Node::StageROS2Node | world file does not exist");
    }
    RCLCPP_INFO(this->get_logger(), "World file: %s", world.c_str());

    // initialize libstage
    Stg::Init(&argc, &argv);

    if (gui)
    {
      this->world = new Stg::WorldGui(600, 400, "Stage (ROS)");
    }
    else
    {
      this->world = new Stg::World();
    }

    this->world->Load(world.c_str());

    // todo: reverse the order of these next lines? try it .

    this->world->AddUpdateCallback((Stg::world_callback_t) s_update, this);

    // inspect every model to locate the things we care about
    this->world->ForEachDescendant((Stg::model_callback_t) ghfunc, this);
  }

  /**
   * @brief Destroy the StageROS2Node object
   */
  ~StageROS2Node()
  {
    for (std::vector<StageRobot const*>::iterator r =
             this->robotmodels_.begin();
         r != this->robotmodels_.end(); ++r)
    {
      delete *r;
    }
  }

  /**
   * @brief Subscribe to models of interest (currently, find and subscribe to
   *        the first 'laser' model and the first 'position' model)
   * @return int 0 on success (both models subscribed), -1 otherwise
   */
  int SubscribeModels()
  {
    // Parameters declaration
    for (size_t r = 0; r < this->positionmodels.size(); r++)
    {
      StageRobot* new_robot = new StageRobot;
      new_robot->robot_id = static_cast<int>(r);
      new_robot->positionmodel = this->positionmodels[r];
      new_robot->positionmodel->Subscribe();

      RCLCPP_INFO(this->get_logger(),
                  "Subscribed to Stage position model \"%s\"",
                  this->positionmodels[r]->Token());

      for (size_t s = 0; s < this->lasermodels.size(); s++)
      {
        if (this->lasermodels[s] and
            this->lasermodels[s]->Parent() == new_robot->positionmodel)
        {
          new_robot->lasermodels.push_back(this->lasermodels[s]);
          this->lasermodels[s]->Subscribe();
          RCLCPP_INFO(this->get_logger(), "subscribed to Stage ranger \"%s\"",
                      this->lasermodels[s]->Token());
        }
      }

      for (size_t s = 0; s < this->cameramodels.size(); s++)
      {
        if (this->cameramodels[s] and
            this->cameramodels[s]->Parent() == new_robot->positionmodel)
        {
          new_robot->cameramodels.push_back(this->cameramodels[s]);
          this->cameramodels[s]->Subscribe();

          RCLCPP_INFO(this->get_logger(),
                      "subscribed to Stage camera model \"%s\"",
                      this->cameramodels[s]->Token());
        }
      }

      // TODO - print the topic names nicely as well
      RCLCPP_INFO(
          this->get_logger(), "Robot %s provided %lu rangers and %lu cameras",
          new_robot->positionmodel->Token(), new_robot->lasermodels.size(),
          new_robot->cameramodels.size());

      new_robot->odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(
          mapName(ODOM, r, static_cast<Stg::Model*>(new_robot->positionmodel)),
          10);
      new_robot->ground_truth_pub =
          this->create_publisher<nav_msgs::msg::Odometry>(
              mapName(BASE_POSE_GROUND_TRUTH, r,
                      static_cast<Stg::Model*>(new_robot->positionmodel)),
              10);
      new_robot->cmdvel_sub =
          this->create_subscription<geometry_msgs::msg::Twist>(
              mapName(CMD_VEL, r,
                      static_cast<Stg::Model*>(new_robot->positionmodel)),
              10,
              [&, this,
               new_robot](const geometry_msgs::msg::Twist::SharedPtr msg)
              { this->cmdvelReceived(msg, new_robot->robot_id); });

      for (size_t s = 0; s < new_robot->lasermodels.size(); ++s)
      {
        if (new_robot->lasermodels.size() == 1)
          new_robot->laser_pubs.push_back(
              this->create_publisher<sensor_msgs::msg::LaserScan>(
                  mapName(BASE_SCAN, r,
                          static_cast<Stg::Model*>(new_robot->positionmodel)),
                  10));
        else
          new_robot->laser_pubs.push_back(
              this->create_publisher<sensor_msgs::msg::LaserScan>(
                  mapName(BASE_SCAN, r, s,
                          static_cast<Stg::Model*>(new_robot->positionmodel)),
                  10));
      }

      for (size_t s = 0; s < new_robot->cameramodels.size(); ++s)
      {
        if (new_robot->cameramodels.size() == 1)
        {
          new_robot->image_pubs.push_back(
              this->create_publisher<sensor_msgs::msg::Image>(
                  mapName(IMAGE, r,
                          static_cast<Stg::Model*>(new_robot->positionmodel)),
                  10));
          new_robot->depth_pubs.push_back(
              this->create_publisher<sensor_msgs::msg::Image>(
                  mapName(DEPTH, r,
                          static_cast<Stg::Model*>(new_robot->positionmodel)),
                  10));
          new_robot->camera_pubs.push_back(
              this->create_publisher<sensor_msgs::msg::CameraInfo>(
                  mapName(CAMERA_INFO, r,
                          static_cast<Stg::Model*>(new_robot->positionmodel)),
                  10));
        }
        else
        {
          new_robot->image_pubs.push_back(
              this->create_publisher<sensor_msgs::msg::Image>(
                  mapName(IMAGE, r, s,
                          static_cast<Stg::Model*>(new_robot->positionmodel)),
                  10));
          new_robot->depth_pubs.push_back(
              this->create_publisher<sensor_msgs::msg::Image>(
                  mapName(DEPTH, r, s,
                          static_cast<Stg::Model*>(new_robot->positionmodel)),
                  10));
          new_robot->camera_pubs.push_back(
              this->create_publisher<sensor_msgs::msg::CameraInfo>(
                  mapName(CAMERA_INFO, r, s,
                          static_cast<Stg::Model*>(new_robot->positionmodel)),
                  10));
        }
      }

      this->robotmodels_.push_back(new_robot);
    }
    clock_pub_ =
        this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

    // advertising reset service
    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
        "reset_positions",
        std::bind(&StageROS2Node::cb_reset_srv, this, std::placeholders::_1,
                  std::placeholders::_2));

    return (0);
  }

  /**
   * @brief our callback
   */
  void WorldCallback()
  {
    if (!rclcpp::ok())
    {
      RCLCPP_INFO(this->get_logger(), "rclcpp::ok() is false. Quitting.");
      this->world->QuitAll();
      return;
    }

    std::scoped_lock lock(msg_lock);

    this->sim_time = rclcpp::Time(world->SimTimeNow() * 1e3);
    // We're not allowed to publish clock==0, because it used as a special
    // value in parts of ROS, #4027.
    if (this->sim_time.seconds() == 0 && this->sim_time.nanoseconds() == 0)
    {
      RCLCPP_DEBUG(
          this->get_logger(),
          "Skipping initial simulation step, to avoid publishing clock==0");
      return;
    }

    // TODO make this only affect one robot if necessary
    if ((this->base_watchdog_timeout.seconds() > 0.0) &&
        ((this->sim_time - this->base_last_cmd) >= this->base_watchdog_timeout))
    {
      for (size_t r = 0; r < this->positionmodels.size(); r++)
        this->positionmodels[r]->SetSpeed(0.0, 0.0, 0.0);
    }

    // INIT TFs
    if (!is_tf_static_init && param_pub_static_tf)
    {
      for (size_t r = 0; r < this->robotmodels_.size(); ++r)
      {
        StageRobot const* robotmodel = this->robotmodels_[r];

        // loop on the laser devices for the current robot
        for (size_t s = 0; s < robotmodel->lasermodels.size(); ++s)
        {
          Stg::ModelRanger const* lasermodel = robotmodel->lasermodels[s];

          // Also publish the base->base_laser_link Tx.  This could eventually
          // move into being retrieved from the param server as a static Tx.
          Stg::Pose lp = lasermodel->GetPose();
          tf2::Quaternion laserQ;
          laserQ.setRPY(0.0, 0.0, lp.a);
          tf2::Transform txLaser(
              laserQ,
              tf2::Vector3(lp.x, lp.y,
                           robotmodel->positionmodel->GetGeom().size.z + lp.z));

          geometry_msgs::msg::TransformStamped tfLaser;

          if (robotmodel->lasermodels.size() > 1)
          {
            tfLaser.header.frame_id = std::string(
                mapName(param_frame_base_link.c_str(), r,
                        static_cast<Stg::Model*>(robotmodel->positionmodel)));
            tfLaser.child_frame_id = std::string(
                mapName(param_frame_laser.c_str(), r, s,
                        static_cast<Stg::Model*>(robotmodel->positionmodel)));
          }
          else
          {
            tfLaser.header.frame_id = std::string(
                mapName(param_frame_base_link.c_str(), r,
                        static_cast<Stg::Model*>(robotmodel->positionmodel)));
            tfLaser.child_frame_id = std::string(
                mapName(param_frame_laser.c_str(), r,
                        static_cast<Stg::Model*>(robotmodel->positionmodel)));
          }

          tfLaser.header.stamp = sim_time;
          tfLaser.transform = tf2::toMsg(txLaser);

          tf_static->sendTransform(tfLaser);
        }

        geometry_msgs::msg::TransformStamped tfRobotBase;

        tfRobotBase.header.frame_id = std::string(
            mapName(param_frame_base_footprint.c_str(), r,
                    static_cast<Stg::Model*>(robotmodel->positionmodel)));
        tfRobotBase.header.stamp = sim_time;
        tfRobotBase.child_frame_id = std::string(
            mapName(param_frame_base_link.c_str(), r,
                    static_cast<Stg::Model*>(robotmodel->positionmodel)));
        tfRobotBase.transform = tf2::toMsg(tf2::Transform::getIdentity());

        tf_static->sendTransform(tfRobotBase);

        // cameras
        for (size_t s = 0; s < robotmodel->cameramodels.size(); ++s)
        {
          Stg::ModelCamera* cameramodel = robotmodel->cameramodels[s];

          Stg::Pose lp = cameramodel->GetPose();
          tf2::Quaternion Q;
          Q.setRPY((cameramodel->getCamera().pitch() * M_PI / 180.0) - M_PI,
                   0.0,
                   lp.a + (cameramodel->getCamera().yaw() * M_PI / 180.0) -
                       robotmodel->positionmodel->GetPose().a);

          tf2::Transform tr(
              Q,
              tf2::Vector3(lp.x, lp.y,
                           robotmodel->positionmodel->GetGeom().size.z + lp.z));

          geometry_msgs::msg::TransformStamped tfCamera;

          if (robotmodel->cameramodels.size() > 1)
          {
            tfCamera.header.frame_id = std::string(
                mapName(param_frame_base_link.c_str(), r,
                        static_cast<Stg::Model*>(robotmodel->positionmodel)));
            tfCamera.child_frame_id = std::string(
                mapName(param_frame_camera.c_str(), r, s,
                        static_cast<Stg::Model*>(robotmodel->positionmodel)));
          }
          else
          {
            tfCamera.header.frame_id = std::string(
                mapName(param_frame_base_link.c_str(), r,
                        static_cast<Stg::Model*>(robotmodel->positionmodel)));
            tfCamera.child_frame_id = std::string(
                mapName(param_frame_camera.c_str(), r,
                        static_cast<Stg::Model*>(robotmodel->positionmodel)));
          }

          tfCamera.header.stamp = sim_time;
          tfCamera.transform = tf2::toMsg(tr);

          tf_static->sendTransform(tfCamera);
        }
      }

      is_tf_static_init = true;
    }

    // loop on the robot models
    for (size_t r = 0; r < this->robotmodels_.size(); ++r)
    {
      StageRobot const* robotmodel = this->robotmodels_[r];

      // loop on the laser devices for the current robot
      for (size_t s = 0; s < robotmodel->lasermodels.size(); ++s)
      {
        Stg::ModelRanger const* lasermodel = robotmodel->lasermodels[s];
        const std::vector<Stg::ModelRanger::Sensor>& sensors =
            lasermodel->GetSensors();

        if (sensors.size() > 1)
        {
          RCLCPP_WARN(
              this->get_logger(),
              "ROS Stage currently supports rangers with 1 sensor only.");
        }

        // for now we access only the zeroth sensor of the ranger - good
        // enough for most laser models that have a single beam origin
        const Stg::ModelRanger::Sensor& sensor = sensors[0];

        if (sensor.ranges.size())
        {
          // Translate into ROS message format and publish
          sensor_msgs::msg::LaserScan msg;
          msg.angle_min = -sensor.fov / 2.0;
          msg.angle_max = +sensor.fov / 2.0;
          msg.angle_increment = sensor.fov / (double) (sensor.sample_count - 1);
          msg.range_min = sensor.range.min;
          msg.range_max = sensor.range.max;
          msg.ranges.resize(sensor.ranges.size());
          msg.intensities.resize(sensor.intensities.size());

          for (unsigned int i = 0; i < sensor.ranges.size(); i++)
          {
            msg.ranges[i] = sensor.ranges[i];
            msg.intensities[i] = sensor.intensities[i];
          }

          if (robotmodel->lasermodels.size() > 1)
            msg.header.frame_id =
                mapName(param_frame_laser.c_str(), r, s,
                        static_cast<Stg::Model*>(robotmodel->positionmodel));
          else
            msg.header.frame_id =
                mapName(param_frame_laser.c_str(), r,
                        static_cast<Stg::Model*>(robotmodel->positionmodel));

          msg.header.stamp = sim_time;
          robotmodel->laser_pubs[s]->publish(msg);
        }
      }

      // Get ground-truth pose
      Stg::Pose gpose = robotmodel->positionmodel->GetGlobalPose();

      // Get latest odometry data
      // Translate into ROS message format and publish
      tf2::Quaternion robotQ;
      nav_msgs::msg::Odometry odom_msg;
      if (param_use_odom_model)
      {
        robotQ.setRPY(0, 0, robotmodel->positionmodel->est_pose.a);
        odom_msg.pose.pose.position.x = robotmodel->positionmodel->est_pose.x;
        odom_msg.pose.pose.position.y = robotmodel->positionmodel->est_pose.y;
      }
      else
      {
        robotQ.setRPY(0, 0, gpose.a);
        odom_msg.pose.pose.position.x = gpose.x;
        odom_msg.pose.pose.position.y = gpose.y;
      }
      odom_msg.pose.pose.orientation = tf2::toMsg(robotQ);
      Stg::Velocity v = robotmodel->positionmodel->GetVelocity();
      odom_msg.twist.twist.linear.x = v.x;
      odom_msg.twist.twist.linear.y = v.y;
      odom_msg.twist.twist.angular.z = v.a;

      //@todo Publish stall on a separate topic when one becomes available
      // this->odomMsgs[r].stall = this->positionmodels[r]->Stall();
      //
      odom_msg.header.frame_id =
          mapName(param_frame_odom.c_str(), r,
                  static_cast<Stg::Model*>(robotmodel->positionmodel));
      odom_msg.header.stamp = sim_time;
      odom_msg.child_frame_id = std::string(
          mapName(param_frame_base_footprint.c_str(), r,
                  static_cast<Stg::Model*>(robotmodel->positionmodel)));

      robotmodel->odom_pub->publish(odom_msg);

      // broadcast odometry transform
      tf2::Quaternion odomQ;
      tf2::fromMsg(odom_msg.pose.pose.orientation, odomQ);
      tf2::Transform txOdom(odomQ,
                            tf2::Vector3(odom_msg.pose.pose.position.x,
                                         odom_msg.pose.pose.position.y, 0.0));

      geometry_msgs::msg::TransformStamped tfOdom;

      tfOdom.header.frame_id = std::string(
          mapName(param_frame_odom.c_str(), r,
                  static_cast<Stg::Model*>(robotmodel->positionmodel)));

      tfOdom.header.stamp = sim_time;
      tfOdom.child_frame_id = std::string(
          mapName(param_frame_base_footprint.c_str(), r,
                  static_cast<Stg::Model*>(robotmodel->positionmodel)));
      tfOdom.transform = tf2::toMsg(txOdom);

      tf->sendTransform(tfOdom);

      // Also publish the ground truth pose and velocity
      tf2::Quaternion q_gpose;
      q_gpose.setRPY(0.0, 0.0, gpose.a);
      tf2::Transform gt(q_gpose, tf2::Vector3(gpose.x, gpose.y, 0.0));
      // Velocity is 0 by default and will be set only if there is previous pose
      // and time delta>0
      Stg::Velocity gvel(0, 0, 0, 0);
      if (this->base_last_globalpos.size() > r)
      {
        Stg::Pose prevpose = this->base_last_globalpos.at(r);
        double dT = (this->sim_time - this->base_last_globalpos_time).seconds();
        if (dT > 0)
          gvel = Stg::Velocity((gpose.x - prevpose.x) / dT,
                               (gpose.y - prevpose.y) / dT,
                               (gpose.z - prevpose.z) / dT,
                               Stg::normalize(gpose.a - prevpose.a) / dT);
        this->base_last_globalpos.at(r) = gpose;
      }
      else  // There are no previous readings, adding current pose...
        this->base_last_globalpos.push_back(gpose);

      nav_msgs::msg::Odometry ground_truth_msg;
      ground_truth_msg.pose.pose.position.x = gt.getOrigin().x();
      ground_truth_msg.pose.pose.position.y = gt.getOrigin().y();
      ground_truth_msg.pose.pose.position.z = gt.getOrigin().z();
      ground_truth_msg.pose.pose.orientation.x = gt.getRotation().x();
      ground_truth_msg.pose.pose.orientation.y = gt.getRotation().y();
      ground_truth_msg.pose.pose.orientation.z = gt.getRotation().z();
      ground_truth_msg.pose.pose.orientation.w = gt.getRotation().w();
      ground_truth_msg.twist.twist.linear.x = gvel.x;
      ground_truth_msg.twist.twist.linear.y = gvel.y;
      ground_truth_msg.twist.twist.linear.z = gvel.z;
      ground_truth_msg.twist.twist.angular.z = gvel.a;

      ground_truth_msg.header.frame_id = param_frame_gt.c_str();
      ground_truth_msg.header.stamp = sim_time;
      ground_truth_msg.child_frame_id =
          mapName(param_frame_base_footprint.c_str(), r,
                  static_cast<Stg::Model*>(robotmodel->positionmodel));

      robotmodel->ground_truth_pub->publish(ground_truth_msg);

      if (param_pub_gt_tf)
      {
        tf2::Transform txGt = gt * txOdom.inverse();

        geometry_msgs::msg::TransformStamped tfGt;
        tfGt.header.frame_id = param_frame_gt.c_str();
        tfGt.header.stamp = sim_time;
        tfGt.child_frame_id =
            mapName(param_frame_odom.c_str(), r,
                    static_cast<Stg::Model*>(robotmodel->positionmodel));
        tfGt.transform = tf2::toMsg(txGt);

        tf->sendTransform(tfGt);
      }

      // cameras
      for (size_t s = 0; s < robotmodel->cameramodels.size(); ++s)
      {
        Stg::ModelCamera* cameramodel = robotmodel->cameramodels[s];
        // Get latest image data
        // Translate into ROS message format and publish
        if (robotmodel->image_pubs[s]->get_subscription_count() > 0 &&
            cameramodel->FrameColor())
        {
          sensor_msgs::msg::Image image_msg;

          image_msg.height = cameramodel->getHeight();
          image_msg.width = cameramodel->getWidth();
          image_msg.encoding = "rgba8";
          // this->imageMsgs[r].is_bigendian="";
          image_msg.step = image_msg.width * 4;
          image_msg.data.resize(image_msg.width * image_msg.height * 4);

          memcpy(&(image_msg.data[0]), cameramodel->FrameColor(),
                 image_msg.width * image_msg.height * 4);

          // invert the opengl weirdness
          int height = image_msg.height - 1;
          int linewidth = image_msg.width * 4;

          char* temp = new char[linewidth];
          for (int y = 0; y < (height + 1) / 2; y++)
          {
            memcpy(temp, &image_msg.data[y * linewidth], linewidth);
            memcpy(&(image_msg.data[y * linewidth]),
                   &(image_msg.data[(height - y) * linewidth]), linewidth);
            memcpy(&(image_msg.data[(height - y) * linewidth]), temp,
                   linewidth);
          }

          if (robotmodel->cameramodels.size() > 1)
            image_msg.header.frame_id =
                mapName(param_frame_camera.c_str(), r, s,
                        static_cast<Stg::Model*>(robotmodel->positionmodel));
          else
            image_msg.header.frame_id =
                mapName(param_frame_camera.c_str(), r,
                        static_cast<Stg::Model*>(robotmodel->positionmodel));
          image_msg.header.stamp = sim_time;

          robotmodel->image_pubs[s]->publish(image_msg);
        }

        // Get latest depth data
        // Translate into ROS message format and publish
        // Skip if there are no subscribers
        if (robotmodel->depth_pubs[s]->get_subscription_count() > 0 &&
            cameramodel->FrameDepth())
        {
          sensor_msgs::msg::Image depth_msg;
          depth_msg.height = cameramodel->getHeight();
          depth_msg.width = cameramodel->getWidth();
          depth_msg.encoding = this->isDepthCanonical
                                   ? sensor_msgs::image_encodings::TYPE_32FC1
                                   : sensor_msgs::image_encodings::TYPE_16UC1;
          // this->depthMsgs[r].is_bigendian="";
          int sz = this->isDepthCanonical ? sizeof(float) : sizeof(uint16_t);
          size_t len = depth_msg.width * depth_msg.height;
          depth_msg.step = depth_msg.width * sz;
          depth_msg.data.resize(len * sz);

          // processing data according to REP118
          if (this->isDepthCanonical)
          {
            double nearClip = cameramodel->getCamera().nearClip();
            double farClip = cameramodel->getCamera().farClip();
            memcpy(&(depth_msg.data[0]), cameramodel->FrameDepth(), len * sz);
            float* data = (float*) &(depth_msg.data[0]);
            for (size_t i = 0; i < len; ++i)
              if (data[i] <= nearClip)
                data[i] = -INFINITY;
              else if (data[i] >= farClip)
                data[i] = INFINITY;
          }
          else
          {
            int nearClip = (int) (cameramodel->getCamera().nearClip() * 1000);
            int farClip = (int) (cameramodel->getCamera().farClip() * 1000);
            for (size_t i = 0; i < len; ++i)
            {
              int v = (int) (cameramodel->FrameDepth()[i] * 1000);
              if (v <= nearClip || v >= farClip) v = 0;
              ((uint16_t*) &(depth_msg.data[0]))[i] =
                  (uint16_t) ((v <= nearClip || v >= farClip) ? 0 : v);
            }
          }

          // invert the opengl weirdness
          int height = depth_msg.height - 1;
          int linewidth = depth_msg.width * sz;

          char* temp = new char[linewidth];
          for (int y = 0; y < (height + 1) / 2; y++)
          {
            memcpy(temp, &depth_msg.data[y * linewidth], linewidth);
            memcpy(&(depth_msg.data[y * linewidth]),
                   &(depth_msg.data[(height - y) * linewidth]), linewidth);
            memcpy(&(depth_msg.data[(height - y) * linewidth]), temp,
                   linewidth);
          }

          if (robotmodel->cameramodels.size() > 1)
            depth_msg.header.frame_id =
                mapName(param_frame_camera.c_str(), r, s,
                        static_cast<Stg::Model*>(robotmodel->positionmodel));
          else
            depth_msg.header.frame_id =
                mapName(param_frame_camera.c_str(), r,
                        static_cast<Stg::Model*>(robotmodel->positionmodel));
          depth_msg.header.stamp = sim_time;
          robotmodel->depth_pubs[s]->publish(depth_msg);
        }

        // sending camera's tf and info only if image or depth topics are
        // subscribed to
        if ((robotmodel->image_pubs[s]->get_subscription_count() > 0 &&
             cameramodel->FrameColor()) ||
            (robotmodel->depth_pubs[s]->get_subscription_count() > 0 &&
             cameramodel->FrameDepth()))
        {
          sensor_msgs::msg::CameraInfo camera_msg;
          if (robotmodel->cameramodels.size() > 1)
            camera_msg.header.frame_id =
                mapName(param_frame_camera.c_str(), r, s,
                        static_cast<Stg::Model*>(robotmodel->positionmodel));
          else
            camera_msg.header.frame_id =
                mapName(param_frame_camera.c_str(), r,
                        static_cast<Stg::Model*>(robotmodel->positionmodel));
          camera_msg.header.stamp = sim_time;
          camera_msg.height = cameramodel->getHeight();
          camera_msg.width = cameramodel->getWidth();

          double fx, fy, cx, cy;
          cx = camera_msg.width / 2.0;
          cy = camera_msg.height / 2.0;
          double fovh = cameramodel->getCamera().horizFov() * M_PI / 180.0;
          double fovv = cameramodel->getCamera().vertFov() * M_PI / 180.0;
          // double fx_
          // = 1.43266615300557*this->cameramodels[r]->getWidth()/tan(fovh);
          // double fy_
          // = 1.43266615300557*this->cameramodels[r]->getHeight()/tan(fovv);
          fx = cameramodel->getWidth() / (2 * tan(fovh / 2));
          fy = cameramodel->getHeight() / (2 * tan(fovv / 2));

          // RCLCPP_INFO(this->get_logger(), "fx=%.4f,%.4f; fy=%.4f,%.4f",
          //             fx, fx_, fy, fy_);

          camera_msg.d.resize(4, 0.0);

          camera_msg.k[0] = fx;
          camera_msg.k[2] = cx;
          camera_msg.k[4] = fy;
          camera_msg.k[5] = cy;
          camera_msg.k[8] = 1.0;

          camera_msg.r[0] = 1.0;
          camera_msg.r[4] = 1.0;
          camera_msg.r[8] = 1.0;

          camera_msg.p[0] = fx;
          camera_msg.p[2] = cx;
          camera_msg.p[5] = fy;
          camera_msg.p[6] = cy;
          camera_msg.p[10] = 1.0;

          robotmodel->camera_pubs[s]->publish(camera_msg);
        }
      }
    }

    this->base_last_globalpos_time = this->sim_time;
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = sim_time;
    this->clock_pub_->publish(clock_msg);
  }

  /**
   * @brief Do one update of the world (may pause if the next update time has
   *        not yet arrived)
   * @return true
   * @return false
   */
  bool UpdateWorld() { return this->world->UpdateAll(); }

  /**
   * @brief Message callback for a MsgBaseVel message, which set velocities
   * @param[in] msg cmd_vel twist message (v, vn, w)
   * @param[in] idx robot index
   */
  void cmdvelReceived(const geometry_msgs::msg::Twist::SharedPtr msg, int idx)
  {
    std::scoped_lock lock(msg_lock);
    this->positionmodels[idx]->SetSpeed(msg->linear.x, msg->linear.y,
                                        msg->angular.z);
    this->base_last_cmd = this->sim_time;
  }

  /**
   * @brief Stage reset service (empty service)
   * @param[in] request empty request
   * @param[out] response empty response
   * @return true service finished cleanly
   * @return false otherwise
   */
  bool cb_reset_srv(
      const std_srvs::srv::Empty::Request::SharedPtr /* request */,
      const std_srvs::srv::Empty::Response::SharedPtr /* response */)
  {
    RCLCPP_INFO(this->get_logger(), "Resetting stage!");
    for (size_t r = 0; r < this->positionmodels.size(); r++)
    {
      this->positionmodels[r]->SetPose(this->initial_poses[r]);
      this->positionmodels[r]->SetStall(false);
    }
    return true;
  }

 private:

  static void ghfunc(Stg::Model* mod, StageROS2Node* node)
  {
    // printf( "inspecting %s, parent\n", mod->Token() );

    if (dynamic_cast<Stg::ModelRanger*>(mod))
    {
      node->lasermodels.push_back(dynamic_cast<Stg::ModelRanger*>(mod));
    }
    if (dynamic_cast<Stg::ModelPosition*>(mod))
    {
      Stg::ModelPosition* p = dynamic_cast<Stg::ModelPosition*>(mod);
      // remember initial poses
      node->positionmodels.push_back(p);
      node->initial_poses.push_back(p->GetGlobalPose());
    }
    if (dynamic_cast<Stg::ModelCamera*>(mod))
    {
      node->cameramodels.push_back(dynamic_cast<Stg::ModelCamera*>(mod));
    }
  }

  static bool s_update(Stg::World* /* world */, StageROS2Node* node)
  {
    node->WorldCallback();
    // We return false to indicate that we want to be called again (an
    // odd convention, but that's the way that Stage works).
    return false;
  }

  const char* mapName(const char* name, size_t robotID, Stg::Model* mod) const
  {
    // RCLCPP_INFO(this->get_logger(), "Robot %lu: Device %s", robotID, name);
    bool umn = this->use_model_names;

    if ((positionmodels.size() > 1) || umn)
    {
      static char buf[100];
      std::size_t found =
          std::string(((Stg::Ancestor*) mod)->Token()).find(":");

      if ((found == std::string::npos) && umn)
      {
        snprintf(buf, sizeof(buf), "%s/%s", ((Stg::Ancestor*) mod)->Token(),
                 name);
      }
      else
      {
        snprintf(buf, sizeof(buf), "robot_%u/%s", (unsigned int) robotID, name);
      }

      return buf;
    }
    else
    {
      return name;
    }
  }

  const char* mapName(const char* name, size_t robotID, size_t deviceID,
                      Stg::Model* mod) const
  {
    // RCLCPP_INFO(this->get_logger(), "Robot %lu: Device %s:%lu", robotID,
    //             name, deviceID);
    bool umn = this->use_model_names;

    if ((positionmodels.size() > 1) || umn)
    {
      static char buf[100];
      std::size_t found =
          std::string(((Stg::Ancestor*) mod)->Token()).find(":");

      if ((found == std::string::npos) && umn)
      {
        snprintf(buf, sizeof(buf), "%s/%s_%u", ((Stg::Ancestor*) mod)->Token(),
                 name, (unsigned int) deviceID);
      }
      else
      {
        snprintf(buf, sizeof(buf), "robot_%u/%s_%u", (unsigned int) robotID,
                 name, (unsigned int) deviceID);
      }

      return buf;
    }
    else
    {
      static char buf[100];
      snprintf(buf, sizeof(buf), "%s_%u", name, (unsigned int) deviceID);
      return buf;
    }
  }
};

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    puts(USAGE);
    exit(-1);
  }

  rclcpp::init(argc, argv);

  bool gui = true;
  bool use_model_names = false;
  bool start_paused = false;

  for (int i = 0; i < (argc - 1); i++)
  {
    if (!strcmp(argv[i], "-g")) gui = false;
    if (!strcmp(argv[i], "-u")) use_model_names = true;
    if (!strcmp(argv[i], "-p")) start_paused = true;
  }

  std::shared_ptr<StageROS2Node> stageros2 = std::make_shared<StageROS2Node>(
      argc - 1, argv, gui, argv[argc - 1], use_model_names);

  if (stageros2->SubscribeModels() != 0) exit(-1);

  std::thread t([&, stageros2]() { rclcpp::spin(stageros2); });

  if (start_paused)
  {
    stageros2->world->Stop();
  }
  else
  {
    stageros2->world->Start();
  }

  Stg::World::Run();

  t.join();

  exit(0);
}
