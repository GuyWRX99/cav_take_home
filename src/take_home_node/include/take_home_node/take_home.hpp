#pragma once
#include <mutex>                                  
#include <deque>                                 
#include <utility>
#include <cmath>
#include <cfloat>                                
#include <rclcpp/rclcpp.hpp>                      
#include <std_msgs/msg/float32.hpp>               
#include <std_msgs/msg/float64.hpp>              
#include <nav_msgs/msg/odometry.hpp>             
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>   
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>
#include <novatel_oem7_msgs/msg/rawimu.hpp>

class TakeHome : public rclcpp::Node {
 public:
  TakeHome(const rclcpp::NodeOptions& options);


 private:

  //Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr metric_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_long_rr_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_long_rl_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_long_fr_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_long_fl_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr jitter_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lap_time_pub_;

  //Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheel_speed_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr steering_subscriber_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr curvilinear_distance_subscriber_;

  //Recent data
  nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_;
  raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_speed_msg_;
  raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg_;
  std_msgs::msg::Float32::ConstSharedPtr curv_dist_msg_; 

  //Functions
  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void wheel_speed_callback(const raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr msg);
  void steering_callback(const raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr msg);
  void imu_callback(const novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg);
  void curvilinear_distance_callback(const std_msgs::msg::Float32::ConstSharedPtr msg);
  float safe_slip(float wheel_ms, float v_x);
  double compute_variance(const std::deque<std::pair<double,double>> &q);
  void on_timer();

  //Mutexes
  std::mutex mtx_data_, mtx_imu_;
  //IMU jitter data 
  std::deque<std::pair<double,double>> imu_queue_;
  double last_imu_time_ = -1.0;

  //Timer
  rclcpp::TimerBase::SharedPtr timer_;
  //Store multiple time stamps
  std::deque<double> timestamps_;

  //Ensuring data is received
  bool have_odom_ = false;
  bool have_wheel_speed_ = false;
  bool have_steering_ = false;
  //Lap time
  bool first_zero_ = false;
  double lap_start_ = 0.0;
  double last_distance_ = 0.0;
};

