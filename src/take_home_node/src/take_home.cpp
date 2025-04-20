#include "take_home_node/take_home.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <cmath>
#include <deque>
#include <chrono>
using namespace std::chrono_literals;

TakeHome::TakeHome(const rclcpp::NodeOptions& options)
    : Node("take_home_metrics", options) {

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    // Look at the hpp file to define all class variables, including subscribers
    // A subscriber will "listen" to a topic and whenever a message is published to it, the subscriber
    // will pass it onto the attached callback (`TakeHome::odometry_callback` in this instance)
    
    //Odometry subscriber for PT A
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/vehicle/uva_odometry", qos_profile,
      std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1));
    //Top IMU subscriber for PT B
    imu_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "/novatel_top/rawimu", qos_profile,
      std::bind(&TakeHome::imu_callback, this, std::placeholders::_1));
    //Curvilinear distance subscriber for pt C
    curvilinear_distance_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "/curvilinear_distance", qos_profile,
        std::bind(&TakeHome::curvilinear_distance_callback, this, std::placeholders::_1));
    wheel_speed_subscriber_ = create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>("/raptor_dbw_interface/wheel_speed_report", qos_profile,
    std::bind(&TakeHome::wheel_speed_callback, this, std::placeholders::_1));
    steering_subscriber_    = create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>("/raptor_dbw_interface/steering_extended_report", qos_profile,
    std::bind(&TakeHome::steering_callback, this, std::placeholders::_1));

      
      //Publishers 
      metric_publisher_ = this->create_publisher<std_msgs::msg::Float32>("metrics_output", qos_profile);
      slip_long_rr_pub_ = this->create_publisher<std_msgs::msg::Float32>("/slip/long/rr", qos_profile);
      slip_long_rl_pub_ = this->create_publisher<std_msgs::msg::Float32>("/slip/long/rl", qos_profile);
      slip_long_fl_pub_ = this->create_publisher<std_msgs::msg::Float32>("/slip/long/fl", qos_profile);
      slip_long_fr_pub_ = this->create_publisher<std_msgs::msg::Float32>("/slip/long/fr", qos_profile);
      jitter_pub_ = this->create_publisher<std_msgs::msg::Float32>("/imu_top/jitter", qos_profile);
      lap_time_pub_ = this->create_publisher<std_msgs::msg::Float32>("/lap_time", qos_profile);
      
      //Timer set for .1 sec (change maybe if it still seeing noise)
      timer_ = create_wall_timer(100ms, std::bind(&TakeHome::on_timer, this));

}

// 
/**
 * Whenever a message is published to the topic "vehicle/uva_odometry" the subscriber passes the message onto this callback
 * To see what is in each message look for the corresponding .msg file in this repository
 * For instance, when running `ros2 bag info` on the given bag, we see the wheel speed report has message type of raptor_dbw_msgs/msgs/WheelSpeedReport
 * and from the corresponding WheelSpeedReport.msg file we see that we can do msg->front_left for the front left speed for instance.
 * For the built in ROS2 messages, we can find the documentation online: e.g. https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
 */

 //Get most recent odometry data 
 void TakeHome::odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lk(mtx_data_);
  odom_msg_ = msg;
} //dont touch

//Get most recent wheel speed data 
void TakeHome::wheel_speed_callback(const raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr msg){
  std::lock_guard<std::mutex> lk(mtx_data_);
  wheel_speed_msg_ = msg;
} //dont touch

//Get most recent steering data
void TakeHome::steering_callback(const raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr msg){
  std::lock_guard<std::mutex> lk(mtx_data_);
  steering_msg_ = msg;
} //dont touch

//Get most recent curvilinear info
void TakeHome::curvilinear_distance_callback(const std_msgs::msg::Float32::ConstSharedPtr msg){
  //Get info from topic
  std::lock_guard<std::mutex> lk(mtx_data_);
  curv_dist_msg_ = msg;
}

//IMU store timing info
void TakeHome::imu_callback(const novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lk(mtx_data_);
  //Convert to sec and nano
  double t = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
  //Calculate difference  
  if (last_imu_time_ >= 0.0) {
    imu_queue_.push_front({t - last_imu_time_, last_imu_time_});
  }
  last_imu_time_ = t;
}

//Compute on timer
void TakeHome::on_timer(){
  //Slip Ratios
  //Sanity Check
  {
  std::lock_guard<std::mutex> lk(mtx_data_);
  if (!odom_msg_ || !wheel_speed_msg_ || !steering_msg_) {
    return;  
  }
  //Get odometry data 
  double vx  = odom_msg_->twist.twist.linear.x;
  double vy  = odom_msg_->twist.twist.linear.y;
  double wz  = odom_msg_->twist.twist.angular.z;

  //km/h to m/s
  const float k2m = 1.0f/3.6f;
  float wrr = wheel_speed_msg_->rear_right * k2m;
  float wrl = wheel_speed_msg_->rear_left * k2m;
  float wfr = wheel_speed_msg_->front_right * k2m;
  float wfl = wheel_speed_msg_->front_left * k2m;

  // steering angle to radians
  float delta = (steering_msg_->primary_steering_angle_fbk / 15.0f) * static_cast<float>(M_PI/180.0);

  // Rear right
  float vrr = vx - 0.5f * wz * 1.523f;
  float k_rr = safe_slip(wrr, vrr);

  // Rear left
  float vrl = vx + 0.5f * wz * 1.523f;
  float k_rl = safe_slip(wrl, vrl);

  // Front right
  float vfr_x = vx - 0.5f * wz * 1.638f;
  float vfr_y = vy + wz * 1.7238f;
  float vfr_d = std::cos(delta)*vfr_x - std::sin(delta)*vfr_y;
  float k_fr = safe_slip(wfr, vfr_d);

  // Front left
  float vfl_x = vx + 0.5f * wz * 1.638f;
  float vfl_y = vy + wz * 1.7238f;
  float vfl_d = std::cos(delta)*vfl_x - std::sin(delta)*vfl_y;
  float k_fl  = safe_slip(wfl, vfl_d);

  std_msgs::msg::Float32 out;
  out.data = k_rr; 
  slip_long_rr_pub_->publish(out);
  out.data = k_rl; 
  slip_long_rl_pub_->publish(out);
  out.data = k_fr; 
  slip_long_fr_pub_->publish(out);
  out.data = k_fl; 
  slip_long_fl_pub_->publish(out);
  }
  //Jitter
  {
  std::lock_guard<std::mutex> lk(mtx_data_);
  //Only keep new data 
  while (!imu_queue_.empty() && (last_imu_time_ - imu_queue_.back().second > 1.0)){
     imu_queue_.pop_back();
  }
  double var = compute_variance(imu_queue_);
  if (var >= 0.0) {
    std_msgs::msg::Float32 msg;
    msg.data = var;
    jitter_pub_->publish(msg);
  }
}

  //Lap Time
  {
  std::lock_guard<std::mutex> lk(mtx_data_);
  //Sanity Check
  if (!curv_dist_msg_){
    return;
  }
  float dist = curv_dist_msg_->data;
  bool start = last_distance_ > 0.0f;
  bool lapped = (start) && (dist < last_distance_);
  last_distance_ = dist;
  //When you start lap
  if(lapped && !first_zero_) {
    first_zero_ = true;
    lap_start_ = this->now().seconds();
  } else if(lapped && first_zero_){
    float stop = this->now().seconds();
    float lap_time = std::fabs(static_cast<float>(stop - lap_start_));
    std_msgs::msg::Float32 msg;
    msg.data = lap_time;
    lap_time_pub_->publish(msg);
    //Reset
    lap_start_ = this->now().seconds();
    first_zero_ = false;
  }
}
}
float TakeHome::safe_slip(float wheel_ms, float v_x){
  //Sanity Check
  if(std::fabs(v_x) < 1e-6f){
    return 0.0;
  }
  float compute = (wheel_ms - v_x) / v_x;
  if(std::isnan(compute) || std::isinf(compute)){
    return wheel_ms - v_x;
  }
  return compute;
}

//Variance for Jitter
double TakeHome::compute_variance(const std::deque<std::pair<double,double>> &q){
  int size = q.size();
  //Sanity Check
  if(size < 2){
    return -1;
  }
  std::vector<double> vals; 
  vals.reserve(size);
  double sum = 0.0;
  //Get first element from the deque and calculate sum
  for (auto &p : q) {
    vals.push_back(p.first);
    sum += p.first;
  }
  double mean = sum / size;
  double var = 0.0;
  //Sum of squares 
  for (double v : vals) {
    double diff = v - mean;
    var += diff*diff;
  }
  return var / (size - 1);
}
RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome);

