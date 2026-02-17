#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sstream>
#include <string>
#include <cmath>
#include <memory>

class EncoderOdomNode : public rclcpp::Node
{
public:
  EncoderOdomNode()
  : Node("encoder_odom_node"),
    fd_(-1), 
    port_(""),
    buffer_(""),
    frame_id_("odom"),
    child_frame_id_("base_link"),
    publish_tf_(true),
    first_read_(true),
    x_(0.0), y_(0.0), theta_(0.0),
    last_left_counts_(0), last_right_counts_(0)
  {
    // ================= PARAMETERS =================
    this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    this->declare_parameter<int>("baud", 115200);
    this->declare_parameter<double>("wheel_radius", 0.075);
    this->declare_parameter<double>("wheel_base",   0.60);
    this->declare_parameter<int>("ticks_per_rev_left",  500);
    this->declare_parameter<int>("ticks_per_rev_right", 826);
    this->declare_parameter<std::string>("frame_id", "odom");
    this->declare_parameter<std::string>("child_frame_id", "base_link");
    this->declare_parameter<bool>("publish_tf", true);

    port_  = this->get_parameter("port").as_string();
    baud_  = this->get_parameter("baud").as_int();
    R_     = this->get_parameter("wheel_radius").as_double();
    L_     = this->get_parameter("wheel_base").as_double();
    ticks_per_rev_left_  = this->get_parameter("ticks_per_rev_left").as_int();
    ticks_per_rev_right_ = this->get_parameter("ticks_per_rev_right").as_int();
    frame_id_       = this->get_parameter("frame_id").as_string();
    child_frame_id_ = this->get_parameter("child_frame_id").as_string();
    publish_tf_     = this->get_parameter("publish_tf").as_bool();

    if (!openSerial()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open serial port %s", port_.c_str());
      throw std::runtime_error("Cannot open serial port");
    }

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    imu_pub_  = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&EncoderOdomNode::cmdVelCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&EncoderOdomNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Driver started. Port: %s, Frame: %s", port_.c_str(), frame_id_.c_str());
  }

  ~EncoderOdomNode() override { if (fd_ >= 0) close(fd_); }

private:
  bool openSerial() {
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;
    struct termios tio;
    if (tcgetattr(fd_, &tio) != 0) return false;
    cfmakeraw(&tio);
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;
    return tcsetattr(fd_, TCSANOW, &tio) == 0;
  }

  void sendSerialCommand(std::string cmd) {
    cmd += "\n";
    write(fd_, cmd.c_str(), cmd.size());
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    int speed = static_cast<int>(msg->linear.x * 400.0); 
    int steer = static_cast<int>(msg->angular.z * 200.0);
    sendSerialCommand(std::to_string(speed) + " " + std::to_string(steer));
  }

  bool readLine(std::string &line) {
    char buf[256];
    ssize_t n = read(fd_, buf, sizeof(buf));
    if (n <= 0) return false;
    buffer_.append(buf, n);
    auto pos = buffer_.find('\n');
    if (pos == std::string::npos) return false;
    line = buffer_.substr(0, pos);
    buffer_.erase(0, pos + 1);
    return true;
  }

  void timerCallback() {
    std::string line;
    while (readLine(line)) {
      if (line.rfind("ENC", 0) == 0) processEncoderLine(line);
      else if (line.rfind("IMU", 0) == 0) processImuLine(line);
    }
  }

  void processImuLine(const std::string &line) {
    std::istringstream iss(line);
    std::string tag;
    double ax, ay, az, gx, gy, gz;
    if (!(iss >> tag >> ax >> ay >> az >> gx >> gy >> gz)) return;

    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "imu_link";
    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;
    imu_msg.angular_velocity.x = gx;
    imu_msg.angular_velocity.y = gy;
    imu_msg.angular_velocity.z = gz;
    imu_msg.orientation_covariance[0] = -1;
    imu_pub_->publish(imu_msg);
  }

  void processEncoderLine(const std::string &line) {
    std::istringstream iss(line);
    std::string tag;
    long left_counts, right_counts;
    if (!(iss >> tag >> left_counts >> right_counts)) return;

    rclcpp::Time current_time = this->now();
    if (first_read_) {
      last_left_counts_ = left_counts; 
      last_right_counts_ = right_counts;
      last_time_ = current_time; 
      first_read_ = false; 
      return;
    }

    double dt = (current_time - last_time_).seconds();
    if (dt <= 0.0) return;
    // أضف إشارة السالب (-) قبل القوس لعكس اتجاه العجلة اليسرى برمجياً
     double dist_L = -((double)(left_counts - last_left_counts_) / ticks_per_rev_left_) * 2.0 * M_PI * R_;
     double dist_R = ((double)(right_counts - last_right_counts_) / ticks_per_rev_right_) * 2.0 * M_PI * R_;
    last_left_counts_ = left_counts; 
    last_right_counts_ = right_counts;
    last_time_ = current_time;

    double v = (dist_R + dist_L) / (2.0 * dt);
    double omega = (dist_R - dist_L) / (L_ * dt);
    theta_ += omega * dt;
    x_ += v * cos(theta_) * dt;
    y_ += v * sin(theta_) * dt;

    publishOdom(current_time, v, omega);
  }

  void publishOdom(const rclcpp::Time &stamp, double v, double omega) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = stamp;
      tf_msg.header.frame_id = frame_id_;
      tf_msg.child_frame_id = child_frame_id_;
      tf_msg.transform.translation.x = x_;
      tf_msg.transform.translation.y = y_;
      tf_msg.transform.rotation.x = q.x();
      tf_msg.transform.rotation.y = q.y();
      tf_msg.transform.rotation.z = q.z();
      tf_msg.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(tf_msg);
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = omega;
    odom_pub_->publish(odom);
  }

  // --- Member Variables (Reordered to fix warnings) ---
  int fd_;
  std::string port_;
  std::string buffer_;
  std::string frame_id_;
  std::string child_frame_id_;
  bool publish_tf_;
  bool first_read_;
  int baud_;
  double R_, L_;
  int ticks_per_rev_left_, ticks_per_rev_right_;
  double x_, y_, theta_;
  long last_left_counts_, last_right_counts_;
  rclcpp::Time last_time_;
  
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncoderOdomNode>());
  rclcpp::shutdown();
  return 0;
}