#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class Patrol final : public rclcpp::Node {
public:
  explicit Patrol() : Node("patrol_node") {
    laser_scan_subscriber =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&Patrol::scan_callback, this, std::placeholders::_1));

    cmd_vel_publisher =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    control_loop_timer = this->create_wall_timer(
        100ms, std::bind(&Patrol::control_loop_timer_callback, this));

    cmd_vel_message.linear.x = 0.1;
  }

private:
  float calculate_distance(std::vector<float> const &v, int m, int n) {

    std::vector<float> vec(v.cbegin() + m, v.cbegin() + n + 1);

    float minimum_distance_in_range = MAXFLOAT;

    for (auto i : vec) {
      if (!std::isinf(i) && i < minimum_distance_in_range) {
        minimum_distance_in_range = i;
      }
    }

    return minimum_distance_in_range;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    float max_distance = 0.0;
    float max_distance_angle = 0.0;

    int i = 0;

    for (std::vector<float>::iterator it = msg->ranges.begin();
         it != msg->ranges.end(); it++, i++) {

      float radians = ((i - 360) / 2.0 * M_PI) / 180;

      if (radians > -M_PI_2 && radians < M_PI_2) {
        float distance = calculate_distance(msg->ranges, i - 50, i + 50);

        if (distance > max_distance) {
          max_distance = distance;
          max_distance_angle = radians;
        }
      }
    }

    direction_ = max_distance_angle;
  }

  void control_loop_timer_callback() {
    cmd_vel_message.angular.z = direction_ / 2;
    cmd_vel_publisher->publish(cmd_vel_message);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_subscriber;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;

  rclcpp::TimerBase::SharedPtr control_loop_timer;

  geometry_msgs::msg::Twist cmd_vel_message;

  float direction_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}
