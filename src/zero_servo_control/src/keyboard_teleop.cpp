#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <termios.h> // POSIX 터미널 제어 API
#include <unistd.h>  // UNIX 표준 함수

// 터미널 입력을 위한 함수
int getch()
{
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  int c = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return c;
}

class TeleopNode : public rclcpp::Node
{
public:
  TeleopNode() : Node("keyboard_teleop_node"), linear_speed_(0.3), angular_speed_(0.1)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 10);
    
    RCLCPP_INFO(this->get_logger(), "Keyboard Teleop Node Started.");
    RCLCPP_INFO(this->get_logger(), "Use 'w, a, s, d' to move, 'q, e' to rotate. 'x' to stop.");

    // 별도의 스레드에서 키 입력을 계속 감지
    input_thread_ = std::thread([this]() {
      while (rclcpp::ok()) {
        char c = getch();
        handle_key_input(c);
      }
    });
  }

private:
  void handle_key_input(char c)
  {
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    twist_msg->header.stamp = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "%d, %u", twist_msg->header.stamp.sec, twist_msg->header.stamp.nanosec);
    twist_msg->header.frame_id = "link1"; // servo.yaml의 planning_frame과 일치

    switch (c)
    {
      case 'w': twist_msg->twist.linear.x = -linear_speed_; break;
      case 's': twist_msg->twist.linear.x = linear_speed_; break;
      case 'a': twist_msg->twist.linear.y = -linear_speed_; break;
      case 'd': twist_msg->twist.linear.y = linear_speed_; break;
      case 'q': twist_msg->twist.angular.z = angular_speed_; break;
      case 'e': twist_msg->twist.angular.z = -angular_speed_; break;
      case 'f': twist_msg->twist.linear.z = linear_speed_; break;
      case 'g': twist_msg->twist.linear.z = -linear_speed_; break;
      case 'x': // 정지 명령
        break;
    }
    publisher_->publish(std::move(twist_msg));
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  std::thread input_thread_;
  double linear_speed_;
  double angular_speed_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopNode>());
  rclcpp::shutdown();
  return 0;
}
