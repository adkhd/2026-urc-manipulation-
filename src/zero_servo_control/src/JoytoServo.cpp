#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <thread>
#include <vector>
#include <map>
#include <string>
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// 토픽 설정
const std::string JOY_TOPIC = "/joy";
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string EEF_FRAME_ID = "link5";
const std::string BASE_FRAME_ID = "link0";

// ==========================================
// [스케일 계산 로직]
// MoveIt Servo 설정(yaml) 기준값 (가정)
// linear_scale: 0.1 (Joystick 1.0 -> 0.1 m/s)
// rotational_scale: 1.5 (Joystick 1.0 -> 1.5 rad/s)
const double REF_VEL_LIN = 0.1; 
const double REF_VEL_ROT = 1.5;

// 우리가 원하는 목표 최대 속도
const double TARGET_VEL_J1_Y = 0.03;   // m/s (J1 및 Twist Y축)
const double TARGET_VEL_LIN_XZ = 0.1;  // m/s (Twist X, Z축)
const double TARGET_VEL_ROT_STD = 1.0; // rad/s (J2, J3, J4)
// J5는 그대로 (1.0 * REF_VEL_ROT = 1.5 rad/s)

// 최종 적용될 Base Multiplier (Joystick Input 1.0에 곱해질 값)
const double BASE_SCALE_Y  = TARGET_VEL_J1_Y / REF_VEL_LIN;    // 0.03 / 0.1 = 0.3
const double BASE_SCALE_J1  = TARGET_VEL_J1_Y / REF_VEL_ROT ;    // 0.03 / 0.1 = 0.3
const double BASE_SCALE_XZ    = TARGET_VEL_LIN_XZ / REF_VEL_LIN;  // 0.1 / 0.1 = 1.0
const double BASE_SCALE_ROT   = TARGET_VEL_ROT_STD / REF_VEL_ROT; // 1.0 / 1.5 = 0.666...
const double BASE_SCALE_J5    = 1.0;                              // 그대로
// ==========================================

// 속도 프리셋 (20% ~ 100%)
const std::vector<double> SPEED_PRESETS = {0.2, 0.4, 0.6, 0.8, 1.0};

enum Axis {
  LEFT_STICK_X = 0, LEFT_STICK_Y = 1, LEFT_TRIGGER = 2,
  RIGHT_STICK_X = 3, RIGHT_STICK_Y = 4, RIGHT_TRIGGER = 5,
  D_PAD_X = 6, D_PAD_Y = 7
};

enum Button {
  A = 0, B = 1, X = 2, Y = 3,
  LEFT_BUMPER = 4, RIGHT_BUMPER = 5,
  CHANGE_VIEW = 6, MENU = 7, HOME = 8,
  LEFT_STICK_CLICK = 9, RIGHT_STICK_CLICK = 10
};

std::map<Axis, double> AXIS_DEFAULTS = { 
  { LEFT_TRIGGER, 1.0 }, 
  { RIGHT_TRIGGER, 1.0 } 
};

namespace zero_servo
{
class JoyToServoPub : public rclcpp::Node
{
public:
  JoyToServoPub(const rclcpp::NodeOptions& options)
    : Node("joy_to_twist_publisher", options), 
      frame_to_publish_(BASE_FRAME_ID),
      speed_level_index_(4),  // 기본 100% (Index 4)
      e_stop_active_(false),
      last_menu_button_(false),
      last_speed_button_(false),
      last_home_button_(false),
      last_solenoid_button_(false),
      gripper_mode_(false),
      gripper_norm_(0.5),
      serial_fd_(-1)
  {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) { 
          return joyCB(msg); 
        });

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        TWIST_TOPIC, rclcpp::SystemDefaultsQoS());
    
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
        JOINT_TOPIC, rclcpp::SystemDefaultsQoS());
    
    // 그리퍼 시리얼 초기화 (포트가 없어도 노드가 죽지 않도록 예외처리 필요)
    initSerial("/dev/arduino_mega");

    // 초기 스케일 설정
    updateScales();
    
    RCLCPP_INFO(this->get_logger(), 
                "Joy Node Started. Speed Level: %.0f%%", 
                SPEED_PRESETS[speed_level_index_] * 100.0);
  }

  ~JoyToServoPub() override {
      if (serial_fd_ >= 0) close(serial_fd_);
  }

private:
  void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
  {
    // 버튼/축 크기 검증
    if (msg->buttons.size() < 11 || msg->axes.size() < 8) return;

    // 1. E-STOP 토글 (MENU 버튼)
    bool menu_btn = (msg->buttons[MENU] != 0);
    if (menu_btn && !last_menu_button_) {
        e_stop_active_ = !e_stop_active_;
        if (e_stop_active_) {
            RCLCPP_WARN(this->get_logger(), "!!! EMERGENCY STOP ACTIVATED !!!");
        } else {
            RCLCPP_INFO(this->get_logger(), "E-STOP Released.");
        }
    }
    last_menu_button_ = menu_btn;

    // E-STOP 상태면 0 명령 전송
    if (e_stop_active_) {
        auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();
        joint_msg->header.stamp = this->now();
        joint_msg->joint_names = {"Joint1", "Joint2", "Joint3", "Joint4", "Joint5"};
        joint_msg->velocities.resize(5, 0.0);
        joint_pub_->publish(std::move(joint_msg));
        return; 
    }

    // 2. 그리퍼 모드 토글 (HOME 버튼)
    bool home_btn = (msg->buttons[HOME] != 0);
    if (home_btn && !last_home_button_) {
        gripper_mode_ = !gripper_mode_;
        RCLCPP_INFO(this->get_logger(), "Gripper Mode: %s", gripper_mode_ ? "ON" : "OFF");
    }
    last_home_button_ = home_btn;

    if (gripper_mode_) {
        processGripperJoy(msg);
        return; // 그리퍼 모드일 때는 팔 움직임 차단
    }

    // 3. 속도 레벨 순환 (LEFT_STICK_CLICK)
    bool speed_btn = (msg->buttons[LEFT_STICK_CLICK] != 0);
    if (speed_btn && !last_speed_button_) {
        speed_level_index_ = (speed_level_index_ + 1) % SPEED_PRESETS.size();
        updateScales();
        RCLCPP_INFO(this->get_logger(), 
                    "Speed Level: %.0f%%", 
                    SPEED_PRESETS[speed_level_index_] * 100.0);
    }
    last_speed_button_ = speed_btn;

    // 4. 프레임 변경 (CHANGE_VIEW)
    if (msg->buttons[CHANGE_VIEW]) {
        if (frame_to_publish_ == EEF_FRAME_ID) frame_to_publish_ = BASE_FRAME_ID;
        else frame_to_publish_ = EEF_FRAME_ID;
    }

    // 5. 명령 변환 및 발행
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg)) {
        // Twist 발행
        twist_msg->header.frame_id = frame_to_publish_;
        twist_msg->header.stamp = this->now();
        twist_pub_->publish(std::move(twist_msg));
    } else {
        // Joint 발행
        joint_msg->header.stamp = this->now();
        joint_msg->header.frame_id = BASE_FRAME_ID;
        joint_pub_->publish(std::move(joint_msg));
    }
  }

  void updateScales() {
      double multiplier = SPEED_PRESETS[speed_level_index_];
      
      // J1 및 Twist Y축: (0.03 / 0.1) * 프리셋
      current_scale_j1_ = BASE_SCALE_J1 * multiplier;
      current_scale_y_ = BASE_SCALE_Y * multiplier;
      // Twist X, Z축: (0.1 / 0.1) * 프리셋
      current_scale_xz_ = BASE_SCALE_XZ * multiplier;
      
      // J2, J3, J4: (1.0 / 1.5) * 프리셋
      current_scale_rot_ = BASE_SCALE_ROT * multiplier;
      
      // J5: 1.0 * 프리셋
      current_scale_j5_ = BASE_SCALE_J5 * multiplier;
  }

  bool convertJoyToCmd(
      const std::vector<float>& axes, 
      const std::vector<int>& buttons,
      std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
      std::unique_ptr<control_msgs::msg::JointJog>& joint)
  {
      // Joint Control (Buttons / D-Pad / Bumpers)
      bool joint_mode = (buttons[A] || buttons[B] || buttons[X] || buttons[Y] || 
                        std::abs(axes[D_PAD_X]) > 0.1 || std::abs(axes[D_PAD_Y]) > 0.1 ||
                        buttons[LEFT_BUMPER] || buttons[RIGHT_BUMPER]);
      
      if (joint_mode) {
        // Joint 1 (Linear) - D-Pad Y
        // 0.03 m/s 제한 적용
        joint->joint_names.push_back("Joint1");
        joint->velocities.push_back(axes[D_PAD_Y] * current_scale_j1_);

        // Joint 2 - D-Pad X
        // 1.0 rad/s 제한 적용
        joint->joint_names.push_back("Joint2");
        joint->velocities.push_back(axes[D_PAD_X] * current_scale_rot_);

        // Joint 3 (LB/RB)
        // 1.0 rad/s 제한 적용
        joint->joint_names.push_back("Joint3");
        double j3_vel = (buttons[RIGHT_BUMPER] - buttons[LEFT_BUMPER]) * current_scale_rot_;
        joint->velocities.push_back(j3_vel);

        // Joint 4 (Y/A)
        // 1.0 rad/s 제한 적용
        joint->joint_names.push_back("Joint4");
        joint->velocities.push_back((buttons[Y] - buttons[A]) * current_scale_rot_);

        // Joint 5 (B/X)
        // 1.5 rad/s (그대로) 적용
        joint->joint_names.push_back("Joint5");
        joint->velocities.push_back((buttons[B] - buttons[X]) * current_scale_j5_);
        
        return false; // JointJog 모드
      }

      // Cartesian Control (Sticks) - Twist 모드
      
      // Z축 (수직): Right Stick Y -> 0.1 m/s 기준
      twist->twist.linear.z = axes[RIGHT_STICK_Y] * current_scale_xz_;
      
      // [중요] Y축 (좌우, J1 대응): Right Stick X -> 0.03 m/s 기준
      twist->twist.linear.y = axes[RIGHT_STICK_X] * current_scale_y_;

      // X축 (전후): Triggers -> 0.1 m/s 기준
      double lin_x_right = -0.5 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
      double lin_x_left = 0.5 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS.at(LEFT_TRIGGER));
      twist->twist.linear.x = (lin_x_right + lin_x_left) * current_scale_xz_;

      return true; // Twist 모드
  }
  int last_sent_gripper_val_ = -1; // 초기값 -1
  // --- 그리퍼/시리얼 관련 함수 (기존 코드 유지) ---
  void processGripperJoy(const sensor_msgs::msg::Joy::ConstSharedPtr& msg) {
      if (serial_fd_ < 0) return;
      double axis = msg->axes[RIGHT_STICK_Y];
      if (axis < -1.0) axis = -1.0;
      if (axis >  1.0) axis =  1.0;
      const double max_speed = 0.05;
      gripper_norm_ += axis * max_speed;
      if (gripper_norm_ < 0.0) gripper_norm_ = 0.0;
      if (gripper_norm_ > 1.0) gripper_norm_ = 1.0;
      
      int current_val = static_cast<int>(gripper_norm_ * 1000.0 + 0.5);
    
    // 너무 자주 보내지 않게 + 값이 바뀌었을 때만 전송
     if (current_val != last_sent_gripper_val_) {
        sendGripperPosition(gripper_norm_);
        last_sent_gripper_val_ = current_val;
        RCLCPP_INFO(this->get_logger(), "Sent G: %d", current_val); 
    }
      bool solenoid_btn = (msg->buttons[RIGHT_BUMPER] != 0);
      if (solenoid_btn && !last_solenoid_button_) sendSolenoid();
      last_solenoid_button_ = solenoid_btn;
  }

  void sendGripperPosition(double normalized) {
      if (serial_fd_ < 0) return;
      int value = static_cast<int>(normalized * 1000.0 + 0.5);
      char buf[32];
      int len = snprintf(buf, sizeof(buf), "G %d\n", value);
      if (len > 0) { ssize_t w = write(serial_fd_, buf, len); (void)w; }
  }

  void sendSolenoid() {
      if (serial_fd_ < 0) return;
      const char* cmd = "S\n";
      ssize_t w = write(serial_fd_, cmd, 2); (void)w;
  }

  void initSerial(const std::string& port_name) {
      serial_fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (serial_fd_ < 0) {
        // [추가] 연결 실패 시 에러 로그 띄우기
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_name.c_str());
        return; 
    }
      termios tty{};
      if (tcgetattr(serial_fd_, &tty) != 0) { close(serial_fd_); serial_fd_ = -1; return; }
      cfsetospeed(&tty, B115200); cfsetispeed(&tty, B115200);
      tty.c_cflag |= (CLOCAL | CREAD); tty.c_cflag &= ~CSIZE; tty.c_cflag |= CS8;
      tty.c_cflag &= ~PARENB; tty.c_cflag &= ~CSTOPB; tty.c_cflag &= ~CRTSCTS;
      tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); tty.c_iflag &= ~(IXON | IXOFF | IXANY);
      tty.c_oflag &= ~OPOST;
      tty.c_cc[VMIN]  = 0; tty.c_cc[VTIME] = 1;
      tcsetattr(serial_fd_, TCSANOW, &tty);
  }

  // 멤버 변수
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  
  std::string frame_to_publish_;
  size_t speed_level_index_;
  
  // 현재 계산된 스케일들
  double current_scale_j1_; // J1 & Twist Y
  double current_scale_y_; // J1 & Twist Y
  double current_scale_rot_;  // J2, J3, J4
  double current_scale_j5_;   // J5
  double current_scale_xz_;   // Twist X, Z
  
  bool e_stop_active_;
  bool last_menu_button_;
  bool last_speed_button_;
  bool last_home_button_;
  bool last_solenoid_button_;
  bool gripper_mode_;
  double gripper_norm_;
  int serial_fd_;
};

} // namespace zero_servo

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(zero_servo::JoyToServoPub)