#include <unordered_map>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "unitree_nav_interfaces/srv/set_body_rpy.hpp"

//TODO convert to SMACC if I get the time
enum class State
{
    IDLE
  ,SWEEPING_YAW
  ,SETTING_RPY
};

//TODO find a better way of doing this
std::unordered_map<State, std::string> STATE_NAMES;

void init_enums() {
  STATE_NAMES[State::IDLE] = "IDLE";
  STATE_NAMES[State::SWEEPING_YAW] = "SWEEPING_YAW";
  STATE_NAMES[State::SETTING_RPY] = "SETTING_RPY";
}

class Sandbox : public rclcpp::Node
{
public:
  Sandbox()
  : Node("sandbox")
  {

    //Timers
    timer_ = create_wall_timer(
      static_cast<std::chrono::milliseconds>(static_cast<int>(interval_ * 1000.0)), 
      std::bind(&Sandbox::timer_callback, this)
    );

    RCLCPP_INFO_STREAM(get_logger(), "sandbox node started");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;

  State state_ = State::IDLE;
  State state_last_ = state_;
  State state_next_ = state_;
  double rate_ = 100.0; //Hz
  double interval_ = 1.0 / rate_; //seconds

  void timer_callback() {
      
    auto new_state = state_ != state_last_;

    if (new_state) {
      RCLCPP_INFO_STREAM(get_logger(), "Sandbox state changed to " << STATE_NAMES[state_]);

      state_last_ = state_;
    }


    //State machine here



    state_ = state_next_;
  }
};

int main(int argc, char** argv)
{
  init_enums();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sandbox>());
  rclcpp::shutdown();
  return 0;
}