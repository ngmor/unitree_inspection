#include <string>
#include <future>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "unitree_nav_interfaces/srv/set_body_rpy.hpp"
#include "unitree_ocr_interfaces/msg/detection.hpp"
#include "unitree_ocr_interfaces/msg/detections.hpp"

using namespace std::chrono_literals;

//https://stackoverflow.com/questions/11714325/how-to-get-enum-item-name-from-its-value
#define STATES \
X(STARTUP, "STARTUP") \
X(SETTING_RPY_START, "SETTING_RPY_START") \
X(SETTING_RPY_WAIT, "SETTING_RPY_WAIT") \

#define X(state, name) state,
enum class State : size_t {STATES};
#undef X

#define X(state, name) name,
std::vector<std::string> STATE_NAMES = {STATES};
#undef X

//https://stackoverflow.com/questions/11421432/how-can-i-output-the-value-of-an-enum-class-in-c11
template <typename Enumeration>
auto to_value(Enumeration const value)
  -> typename std::underlying_type<Enumeration>::type
{
  return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}

auto get_state_name(State state) {
  return STATE_NAMES[to_value(state)];
}

class Tricks : public rclcpp::Node
{
public:
  Tricks()
  : Node("tricks")
  {

    //Parameters
    auto param = rcl_interfaces::msg::ParameterDescriptor{};

    //Timers
    timer_ = create_wall_timer(
      static_cast<std::chrono::milliseconds>(static_cast<int>(interval_ * 1000.0)), 
      std::bind(&Tricks::timer_callback, this)
    );

    //Clients
    cli_set_rpy_ = create_client<unitree_nav_interfaces::srv::SetBodyRPY>("set_body_rpy");

    RCLCPP_INFO_STREAM(get_logger(), "tricks node started");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<unitree_nav_interfaces::srv::SetBodyRPY>::SharedPtr cli_set_rpy_;

  State state_ = State::STARTUP;
  State state_last_ = state_;
  State state_next_ = state_;
  double rate_ = 100.0; //Hz
  double interval_ = 1.0 / rate_; //seconds

  void timer_callback() {

    state_ = state_next_;

    auto new_state = state_ != state_last_;

    if (new_state) {
      RCLCPP_INFO_STREAM(get_logger(), "Inspection state changed to " << get_state_name(state_));

      state_last_ = state_;
    }

    //State machine here
    switch (state_) {
      case State::STARTUP:
      {
        break;
      }
      default:
        break;
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Tricks>());
  rclcpp::shutdown();
  return 0;
}