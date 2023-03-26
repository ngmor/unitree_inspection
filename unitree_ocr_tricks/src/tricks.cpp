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
X(IDLE, "IDLE") \

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

class Inspection : public rclcpp::Node
{
public:
  Inspection()
  : Node("tricks")
  {

    //Parameters
    auto param = rcl_interfaces::msg::ParameterDescriptor{};

    //Timers
    timer_ = create_wall_timer(
      static_cast<std::chrono::milliseconds>(static_cast<int>(interval_ * 1000.0)), 
      std::bind(&Inspection::timer_callback, this)
    );

    RCLCPP_INFO_STREAM(get_logger(), "tricks node started");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;

	double rate_ = 100.0; //Hz
  double interval_ = 1.0 / rate_; //seconds

  void timer_callback() {

  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Inspection>());
  rclcpp::shutdown();
  return 0;
}