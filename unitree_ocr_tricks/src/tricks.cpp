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
X(WAIT_FOR_COMMAND, "WAIT_FOR_COMMAND") \
X(STAND_START, "STAND_START") \

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


std::unordered_map<std::string, State> VALID_COMMANDS {
   {"lay down", State::STARTUP}
  ,{"stand", State::STAND_START}
};

class Tricks : public rclcpp::Node
{
public:
  Tricks()
  : Node("tricks")
  {

    //Parameters
    auto param = rcl_interfaces::msg::ParameterDescriptor{};

    param.description = "Consecutive frames of detection before detected command is accepted";
    declare_parameter("detection_count_threshold", 3, param);
    detection_count_threshold_ = get_parameter("detection_count_threshold").get_parameter_value().get<int>();

    //Timers
    timer_ = create_wall_timer(
      static_cast<std::chrono::milliseconds>(static_cast<int>(interval_ * 1000.0)), 
      std::bind(&Tricks::timer_callback, this)
    );

    //Subscribers
    sub_detected_text_ = create_subscription<unitree_ocr_interfaces::msg::Detections>(
      "detected_text",
      10,
      std::bind(&Tricks::detected_text_callback, this, std::placeholders::_1)
    );

    //Clients
    cli_set_rpy_ = create_client<unitree_nav_interfaces::srv::SetBodyRPY>("set_body_rpy");

    RCLCPP_INFO_STREAM(get_logger(), "tricks node started");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<unitree_ocr_interfaces::msg::Detections>::SharedPtr sub_detected_text_;
  rclcpp::Client<unitree_nav_interfaces::srv::SetBodyRPY>::SharedPtr cli_set_rpy_;

  State state_ = State::STARTUP;
  State state_last_ = state_;
  State state_next_ = state_;
  double rate_ = 100.0; //Hz
  double interval_ = 1.0 / rate_; //seconds
  rclcpp::Client<unitree_nav_interfaces::srv::SetBodyRPY>::SharedFuture rpy_future_;
  int detection_count_threshold_;
  unitree_ocr_interfaces::msg::Detections::SharedPtr detected_text_;

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
        state_next_ = State::SETTING_RPY_START;
        break;
      }
      case State::SETTING_RPY_START:
      {
        //Make dog look up to read signs
        if (cli_set_rpy_->wait_for_service(0s)) {
          std::shared_ptr<unitree_nav_interfaces::srv::SetBodyRPY::Request> rpy_request =
            std::make_shared<unitree_nav_interfaces::srv::SetBodyRPY::Request>();

            rpy_request->pitch = -0.6; //TODO make parameter?

          rpy_future_ = cli_set_rpy_->async_send_request(rpy_request).future.share();
          state_next_ = State::SETTING_RPY_WAIT;
        }
        break;
      }
      case State::SETTING_RPY_WAIT:
      {
        if (rpy_future_.wait_for(0s) == std::future_status::ready) {
            state_next_ = State::WAIT_FOR_COMMAND;
        }
        break;
      }
      case State::WAIT_FOR_COMMAND:
      {

        //Iterate through detected text
        for (const auto & detection : detected_text_->data) {
          //Skip anything that doesn't have enough detections
          if (detection.count < detection_count_threshold_) {
            continue;
          }

          auto detection_itr = VALID_COMMANDS.find(detection.text);

          //If text is in valid commands, begin that command
          if (detection_itr != VALID_COMMANDS.end()) {
            //Go to the appropriate next state to trigger command
            state_next_ = detection_itr->second;
          }
        }
        break;
      }
      case State::STAND_START:
      {
        //TODO
        state_next_ = State::STARTUP;
      }
      default:
        break;
    }
  }

  void detected_text_callback(const unitree_ocr_interfaces::msg::Detections::SharedPtr msg) {
    detected_text_ = msg;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Tricks>());
  rclcpp::shutdown();
  return 0;
}