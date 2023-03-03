#include <string>
#include <future>
#include <unordered_map>
#include <unordered_set>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "unitree_nav_interfaces/srv/set_body_rpy.hpp"
#include "unitree_ocr_interfaces/msg/detection.hpp"
#include "unitree_ocr_interfaces/msg/detections.hpp"

using namespace std::chrono_literals;

//TODO convert to SMACC if I get the time
enum class State
{
   IDLE
  ,SET_SWEEP_TARGET
  ,SETTING_RPY_START //TODO turn start wait pattern into a class
  ,SETTING_RPY_WAIT
  ,EVALUATE_DETECTION
};

//TODO find a better way of doing this
std::unordered_map<State, std::string> STATE_NAMES = {
   {State::IDLE, "IDLE"}
  ,{State::SET_SWEEP_TARGET, "SET_SWEEP_TARGET"}
  ,{State::SETTING_RPY_START, "SETTING_RPY_START"}
  ,{State::SETTING_RPY_WAIT, "SETTING_RPY_WAIT"}
  ,{State::EVALUATE_DETECTION, "EVALUATE_DETECTION"}
};

//Detections that are valid on a sign post.
std::unordered_set<std::string> VALID_DETECTIONS = {
   "100"
  ,"200"
  ,"300"
  ,"400"
  ,"500"
  ,"600"
  ,"700"
  ,"800"
  ,"900"
};

//TODO make parameters for all these things
constexpr uint16_t CONSECUTIVE_FRAMES_THRESHOLD = 5; //number of frames before any text is considered detected
constexpr uint16_t DETECTION_COUNT_THRESHOLD = 10; //number of frames before specific text is considered detected

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

    //Subscribers
    sub_detected_text_ = create_subscription<unitree_ocr_interfaces::msg::Detections>(
      "detected_text",
      10,
      std::bind(&Sandbox::detected_text_callback, this, std::placeholders::_1)
    );

    //Services
    srv_reset_rpy_ = create_service<std_srvs::srv::Empty>(
      "reset_rpy",
      std::bind(&Sandbox::reset_rpy_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );
    srv_sweep_pitch_ = create_service<std_srvs::srv::Empty>(
      "sweep_pitch",
      std::bind(&Sandbox::sweep_pitch_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );
    srv_stop_sweep_ = create_service<std_srvs::srv::Empty>(
      "stop_sweep",
      std::bind(&Sandbox::stop_sweep_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );
    srv_inspect_for_text_ = create_service<std_srvs::srv::Empty>(
      "inspect_for_text",
      std::bind(&Sandbox::inspect_for_text_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );

    //Clients
    cli_set_rpy_ = create_client<unitree_nav_interfaces::srv::SetBodyRPY>("set_body_rpy");

    RCLCPP_INFO_STREAM(get_logger(), "sandbox node started");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<unitree_ocr_interfaces::msg::Detections>::SharedPtr sub_detected_text_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset_rpy_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_sweep_pitch_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_stop_sweep_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_inspect_for_text_;
  rclcpp::Client<unitree_nav_interfaces::srv::SetBodyRPY>::SharedPtr cli_set_rpy_;

  State state_ = State::IDLE;
  State state_last_ = state_;
  State state_next_ = state_;
  double rate_ = 100.0; //Hz
  double interval_ = 1.0 / rate_; //seconds
  std::shared_ptr<unitree_nav_interfaces::srv::SetBodyRPY::Request> rpy_request_ =
    std::make_shared<unitree_nav_interfaces::srv::SetBodyRPY::Request>();
  rclcpp::Client<unitree_nav_interfaces::srv::SetBodyRPY>::SharedFuture rpy_future_;
  bool rpy_call_complete_ = false;
  bool sweeping_ = false;
  //TODO make parameters
  double sweep_time_ = 10.0;
  double sweep_max_ = 0.6;
  double sweep_rate_ = 2*sweep_max_ / sweep_time_;
  double sweep_angle_ = 0.0;
  bool sweeping_upwards_ = true;
  rclcpp::Time last_sweep_time_;
  bool inspecting_for_text_ = false;
  unitree_ocr_interfaces::msg::Detections::SharedPtr detected_text_;

  void timer_callback() {
    state_ = state_next_;

    auto new_state = state_ != state_last_;

    if (new_state) {
      RCLCPP_INFO_STREAM(get_logger(), "Sandbox state changed to " << STATE_NAMES[state_]);

      state_last_ = state_;
    }

    //State machine here
    switch (state_) {
      case State::IDLE:
      {
        sweeping_ = false;
        inspecting_for_text_ = false;
        break;
      }
      case State::SETTING_RPY_START:
      {
        if (!cli_set_rpy_->wait_for_service(0s)) {
          RCLCPP_ERROR_STREAM(get_logger(), "Set RPY service not available.");
          state_next_ = State::IDLE;
        } else {
          rpy_future_ = cli_set_rpy_->async_send_request(rpy_request_).future.share();
          state_next_ = State::SETTING_RPY_WAIT;
        }
        break;
      }
      case State::SETTING_RPY_WAIT:
      {
        if (rpy_future_.wait_for(0s) == std::future_status::ready) {

          if (sweeping_) {
            state_next_ = State::SET_SWEEP_TARGET;
          } else {
            state_next_ = State::IDLE;
          }
        }
        break;
      }
      case State::SET_SWEEP_TARGET:
      {
        //Calculate how much ton increment the sweep angle by
        auto time_delta = static_cast<double>(get_clock()->now().nanoseconds() -
          last_sweep_time_.nanoseconds()) * 1.0e-9;

        auto pitch_delta = sweep_rate_*time_delta;

        // Change sweep direction if we've hit limits
        if (sweep_angle_ < -sweep_max_) {
          sweeping_upwards_ = false;
        } else if (sweep_angle_ > sweep_max_) {
          sweeping_upwards_ = true;
        }

        //update sweep angle based on the direction we're going
        if (sweeping_upwards_) {
          sweep_angle_ -= pitch_delta;
        } else {
          sweep_angle_ += pitch_delta;
        }

        //update message
        rpy_request_->pitch = sweep_angle_;

        //Update sweep timestamp
        last_sweep_time_ = get_clock()->now();

        //Call service to update sweep
        state_next_ = State::SETTING_RPY_START;

        break;
      }

      case State::EVALUATE_DETECTION:
      {

        if (detected_text_->count < CONSECUTIVE_FRAMES_THRESHOLD) { //TODO add timeout, handling retriggers gets complicated
          //if we lose the detections, return to sweep
          start_sweep_pitch();
        }
        
        //Iterate through detected text
        for (const auto & detection : detected_text_->data) {
          //Skip anything that doesn't have enough detections
          if(detection.count < DETECTION_COUNT_THRESHOLD) {
            continue;
          }

          //If text is in valid detections, mark that we've found it and reset body RPY before going back to idle
          if(VALID_DETECTIONS.find(detection.text) != VALID_DETECTIONS.end()) {
            RCLCPP_INFO_STREAM(get_logger(), "Text found: " << detection.text);
            *rpy_request_ = unitree_nav_interfaces::srv::SetBodyRPY::Request {};
            sweeping_ = false;
            inspecting_for_text_ = false;
            state_next_ = State::SETTING_RPY_START;
          }

        }


        
      }
      default:
        break;
    }
  }

  void detected_text_callback(const unitree_ocr_interfaces::msg::Detections::SharedPtr msg) {
    //Don't need to process this data if not actively looking for it
    if (!inspecting_for_text_) {return;}

    if (state_ != State::EVALUATE_DETECTION && msg->count > CONSECUTIVE_FRAMES_THRESHOLD) {
      state_next_ = State::EVALUATE_DETECTION;
    }

    detected_text_ = msg;

  }

  void start_sweep_pitch() {
    sweeping_ = true;
    last_sweep_time_ = get_clock()->now();
    state_next_ = State::SET_SWEEP_TARGET;
  }

  //reset dog roll pitch and pitch
  void reset_rpy_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  ) {

    //Set all values to 0
    *rpy_request_ = unitree_nav_interfaces::srv::SetBodyRPY::Request {};

    //Begin setting RPY
    state_next_ = State::SETTING_RPY_START;

    //Reset sweeping variables
    //Disable sweeping
    sweeping_ = false;
    sweeping_upwards_ = true;
    sweep_angle_ = 0.0;
  }

  void sweep_pitch_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  ) {

    //TODO - validate state?
    start_sweep_pitch();

  }

  void stop_sweep_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  ) {
    //Disable sweeping
    sweeping_ = false;
    state_next_ = State::IDLE;
  }

  void inspect_for_text_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  ) {

    //TODO - validate state?
    start_sweep_pitch();

    inspecting_for_text_ = true;

  }
};



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sandbox>());
  rclcpp::shutdown();
  return 0;
}