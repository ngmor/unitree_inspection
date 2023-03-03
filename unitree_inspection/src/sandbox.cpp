#include <unordered_map>
#include <string>
#include <future>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "unitree_nav_interfaces/srv/set_body_rpy.hpp"

using namespace std::chrono_literals;

//TODO convert to SMACC if I get the time
enum class State
{
    IDLE
  ,SWEEPING_PITCH
  ,SETTING_RPY_START //TODO turn start wait pattern into a class
  ,SETTING_RPY_WAIT
};

//TODO find a better way of doing this
std::unordered_map<State, std::string> STATE_NAMES;

void init_enums() {
  STATE_NAMES[State::IDLE] = "IDLE";
  STATE_NAMES[State::SWEEPING_PITCH] = "SWEEPING_PITCH";
  STATE_NAMES[State::SETTING_RPY_START] = "SETTING_RPY_START";
  STATE_NAMES[State::SETTING_RPY_WAIT] = "SETTING_RPY_WAIT";
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

    //Clients
    cli_set_rpy_ = create_client<unitree_nav_interfaces::srv::SetBodyRPY>("set_body_rpy");

    RCLCPP_INFO_STREAM(get_logger(), "sandbox node started");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset_rpy_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_sweep_pitch_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_stop_sweep_;
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
            state_next_ = State::SWEEPING_PITCH;
          } else {
            state_next_ = State::IDLE;
          }
        }
        break;
      }
      case State::SWEEPING_PITCH:
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
      default:
        break;
    }
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

    sweeping_ = true;

    last_sweep_time_ = get_clock()->now();

    state_next_ = State::SWEEPING_PITCH;

  }

  void stop_sweep_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  ) {
    //Disable sweeping
    sweeping_ = false;
    state_next_ = State::IDLE;
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