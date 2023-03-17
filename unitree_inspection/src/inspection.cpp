#include <string>
#include <future>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "unitree_nav_interfaces/srv/set_body_rpy.hpp"
#include "unitree_nav_interfaces/srv/nav_to_pose.hpp"
#include "unitree_ocr_interfaces/msg/detection.hpp"
#include "unitree_ocr_interfaces/msg/detections.hpp"
#include "unitree_inspection_interfaces/srv/go_to_inspection_point.hpp"

using namespace std::chrono_literals;

//https://stackoverflow.com/questions/11714325/how-to-get-enum-item-name-from-its-value
#define STATES \
X(IDLE, "IDLE") \
X(SET_SWEEP_TARGET, "SET_SWEEP_TARGET") \
X(SETTING_RPY_START, "SETTING_RPY_START") \
X(SETTING_RPY_WAIT, "SETTING_RPY_WAIT") \
X(EVALUATE_DETECTION, "EVALUATE_DETECTION") \
X(SEND_GOAL, "SEND_GOAL") \
X(WAIT_FOR_GOAL_RESPONSE, "WAIT_FOR_GOAL_RESPONSE") \
X(WAIT_FOR_MOVEMENT_COMPLETE, "WAIT_FOR_MOVEMENT_COMPLETE") \

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

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

std::tuple<double, double, double> quaternion_to_rpy(const geometry_msgs::msg::Quaternion & q);
geometry_msgs::msg::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw);

//TODO make parameters for all these things
constexpr uint16_t CONSECUTIVE_FRAMES_THRESHOLD = 5; //number of frames before any text is considered detected
constexpr uint16_t DETECTION_COUNT_THRESHOLD = 10; //number of frames before specific text is considered detected

class Inspection : public rclcpp::Node
{
public:
  Inspection()
  : Node("inspection")
  {

    //Parameters
    auto param = rcl_interfaces::msg::ParameterDescriptor{};

    //Check if required parameters were provided
    bool required_parameters_received = true;

    param.description = "The frame in which poses are sent.";
    declare_parameter("pose_frame", "map", param);
    goal_msg_.pose.header.frame_id = get_parameter("pose_frame").get_parameter_value().get<std::string>();

    param.description =
      "List of text labels for each inspection point. Arbitrary length, but must match length of x, y, theta.";
    declare_parameter("inspection_points.text", std::vector<std::string> {}, param);
    auto inspection_points_text = get_parameter("inspection_points.text").as_string_array();

    param.description =
      "List of x positions of inspection points (m). Arbitrary length, but must match length of text, y, theta.";
    declare_parameter("inspection_points.x", std::vector<double> {}, param);
    auto inspection_points_x = get_parameter("inspection_points.x").as_double_array();

    param.description =
      "List of y positions of inspection points (m). Arbitrary length, but must match length of text, x, theta.";
    declare_parameter("inspection_points.y", std::vector<double> {}, param);
    auto inspection_points_y = get_parameter("inspection_points.y").as_double_array();

    param.description =
      "List of theta positions of inspection points (rad). Arbitrary length, but must match length of text, x, y.";
    declare_parameter("inspection_points.theta", std::vector<double> {}, param);
    auto inspection_points_theta = get_parameter("inspection_points.theta").as_double_array();

    //If vectors differ in size, exit node
    if (
       (inspection_points_text.size() != inspection_points_x.size())
    || (inspection_points_text.size() != inspection_points_y.size())
    || (inspection_points_text.size() != inspection_points_theta.size())
    ) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Size mismatch between input inspection point lists ("
        << "text: " << inspection_points_text.size() << " elements, "
        << "x: " << inspection_points_x.size() << " elements, "
        << "y: " << inspection_points_y.size() << " elements, "
        << "theta: " << inspection_points_theta.size() << " elements)"
      );
      required_parameters_received = false;
    }

    //Abort if any required parameters were not provided
    if (!required_parameters_received) {
      throw std::logic_error(
        "Required parameters were not received or were invalid. Please provide valid parameters."
      );
    }

    //Timers
    timer_ = create_wall_timer(
      static_cast<std::chrono::milliseconds>(static_cast<int>(interval_ * 1000.0)), 
      std::bind(&Inspection::timer_callback, this)
    );

    //Subscribers
    sub_detected_text_ = create_subscription<unitree_ocr_interfaces::msg::Detections>(
      "detected_text",
      10,
      std::bind(&Inspection::detected_text_callback, this, std::placeholders::_1)
    );

    //Services
    srv_reset_rpy_ = create_service<std_srvs::srv::Empty>(
      "reset_rpy",
      std::bind(&Inspection::reset_rpy_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );
    srv_sweep_pitch_ = create_service<std_srvs::srv::Empty>(
      "sweep_pitch",
      std::bind(&Inspection::sweep_pitch_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );
    srv_stop_sweep_ = create_service<std_srvs::srv::Empty>(
      "stop_sweep",
      std::bind(&Inspection::stop_sweep_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );
    srv_inspect_for_text_ = create_service<std_srvs::srv::Empty>(
      "inspect_for_text",
      std::bind(&Inspection::inspect_for_text_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );
    srv_nav_to_pose_ = create_service<unitree_nav_interfaces::srv::NavToPose>(
      "unitree_nav_to_pose",
      std::bind(&Inspection::srv_nav_to_pose_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );
    srv_cancel_nav_ = create_service<std_srvs::srv::Empty>(
      "unitree_cancel_nav",
      std::bind(&Inspection::cancel_nav_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );
    srv_go_to_inspection_point_ = create_service<unitree_inspection_interfaces::srv::GoToInspectionPoint>(
      "go_to_inspection_point",
      std::bind(&Inspection::go_to_inspection_point_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );

    //Clients
    cli_set_rpy_ = create_client<unitree_nav_interfaces::srv::SetBodyRPY>("set_body_rpy");

    //Action Clients
    act_nav_to_pose_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this,
      "navigate_to_pose"
    );

    //Construct inspection points map
    for (size_t i = 0; i < inspection_points_text.size(); i++) {
      inspection_points_[inspection_points_text.at(i)] = Pose2D {
        inspection_points_x.at(i),
        inspection_points_y.at(i),
        inspection_points_theta.at(i),
      };
    }

    RCLCPP_INFO_STREAM(get_logger(), "inspection node started");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<unitree_ocr_interfaces::msg::Detections>::SharedPtr sub_detected_text_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset_rpy_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_sweep_pitch_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_stop_sweep_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_inspect_for_text_;
  rclcpp::Service<unitree_nav_interfaces::srv::NavToPose>::SharedPtr srv_nav_to_pose_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_cancel_nav_;
  rclcpp::Service<unitree_inspection_interfaces::srv::GoToInspectionPoint>::SharedPtr srv_go_to_inspection_point_;
  rclcpp::Client<unitree_nav_interfaces::srv::SetBodyRPY>::SharedPtr cli_set_rpy_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr act_nav_to_pose_;

  std::unordered_map<std::string, Pose2D> inspection_points_ {};
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
  Pose2D goal_pose_;
  bool navigating_to_points_ = false;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal goal_msg_ {};
  bool goal_response_received_ = false;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_ {};
  std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback_ = nullptr;
  std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult>
    result_ = nullptr;

  void timer_callback() {
    state_ = state_next_;

    auto new_state = state_ != state_last_;

    if (new_state) {
      RCLCPP_INFO_STREAM(get_logger(), "Inspection state changed to " << get_state_name(state_));

      state_last_ = state_;
    }

    //State machine here
    switch (state_) {
      case State::IDLE:
      {
        sweeping_ = false;
        inspecting_for_text_ = false;
        navigating_to_points_ = false;
        break;
      }
      case State::SEND_GOAL:
      {
        if(act_nav_to_pose_->wait_for_action_server(0s)) {
          //Reset status flags and pointers
          goal_response_received_ = false;
          goal_handle_ = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr {};
          result_ = nullptr;

          //Construct and send goal
          goal_msg_.pose.pose.position.x = goal_pose_.x;
          goal_msg_.pose.pose.position.y = goal_pose_.y;
          goal_msg_.pose.pose.orientation = rpy_to_quaternion(0.0, 0.0, goal_pose_.theta);

          //End navigation to points if we're returning to the origin
          if ((goal_pose_.x == 0.0) && (goal_pose_.y == 0.0) && (goal_pose_.theta == 0.0)) {
            navigating_to_points_ = false;
          }

          auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
          send_goal_options.goal_response_callback = 
            std::bind(&Inspection::goal_response_callback, this, std::placeholders::_1);
          send_goal_options.feedback_callback =
            std::bind(&Inspection::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
          send_goal_options.result_callback =
            std::bind(&Inspection::result_callback, this, std::placeholders::_1);
          act_nav_to_pose_->async_send_goal(goal_msg_, send_goal_options);

          state_next_ = State::WAIT_FOR_GOAL_RESPONSE;
        } else {
          RCLCPP_ERROR_STREAM(get_logger(), "Action server not available, aborting.");
          state_next_ = State::IDLE;
        }

        break;
      }
      case State::WAIT_FOR_GOAL_RESPONSE:
      {
        //TODO add timeout
        if (goal_response_received_) {
          if (goal_handle_) {
            RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
            state_next_ = State::WAIT_FOR_MOVEMENT_COMPLETE;
          } else {
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was rejected by server");
            state_next_ = State::IDLE;
          }
        }

        break;
      }
      case State::WAIT_FOR_MOVEMENT_COMPLETE:
      {
        if (result_) {
          if (navigating_to_points_) {
            start_inspecting_for_text();
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
          } else if (navigating_to_points_) {
            state_next_ = State::SEND_GOAL;
          } else {
            state_next_ = State::IDLE;
          }
        }
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

          auto detection_itr = inspection_points_.find(detection.text);

          //If text is in valid detections, mark that we've found it and reset body RPY before going back to idle
          if(detection_itr != inspection_points_.end()) {
            goal_pose_ = detection_itr->second;
            RCLCPP_INFO_STREAM(
              get_logger(),
              "Text found: " << detection.text << "\nNext destination: (" 
              << goal_pose_.x << ", " << goal_pose_.y << ", " << goal_pose_.theta << ")"
            );
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

  void start_inspecting_for_text() {
    sweep_angle_ = 0.0;
    sweeping_upwards_ = true;
    start_sweep_pitch();
    inspecting_for_text_ = true;
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
    start_inspecting_for_text();
  }

  void go_to_inspection_point_callback(
    const std::shared_ptr<unitree_inspection_interfaces::srv::GoToInspectionPoint::Request> request,
    std::shared_ptr<unitree_inspection_interfaces::srv::GoToInspectionPoint::Response>
  )
  {
    //If requested inspection point is not in valid inspection points, do nothing
    if(inspection_points_.find(request->inspection_point) == inspection_points_.end()) {
      return;
    }

    //Otherwise, begin navigation to points
    navigating_to_points_ = true;

    goal_pose_ = inspection_points_[request->inspection_point];

    state_next_ = State::SEND_GOAL;
  }

  void srv_nav_to_pose_callback(
    const std::shared_ptr<unitree_nav_interfaces::srv::NavToPose::Request> request,
    std::shared_ptr<unitree_nav_interfaces::srv::NavToPose::Response>
  ) {
    //Store requested pose
    goal_pose_.x = request->x;
    goal_pose_.y = request->y;
    goal_pose_.theta = request->theta;

    //Initiate action call
    state_next_ = State::SEND_GOAL;
  }

  void cancel_nav_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  ) {
    RCLCPP_INFO_STREAM(get_logger(), "Cancelling navigation.");
    act_nav_to_pose_->async_cancel_all_goals();
    state_next_ = State::IDLE;
  }

  void goal_response_callback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle
  ) {
    goal_response_received_ = true;
    goal_handle_ = goal_handle;
    RCLCPP_INFO_STREAM(get_logger(), "Goal response");
  }

  void feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback
  ) {
    //Store result for later use
    feedback_ = feedback;

    if (feedback_) {
      auto [roll, pitch, yaw] = quaternion_to_rpy(feedback_->current_pose.pose.orientation);

      RCLCPP_INFO_STREAM(get_logger(), "x = " << feedback_->current_pose.pose.position.x
                                  << ", y = " << feedback_->current_pose.pose.position.y
                                  << ", theta = " << yaw
      );
    }
  }

  void result_callback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result
  ) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    //Store result for later use
    result_ = std::make_shared<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult>();
    *result_ = result;
  }
};

std::tuple<double, double, double> quaternion_to_rpy(const geometry_msgs::msg::Quaternion & q) {
  //https://answers.ros.org/question/339528/quaternion-to-rpy-ros2/
  tf2::Quaternion q_temp;
  tf2::fromMsg(q, q_temp);
  tf2::Matrix3x3 m(q_temp);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return {roll, pitch, yaw};
}
geometry_msgs::msg::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw) {
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return tf2::toMsg(q);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Inspection>());
  rclcpp::shutdown();
  return 0;
}