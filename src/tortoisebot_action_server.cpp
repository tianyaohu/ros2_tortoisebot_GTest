// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>

// class WaypointActionServer : public rclcpp::Node {
// public:
//   using WaypointAction = tortoisebot_waypoints::action::WaypointAction;
//   using GoalHandle = rclcpp_action::ServerGoalHandle<WaypointAction>;

//   explicit WaypointActionServer(
//       const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
//       : Node("waypoint_action_server", options) {
//     using namespace std::placeholders;

//     this->action_server_ = rclcpp_action::create_server<WaypointAction>(
//         this, "waypoint",
//         std::bind(&WaypointActionServer::handle_goal, this, _1, _2),
//         std::bind(&WaypointActionServer::handle_cancel, this, _1),
//         std::bind(&WaypointActionServer::handle_accepted, this, _1));
//   }

// private:
//   rclcpp_action::Server<WaypointAction>::SharedPtr action_server_;

//   rclcpp_action::GoalResponse
//   handle_goal(const rclcpp_action::GoalUUID &uuid,
//               std::shared_ptr<const WaypointAction::Goal> goal) {
//     RCLCPP_INFO(this->get_logger(),
//                 "Received goal request with position x: %f, y: %f, z: %f",
//                 goal->position.x, goal->position.y, goal->position.z);
//     (void)uuid;
//     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
//   }

//   rclcpp_action::CancelResponse
//   handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
//     RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
//     (void)goal_handle;
//     return rclcpp_action::CancelResponse::ACCEPT;
//   }

//   void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
//     using namespace std::placeholders;
//     std::thread{std::bind(&WaypointActionServer::execute, this, _1),
//                 goal_handle}
//         .detach();
//   }

//   void execute(const std::shared_ptr<GoalHandle> goal_handle) {
//     RCLCPP_INFO(this->get_logger(), "Executing goal");
//     rclcpp::Rate loop_rate(1);
//     const auto goal = goal_handle->get_goal();
//     auto feedback = std::make_shared<WaypointAction::Feedback>();
//     auto result = std::make_shared<WaypointAction::Result>();

//     for (int i = 0; i < 10; ++i) { // Assuming some logic here for
//     demonstration
//       // Check if there is a cancel request
//       if (goal_handle->is_canceling()) {
//         result->success = false;
//         goal_handle->canceled(result);
//         RCLCPP_INFO(this->get_logger(), "Goal canceled");
//         return;
//       }

//       // Update feedback
//       feedback->position = goal->position; // Example feedback update
//       feedback->state = "In Progress";
//       goal_handle->publish_feedback(feedback);
//       RCLCPP_INFO(this->get_logger(), "Published feedback");

//       loop_rate.sleep();
//     }

//     // Goal completed
//     if (rclcpp::ok()) {
//       result->success = true;
//       goal_handle->succeed(result);
//       RCLCPP_INFO(this->get_logger(), "Goal succeeded");
//     }
//   }
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<WaypointActionServer>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }

#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tortoisebot_waypoints/action/waypoint_action.hpp"

using namespace std;

class GoToPoseActionServer : public rclcpp::Node {
public:
  using GoToPose = tortoisebot_waypoints::action::WaypointAction;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<GoToPose>;

  explicit GoToPoseActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("go_to_pose_action_server_node", options) {
    using namespace std::placeholders;

    // init callback groups
    callback_group_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_3 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // init subscription option
    rclcpp::SubscriptionOptions option1;
    option1.callback_group = callback_group_1;

    // init odom sub
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 1, std::bind(&GoToPoseActionServer::odom_callback, this, _1),
        option1);

    // init command vel pub
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    action_server_ = rclcpp_action::create_server<GoToPose>(
        this, "tortoisebot_waypoint",
        std::bind(&GoToPoseActionServer::handle_goal, this, _1, _2),
        std::bind(&GoToPoseActionServer::handle_cancel, this, _1),
        std::bind(&GoToPoseActionServer::handle_accepted, this, _1),
        rcl_action_server_get_default_options(), callback_group_3);
  }

private:
  // attributes
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_3;

  rclcpp_action::Server<GoToPose>::SharedPtr action_server_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // current pos2d
  geometry_msgs::msg::Pose2D cur_pos;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    // set x,y of the current position
    cur_pos.x = msg->pose.pose.position.x;
    cur_pos.y = msg->pose.pose.position.y;

    const float q_x = msg->pose.pose.orientation.x;
    const float q_y = msg->pose.pose.orientation.y;
    const float q_z = msg->pose.pose.orientation.z;
    const float q_w = msg->pose.pose.orientation.w;

    // Yaw (z-axis rotation)
    float sinYaw = 2.0f * (q_w * q_z + q_x * q_y);
    float cosYaw = 1.0f - 2.0f * (q_y * q_y + q_z * q_z);
    float yaw = std::atan2(sinYaw, cosYaw);

    // set theta of current positon
    cur_pos.theta = yaw;
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPose::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),
                "Received goal request with x at %f; y at %f", goal->position.x,
                goal->position.y);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up
    // a new thread
    std::thread{std::bind(&GoToPoseActionServer::execute, this, _1),
                goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GoToPose::Feedback>();

    auto result = std::make_shared<GoToPose::Result>();
    auto move = geometry_msgs::msg::Twist();

    auto desired_pos = goal->position;

    const float MAX_LINEAR_SPEED = 0.2;
    const float MAX_ANGULAR_SPEED = 0.5;
    while (rclcpp::ok()) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        // stop the robot
        move.linear.x = 0;
        move.angular.z = 0;
        vel_pub_->publish(move);
        // set goal state
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // calculate turning speed
      float diff_x = desired_pos.x - cur_pos.x,
            diff_y = desired_pos.y - cur_pos.y;
      float dirc = atan2(diff_y, diff_x);

      // Set linear speed
      float abs_dist = sqrt(diff_x * diff_x + diff_y * diff_y);
      move.linear.x = min(MAX_LINEAR_SPEED, abs_dist / 5);

      // calculate true delta
      float turn_delta;
      if (abs_dist < 0.05) {
        turn_delta = cur_pos.theta - desired_pos.theta;
      } else {
        turn_delta = cur_pos.theta - dirc;
      }

      if (abs(turn_delta) > M_PI) {
        turn_delta =
            turn_delta > 0 ? 2 * M_PI - turn_delta : 2 * M_PI + turn_delta;
      }

      // set angular speed
      move.angular.z = min(MAX_ANGULAR_SPEED, turn_delta / -3);

      vel_pub_->publish(move);

      feedback->position = cur_pos;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(),
                  "Publish feedback, reaching goal in %f distancel; %f turn "
                  "delta; linear x "
                  "speed: %f, angular z speed %f.",
                  abs_dist, abs(turn_delta), move.linear.x, move.angular.z);

      // check if goal is compelete (if we are close enough)
      if (abs_dist + abs(cur_pos.theta - desired_pos.theta) < 0.1) {
        break;
      } else {
        this_thread::sleep_for(100ms);
      }
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->success = true;
      move.linear.x = 0.0;
      move.angular.z = 0.0;
      vel_pub_->publish(move);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

}; // class GoToPoseActionServer

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<GoToPoseActionServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}