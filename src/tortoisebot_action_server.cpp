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

  void normalize_yaw_error(double &err_yaw) {
    // Normalize the yaw error to the range [-π, π]
    while (err_yaw > M_PI) {
      err_yaw -= 2.0 * M_PI;
    }
    while (err_yaw < -M_PI) {
      err_yaw += 2.0 * M_PI;
    }
  }

  void execute(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GoToPose::Feedback>();
    auto result = std::make_shared<GoToPose::Result>();
    auto move = geometry_msgs::msg::Twist();

    // Define desired position and errors
    auto desired_pos = goal->position;
    double desired_yaw =
        atan2(desired_pos.y - cur_pos.y, desired_pos.x - cur_pos.x);
    double err_pos = sqrt(pow(desired_pos.y - cur_pos.y, 2) +
                          pow(desired_pos.x - cur_pos.x, 2));
    double err_yaw = desired_yaw - cur_pos.theta;

    // Normalize the yaw error
    normalize_yaw_error(err_yaw);

    const double MAX_LINEAR_SPEED = 0.2;
    const double MAX_ANGULAR_SPEED = 0.5;

    const double _dist_precision = 0.05;
    const double _yaw_precision = M_PI / 90.0;

    while (rclcpp::ok() &&
           (err_pos > _dist_precision || fabs(err_yaw) > _yaw_precision)) {
      if (err_pos > _dist_precision) {
        desired_yaw =
            atan2(desired_pos.y - cur_pos.y, desired_pos.x - cur_pos.x);
        err_yaw = desired_yaw - cur_pos.theta;
      } else {
        desired_yaw = desired_pos.theta;
        err_yaw = desired_yaw - cur_pos.theta;
      }

      // Normalize the yaw error
      normalize_yaw_error(err_yaw);
      err_pos = sqrt(pow(desired_pos.y - cur_pos.y, 2) +
                     pow(desired_pos.x - cur_pos.x, 2));

      RCLCPP_INFO(this->get_logger(), "Current Yaw: %f", cur_pos.theta);
      RCLCPP_INFO(this->get_logger(), "Desired Yaw: %f", desired_yaw);
      RCLCPP_INFO(this->get_logger(), "Error Yaw: %f", err_yaw);

      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        // Stop the robot
        move.linear.x = 0;
        move.angular.z = 0;
        vel_pub_->publish(move);

        // Set goal state
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // Handle significant yaw error by rotating in place
      if (fabs(err_yaw) > _yaw_precision * 10) {
        move.linear.x = 0;
        move.angular.z =
            std::min(MAX_ANGULAR_SPEED,
                     err_yaw * 0.5); // Proportional control for angular speed
      } else {
        // Regular movement towards the target
        move.angular.z = (fabs(err_yaw) > _yaw_precision)
                             ? std::min(MAX_ANGULAR_SPEED, err_yaw)
                             : 0.0;
        move.linear.x = (err_pos > _dist_precision)
                            ? std::min(MAX_LINEAR_SPEED, err_pos * 0.5)
                            : 0;
      }

      vel_pub_->publish(move);

      //   if (err_pos > _dist_precision) {
      //     move.linear.x =
      //         std::min(MAX_LINEAR_SPEED,
      //                  err_pos * 0.5); // Proportional control for linear
      //                  speed
      //     move.angular.z = (fabs(err_yaw) > _yaw_precision)
      //                          ? (std::min(MAX_ANGULAR_SPEED, err_yaw))
      //                          : 0.0; // Proportional control for angular
      //                          speed
      //   } else if (fabs(err_yaw) > _yaw_precision) {
      //     move.linear.x = 0;
      //     move.angular.z =
      //         std::min(MAX_ANGULAR_SPEED,
      //                  err_yaw * 0.5); // Proportional control for final
      //                  pivot
      //   } else {
      //     move.linear.x = 0;
      //     move.angular.z = 0;
      //   }

      //   vel_pub_->publish(move);

      feedback->position = cur_pos;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(),
                  "Publish feedback, reaching goal in %f distance; %f turn "
                  "delta; linear x speed: %f, angular z speed %f.",
                  err_pos, fabs(err_yaw), move.linear.x, move.angular.z);

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop the robot
    move.linear.x = 0;
    move.angular.z = 0;
    vel_pub_->publish(move);

    // Return success
    if (err_pos < _dist_precision && fabs(err_yaw) < _yaw_precision) {
      result->success = true;
      goal_handle->succeed(result);
    }
  }

  //   void execute(const std::shared_ptr<GoalHandleMove> goal_handle) {
  //     RCLCPP_INFO(this->get_logger(), "Executing goal");

  //     const auto goal = goal_handle->get_goal();
  //     auto feedback = std::make_shared<GoToPose::Feedback>();
  //     auto result = std::make_shared<GoToPose::Result>();
  //     auto move = geometry_msgs::msg::Twist();

  //     // Define desired position and errors
  //     auto desired_pos = goal->position;
  //     double desired_yaw =
  //         atan2(desired_pos.y - cur_pos.y, desired_pos.x - cur_pos.x);
  //     double err_pos = sqrt(pow(desired_pos.y - cur_pos.y, 2) +
  //                           pow(desired_pos.x - cur_pos.x, 2));
  //     double err_yaw = desired_yaw - cur_pos.theta;

  //     // Normalize the yaw error
  //     normalize_yaw_error(err_yaw);

  //     const double MAX_LINEAR_SPEED = 0.2;
  //     const double MAX_ANGULAR_SPEED = 0.5;

  //     float _dist_precision(0.05);
  //     float _yaw_precision(M_PI / 90.0f);

  //     while (rclcpp::ok() && err_pos > _dist_precision) {
  //       desired_yaw = atan2(desired_pos.y - cur_pos.y, desired_pos.x -
  //       cur_pos.x); err_yaw = desired_yaw - cur_pos.theta;
  //       // Normalize the yaw error
  //       normalize_yaw_error(err_yaw);
  //       err_pos = sqrt(pow(desired_pos.y - cur_pos.y, 2) +
  //                      pow(desired_pos.x - cur_pos.x, 2));

  //       RCLCPP_INFO(this->get_logger(), "Current Yaw: %f", cur_pos.theta);
  //       RCLCPP_INFO(this->get_logger(), "Desired Yaw: %f", desired_yaw);
  //       RCLCPP_INFO(this->get_logger(), "Error Yaw: %f", err_yaw);

  //       // Check if there is a cancel request
  //       if (goal_handle->is_canceling()) {
  //         // Stop the robot
  //         move.linear.x = 0;
  //         move.angular.z = 0;
  //         vel_pub_->publish(move);

  //         // Set goal state
  //         result->success = false;
  //         goal_handle->canceled(result);
  //         RCLCPP_INFO(this->get_logger(), "Goal canceled");
  //         return;
  //       } else if (fabs(err_yaw) > _yaw_precision) {
  //         RCLCPP_INFO(this->get_logger(), "Fix yaw");
  //         move.linear.x = 0;
  //         move.angular.z = (err_yaw > 0 ? MAX_ANGULAR_SPEED :
  //         -MAX_ANGULAR_SPEED); vel_pub_->publish(move);
  //       } else {
  //         RCLCPP_INFO(this->get_logger(), "Go to point");
  //         move.linear.x = std::min(MAX_LINEAR_SPEED, err_pos / 2.0f);
  //         move.angular.z = 0;
  //         vel_pub_->publish(move);
  //       }

  //       feedback->position = cur_pos;
  //       goal_handle->publish_feedback(feedback);
  //       RCLCPP_INFO(this->get_logger(),
  //                   "Publish feedback, reaching goal in %f distance; %f turn
  //                   " "delta; linear x speed: %f, angular z speed %f.",
  //                   err_pos, fabs(err_yaw), move.linear.x, move.angular.z);

  //       // Check if goal is complete (if we are close enough)
  //       if (err_pos < 0.05 && fabs(cur_pos.theta - desired_pos.theta) < 0.1)
  //       {
  //         break;
  //       } else {
  //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
  //       }
  //     }

  //     // Stop the robot
  //     move.linear.x = 0;
  //     move.angular.z = 0;
  //     vel_pub_->publish(move);

  //     // Return success
  //     if (err_pos < _dist_precision) {
  //       result->success = true;
  //       goal_handle->succeed(result);
  //     }
  //   }

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