#include "tortoisebot_waypoints/action/waypoint_action.hpp"
#include <geometry_msgs/msg/pose2_d.hpp>
#include <gtest/gtest.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::placeholders;
using namespace std::chrono_literals;

class RclCppFixture {
public:
  RclCppFixture() { rclcpp::init(0, nullptr); }
  ~RclCppFixture() { rclcpp::shutdown(); }
};
RclCppFixture g_rclcppfixture;

class WaypointActionClientTest : public ::testing::Test {
protected:
  void SetUp() override {
    node_ = rclcpp::Node::make_shared("waypoint_action_client_test");
    action_client_ = rclcpp_action::create_client<
        tortoisebot_waypoints::action::WaypointAction>(node_,
                                                       "tortoisebot_waypoint");

    odom_subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&WaypointActionClientTest::odom_callback, this, _1));

    // Ensure the action server is up before proceeding.
    ASSERT_TRUE(action_client_->wait_for_action_server(10s));

    // Send a goal and wait for the result.
    send_goal(1.0, 1.0, 0.0);
  }

  void send_goal(double goal_x, double goal_y, double goal_theta) {
    using Goal = tortoisebot_waypoints::action::WaypointAction::Goal;
    auto goal_msg = Goal();
    goal_msg.position.x = goal_x;
    goal_msg.position.y = goal_y;
    goal_msg.position.theta = goal_theta;

    auto goal_handle_future = action_client_->async_send_goal(goal_msg);
    if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      ADD_FAILURE() << "Failed to send goal to action server.";
      return;
    }

    auto result_future =
        action_client_->async_get_result(goal_handle_future.get());
    if (rclcpp::spin_until_future_complete(node_, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      ADD_FAILURE() << "Failed to get result from action server.";
      return;
    }

    success_ = result_future.get().result->success;

    // Wait for odom to be updated
    rclcpp::Rate rate(10);
    for (int i = 0; i < 100 && !odom_received_; ++i) {
      rclcpp::spin_some(node_);
      rate.sleep();
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;
    current_pose_.theta = get_yaw_from_quaternion(msg->pose.pose.orientation);
    odom_received_ = true;
  }

  double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q) {
    tf2::Quaternion tf2_q(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<
      tortoisebot_waypoints::action::WaypointAction>::SharedPtr action_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  geometry_msgs::msg::Pose2D current_pose_;
  bool odom_received_ = false;
  bool success_ = false;
};

TEST_F(WaypointActionClientTest, TestPositionAccuracy) {
  EXPECT_TRUE(success_);
  EXPECT_NEAR(current_pose_.x, 1.0, 0.05);
  EXPECT_NEAR(current_pose_.y, 1.0, 0.05);
}

TEST_F(WaypointActionClientTest, TestOrientationAccuracy) {
  EXPECT_TRUE(success_);
  EXPECT_NEAR(current_pose_.theta, 0.0, 0.1);
}