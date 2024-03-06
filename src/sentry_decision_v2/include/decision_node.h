#ifndef _DECISION_NODE_H
#define _DECISION_NODE_H

#include "./sentry_decision_v2/structs.h"
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include "auto_aim_interfaces/msg/uart.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "Json/json.h"

namespace sentry
{
    using namespace std::chrono_literals;
    class RobotDecisionNode : public rclcpp::Node
    {
    public:
        RobotDecisionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~RobotDecisionNode();

    private:
        void initParam();
        void initSubscribers();
        void initPublishers();
        void init();
        void uartSubCallback(const auto_aim_interfaces::msg::Uart::SharedPtr msg);
        bool readJsonFile();

    private:
        std::shared_ptr<rclcpp::Rate> local_rate_;
        Blackboard blackboard;
        std::deque<Mission> missionList;
        std::string decisions_file, waypoints_file;
        std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
        rclcpp::Subscription<auto_aim_interfaces::msg::Uart>::SharedPtr uart_sub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mark_pub_;

        std::vector<std::shared_ptr<Way_Point>> waypoints;
        std::vector<std::shared_ptr<Decision_Warp>> decisions;
    };
} // namespace sentry

#endif // _DECISION_NODE_H