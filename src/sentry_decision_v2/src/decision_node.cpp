#include <cstdio>
#include "../include/decision_node.h"

namespace sentry
{
  RobotDecisionNode::RobotDecisionNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("robot_decision_node", options)
  {
    RCLCPP_INFO(
        this->get_logger(),
        "RobotDecision node...starting");
    initParam();
    initSubscribers();
    initPublishers();
    readJsonFile();
    init();
  }

  RobotDecisionNode::~RobotDecisionNode()
  {
  }

  void RobotDecisionNode::initParam()
  {
    this->declare_parameter<std::string>("waypoints_file", "default_waypoints.json");
    this->declare_parameter<std::string>("decisions_file", "default_decisions.json");
    this->get_parameter("waypoints_file", waypoints_file);
    this->get_parameter("decisions_file", decisions_file);
  }

  void RobotDecisionNode::initSubscribers()
  {
    uart_sub_ = this->create_subscription<auto_aim_interfaces::msg::Uart>("/uart", rclcpp::QoS(10).reliable().keep_last(1), std::bind(&RobotDecisionNode::uartSubCallback, this, std::placeholders::_1));
  }

  void RobotDecisionNode::initPublishers()
  {
    mark_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("decision_mark", rclcpp::QoS(10).transient_local().keep_last(1));
  }

  void RobotDecisionNode::init()
  {
  }

  void RobotDecisionNode::uartSubCallback(const auto_aim_interfaces::msg::Uart::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lck(blackboard.mutex);
    blackboard._hp = msg->sentry_hp;
    blackboard._oupost_hp_remaining = msg->outpost_hp;
    blackboard._stage = msg->game_stage;
    blackboard._time_remaining = msg->remain_time;
  }

  bool RobotDecisionNode::readJsonFile()
  {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("sentry_decision_v2");
    Json::Reader jsonReader;
    Json::Value jsonValue;
    std::ifstream jsonFile_waypoints(package_share_directory + "/JsonFile/" + waypoints_file);
    if (!jsonReader.parse(jsonFile_waypoints, jsonValue, true))
    {
      RCLCPP_ERROR(this->get_logger(), "JSON FILE I/O ERR !");
      jsonFile_waypoints.close();
      return false;
    }
    Json::Value arrayValue = jsonValue["data"];
    for (int i = 0; i < int(arrayValue.size()); ++i)
    {
      std::shared_ptr<Way_Point> temp(new Way_Point);
      temp->id = arrayValue[i]["id"].asInt();
      temp->name = arrayValue[i]["name"].asCString();
      temp->x = arrayValue[i]["x"].asDouble();
      temp->y = arrayValue[i]["y"].asDouble();
      temp->w = arrayValue[i]["w"].asDouble();
      waypoints.emplace_back(temp);
    }
    arrayValue.clear();
    jsonValue.clear();
    jsonFile_waypoints.close();
    std::ifstream jsonFile_decisions(package_share_directory + "/JsonFile/" + decisions_file);
    if (!jsonReader.parse(jsonFile_decisions, jsonValue, true))
    {
      RCLCPP_ERROR(this->get_logger(), "JSON FILE I/O ERR !");
      jsonFile_decisions.close();
      return false;
    }
    arrayValue = jsonValue["data"];
    for (int i = 0; i < int(arrayValue.size()); ++i)
    {
      std::shared_ptr<Decision_Warp> temp(new Decision_Warp);
      temp->id = arrayValue[i]["id"].asInt();
      temp->group = arrayValue[i]["group"].asCString();
      temp->name = arrayValue[i]["name"].asCString();
      temp->waypointID = arrayValue[i]["wayPointID"].asInt();
      temp->weight = arrayValue[i]["weight"].asInt();
      temp->start_time = arrayValue[i]["start_time"].asInt();
      temp->end_time = arrayValue[i]["end_time"].asInt();
      temp->minHP = arrayValue[i]["minHP"].asInt();
      temp->maxHP = arrayValue[i]["maxHP"].asInt();
      temp->outpost_minHP = arrayValue[i]["out_post_HP_min"].asInt();
      Json::Value actions_array = arrayValue[i]["actions"];
      for (int j = 0; j < int(actions_array.size()); ++j)
      {
        temp->actions.emplace_back(actions_array[j].asCString());
      }
      decisions.emplace_back(temp);
    }
    jsonFile_decisions.close();
    return true;
  }
} // namespace sentry

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto my_node = std::make_shared<sentry::RobotDecisionNode>();
  rclcpp::spin(my_node);
  rclcpp::shutdown();
  return 0;
}
