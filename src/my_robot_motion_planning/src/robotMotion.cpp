#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <geometry_msgs/Pose.h>
#include <nlohmann/json.hpp>
#include <iostream>

using json = nlohmann::json;

using namespace std;

ros::Duration total_time(0.0);

bool loadPoseWaypointsFromJSON(
    const std::string& filename,
    std::vector<geometry_msgs::Pose>& waypoints)
{
  std::ifstream file(filename);
  if (!file.is_open())
  {
    ROS_ERROR("Failed to open JSON file: %s", filename.c_str());
    return false;
  }

  json j;
  file >> j;

  for (const auto& item : j)
  {
    geometry_msgs::Pose pose;

    pose.position.x = item.at("x").get<double>();
    pose.position.y = item.at("y").get<double>();
    pose.position.z = item.at("z").get<double>();

    pose.orientation.x = item.at("qx").get<double>();
    pose.orientation.y = item.at("qy").get<double>();
    pose.orientation.z = item.at("qz").get<double>();
    pose.orientation.w = item.at("qw").get<double>();

    waypoints.push_back(pose);
  }

  ROS_INFO("Loaded %lu waypoints from JSON", waypoints.size());
  return true;
}

bool loadJointWaypointsFromJSON(
    const std::string& filename,
    std::vector<vector<double>>& waypoints)
{
  std::ifstream file(filename);
  if (!file.is_open())
  {
    ROS_ERROR("Failed to open JSON file: %s", filename.c_str());
    return false;
  }

  json j;
  file >> j;

  for (const auto& item : j)
  {
    vector<double> jointPose;

    jointPose = item.at("solutions").get<vector<double>>();

    waypoints.push_back(jointPose);
  }

  ROS_INFO("Loaded %lu Joint waypoints from JSON", waypoints.size());
  return true;
}

void displayWaypointsPair(const vector<pair<int, geometry_msgs::Pose>> &failed_poses) {
  for(int i=0; i<failed_poses.size(); ++i) {
    cout << "Waypoint number - " << failed_poses[i].first << endl;
  }
  return;
}

void displayTimeStamps(const vector<ros::Duration> &timestamps) {
  for(int i=0; i<timestamps.size(); ++i) {
    cout << "Time taken for " << i + 1 << "th waypoint is "<<timestamps[i] << endl;
    total_time += timestamps[i];
  }
  cout << "Total Time taken to traverse the given points " << total_time << endl;
}

void returnHome(moveit::planning_interface::MoveGroupInterface& move_group) {
  std::cout << "Motion completed ! Now heading to home pose." << std::endl;
  std::vector<double> home_pose = {
        0.0, -1.57, 1.57, 0.0, 0.0, 0.0
    };
  move_group.setJointValueTarget(home_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) ==
                  moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
      ROS_INFO("Home Plan successful, executing...");
      move_group.execute(plan);
  }
}

void executeMotionByPose(const vector<geometry_msgs::Pose> &waypoints, vector<pair<int, geometry_msgs::Pose>> &failed_poses, vector<ros::Duration> &timestamps) {
  static const std::string PLANNING_GROUP = "vin_robot";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  returnHome(move_group); // To make sure to start from home pose
  for(int i=0; i<waypoints.size(); ++i) {
    ros::Time exec_start = ros::Time::now();
    move_group.setPlanningPipelineId("ompl");

    move_group.setPlannerId("RRT");

    move_group.setPlanningTime(5.0);


    ROS_INFO_STREAM("Planning frame: "
                    << move_group.getPlanningFrame());
    ROS_INFO_STREAM("End effector: "
                    << move_group.getEndEffectorLink());

    geometry_msgs::Pose target_pose;
    target_pose.position.x = waypoints[i].position.x;
    target_pose.position.y = waypoints[i].position.y;
    target_pose.position.z = waypoints[i].position.z;

    target_pose.orientation.w = 0.5;
    target_pose.orientation.x = -0.5;
    target_pose.orientation.y = -0.5;
    target_pose.orientation.z = -0.5;

    move_group.setStartStateToCurrentState();

    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        ROS_INFO("Plan successful, executing...");
        move_group.execute(plan);
        ros::Time exec_end = ros::Time::now();
        timestamps.push_back(exec_end - exec_start);
    }
    else
    {
        ROS_WARN("Planning failed");
        pair<int, geometry_msgs::Pose> failed_pair;
        failed_pair.first = i+1;
        failed_pair.second = waypoints[i];
        failed_poses.push_back(failed_pair);
    }
  }
  returnHome(move_group); // To return to the home pose
}

void executeMotionByJoints(vector<vector<double>> &joint_waypoints, vector<pair<int, vector<double>>> &failed_joint_poses, vector<ros::Duration> &joint_timestamps) {
  static const std::string PLANNING_GROUP = "vin_robot";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setPlanningPipelineId("ompl");
  move_group.setPlannerId("RRT");

  returnHome(move_group); // To make sure to start from home pose
  for(int i=0; i<joint_waypoints.size(); ++i) {
    ros::Time exec_start = ros::Time::now();
    move_group.setPlanningTime(5.0);

    ROS_INFO_STREAM("Planning frame: "
                    << move_group.getPlanningFrame());
    ROS_INFO_STREAM("End effector: "
                    << move_group.getEndEffectorLink());

    vector<double> target_pose = joint_waypoints[i];

    move_group.setStartStateToCurrentState();

    move_group.setJointValueTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        ROS_INFO("Plan successful, executing...");
        move_group.execute(plan);
        ros::Time exec_end = ros::Time::now();
        joint_timestamps.push_back(exec_end - exec_start);
    }
    else
    {
        ROS_WARN("Planning failed");
        pair<int, vector<double>> failed_pair;
        failed_pair.first = i+1;
        failed_pair.second = target_pose;
        failed_joint_poses.push_back(failed_pair);
    }
  }
  returnHome(move_group); // To make sure to start from home pose
}

int main(int argc, char** argv)
{
  int opt_type = std::stoi(argv[1]);
  ros::init(argc, argv, "moveit_pose_goal_cpp");
  ros::AsyncSpinner spinner(1);
  ros::Subscriber waypoint_sub_;
  spinner.start();

  ros::NodeHandle n;

  switch(opt_type) {
    case 1: {
      vector<pair<int, geometry_msgs::Pose>> failed_poses;
      vector<ros::Duration> timestamps;
      vector<geometry_msgs::Pose> waypoints;
      string filename = "./points.json";
      loadPoseWaypointsFromJSON(filename, waypoints);
      executeMotionByPose(waypoints, failed_poses, timestamps);
      displayWaypointsPair(failed_poses);
      displayTimeStamps(timestamps);
      break;
    }
    case 2: {
      vector<pair<int, vector<double>>> failed_joint_poses;
      vector<ros::Duration> joint_timestamps;
      vector<vector<double>> joint_waypoints;
      string filename = "./ik_sorted_solutions.json";
      loadJointWaypointsFromJSON(filename, joint_waypoints);
      executeMotionByJoints(joint_waypoints, failed_joint_poses, joint_timestamps);
      displayTimeStamps(joint_timestamps);
    }
  }

  ros::shutdown();
  return 0;
}