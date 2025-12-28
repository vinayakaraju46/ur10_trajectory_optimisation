#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <random>
#include <cmath>
#include <fstream>

#include <nlohmann/json.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/move_group_interface/move_group_interface.h>

using json = nlohmann::json;

struct JointNode
{
    int pose_number;
    std::vector<double> joint_angles;  // one solution of IK
};

std::vector<int> path;
std::vector<JointNode> nodes;
std::vector<bool> visited(nodes.size(), false);

/* Generate a random point uniformly inside a sphere */
geometry_msgs::Point generateRandomPointInSphere(double radius)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<double> dist(0.0, 1.0);

    double u = dist(gen);
    double v = dist(gen);
    double w = dist(gen);

    double theta = 2.0 * M_PI * u;
    double phi   = acos(2.0 * v - 1.0);
    double r     = radius * cbrt(w);

    geometry_msgs::Point p;
    p.x = r * sin(phi) * cos(theta);
    p.y = r * sin(phi) * sin(theta);
    p.z = r * cos(phi);

    return p;
}

/* Generate random quaternion from roll, pitch, yaw */
tf2::Quaternion randomQuaternion()
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<double> angle(-M_PI, M_PI);

    double roll  = angle(gen);
    double pitch = angle(gen);
    double yaw   = angle(gen);

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return q;
}

double jointDistance(const JointNode& a, const JointNode& b)
{
    double dist = 0.0;
    for (size_t i = 0; i < a.joint_angles.size(); ++i)
    {
        double diff = a.joint_angles[i] - b.joint_angles[i];
        dist += diff * diff;
    }
    return std::sqrt(dist);
}

std::vector<int> nearestNeighbourTSPFromHome(
    const std::vector<JointNode>& nodes,
    const std::vector<double>& home_pose)
{
    std::vector<int> path;
    std::vector<bool> visited(nodes.size(), false);

    std::vector<double> current = home_pose;

    JointNode currentNode;
    currentNode.pose_number = -1;
    currentNode.joint_angles = home_pose;

    for (size_t step = 0; step < nodes.size(); ++step)
    {
        double best_dist = std::numeric_limits<double>::max();
        int best_idx = -1;

        for (size_t i = 0; i < nodes.size(); ++i)
        {
            if (visited[i]) continue;

            double dist = jointDistance(currentNode, nodes[i]);
            if (dist < best_dist)
            {
                best_dist = dist;
                best_idx = i;
            }
        }

        if (best_idx == -1)
            break;

        visited[best_idx] = true;
        path.push_back(best_idx);
        currentNode.joint_angles = nodes[best_idx].joint_angles;
    }

    return path;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "random_quaternion_marker_node");
    ros::NodeHandle nh("~");

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>(
        "/random_quaternion_markers", 1, true  // latched
    );

    int num_points;
    double radius;
    std::string output_file;

    robot_model_loader::RobotModelLoader model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = model_loader.getModel();

    const robot_model::JointModelGroup* joint_model_group =
        kinematic_model->getJointModelGroup("vin_robot");

    // Create a RobotState to use IK
    robot_state::RobotState state(kinematic_model);

    // For IK, use the solver pointer
    const kinematics::KinematicsBaseConstPtr solver = joint_model_group->getSolverInstance();

    // JointNode home_pose;
    // home_pose.pose_number = -1;
    // home_pose.joint_angles = {0.0, -1.526, 1.526, 0.0, 0.0, 0.0}; // example home pose


    nh.param("num_points", num_points, 100);
    nh.param("radius", radius, 0.2);
    nh.param("output_file", output_file, std::string("./points.json"));

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "random_points";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    marker.color.r = 0.0;
    marker.color.g = 0.7;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    marker.pose.position.x = 0.5;
    marker.pose.position.y = 0.5;
    marker.pose.position.z = 0.5;
    marker.pose.orientation.w = 1.0;

    json data = json::array();
    json ik_data = json::array();

    for (int i = 0; i < num_points; ++i)
    {
        geometry_msgs::Point p = generateRandomPointInSphere(radius);
        marker.points.push_back(p);

        tf2::Quaternion q = randomQuaternion();

        data.push_back({
            {"pose_number", i},
            {"x", marker.pose.position.x + p.x},
            {"y", marker.pose.position.y + p.y},
            {"z", marker.pose.position.z + p.z},
            {"qx", -0.5},
            {"qy", -0.5},
            {"qz", -0.5},
            {"qw", 0.5}
        });
        geometry_msgs::Pose target_pose;
        target_pose.position.x = marker.pose.position.x + p.x;
        target_pose.position.y = marker.pose.position.y + p.y;
        target_pose.position.z = marker.pose.position.z + p.z;
        target_pose.orientation.w = 0.5;
        target_pose.orientation.x = -0.5;
        target_pose.orientation.y = -0.5;
        target_pose.orientation.z = -0.5;
        std::vector<std::vector<double>> solutions;
        kinematics::KinematicsResult result;
        kinematics::KinematicsQueryOptions options;
        std::vector<double> seed = {0.0, -1.526, 1.526, 0.0, 0.0, 0.0};

        bool found = solver->getPositionIK({target_pose}, seed, solutions, result, options);

        if (found && solutions.size() > 0)
        {
            ROS_INFO("IK solution found!");
            ik_data.push_back({
                {"pose_number", i},
                {"number_of_solutions", solutions.size()},
                {"solutions", solutions}
            });
        }
        else
        {
            ROS_WARN("IK solution not found!");
            ik_data.push_back({
                {"pose_number", i},
                {"number_of_solutions", 0},
                {"solutions", json::array()}
            });
        }
    }

    marker_pub.publish(marker);
    ROS_INFO("Published %d markers", num_points);

    std::ofstream file(output_file);
    file << data.dump(4);
    file.close();

    // Save IK solutions JSON
    std::ofstream ik_file("ik_solutions.json");
    ik_file << ik_data.dump(4);
    ik_file.close();

    ROS_INFO("Saved JSON to %s", output_file.c_str());


    std::vector<double> home_pose = {
        0.0, -1.57, 1.57, 0.0, 0.0, 0.0
    };

    

    for (const auto& entry : ik_data)
    {
        if (entry["number_of_solutions"].get<int>() == 0)
            continue;

        JointNode node;
        node.pose_number = entry["pose_number"];
        node.joint_angles =
            entry["solutions"][0].get<std::vector<double>>(); // first IK

        nodes.push_back(node);
    }

    ROS_INFO("Loaded %lu valid IK poses", nodes.size());

    /* ---------- RUN TSP ---------- */
    std::vector<int> tsp_path =
        nearestNeighbourTSPFromHome(nodes, home_pose);
    
    json ik_sorted_data = json::array();

    /* ---------- PRINT RESULT ---------- */
    ROS_INFO("TSP path starting from HOME:");
    for (int idx : tsp_path)
    {
        ROS_INFO("Visit pose %d", nodes[idx].pose_number);
        ik_sorted_data.push_back({
            {"pose_number", nodes[idx].pose_number},
            {"number_of_solutions", 1},
            {"solutions", nodes[idx].joint_angles}
        });
    }

    // Save Sorted IK solutions JSON
    std::ofstream ik_sorted_file("ik_sorted_solutions.json");
    ik_sorted_file << ik_sorted_data.dump(4);
    ik_sorted_file.close();

    ros::spin();

    ros::shutdown();
    return 0;
}
