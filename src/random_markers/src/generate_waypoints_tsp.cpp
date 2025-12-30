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

#include <chrono>

using json = nlohmann::json;

struct JointNode
{
    int pose_number;
    std::vector<double> joint_angles;  // one solution of IK
    std::vector<std::vector<double>> solutions;
    int best_solution_index = 0;
};

std::vector<int> path;
std::vector<JointNode> nodes;
std::vector<bool> visited(nodes.size(), false);
int tsp_type = 1;

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

double jointDistanceMultipleSolutions(const std::vector<double>& a, const std::vector<double>& b)
{
    double dist = 0.0;
    for (size_t i = 0; i < a.size(); ++i)
    {
        double diff = a[i] - b[i];
        dist += diff * diff;
    }
    return std::sqrt(dist);
}

std::vector<std::vector<double>> buildDistanceMatrix(
    const std::vector<JointNode>& nodes)
{
    size_t N = nodes.size();
    std::vector<std::vector<double>> dist(N, std::vector<double>(N, 0.0));

    for (size_t i = 0; i < N; ++i)
    {
        for (size_t j = i + 1; j < N; ++j)
        {
            double d = jointDistance(nodes[i],
                                     nodes[j]);
            dist[i][j] = d;
            dist[j][i] = d;  // symmetry
        }
    }
    return dist;
}


std::vector<int> nearestNeighbourTSPFromHome(
    const std::vector<JointNode>& nodes,
    const std::vector<double>& home_pose)
{
    std::vector<int> path;
    std::vector<bool> visited(nodes.size(), false);

    std::vector<double> current = home_pose;

    JointNode currentNode;
    currentNode.pose_number = 0;
    currentNode.joint_angles = home_pose;

    for (size_t step = 1; step < nodes.size(); ++step)
    {
        double best_dist = std::numeric_limits<double>::max();
        int best_idx = -1;

        for (size_t i = 1; i < nodes.size(); ++i)
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


std::vector<std::pair<int,int>>
nearestNeighbourTSPFromHomeMultipleSolutions(
    const std::vector<JointNode>& nodes,
    const std::vector<double>& home_pose)
{
    std::vector<std::pair<int,int>> path;
    const size_t N = nodes.size();

    std::vector<bool> visited(N, false);

    // Current joint configuration (starts at home)
    std::vector<double> current_joints = home_pose;

    // If node 0 is home, mark it visited
    visited[0] = true;

    for (size_t step = 0; step < N - 1; ++step)
    {
        double best_dist = std::numeric_limits<double>::max();
        int best_node_idx = -1;
        int best_solution_idx = -1;

        for (size_t i = 0; i < N; ++i)
        {
            if (visited[i]) continue;

            for (size_t j = 0; j < nodes[i].solutions.size(); ++j)
            {
                double dist =
                    jointDistanceMultipleSolutions(
                        current_joints,
                        nodes[i].solutions[j]);

                if (dist < best_dist)
                {
                    best_dist = dist;
                    best_node_idx = i;
                    best_solution_idx = j;
                }
            }
        }

        if (best_node_idx == -1)
            break;

        // Move to the selected node + solution
        visited[best_node_idx] = true;
        path.emplace_back(best_node_idx, best_solution_idx);

        current_joints = nodes[best_node_idx].solutions[best_solution_idx];
    }

    return path;
}


std::vector<int> nearestNeighbourMemoizedTSPFromHome(
    const std::vector<JointNode>& nodes,
    const std::vector<double>& home_pose,
    std::vector<std::vector<double>>& dist
) {
    
    size_t N = nodes.size();
    std::vector<bool> visited(N, false);
    std::vector<int> path;

    int current = 0;

    for (size_t step = 1; step < N; ++step)
    {
        int next = -1;
        double best_dist = std::numeric_limits<double>::max();

        for (size_t i = 1; i < N; ++i)
        {
            if (visited[i]) continue;

            double d = dist[current][i];
            if (d < best_dist)
            {
                best_dist = d;
                next = i;
            }
        }

        visited[next] = true;
        path.push_back(next);
        current = next;
    }

    return path;
}


void sortByTSP(json &ik_data) {
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
    

    std::vector<int> tsp_path;
    switch(tsp_type) {
        case 1: {
            auto start = std::chrono::high_resolution_clock::now();
            tsp_path = nearestNeighbourTSPFromHome(nodes, home_pose);
            auto end = std::chrono::high_resolution_clock::now();
            double time_ms =
                std::chrono::duration<double, std::milli>(end - start).count();
            std::cout << "Time taken for OG TSP -> " << time_ms << std::endl;
            break;
        }
        case 2: {
            auto memo_start = std::chrono::high_resolution_clock::now();
            std::vector<std::vector<double>> dist = buildDistanceMatrix(nodes);
            tsp_path = nearestNeighbourMemoizedTSPFromHome(nodes, home_pose, dist);
            auto memo_end = std::chrono::high_resolution_clock::now();
            double memo_time_ms =
                std::chrono::duration<double, std::milli>(memo_end - memo_start).count();
            std::cout << "Time taken for Memoized TSP -> " << memo_time_ms << std::endl;
            break;
        }
        default:
            tsp_path = nearestNeighbourTSPFromHome(nodes, home_pose);
            break;
    }
    
    json ik_sorted_data = json::array();

    ROS_INFO("TSP path starting from HOME:");
    for (int idx : tsp_path)
    {
        // ROS_INFO("Visit pose %d", nodes[idx].pose_number);
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

    return;
}


void sortByTSPMultipleSolution(json &ik_data) {
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
            entry["solutions"][0].get<std::vector<double>>();
        node.solutions =
            entry["solutions"].get<std::vector<std::vector<double>>>(); // first IK

        nodes.push_back(node);
    }

    ROS_INFO("Loaded %lu valid IK poses", nodes.size());
    

    std::vector<std::pair<int,int>> tsp_path;
    
    tsp_path = nearestNeighbourTSPFromHomeMultipleSolutions(nodes, home_pose);
           
    json ik_sorted_data = json::array();

    ROS_INFO("TSP path starting from HOME:");
    for (std::pair<int,int> idx : tsp_path)
    {
        // ROS_INFO("Visit pose %d", nodes[idx].pose_number);
        ik_sorted_data.push_back({
            {"pose_number", nodes[idx.first].pose_number},
            {"number_of_solutions", 1},
            {"solutions", nodes[idx.first].joint_angles},
            {"best_solution_index", idx.second}
        });
    }

    // Save Sorted IK solutions JSON
    std::ofstream ik_sorted_file("ik_sorted_solutions.json");
    ik_sorted_file << ik_sorted_data.dump(4);
    ik_sorted_file.close();

    return;
}




void append_home_pose(json& ik_data) {
    std::vector<double> home_pose = {
        0.0, -1.57, 1.57, 0.0, 0.0, 0.0
    };

    ik_data.push_back({
        {"pose_number", 0},
        {"number_of_solutions", 1},
        {"solutions", {home_pose}}
    });
    
    return ;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "random_quaternion_marker_node");
    ros::NodeHandle nh("~");

    tsp_type = std::stoi(argv[1]);
    int number_of_poses = std::stoi(argv[2]);

    std::cout << "TSP type selected: " << tsp_type << std::endl;

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>(
        "/random_quaternion_markers", 1, true
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

    
    nh.param("num_points", num_points, number_of_poses);
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

    append_home_pose(ik_data);

    for (int i = 0; i < num_points; ++i)
    {
        geometry_msgs::Point p = generateRandomPointInSphere(radius);
        marker.points.push_back(p);

        tf2::Quaternion q = randomQuaternion();

        geometry_msgs::Pose target_pose;
        target_pose.position.x = marker.pose.position.x + p.x;
        target_pose.position.y = marker.pose.position.y + p.y;
        target_pose.position.z = marker.pose.position.z + p.z;
        target_pose.orientation.w = 0.5;
        target_pose.orientation.x = -0.5;
        target_pose.orientation.y = -0.5;
        target_pose.orientation.z = -0.5;

        data.push_back({
            {"pose_number", i},
            {"x", marker.pose.position.x + p.x},
            {"y", marker.pose.position.y + p.y},
            {"z", marker.pose.position.z + p.z},
            {"qx", target_pose.orientation.x},
            {"qy", target_pose.orientation.y},
            {"qz", target_pose.orientation.z},
            {"qw", target_pose.orientation.w}
        });
        
        std::vector<std::vector<double>> solutions;
        kinematics::KinematicsResult result;
        kinematics::KinematicsQueryOptions options;
        std::vector<double> seed = {0.0, -1.526, 1.526, 0.0, 0.0, 0.0};

        bool found = solver->getPositionIK({target_pose}, seed, solutions, result, options);

        if (found && solutions.size() > 0)
        {
            ROS_INFO("IK solution found!");
            ik_data.push_back({
                {"pose_number", i+1},
                {"number_of_solutions", solutions.size()},
                {"solutions", solutions}
            });
        }
        else
        {
            ROS_WARN("IK solution not found!");
            ik_data.push_back({
                {"pose_number", i+1},
                {"number_of_solutions", 0},
                {"solutions", json::array()}
            });
        }
    }

    append_home_pose(ik_data);

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

    // sortByTSP(ik_data);
    sortByTSPMultipleSolution(ik_data);

    ros::spin();

    ros::shutdown();
    return 0;
}
