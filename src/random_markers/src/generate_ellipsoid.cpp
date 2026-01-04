


#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "manipulability_ellipsoid_node");
    ros::NodeHandle nh;

    ros::Publisher marker_pub =
        nh.advertise<visualization_msgs::Marker>("manipulability_ellipsoid", 1, true);

    // --- Load model
    pinocchio::Model model;
    pinocchio::urdf::buildModel(
        "/home/vinayaka/ros_ws/ur10_robot.urdf",
        model
    );

    pinocchio::Data data(model);

    // --- Joint configuration (example)
    Eigen::VectorXd q(model.nq);
    q << 0.0, -1.57, 1.57, 0.0, 0.0, 0.0;  // test pose

    // --- Kinematics
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    // --- Jacobian
    pinocchio::FrameIndex ee_frame_id = model.getFrameId("rounded_tip");

    Eigen::MatrixXd J(6, model.nv);
    pinocchio::computeFrameJacobian(
        model, data, q, ee_frame_id,
        pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
        J
    );

    Eigen::Matrix3d Jv = J.topRows<3>();
    Eigen::Matrix3d A = Jv * Jv.transpose();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(A);

    // --- Marker
    visualization_msgs::Marker ellipsoid;
    ellipsoid.header.frame_id = "base_link";
    ellipsoid.header.stamp = ros::Time::now();
    ellipsoid.ns = "manipulability";
    ellipsoid.id = 0;
    ellipsoid.type = visualization_msgs::Marker::SPHERE;
    ellipsoid.action = visualization_msgs::Marker::ADD;

    Eigen::Vector3d p = data.oMf[ee_frame_id].translation();
    ellipsoid.pose.position.x = p.x();
    ellipsoid.pose.position.y = p.y();
    ellipsoid.pose.position.z = p.z();

    Eigen::Quaterniond q_ellipsoid(solver.eigenvectors());
    ellipsoid.pose.orientation.x = q_ellipsoid.x();
    ellipsoid.pose.orientation.y = q_ellipsoid.y();
    ellipsoid.pose.orientation.z = q_ellipsoid.z();
    ellipsoid.pose.orientation.w = q_ellipsoid.w();

    ellipsoid.scale.x = std::sqrt(solver.eigenvalues()[0]) * 0.2;
    ellipsoid.scale.y = std::sqrt(solver.eigenvalues()[1]) * 0.2;
    ellipsoid.scale.z = std::sqrt(solver.eigenvalues()[2]) * 0.2;

    ellipsoid.color.r = 0.2;
    ellipsoid.color.g = 0.8;
    ellipsoid.color.b = 0.2;
    ellipsoid.color.a = 0.5;

    marker_pub.publish(ellipsoid);

    ros::spin();
    return 0;
}
