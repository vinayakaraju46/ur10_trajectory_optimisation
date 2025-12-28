#!/usr/bin/env python3
import rospy
import moveit_commander
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from copy import deepcopy
import math

class MarkerWaypointsPlanner:
    def __init__(self):
        rospy.init_node("marker_waypoints_ik_planner")

        # MoveIt init
        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("vin_robot")  # <-- your group

        # 🔹 Planner settings (IMPORTANT)
        self.group.set_planner_id("RRT")
        self.group.set_planning_time(5.0)
        self.group.set_num_planning_attempts(10)
        self.group.allow_replanning(True)

        # Optional but recommended
        self.group.set_max_velocity_scaling_factor(0.3)
        self.group.set_max_acceleration_scaling_factor(0.3)

        # Waypoints storage
        self.waypoints = []

        rospy.Subscriber(
            "/random_quaternion_markers",
            Marker,
            self.marker_callback,
            queue_size=1
        )

        rospy.loginfo("Waiting for marker waypoints...")
        while not self.waypoints and not rospy.is_shutdown():
            rospy.sleep(0.5)

        rospy.loginfo(f"Received {len(self.waypoints)} waypoints")
        self.execute_waypoints()

    def marker_callback(self, msg):
        self.waypoints.clear()

        for p in msg.points:
            pose = Pose()
            pose.position.x = p.x
            pose.position.y = p.y
            pose.position.z = p.z

            # 🔹 Orientation: fixed (can be replaced by quaternion list later)
            pose.orientation.w = 1.0

            self.waypoints.append(deepcopy(pose))

    def execute_waypoints(self):
        for idx, pose in enumerate(self.waypoints):
            rospy.loginfo(f"Planning to waypoint {idx + 1}/{len(self.waypoints)}")

            self.group.
            self.group.set_planning_time(15.0)
            self.group.set_pose_target(pose)

            print(self.group.get_planner_id())          # should say RRT
            print(self.group.get_planning_frame())      # check frame
            print(self.group.get_end_effector_link()) 

            success, plan, planning_time, error_code = self.group.plan()

            if not success:
                rospy.logwarn(f"Planning failed for waypoint {idx + 1}")
                self.group.clear_pose_targets()
                continue
            
            self.group.execute(plan, wait=True)
            self.group.stop()
            self.group.clear_pose_targets()

            rospy.sleep(0.2)

        rospy.loginfo("Finished executing all waypoints")

if __name__ == "__main__":
    try:
        MarkerWaypointsPlanner()
    except rospy.ROSInterruptException:
        pass
