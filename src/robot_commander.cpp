#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#define full_Turn 6.28
#define LockPos 0

// Function to create a trajectory point
trajectory_msgs::JointTrajectoryPoint createTrajectoryPoint(double pos1, double pos2, double pos3, double duration) {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(pos1); // Position for Rotator_1_joint
    point.positions.push_back(pos2); // Position for Rotator_2_joint
    point.positions.push_back(pos3); // Position for Rotator_3_joint
    point.velocities.push_back(0.0); // Velocity for Rotator_1_joint
    point.velocities.push_back(0.0); // Velocity for Rotator_2_joint
    point.velocities.push_back(0.0); // Velocity for Rotator_3_joint
    point.accelerations.push_back(0.0); // Acceleration for Rotator_1_joint
    point.accelerations.push_back(0.0); // Acceleration for Rotator_2_joint
    point.accelerations.push_back(0.0); // Acceleration for Rotator_3_joint
    point.effort.push_back(0.0); // Effort for Rotator_1_joint
    point.effort.push_back(0.0); // Effort for Rotator_2_joint
    point.effort.push_back(0.0); // Effort for Rotator_3_joint
    point.time_from_start = ros::Duration(duration); // Duration of the point
    return point;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "joint_controller_node");
    ros::NodeHandle nh;

    // Publishers for the trajectory topics
    ros::Publisher rotator_1_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ROTATOR_1_controller/command", 10);
    ros::Publisher rotator_2_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ROTATOR_2_controller/command", 10);
    ros::Publisher rotator_3_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ROTATOR_3_controller/command", 10);

    ros::Rate loop_rate(1); // Loop rate set to 1 Hz

    ROS_INFO("Starting Joint Command Control.");
    loop_rate.sleep();
    ros::spinOnce();

    // Start time for the trajectory
    ros::Time start_time = ros::Time::now();

    // Rotator 1 trajectory
    trajectory_msgs::JointTrajectory rotator_1_trajectory;
    rotator_1_trajectory.header.stamp = start_time;
    rotator_1_trajectory.header.frame_id = "Rotator_1";
    rotator_1_trajectory.joint_names.push_back("Rotator_1_to_Fixed_base");
    rotator_1_trajectory.points.push_back(createTrajectoryPoint(full_Turn / 4, 0.0, 0.0, 1.0));
    rotator_1_pub.publish(rotator_1_trajectory);
    ROS_INFO("Published joint trajectory commands for Rotator_1.");
    ros::spinOnce();
    loop_rate.sleep();

    // Rotator 2 trajectory
    trajectory_msgs::JointTrajectory rotator_2_trajectory;
    rotator_2_trajectory.header.stamp = ros::Time::now();
    rotator_2_trajectory.header.frame_id = "Rotator_2";
    rotator_2_trajectory.joint_names.push_back("Rotator_2_to_Rotator_1");
    rotator_2_trajectory.points.push_back(createTrajectoryPoint(0.0, full_Turn / 4, 0.0, 1.0));
    rotator_2_pub.publish(rotator_2_trajectory);
    ROS_INFO("Published joint trajectory commands for Rotator_2.");
    ros::spinOnce();
    loop_rate.sleep();

    // Rotator 3 trajectory
    trajectory_msgs::JointTrajectory rotator_3_trajectory;
    rotator_3_trajectory.header.stamp = ros::Time::now();
    rotator_3_trajectory.header.frame_id = "Rotator_3";
    rotator_3_trajectory.joint_names.push_back("Rotator_3_to_Rotator_2");
    rotator_3_trajectory.points.push_back(createTrajectoryPoint(0.0, 0.0, full_Turn / 4, 1.0));
    rotator_3_pub.publish(rotator_3_trajectory);
    ROS_INFO("Published joint trajectory commands for Rotator_3.");
    ros::spinOnce();
    loop_rate.sleep();

    // Return Rotator 1 to initial position
    trajectory_msgs::JointTrajectory rotator_1_trajectory_rev;
    rotator_1_trajectory_rev.header.stamp = ros::Time::now();
    rotator_1_trajectory_rev.header.frame_id = "Rotator_1";
    rotator_1_trajectory_rev.joint_names.push_back("Rotator_1_to_Fixed_base");
    rotator_1_trajectory_rev.points.push_back(createTrajectoryPoint(0.0, 0.0, 0.0, 1.0));
    rotator_1_pub.publish(rotator_1_trajectory_rev);
    ROS_INFO("Returning joint Rotator_1 to initial position.");
    ros::spinOnce();
    loop_rate.sleep();

    // Return Rotator 2 to initial position
    trajectory_msgs::JointTrajectory rotator_2_trajectory_rev;
    rotator_2_trajectory_rev.header.stamp = ros::Time::now();
    rotator_2_trajectory_rev.header.frame_id = "Rotator_2";
    rotator_2_trajectory_rev.joint_names.push_back("Rotator_2_to_Rotator_1");
    rotator_2_trajectory_rev.points.push_back(createTrajectoryPoint(0.0, 0.0, 0.0, 1.0));
    rotator_2_pub.publish(rotator_2_trajectory_rev);
    ROS_INFO("Returning joint Rotator_2 to initial position.");
    ros::spinOnce();
    loop_rate.sleep();

    // Return Rotator 3 to initial position
    trajectory_msgs::JointTrajectory rotator_3_trajectory_rev;
    rotator_3_trajectory_rev.header.stamp = ros::Time::now();
    rotator_3_trajectory_rev.header.frame_id = "Rotator_3";
    rotator_3_trajectory_rev.joint_names.push_back("Rotator_3_to_Rotator_2");
    rotator_3_trajectory_rev.points.push_back(createTrajectoryPoint(0.0, 0.0, 0.0, 1.0));
    rotator_3_pub.publish(rotator_3_trajectory_rev);
    ROS_INFO("Returning joint Rotator_3 to initial position.");
    ros::spinOnce();
    loop_rate.sleep();

    return 0;
}
