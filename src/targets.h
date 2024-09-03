#ifndef __TARGETS_H__
#define __TARGETS_H__

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_msgs/FollowJointTrajectoryActionFeedback.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <vector>
#include <string>

class JointTrajectoryPublisher
{
    public:
        JointTrajectoryPublisher(ros::NodeHandle& nh, const std::string& topic_name)
        {
            pub_ = nh.advertise<trajectory_msgs::JointTrajectory>(topic_name, 10);
        }

        void publish(const std::vector<std::string>& joint_names,
                    const std::vector<std::vector<double>>& positions,
                    const std::vector<std::vector<double>>& velocities,
                    const std::vector<std::vector<double>>& accelerations,
                    const std::vector<std::vector<double>>& efforts,
                    const std::vector<ros::Duration>& time_from_start)
        {
            trajectory_msgs::JointTrajectory msg;
            
            msg.header.stamp = ros::Time::now();
            msg.joint_names = joint_names;
            msg.points.resize(positions.size());
            for (size_t i = 0; i < positions.size(); ++i)
            {
                msg.points[i].positions = positions[i];
                msg.points[i].velocities = velocities[i];
                msg.points[i].accelerations = accelerations[i];
                msg.points[i].effort = efforts[i];
                msg.points[i].time_from_start = time_from_start[i];
            }
            pub_.publish(msg);
        }

    private:
        ros::Publisher pub_;
};

#endif