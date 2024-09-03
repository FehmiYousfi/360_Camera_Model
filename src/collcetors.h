#ifndef __COLLECTORS_H__
#define __COLLECTORS_H__

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_msgs/FollowJointTrajectoryActionFeedback.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <vector>
#include <string>

class JointTrajectoryData
{
public:
    std::vector<double> desired_positions;
    std::vector<double> desired_velocities;
    std::vector<double> desired_accelerations;
    std::vector<double> desired_effort;

    std::vector<double> actual_positions;
    std::vector<double> actual_velocities;
    std::vector<double> actual_accelerations;
    std::vector<double> actual_effort;

    std::vector<double> error_positions;
    std::vector<double> error_velocities;
    std::vector<double> error_accelerations;
    std::vector<double> error_effort;

    void printData() const
    {
        ROS_INFO("Desired Positions: %s", vectorToString(desired_positions).c_str());
        ROS_INFO("Desired Velocities: %s", vectorToString(desired_velocities).c_str());
        ROS_INFO("Desired Accelerations: %s", vectorToString(desired_accelerations).c_str());
        ROS_INFO("Desired Effort: %s", vectorToString(desired_effort).c_str());

        ROS_INFO("Actual Positions: %s", vectorToString(actual_positions).c_str());
        ROS_INFO("Actual Velocities: %s", vectorToString(actual_velocities).c_str());
        ROS_INFO("Actual Accelerations: %s", vectorToString(actual_accelerations).c_str());
        ROS_INFO("Actual Effort: %s", vectorToString(actual_effort).c_str());

        ROS_INFO("Error Positions: %s", vectorToString(error_positions).c_str());
        ROS_INFO("Error Velocities: %s", vectorToString(error_velocities).c_str());
        ROS_INFO("Error Accelerations: %s", vectorToString(error_accelerations).c_str());
        ROS_INFO("Error Effort: %s", vectorToString(error_effort).c_str());
    }

private:
    std::string vectorToString(const std::vector<double>& vec) const
    {
        std::stringstream ss;
        for (size_t i = 0; i < vec.size(); ++i)
        {
            ss << vec[i];
            if (i < vec.size() - 1)
                ss << ", ";
        }
        return ss.str();
    }
};

class GoalStatusData
{
public:
    struct StatusInfo
    {
        std::string goal_id;
        uint8_t status;
        std::string text;
    };
    std::vector<StatusInfo> status_list;
    void printData() const
    {
        for (const auto& status_info : status_list)
        {
            ROS_INFO("Goal ID: %s, Status: %d, Text: %s", 
                     status_info.goal_id.c_str(), 
                     status_info.status, 
                     status_info.text.c_str());
        }
    }
};

class FollowJointTrajectoryResultData
{
public:
    struct StatusInfo
    {
        std::string goal_id;
        uint8_t status;
        std::string text;
    };

    StatusInfo status_info;

    int32_t error_code;
    std::string error_string;

    void printData() const
    {
        ROS_INFO("Goal ID: %s, Status: %d, Status Text: %s",
                 status_info.goal_id.c_str(),
                 status_info.status,
                 status_info.text.c_str());

        ROS_INFO("Result Error Code: %d, Error String: %s",
                 error_code,
                 error_string.c_str());
    }
};

class FollowJointTrajectoryFeedbackData
{
public:
    struct TrajectoryPoint
    {
        std::vector<double> positions;
        std::vector<double> velocities;
        std::vector<double> accelerations;
        std::vector<double> effort;
        
        // Conversion function to populate this struct from trajectory_msgs::JointTrajectoryPoint
        void fromMsg(const trajectory_msgs::JointTrajectoryPoint& msg)
        {
            positions = msg.positions;
            velocities = msg.velocities;
            accelerations = msg.accelerations;
            effort = msg.effort;
        }
    };

    struct FeedbackInfo
    {
        std::vector<std::string> joint_names;
        TrajectoryPoint desired;
        TrajectoryPoint actual;
        TrajectoryPoint error;
    };

    FeedbackInfo feedback_info;

    void printData() const
    {
        ROS_INFO("Joint Names: %s", vectorToString(feedback_info.joint_names).c_str());

        ROS_INFO("Desired Positions: %s", vectorToString(feedback_info.desired.positions).c_str());
        ROS_INFO("Desired Velocities: %s", vectorToString(feedback_info.desired.velocities).c_str());
        ROS_INFO("Desired Accelerations: %s", vectorToString(feedback_info.desired.accelerations).c_str());
        ROS_INFO("Desired Effort: %s", vectorToString(feedback_info.desired.effort).c_str());

        ROS_INFO("Actual Positions: %s", vectorToString(feedback_info.actual.positions).c_str());
        ROS_INFO("Actual Velocities: %s", vectorToString(feedback_info.actual.velocities).c_str());
        ROS_INFO("Actual Accelerations: %s", vectorToString(feedback_info.actual.accelerations).c_str());
        ROS_INFO("Actual Effort: %s", vectorToString(feedback_info.actual.effort).c_str());

        ROS_INFO("Error Positions: %s", vectorToString(feedback_info.error.positions).c_str());
        ROS_INFO("Error Velocities: %s", vectorToString(feedback_info.error.velocities).c_str());
        ROS_INFO("Error Accelerations: %s", vectorToString(feedback_info.error.accelerations).c_str());
        ROS_INFO("Error Effort: %s", vectorToString(feedback_info.error.effort).c_str());
    }

private:
    template <typename T>
    std::string vectorToString(const std::vector<T>& vec) const
    {
        std::stringstream ss;
        for (size_t i = 0; i < vec.size(); ++i)
        {
            ss << vec[i];
            if (i < vec.size() - 1)
                ss << ", ";
        }
        return ss.str();
    }

    std::string vectorToString(const std::vector<std::string>& vec) const
    {
        std::stringstream ss;
        for (size_t i = 0; i < vec.size(); ++i)
        {
            ss << vec[i];
            if (i < vec.size() - 1)
                ss << ", ";
        }
        return ss.str();
    }
};

#endif