#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_msgs/FollowJointTrajectoryActionFeedback.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <vector>
#include <string>

#include "collcetors.h"


class RobotInfoCollector
{
public:
    RobotInfoCollector()
    {
        ros::NodeHandle nh;

        rotator1_feedback_sub_ = nh.subscribe("/ROTATOR_1_controller/follow_joint_trajectory/feedback", 10, &RobotInfoCollector::feedbackCallback, this);
        rotator1_result_sub_ = nh.subscribe("/ROTATOR_1_controller/follow_joint_trajectory/result", 10, &RobotInfoCollector::resultCallback, this);
        rotator1_status_sub_ = nh.subscribe("/ROTATOR_1_controller/follow_joint_trajectory/status", 10, &RobotInfoCollector::statusCallback, this);
        rotator1_state_sub_ = nh.subscribe("/ROTATOR_1_controller/state", 10, &RobotInfoCollector::stateCallback, this);

    }

    void collectInfo()
    {
        ros::spin();
    }

private:

    void feedbackCallback(const control_msgs::FollowJointTrajectoryActionFeedback::ConstPtr& msg)
    {
        FollowJointTrajectoryFeedbackData data;
        data.feedback_info.joint_names = msg->feedback.joint_names;
        data.feedback_info.desired.fromMsg(msg->feedback.desired);
        data.feedback_info.actual.fromMsg(msg->feedback.actual);
        data.feedback_info.error.fromMsg(msg->feedback.error);

        data.printData();
    }

    void resultCallback(const control_msgs::FollowJointTrajectoryActionResult::ConstPtr& msg)
    {
        FollowJointTrajectoryResultData data;

        data.status_info.goal_id = msg->status.goal_id.id;
        data.status_info.status = msg->status.status;
        data.status_info.text = msg->status.text;
        data.error_code = msg->result.error_code;
        data.error_string = msg->result.error_string;

        data.printData();
    }
    void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
    {
        GoalStatusData data;
        for (const auto& status : msg->status_list)
        {
            GoalStatusData::StatusInfo info;
            info.goal_id = status.goal_id.id;
            info.status = status.status;
            info.text = status.text;

            data.status_list.push_back(info);
        }
        data.printData();
    }


    void stateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
    {
        JointTrajectoryData data;

        data.desired_positions = msg->desired.positions;
        data.desired_velocities = msg->desired.velocities;
        data.desired_accelerations = msg->desired.accelerations;
        data.desired_effort = msg->desired.effort;

        data.actual_positions = msg->actual.positions;
        data.actual_velocities = msg->actual.velocities;
        data.actual_accelerations = msg->actual.accelerations;
        data.actual_effort = msg->actual.effort;

        data.error_positions = msg->error.positions;
        data.error_velocities = msg->error.velocities;
        data.error_accelerations = msg->error.accelerations;
        data.error_effort = msg->error.effort;

        data.printData();
    }


    ros::Subscriber rotator1_feedback_sub_;
    ros::Subscriber rotator1_result_sub_;
    ros::Subscriber rotator1_status_sub_;
    ros::Subscriber rotator1_state_sub_;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_info_collector");
    RobotInfoCollector collector;
    collector.collectInfo();
    return 0;
}
