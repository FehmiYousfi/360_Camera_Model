#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>

class RobotInfoCollector
{
public:
    RobotInfoCollector()
    {
        ros::NodeHandle nh;
        tf_listener_ = new tf::TransformListener();
        joint_states_sub_ = nh.subscribe("/joint_states", 10, &RobotInfoCollector::jointStatesCallback, this);
    }

    ~RobotInfoCollector()
    {
        delete tf_listener_;
    }

    void collectInfo()
    {
        ros::Rate rate(10.0);
        while (ros::ok())
        {
            try
            {
                tf::StampedTransform transform;
                tf_listener_->lookupTransform("Fixed_base", "link",ros::Time(0), transform);

                ROS_INFO("Transform from world to base_link:");
                ROS_INFO("Translation: x = %f, y = %f, z = %f",
                         transform.getOrigin().x(),
                         transform.getOrigin().y(),
                         transform.getOrigin().z());
                ROS_INFO("Rotation: x = %f, y = %f, z = %f, w = %f",
                         transform.getRotation().x(),
                         transform.getRotation().y(),
                         transform.getRotation().z(),
                         transform.getRotation().w());
            }
            catch (tf::TransformException& ex)
            {
                ROS_WARN("Transform error: %s", ex.what());
            }
            ROS_INFO("Current joint states:");
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                ROS_INFO("Joint %s: position = %f, velocity = %f, effort = %f",
                         joint_names_[i].c_str(),
                         joint_positions_[i],
                         joint_velocities_[i],
                         joint_efforts_[i]);
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        joint_names_ = msg->name;
        joint_positions_ = msg->position;
        joint_velocities_ = msg->velocity;
        joint_efforts_ = msg->effort;
    }

    tf::TransformListener* tf_listener_;
    ros::Subscriber joint_states_sub_;
    std::vector<std::string> joint_names_;
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_efforts_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_info_collector");
    RobotInfoCollector collector;
    collector.collectInfo();
    return 0;
}
