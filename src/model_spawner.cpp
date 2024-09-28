#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <ros/package.h>
#include <string>
#include <fstream>
#include <sstream>

bool spawnModel(ros::ServiceClient &client, const std::string &model_name, const std::string &model_xml, const std::string &robot_namespace, const geometry_msgs::Pose &pose) {
    gazebo_msgs::SpawnModel srv;
    srv.request.model_name = model_name;
    srv.request.model_xml = model_xml;
    srv.request.robot_namespace = robot_namespace;
    srv.request.initial_pose = pose;

    if (client.call(srv)) {
        return srv.response.success;
    } else {
        ROS_ERROR("Failed to call service spawn_model");
        return false;
    }
}

std::string loadModelXML(const std::string &file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        ROS_ERROR("Unable to open file: %s", file_path.c_str());
        return "";
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "model_spawner");
    ros::NodeHandle nh;

    // Get model spawn parameters from the launch file or set default values
    double arg_x, arg_y, arg_z, arg_R, arg_P, arg_Y;
    nh.param("arg_x", arg_x, 0.0);
    nh.param("arg_y", arg_y, 0.0);
    nh.param("arg_z", arg_z, 1.0);
    nh.param("arg_R", arg_R, 0.0);
    nh.param("arg_P", arg_P, 0.0);
    nh.param("arg_Y", arg_Y, 0.0);

    // Prepare to spawn models
    ros::ServiceClient spawn_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    std::string model_path = ros::package::getPath("360_Camera_Model");
    std::string message_loader = "Model Path is: ";
    std::string custom_prefix = "/urdf/360_Camera_Model.urdf";
    ROS_INFO_STREAM(message_loader << model_path);
    std::string model_xml = loadModelXML(model_path + custom_prefix);
    if (model_xml.empty()) {
        ROS_ERROR("Failed to load model XML.");
        return 1; 
    }

    geometry_msgs::Pose pose;
    pose.position.x = arg_x;
    pose.position.y = arg_y;
    pose.position.z = arg_z;
    pose.orientation.w = 1.0;

    for (int i = 0; i < 5; ++i) {
        std::string model_name = "camera_model_" + std::to_string(i);
        pose.position.x += 1.0;
        if (spawnModel(spawn_client, model_name, model_xml, "", pose)) {
            ROS_INFO("Spawned model: %s", model_name.c_str());
        } else {
            ROS_ERROR("Failed to spawn model: %s", model_name.c_str());
        }
    }

    ros::spin();
    return 0;
}
