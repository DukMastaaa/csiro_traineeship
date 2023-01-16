// ROS
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_datatypes.h"

// Gazebo
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/msgs/logical_camera_image.pb.h"
#include "gazebo/gazebo_client.hh"

// STL
#include <functional>
#include <iostream>

geometry_msgs::Pose ignPoseToGeom(ignition::math::v6::Pose3d ign_pose) {
    auto geom_pose = geometry_msgs::Pose();
    geom_pose.position.x = ign_pose.X();
    geom_pose.position.y = ign_pose.Y();
    geom_pose.position.z= ign_pose.Z();
    
    auto q = tf::createQuaternionFromRPY(
            ign_pose.Roll(), ign_pose.Pitch(), ign_pose.Yaw());
    geom_pose.orientation.w = q.w();
    geom_pose.orientation.x = q.x();
    geom_pose.orientation.y = q.y();
    geom_pose.orientation.z = q.z();

    return geom_pose;
}


class LogicalToMoveit {
private:
    // Gazebo node.
    gazebo::transport::NodePtr gazebo_node;

    // Ros node handle.
    ros::NodeHandle ros_node;

    // Gazebo subscriber to the logical camera.
    gazebo::transport::SubscriberPtr gazebo_sub;

    // ROS publisher.
    ros::Publisher ros_pub;

    // Model name of the target object.
    std::string target_name;

    // ROS topic name to publish to.
    std::string pub_topic;

private:
    // Callback when Gazebo transport called.
    void gazeboCallback(ConstLogicalCameraImagePtr& _msg);

public:
    LogicalToMoveit(
            gazebo::transport::NodePtr gazebo_node_,
            ros::NodeHandle ros_node_,
            const std::string& subscribe_topic_,
            const std::string& target_name_,
            const std::string& pub_topic_);
};


LogicalToMoveit::LogicalToMoveit(
        gazebo::transport::NodePtr gazebo_node_,
        ros::NodeHandle ros_node_,
        const std::string& subscribe_topic,
        const std::string& target_name_,
        const std::string& pub_topic_)
    : gazebo_node{gazebo_node_}, ros_node{ros_node_},
    gazebo_sub{}, ros_pub{},
    target_name{target_name_}, pub_topic{pub_topic_}
{
    gazebo_sub = gazebo_node->Subscribe(subscribe_topic,
            &LogicalToMoveit::gazeboCallback, this);
    ros_pub = ros_node.advertise<geometry_msgs::Pose>(pub_topic, 1, false);
}

void LogicalToMoveit::gazeboCallback(ConstLogicalCameraImagePtr& _msg) {
    ROS_INFO_STREAM("received!\n");  
    auto camera_pose = gazebo::msgs::ConvertIgn(_msg->pose());
    for (int i = 0; i < _msg->model_size(); i++) {
        auto model = _msg->model(i);
        ROS_INFO_STREAM(model.name() << "\n");
        if (model.name() == target_name) {
            auto model_relative_pose = gazebo::msgs::ConvertIgn(model.pose());
            auto model_world_pose = model_relative_pose - camera_pose;
            auto msg = ignPoseToGeom(model_world_pose);
            ros_pub.publish(msg);
            return;
        }
    }
    ROS_WARN_STREAM("target not detected\n");
}


int main(int argc, char** argv) {
    // The name of the parameter in the ROS parameter server
    // which contains the Gazebo topic broadcasting logical camera images.
    const std::string gz_topic_param_name = "/logical_cam_gz_topic";

    // Name of the Gazebo model that we're reporting the pose of.
    const std::string target_name = "stone";

    // ROS topic that we publish the pose to.
    const std::string pub_topic = "/logical_target_pose";

    gazebo::client::setup(argc, argv);
    gazebo::transport::NodePtr gazebo_node(new gazebo::transport::Node());
    gazebo_node->Init();
    ROS_INFO_STREAM("gazebo node initialised\n");

    ros::init(argc, argv, "logical_to_moveit");
    ros::NodeHandle ros_node;
    ROS_INFO_STREAM("ros node initialised\n");

    int return_code = 0;

    std::string subscribe_topic;
    if (ros_node.getParam(gz_topic_param_name, subscribe_topic)) {
        LogicalToMoveit converter{gazebo_node, ros_node, subscribe_topic, target_name, pub_topic};
        while (true) {
            gazebo::common::Time::MSleep(10);
            ros::spinOnce();
        }
    } else {
        ROS_ERROR_STREAM(gz_topic_param_name << " not found in parameter server!\n");
        return_code = 1;
    }

    gazebo::shutdown();
    ROS_INFO_STREAM("shutting down\n");
    return return_code;
}
