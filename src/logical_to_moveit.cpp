// ROS
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"

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

tf::Transform gzPoseToTransform(gazebo::msgs::Pose gz_pose) {
    auto gz_q = gz_pose.orientation();
    auto gz_p = gz_pose.position();
    return tf::Transform{
            tf::Quaternion{gz_q.x(), gz_q.y(), gz_q.z(), gz_q.w()},
            tf::Vector3{gz_p.x(), gz_p.y(), gz_p.z()}};
}


class LogicalToMoveit {
private:
    // Gazebo node.
    gazebo::transport::NodePtr gazebo_node;

    // Ros node handle.
    ros::NodeHandle ros_node;

    // Gazebo subscriber to the logical camera.
    gazebo::transport::SubscriberPtr gazebo_sub;

    // Broadcaster for tf tree.
    tf::TransformBroadcaster broadcaster;

    // Model name of the target object.
    std::string target_name;

private:
    // Callback when Gazebo transport called.
    void gazeboCallback(ConstLogicalCameraImagePtr& _msg);

public:
    LogicalToMoveit(
            gazebo::transport::NodePtr gazebo_node_,
            ros::NodeHandle ros_node_,
            const std::string& subscribe_topic_,
            const std::string& target_name_);
};


LogicalToMoveit::LogicalToMoveit(
        gazebo::transport::NodePtr gazebo_node_,
        ros::NodeHandle ros_node_,
        const std::string& subscribe_topic,
        const std::string& target_name_)
    : gazebo_node{gazebo_node_}, ros_node{ros_node_},
    gazebo_sub{}, broadcaster{}, target_name{target_name_}
{
    gazebo_sub = gazebo_node->Subscribe(subscribe_topic,
            &LogicalToMoveit::gazeboCallback, this);
}

void LogicalToMoveit::gazeboCallback(ConstLogicalCameraImagePtr& _msg) {
    ROS_INFO_STREAM("received!\n");  
    auto now = ros::Time::now();
    auto camera_transform = gzPoseToTransform(_msg->pose());
    broadcaster.sendTransform(tf::StampedTransform(camera_transform, now, "world", "camera"));

    for (int i = 0; i < _msg->model_size(); i++) {
        auto model = _msg->model(i);
        ROS_INFO_STREAM(model.name() << "\n");
        if (model.name() == target_name) {
            auto model_relative_transform = gzPoseToTransform(model.pose());
            broadcaster.sendTransform(tf::StampedTransform(
                    model_relative_transform, now, "camera", target_name));
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
        LogicalToMoveit converter{gazebo_node, ros_node, subscribe_topic, target_name};
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
