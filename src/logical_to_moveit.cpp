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

    // Model name of the target object.
    std::string target_name;

    // Most recent pose from the logical camera sensor.
    geometry_msgs::Pose recent_pose;

    ros::Timer timer;

private:
    // Callback when Gazebo transport called.
    void gazebo_callback(ConstLogicalCameraImagePtr& _msg);

    // Callback for timer.
    void timer_callback(const ros::TimerEvent& e);

public:
    LogicalToMoveit(
            gazebo::transport::NodePtr gazebo_node_,
            ros::NodeHandle ros_node_,
            const std::string& camera_model_name_,
            const std::string& camera_link_name_,
            const std::string& target_name_);
    
    geometry_msgs::Pose pose();
};


LogicalToMoveit::LogicalToMoveit(
        gazebo::transport::NodePtr gazebo_node_,
        ros::NodeHandle ros_node_,
        const std::string& camera_model_name_,
        const std::string& camera_link_name_,
        const std::string& target_name_)
    : gazebo_node{gazebo_node_}, ros_node{ros_node_}, gazebo_sub{},
    target_name{target_name_},
    recent_pose{}, timer{}
{
    const std::string subscribe_topic = "~/" + camera_model_name_
            + "/" + camera_link_name_ + "/logical_camera/models";
    gazebo_sub = gazebo_node->Subscribe(subscribe_topic,
            &LogicalToMoveit::gazebo_callback, this);
    timer = ros_node.createTimer(
            ros::Duration(0.5),
            boost::bind(&LogicalToMoveit::timer_callback, this, _1));
}

void LogicalToMoveit::gazebo_callback(ConstLogicalCameraImagePtr& _msg) {
    ROS_INFO_STREAM("received!\n");
    auto cameraPose = gazebo::msgs::ConvertIgn(_msg->pose());
    for (int i = 0; i < _msg->model_size(); i++) {
        auto model = _msg->model(i);
        ROS_INFO_STREAM(model.name() << "\n");
        if (model.name() == target_name) {
            auto modelRelativePose = gazebo::msgs::ConvertIgn(model.pose());
            auto modelWorldPose = modelRelativePose - cameraPose;
            recent_pose = ignPoseToGeom(modelWorldPose);
            return;
        }
    }
    ROS_WARN_STREAM("target not detected\n");
}

void LogicalToMoveit::timer_callback(const ros::TimerEvent& e) {
    ROS_INFO_STREAM(recent_pose << "\n");
}

geometry_msgs::Pose LogicalToMoveit::pose() {
    return recent_pose;
}


int main(int argc, char** argv) {
    const std::string camera_model_name = "post";
    const std::string camera_link_name = "link_for_camera";
    const std::string target_name = "stone";

    // new is ew
    gazebo::client::setup(argc, argv);
    gazebo::transport::NodePtr gazebo_node(new gazebo::transport::Node());
    gazebo_node->Init();
    ROS_INFO_STREAM("gazebo node initialised\n");

    ros::init(argc, argv, "logical_to_moveit");
    ros::NodeHandle ros_node;
    ROS_INFO_STREAM("ros node initialised\n");
    
    LogicalToMoveit converter{
            gazebo_node, ros_node,
            camera_model_name, camera_link_name, target_name};

    while (true) {
        gazebo::common::Time::MSleep(10);
        ros::spinOnce();
    }

    gazebo::shutdown();

    ROS_INFO_STREAM("shutting down\n");
}

