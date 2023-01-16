
// ROS
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


#include <moveit_msgs/DisplayRobotState.h>

#include <moveit_msgs/DisplayTrajectory.h>


#include <moveit_msgs/AttachedCollisionObject.h>

#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// STL
#include <iostream>
#include <atomic>
#include <thread>

namespace rvt = rviz_visual_tools;


// This class wraps a CollisionObject to encapsulate away
// target geometry and transformations from the moving code.
class Target {
public:

    moveit_msgs::CollisionObject collision_object;

public:
    // Sets up the wrapped collision object and adds it to the given planning interface.
    Target(const std::string& id, const std::string& frame_id,
            moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);

    // Initialises the given collision object with the target's geometry and initial pose.
    
    static void add_geometry(moveit_msgs::CollisionObject& collision_object);

    // Moves the collision object to the given transform.
    void move(const tf::StampedTransform& newTransform,
        moveit::planning_interface::PlanningSceneInterface& planning_interface);
};

Target::Target(const std::string& id, const std::string& frame_id,
        moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
    : collision_object{}
{
    collision_object.header.frame_id = frame_id;
    collision_object.id = id;
    add_geometry(collision_object);  // initialises geometry and pose
    collision_object.operation = collision_object.ADD;

    // Add the object into the world
    std::vector<moveit_msgs::CollisionObject> cobjs;
    cobjs.push_back(collision_object);
    planning_scene_interface.addCollisionObjects(cobjs);

    collision_object.primitives.clear();
}

void Target::add_geometry(moveit_msgs::CollisionObject& collision_object) {
    shape_msgs::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions.resize(3);
    box.dimensions[box.BOX_X] = 0.025;
    box.dimensions[box.BOX_Y] = 0.032;
    box.dimensions[box.BOX_Z] = 0.064;
    collision_object.primitives.push_back(box);

    geometry_msgs::Pose box_pose;
    box_pose.orientation.x = 1.0;
    box_pose.position.x = 0;
    box_pose.position.y = 0;
    box_pose.position.z = 0;
    collision_object.primitive_poses.push_back(box_pose);
}

void Target::move(const tf::StampedTransform& newTransform,
        moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
    collision_object.operation = collision_object.MOVE;
    geometry_msgs::Pose& box_pose = collision_object.pose;
    box_pose.orientation.w = newTransform.getRotation().w();
    box_pose.orientation.x = newTransform.getRotation().x();
    box_pose.orientation.y = newTransform.getRotation().y();
    box_pose.orientation.z = newTransform.getRotation().z();
    box_pose.position.x = newTransform.getOrigin().x();
    box_pose.position.y = newTransform.getOrigin().y();
    box_pose.position.z = newTransform.getOrigin().z();
    planning_scene_interface.applyCollisionObject(collision_object);
}



class StoneMover {
private:
    static const std::string TARGET_ID;

    // Ros node handle.
    ros::NodeHandle ros_node;

    // Objects for MoveIt integration.
    moveit::planning_interface::MoveGroupInterface& move_group_interface;
    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface;
    moveit_visual_tools::MoveItVisualTools& visual_tools;

    // Target that wraps a CollisionObject.
    Target target;

    // Transform listener to receive pose of target.
    tf::TransformListener transform_listener;

    // Timer that updates the pose of the collision object.
    ros::Timer cameraUpdateTimer;

    // Stores whether the target is currently attached to the arm.
    // This controls whether the pose of the target is updated by the subscriber.
    bool object_attached;

    // Mutex to protect accesses to object_attached.
    std::mutex object_attached_mutex;

private:
    void poseCallback(const ros::TimerEvent& event);

public:
    StoneMover(
            ros::NodeHandle ros_node_,
            moveit::planning_interface::MoveGroupInterface& move_group_interface_,
            moveit::planning_interface::PlanningSceneInterface& planning_scene_interface_,
            moveit_visual_tools::MoveItVisualTools& visual_tools_);
    
};

const std::string StoneMover::TARGET_ID = "stone";

StoneMover::StoneMover(
        ros::NodeHandle ros_node_,
        moveit::planning_interface::MoveGroupInterface& move_group_interface_,
        moveit::planning_interface::PlanningSceneInterface& planning_scene_interface_,
        moveit_visual_tools::MoveItVisualTools& visual_tools_)
    : ros_node{ros_node_},
    move_group_interface{move_group_interface_},
    planning_scene_interface{planning_scene_interface_},
    visual_tools{visual_tools_},
    target{
        StoneMover::TARGET_ID,
        move_group_interface_.getPlanningFrame(),
        planning_scene_interface_
    },
    transform_listener{},
    cameraUpdateTimer{},
    object_attached{false}, object_attached_mutex{}
{
    // const uint32_t queue_size = 1;  // We don't care about old measurements
    // pose_sub = ros_node.subscribe(sub_topic_, queue_size, &StoneMover::poseCallback, this);
    cameraUpdateTimer = ros_node.createTimer(ros::Duration(0.1), &StoneMover::poseCallback, this);
}


void StoneMover::poseCallback(const ros::TimerEvent& event) {
    // Acquire the mutex for the whole time we're in this function
    std::lock_guard<std::mutex> lock_guard(object_attached_mutex);

    if (object_attached) {
        // We shouldn't update the pose as MoveIt knows the target is attached to the arm
        return;
    }

    tf::StampedTransform target_transform;
    try {
        transform_listener.lookupTransform("panda_link0", TARGET_ID, ros::Time(0), target_transform);
        target.move(target_transform, planning_scene_interface);
    }
    catch (tf::TransformException e) {
        ROS_WARN_STREAM(e.what() << '\n');
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "stone_mover");
    ros::NodeHandle nh;

    // We need to spin asynchronously for the move group interface to work.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    const std::string PLANNING_GROUP = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Remote control
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    StoneMover stone_mover{nh, move_group_interface, planning_scene_interface, visual_tools};

    ros::waitForShutdown();
}
