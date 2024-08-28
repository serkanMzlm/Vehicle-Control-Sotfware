#include "commander_visualization.hpp"

geometry_msgs::msg::TransformStamped visualizationTf2(State_t state, std::string frame_id)
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = rclcpp::Clock().now();
    t.header.frame_id = frame_id;
    t.child_frame_id = "base_link";

    t.transform.translation.x = state.position.x;
    t.transform.translation.y = state.position.y;
    t.transform.translation.z = state.position.z;

    // euler to quaternion
    // tf2::Quaternion q;
    // q.setRPY(msg->pose.orientation.x,
    //          msg->pose.orientation.y,
    //          msg->pose.orientation.z);

    t.transform.rotation.x = state.quaternion.q[0];
    t.transform.rotation.y = state.quaternion.q[1];
    t.transform.rotation.z = state.quaternion.q[2];
    t.transform.rotation.w = state.quaternion.q[3];
    return t;
}

geometry_msgs::msg::PoseStamped visualizationPath(Position_t pose, std::string frame_id)
{
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.header.stamp = rclcpp::Clock().now();
    pose_stamped.pose.position.x = pose.x;
    pose_stamped.pose.position.y = pose.y;
    pose_stamped.pose.position.z = pose.z;
    return pose_stamped;
}