#include <ros/ros.h>
#include <actionlib_msgs/GoalID.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_canceler_node");
    ros::NodeHandle nh;

    // Publisher to cancel goals
    ros::Publisher cancel_goal_pub = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10);

    // Prepare a cancellation message
    actionlib_msgs::GoalID cancel_msg;
    cancel_msg.id = "";  // You can specify the goal ID you want to cancel here

    // Sleep briefly to allow the publisher to connect
    ros::Duration(1.0).sleep();

    // Publish the cancellation message
    cancel_goal_pub.publish(cancel_msg);

    ROS_INFO("Goal cancellation request sent.");

    ros::spin();
    return 0;
}

