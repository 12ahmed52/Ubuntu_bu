#include <ros/ros.h>
#include "behaviortree_cpp/bt_factory.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>

class Approach : public BT::SyncActionNode {
public:
    Approach(const std::string& name, const BT::NodeConfiguration &config) : BT::SyncActionNode(name, config)
    {
    }

     static BT::PortsList providedPorts()
        {
            return {};
        }

    void approach()
    {
        MoveBaseClient ac("move_base", true);
        if (!ac.waitForServer(ros::Duration(5.0))) {
            ROS_ERROR("Failed to connect to the move_base server.");
        }

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);   
        tag_point.header.frame_id = "tag_0";
        tag_point.point.x = 0.0;
        tag_point.point.y = 0.0;
        tag_point.point.z = 0.8;
    try {
        if (tfBuffer.canTransform("map", tag_point.header.frame_id, ros::Time(0), ros::Duration(5.0)))
        {
        std::cout<<"HERE!!!"<<std::endl;
        geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("map", tag_point.header.frame_id, ros::Time(0));
        geometry_msgs::PointStamped odom_point;
        tf2::doTransform(tag_point, odom_point, transformStamped);
        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, 0); // Roll, pitch, yaw (in radians)
        // Create and populate the goal message
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map"; // Goal is in the "odom" frame
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position = odom_point.point; // Use the transformed point
        goal.target_pose.pose.orientation = tf2::toMsg(orientation);

        // Send the goal
        ROS_INFO("Sending goal...");
        ac.sendGoalAndWait(goal);

        // Wait for the result

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal reached!");
        } else {
            ROS_ERROR("Failed to reach the goal.");
            
        }
    }
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("Transform failed: %s", ex.what());
    }  

    }
    BT::NodeStatus tick() override
    {
        if(!goal_reached)
        {
            approach();
            goal_reached = true;
        }
        return BT::NodeStatus::SUCCESS;
    
    }


private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_vel_pub_;
    geometry_msgs::Twist twist_msg;
    double linear_speed;
    double desired_x_linear;
    double april_depth;
    double current_resultant_distance;
    std::shared_ptr<geometry_msgs::Point> last_position;
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    move_base_msgs::MoveBaseGoal goal;
    geometry_msgs::PointStamped tag_point;
    bool goal_reached = false;
    
};


