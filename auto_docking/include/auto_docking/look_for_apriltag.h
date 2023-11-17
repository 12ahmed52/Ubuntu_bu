#include <ros/ros.h>
#include "behaviortree_cpp/bt_factory.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "auto_docking/AprilTagDetection.h"
#include "auto_docking/AprilTagDetectionArray.h"

class LookForAprilTag : public BT::SyncActionNode {
public:
    LookForAprilTag(const std::string& name, const BT::NodeConfiguration &config) : BT::SyncActionNode(name, config) {
        // Initialize the ROS subscriber
        tag_sub_ = nh_.subscribe("/tag_detections", 1, &LookForAprilTag::apriltagCallback, this);
        odom_sub_ = nh_.subscribe("/odom", 1, &LookForAprilTag::odomCallback, this);

        // Publisher for velocity commands
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        
        rotation_angle = 0.0;
        initial_yaw_ = 0.0;
        current_yaw = 0.0;
        desired_z_rotation = 0.0;
        rotation_speed_ = 0.2;
        is_rotatoin_finished = false;

    }

     static BT::PortsList providedPorts()
        {
            return {};
        }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
    {
        // Convert IMU quaternion to Euler angles
        geometry_msgs::Pose pose = odom_msg->pose.pose;
        geometry_msgs::Quaternion orientation = pose.orientation;
        tf2::Quaternion odom_quat;
        tf2::fromMsg(pose.orientation, odom_quat);
        double roll, pitch;
        tf2::Matrix3x3(odom_quat).getRPY(roll, pitch, current_yaw);
    }
    
    void apriltagCallback(const auto_docking::AprilTagDetectionArray::ConstPtr &msg)
    {
        if (msg->detections.empty())
            aptiltag_flag = false;
        else
            aptiltag_flag = true;
    }

    
    double normalizeAngle(double angle)
    {
        while (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }
        while (angle <= -M_PI) {
            angle += 2.0 * M_PI;
        }
        return angle;
    }
    
    void rotate_robot(double rotate_angle)
    {

        initial_yaw_ = current_yaw;
        is_rotatoin_finished = false;
        
        while(!is_rotatoin_finished)
        {
          
            double current_angle = current_yaw - initial_yaw_;
            double remaining_rotation = normalizeAngle(rotate_angle - current_angle);
            if (fabs(remaining_rotation) > 0.05)
            {
                // Rotate in the appropriate direction
                desired_z_rotation = remaining_rotation > 0 ? rotation_speed_ : -rotation_speed_;
                publish_msg();
                ros::Rate rate(10);
                rate.sleep();
                ros::spinOnce();
            
            }
            else{
                desired_z_rotation = 0.0;
                publish_msg();  
                ros::Rate rate(10);
                rate.sleep(); 
                ros::Rate loop_rate(1.0/2.0);
                loop_rate.sleep();
                is_rotatoin_finished = true;  
            }
        }
    }
    
    void publish_msg()
    {
        twist_msg.angular.z = desired_z_rotation;
        cmd_vel_pub_.publish(twist_msg);
    }

    BT::NodeStatus tick() override
    {
        number_of_tryes = 0;
        while(number_of_tryes != 12)
        {
            ros::Rate rate(10);
            rate.sleep();
            ros::spinOnce();
            if(!aptiltag_flag)
            {
                rotate_robot(field_of_view);
                number_of_tryes++;
            }
            else
                break;
        }
        if(!aptiltag_flag)
        {
            ROS_INFO("NO APRIL TAG FOUND!!");
            return BT::NodeStatus::FAILURE;
        }   
        else
            return BT::NodeStatus::SUCCESS;
            
    }


private:
    ros::NodeHandle nh_;
    ros::Subscriber tag_sub_;
    ros::Subscriber odom_sub_;
    bool aptiltag_flag = false;
    char number_of_tryes = 0;
    const float field_of_view = 0.523599; //30 degrees
    ros::Publisher cmd_vel_pub_;
    double rotation_angle;
    double initial_yaw_;
    double current_yaw;
    double desired_z_rotation;
    double rotation_speed_;
    bool is_rotatoin_finished;
    geometry_msgs::Twist twist_msg;
};


