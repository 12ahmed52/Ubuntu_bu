#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class RotateTurtlebotNode {
public:
    RotateTurtlebotNode() {
        // Initialize the ROS node handle
        nh_ = ros::NodeHandle("~");

        // Subscriber for IMU data
        imu_sub_ = nh_.subscribe("/imu", 10, &RotateTurtlebotNode::imuCallback, this);

        // Publisher for velocity commands
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Initialize desired rotation angle (in radians)
        desired_angle_ = -(-6.497 * M_PI) / 180;  // 90 degrees

        // Set rotation speed
        rotation_speed_ = 0.2;  // Adjust as needed

        // Initialize variables
        initial_yaw_ = 0.0;
        is_rotating_ = false;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
        // Convert IMU quaternion to Euler angles
        tf2::Quaternion imu_quat;
        tf2::fromMsg(imu_msg->orientation, imu_quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(imu_quat).getRPY(roll, pitch, yaw);

        // Initialize initial yaw angle if not set
        if (!is_rotating_) {
            initial_yaw_ = yaw;
            is_rotating_ = true;
        }

        // Calculate the remaining rotation needed to reach the desired angle
        double current_angle = yaw - initial_yaw_;
        double remaining_rotation = normalizeAngle(desired_angle_ - current_angle);

        // Create a Twist message to send control commands
        geometry_msgs::Twist twist_msg;

        // Check if the TurtleBot3 has reached the desired angle
        if (fabs(remaining_rotation) > 0.01) {
            // Rotate in the appropriate direction
            twist_msg.angular.z = remaining_rotation > 0 ? rotation_speed_ : -rotation_speed_;
        } else {
            // Stop the rotation
            twist_msg.angular.z = 0.0;
            desired_angle_ = 0;
            is_rotating_ = false;  // Reset rotation flag
        }

        // Publish the control command
        cmd_vel_pub_.publish(twist_msg);
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }
        while (angle <= -M_PI) {
            angle += 2.0 * M_PI;
        }
        return angle;
    }
private:
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Publisher cmd_vel_pub_;
    double desired_angle_;
    double rotation_speed_;
    double initial_yaw_;
    bool is_rotating_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rotate_turtlebot");
    RotateTurtlebotNode node;
    ros::spin();
    return 0;
}

