#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

enum States
{
    NOT_ALIGNED,
    ALIGNED,
    NOT_OFFSETED,
    OFFSETED,
    NOT_FAR,
    FAR 
};

enum Actions
{
    ALIGN,
    APPROACH,
    DEOFFSET
};

class RotateTurtlebotNode {
public:
    RotateTurtlebotNode() {
        // Initialize the ROS node handle
        nh_ = ros::NodeHandle("~");

        // Subscriber for IMU data
        imu_sub_ = nh_.subscribe("/imu", 10, &RotateTurtlebotNode::imuCallback, this);

        // Publisher for velocity commands
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        
        odom_sub_ = nh_.subscribe("/odom", 10, &RotateTurtlebotNode::odomCallback, this);

        // Initialize desired rotation angle (in radians)
        desired_angle_ = 0.0;

        // Set rotation speed
        rotation_speed_ = 0.2;
        linear_speed = 0.1;
        // Initialize variables
        initial_yaw_ = 0.0;
        current_x = 0.0;
        current_resultant_distance = 0;
        current_y = 0;
        initial_resultant_distance = 0.0;
        april_depth = 0.0;
        is_rotating_ = false;
        DataArrived = false;
        state = FAR;
        last_position = nullptr;
        
        x_offset = 0.0;
        desired_z_rotation = 0.0;
        desired_x_linear = 0.0;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
    {
        // Convert IMU quaternion to Euler angles
        tf2::Quaternion imu_quat;
        tf2::fromMsg(imu_msg->orientation, imu_quat);
        double roll, pitch;
        tf2::Matrix3x3(imu_quat).getRPY(roll, pitch, current_yaw);
        if(!is_rotating_)
            DataArrived = true;
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
    {
        const geometry_msgs::Point& current_position = odom_msg->pose.pose.position;
        if (last_position == nullptr) {
            last_position = std::make_shared<geometry_msgs::Point>(current_position);
        } else {
            double delta_x = current_position.x - last_position->x;
            double delta_y = current_position.y - last_position->y;
            double distance_increment = sqrt(delta_x * delta_x + delta_y * delta_y);
            current_resultant_distance += distance_increment;
            *last_position = current_position;
            }
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
    
    double getDesiredAngle()
    {
          tf2_ros::Buffer tf_buffer;
          tf2_ros::TransformListener tf_listener(tf_buffer);
          double roll, pitch, yaw;

        try
        {
           
            if (tf_buffer.canTransform("camera_rgb_optical_frame", "tag_0", ros::Time(0), ros::Duration(5.0)))
            {
                geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform("camera_rgb_optical_frame", "tag_0", ros::Time(0));
                tf2::Quaternion rotation;
                tf2::fromMsg(transformStamped.transform.rotation, rotation);
                tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
            }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Failed to lookup transform: %s", ex.what());
        }
        
        return (-1 * pitch);
    }
    
    States getCurrentState()
    {
        return state;
    }
    void move_robot(double distance)
    {   
        start:
        double remaining_distance = abs(distance - current_resultant_distance);
        std::cout<<remaining_distance<<"         "<< current_resultant_distance <<std::endl; 
        if (fabs(remaining_distance) > 0.02)
        {
            desired_x_linear = linear_speed;
            publish_msg();
            ros::Rate rate(10);
            rate.sleep();
            ros::spinOnce();
            goto start; 
        }
        else{
            desired_x_linear = 0.0;
            publish_msg();  
            ros::Rate rate(10);
            rate.sleep(); 
            ros::Rate loop_rate(1.0/2.0);   
            
    }
    last_position = nullptr;
    current_resultant_distance = 0.0;
    }

    void rotate_robot(double rotate_angle)
    {

        initial_yaw_ = current_yaw;
        
        start:
            
        double current_angle = current_yaw - initial_yaw_;
        double remaining_rotation = normalizeAngle(rotate_angle - current_angle);
        if (fabs(remaining_rotation) > 0.02)
        {
            // Rotate in the appropriate direction
            desired_z_rotation = remaining_rotation > 0 ? rotation_speed_ : -rotation_speed_;
            publish_msg();
            ros::Rate rate(10);
            rate.sleep();
            ros::spinOnce();
            goto start;
        }
        else
            desired_z_rotation = 0.0;
            publish_msg();  
            ros::Rate rate(10);
            rate.sleep(); 
            ros::Rate loop_rate(1.0/2.0);
            loop_rate.sleep();  
            
    }
    
    double getOffset()
    {
          tf2_ros::Buffer tf_buffer;
          tf2_ros::TransformListener tf_listener(tf_buffer);
          double offset;

        try
        {
           
            if (tf_buffer.canTransform("camera_rgb_optical_frame", "tag_0", ros::Time(0), ros::Duration(5.0)))
            {
                geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform("camera_rgb_optical_frame", "tag_0", ros::Time(0));
                offset = transformStamped.transform.translation.x;
            }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Failed to lookup transform: %s", ex.what());
        }
        return offset;

    }
    double get_depth()
    {
          double theta = getDesiredAngle();
          tf2_ros::Buffer tf_buffer;
          tf2_ros::TransformListener tf_listener(tf_buffer);
          double depth;

        try
        {
           
            if (tf_buffer.canTransform("camera_rgb_optical_frame", "tag_0", ros::Time(0), ros::Duration(5.0)))
            {
                geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform("camera_rgb_optical_frame", "tag_0", ros::Time(0));
                depth = transformStamped.transform.translation.z;
                depth = depth * cos(theta);
            }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Failed to lookup transform: %s", ex.what());
        }
        return depth;

    }
    
    void AllignRobot()
    {
        if (!is_rotating_ && DataArrived)
        {
            initial_yaw_ = current_yaw;
            is_rotating_ = true;
            desired_angle_ = getDesiredAngle();
        }
        if(DataArrived)
        {
            double current_angle = current_yaw - initial_yaw_;
            double remaining_rotation = normalizeAngle(desired_angle_ - current_angle);
            if (fabs(remaining_rotation) > 0.02)
            {
                // Rotate in the appropriate direction
                desired_z_rotation = remaining_rotation > 0 ? rotation_speed_ : -rotation_speed_;
            }
            else
            {
                // Stop the rotation
                desired_z_rotation = 0.0;
                desired_angle_ = 0.0;
                is_rotating_ = false;  // Reset rotation flag
                DataArrived = false;
                state = ALIGNED;
                publish_msg();
                ros::Rate loop_rate(1.0/2.0);
                loop_rate.sleep();
            }
        }
        
    }
    void DeOffset()
    {
        x_offset = getOffset();
        std::cout<<"offset:"<<x_offset<<std::endl;
        double rotate_angle = 0;
        if(x_offset > 0)
            rotate_angle = -(M_PI/2);
        else
            rotate_angle = (M_PI/2);  
        rotate_robot(rotate_angle);//check counter or clock
        move_robot(abs(x_offset));
        rotate_robot(-rotate_angle);
        state = NOT_OFFSETED;
        AllignRobot();
        state = NOT_OFFSETED;
        ros::Rate loop_rate(1.0/4.0);
        loop_rate.sleep();
        desired_angle_ = getDesiredAngle();
        rotate_robot(desired_angle_);
        state = NOT_OFFSETED;
    }
    
    void approach()
    {
        april_depth = get_depth();
        std::cout<<"april_depth:"<<april_depth<<std::endl;
        move_robot(april_depth - 0.5);
        ros::Rate loop_rate(1.0/2.0);
        loop_rate.sleep();
        state = NOT_ALIGNED;        
    }
    void publish_msg()
    {
        twist_msg.angular.z = desired_z_rotation;
        twist_msg.linear.x = desired_x_linear;
        cmd_vel_pub_.publish(twist_msg);
    }
private:
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_vel_pub_;
    double desired_angle_;
    double rotation_speed_;
    double linear_speed;
    double initial_yaw_;
    double current_yaw;
    double desired_z_rotation;
    double desired_x_linear;
    double x_offset;
    double april_depth;
    double current_x;
    double current_y;
    double current_resultant_distance;
    double initial_resultant_distance;
    bool is_rotating_;
    bool DataArrived;
    States state;
    Actions action;
    geometry_msgs::Twist twist_msg;
    std::shared_ptr<geometry_msgs::Point> last_position;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rotate_turtlebot");
    RotateTurtlebotNode node;
    ros::Rate rate(10);
    while(ros::ok)
    {
        if(node.getCurrentState() == States::FAR)
            node.approach();
        if(node.getCurrentState() == States::NOT_ALIGNED)
            node.AllignRobot();
        if(node.getCurrentState() == States::ALIGNED)
             node.DeOffset();    
        node.publish_msg();  
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}


