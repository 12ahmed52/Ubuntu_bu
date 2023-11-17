#include <ros/ros.h>
#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include "auto_docking/look_for_apriltag.h"
#include "auto_docking/approach.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "auto_docking");
    ros::NodeHandle nh;

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<LookForAprilTag>("LookForAprilTag");
    factory.registerNodeType<Approach>("Approach");
    std::string xml_filename = "/home/ahmed/RoboAds/catkin_ws/src/auto_docking/cfg/at_behavior.xml";
    ROS_INFO("Loading XML : %s", xml_filename.c_str());
    BT::Tree tree = factory.createTreeFromFile(xml_filename);

        // Create the behavior tree loggers
    BT::StdCoutLogger console_logger(tree);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    ros::Rate r(1);
    while (ros::ok()) {
        status = tree.rootNode()->executeTick(); 
        r.sleep();
    }
}

