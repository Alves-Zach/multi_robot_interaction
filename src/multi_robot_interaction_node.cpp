/**
 * /file multi_robot_interaction_node.cpp
 * /author Emek Baris Kucuktabak
 * /brief ROS node of multi_robot_interaction
 * /version 0.1
 * /date 2020-11-06
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <ros/ros.h>
#include <multi_robot_interaction/MultiRobotInteraction.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_robot_interaction");
    ros::NodeHandle nodeHandle("~");

    MultiRobotInteraction multiRobotInteraction(nodeHandle);
    multiRobotInteraction.initialize();
    ros::Rate rate(400); // ROS Rate at 400Hz
    while(ros::ok()){
        multiRobotInteraction.advance();
        ros::spinOnce();
        rate.sleep();
    }
    multiRobotInteraction.exit();
    return 0;
}

