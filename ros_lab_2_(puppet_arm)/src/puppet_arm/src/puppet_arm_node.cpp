/**
 * \file
 * \brief 
 * \author 
 * \version 0.1
 * \date 
 * 
 * \param[in] 
 * 
 * Subscribes to: <BR>
 *    ° 
 * 
 * Publishes to: <BR>
 *    ° 
 *
 * Description
 *
 */


//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

//ROS
#include "ros/ros.h"

// Include here the ".h" files corresponding to the topic type you use.
#include <baxter_core_msgs/JointCommand.h>
#include <sensor_msgs/JointState.h>
// You may have a number of globals here.
// Callback functions...

sensor_msgs::JointState last_state ;
bool state_received = false ;
void jointStateCallback(sensor_msgs::JointState state_msg){
    last_state = state_msg ;
    if(last_state.name.size() < 10) return ;
    state_received = true ;
}


int main (int argc, char** argv)
{

    //ROS Initialization2
    ros::init(argc, argv, "puppet_arm_node");

    // Define your node handles: YOU NEED AT LEAST ONE !
    ros::NodeHandle nh;
    // Read the node parameters if any
    // ...

    // Declare your node's subscriptions and service clients
    ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>("/robot/joint_states", 2, jointStateCallback);
    // Declare you publishers and service servers
    ros::Publisher pub = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1);

    ros::Rate rate(10);   // Or other rate.
    while (ros::ok()){
        ros::spinOnce();

                // %create new message for publishing
        if(state_received)
        {
            baxter_core_msgs::JointCommand data;
            data.mode = data.POSITION_MODE;
            int sign = -1 ;
            for(int i = 0; i < last_state.name.size(); ++i) {
                std::string name = last_state.name[i];
                if (name.find("right") != std::string::npos) {
                    // check whether name containts 'right' letter
                    std::string left_name = name.replace(0, 5, "left");
                    data.names.push_back(left_name);
                    data.command.push_back(sign*last_state.position[i]);
                    sign *= -1 ;
                }
            }

            // publish
            pub.publish(data);
        }

        rate.sleep();
    }
}
