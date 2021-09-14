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
//In order to inform the node which controls the left arm of the current position of the target, a « tf
//broadcaster » broadcasts the transformation between the new frame and the base frame of the
//Baxter robot.

//The controller node is a « tf listener »: it monitors the position and orientation of the target with
//respect to the base frame of the Baxter and calls a service to calculate a joint position which allows
//reaching the goal, if any.

//To use a ROS service, the client node sends a service request message and waits for the
//corresponding service response message. Both messages are strictly typed data structures./
//The service you will need to use is Baxter's Inverse Kinematics Service. Beware: ROS defines a
//more general IK service.
//Most information you will find online using a search engine is likely to beabout the general
//ROS IK service, not Baxter's IK service and it can be very confusing. If you limit
//yourself to the resources given in the present text, you will avoid that confusion.
//Baxter's IK service is defined in the baxter_core_msgs package. To list the service messages defined
//in the package, type: rosservice find baxter_core_msgs/ and type the TAB key twice. The
//inverse kinematics service message is listed as baxter_core_msgs/SolvePositionIK .
//Then if you type rosservice find baxter_core_msgs/SolvePositionIK, you will see the names of the
//two inverse kinematics services, one for each arm of the Baxter.
//To get the description of the message, use either of these methodsTo get the description of the message,
//use either of these methods
//Go to https://github.com/RethinkRobotics/baxter_common/blob/master/baxter_core_msgs/
//srv/SolvePositionIK.srv
//Type rossrv show baxter_core_msgs/SolvePositionIK .
//You will need to write a service client, which is described at :
//http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29

//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

//ROS
#include "ros/ros.h"
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_listener.h>
// Include here the ".h" files corresponding to the topic type you use.
// ...

// You may have a number of globals here.
//...

// Callback functions...

void myFirstCallback(/* Define here the variable which hold the message */){
    // ... Callback function code
}


int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "puppet_hand_node");

    // Define your node handles: YOU NEED AT LEAST ONE !
    ros::NodeHandle nh;
    // Read the node parameters if any
    // ...

    // Declare your node's subscriptions and service clients
    // ...

    // Declare you publishers and service servers
    ros::Publisher pub = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1);
    ros::service::waitForService("/ExternalTools/left/PositionKinematicsNode/IKService");
    ros::ServiceClient inverse_kinematics_client =nh.serviceClient<baxter_core_msgs::SolvePositionIK> ("/ExternalTools/left/PositionKinematicsNode/IKService");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    uint32_t frame_id = 0;
    ros::Rate rate(10);   // Or other rate.
    while (ros::ok()){
        ros::spinOnce();

        // Your node's code goes here.
        geometry_msgs::TransformStamped right_hand_position;
        try{

        right_hand_position = tfBuffer.lookupTransform("base", "frame_09",
                                   ros::Time(0));
        }
        catch (tf2::TransformException &ex)
         {
         ROS_WARN("%s",ex.what());
         ros::Duration(1.0).sleep();
         continue;
         }
       // From this point, I  have all I need.
        //target frame in IK form
        geometry_msgs::PoseStamped ik_pose;
        ik_pose.pose.position.x = right_hand_position.transform.translation.x;
        ik_pose.pose.position.y = right_hand_position.transform.translation.y;
        ik_pose.pose.position.z = right_hand_position.transform.translation.z;

        ik_pose.pose.orientation.x = right_hand_position.transform.rotation.x;
        ik_pose.pose.orientation.y = right_hand_position.transform.rotation.y;
        ik_pose.pose.orientation.z = right_hand_position.transform.rotation.z;
        ik_pose.pose.orientation.w = right_hand_position.transform.rotation.w;

        //ik_pose.header.seq = 'base';
        ik_pose.header.stamp = ros::Time(0);
        ik_pose.header.frame_id = frame_id++;

        //definition of service request
        baxter_core_msgs::SolvePositionIK ik_service_request;
        ik_service_request.request.pose_stamp.push_back(ik_pose);


        if(inverse_kinematics_client.call(ik_service_request)){
            // arrays to preserve
            sensor_msgs::JointState state_msg = ik_service_request.response.joints[0];

            //msgs which should be sent to left_hand (new msgs to publish)
            baxter_core_msgs::JointCommand data;
            data.mode = data.POSITION_MODE;
            for(int i = 0; i < last_state.name.size(); ++i) {
                data.names.push_back(state_msg.name[i]);
                data.command.push_back(state_msg.position[i]);
            }
        }else{
            ROS_ERROR("Failure :: Call to service")
        }


        pub.publish(data)


        rate.sleep();
    }
}
