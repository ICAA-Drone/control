#include <math.h>
#include <ros/ros.h>
#include "mavros_msgs/PositionTarget.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/RCIn.h>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

mavros_msgs::State currentState;
ros::ServiceClient arming_client, set_mode_client;
ros::Subscriber fastSub, stateSub, localPoseSub, rc_sub;
ros::Publisher posPub, testposPub;
mavros_msgs::PositionTarget cur_target_pose;
geometry_msgs::PoseStamped current_pose;
bool RC_control = true;

void stateCallback(const mavros_msgs::StateConstPtr& msg)
{
    currentState = *msg;
}

void localPosCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    current_pose = *msg;
    tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
                        msg->pose.orientation.w);
    tf2::Matrix3x3 m(q);
    // m.getRPY(iris_0_euler.roll, iris_0_euler.pitch, iris_0_euler.yaw);
}

void autoflyCallback(const quadrotor_msgs::PositionCommandConstPtr& msg)
{
    cur_target_pose.header.stamp = msg->header.stamp;
    cur_target_pose.coordinate_frame = 1;
    cur_target_pose.type_mask = 3064;//0
    cur_target_pose.position.x = msg->position.x;
    cur_target_pose.position.y = msg->position.y;
    cur_target_pose.position.z = msg->position.z;

 /*   cur_target_pose.velocity.x = msg->velocity.x;
    cur_target_pose.velocity.y = msg->velocity.y;
    cur_target_pose.velocity.z = msg->velocity.z;
    std::cout<<"Vel-->"<<"X: "<<msg->velocity.x<<"Y: "<<msg->velocity.y<<"Z: "<<msg->velocity.z<<std::endl;

    cur_target_pose.acceleration_or_force.x = msg->acceleration.x;
    cur_target_pose.acceleration_or_force.y = msg->acceleration.y;
    cur_target_pose.acceleration_or_force.z = msg->acceleration.z;
    std::cout<<"Acc-->"<<"X: "<<msg->acceleration.x<<"Y: "<<msg->acceleration.y<<"Z: "<<msg->acceleration.z<<std::endl;*/

    cur_target_pose.yaw = msg->yaw;
    // cur_target_pose.yaw_rate = msg->yaw_dot;
    std::cout<<"Pos-->"<<"X: "<<msg->position.x<<"Y: "<<msg->position.y<<"Z: "<<msg->position.z<<"Yaw-->"<<"yaw: "<<msg->yaw<<std::endl;

    
}

void RC_callback(const mavros_msgs::RCIn::ConstPtr& RCmsg)
{
    if(RCmsg->channels.at(9) >1500)
    {
        RC_control = true;
    }
    else
    {
        RC_control = false;
    }
}

void set_position(double x, double y, double z, double yaw)
{
    cur_target_pose.coordinate_frame = 1;
    cur_target_pose.type_mask = 3064;//0
    cur_target_pose.position.x = x;
    cur_target_pose.position.y = y;
    cur_target_pose.position.z = z;    
    cur_target_pose.yaw = yaw;
}

void timer_callback(const ros::TimerEvent& event)
{
    posPub.publish(cur_target_pose);
    ROS_INFO("publish position:  %f, %f, %f, %f", cur_target_pose.position.x, cur_target_pose.position.y, cur_target_pose.position.z, cur_target_pose.yaw);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fast_auto_node");
    ros::NodeHandle nh;

    localPoseSub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, localPosCallback);
    stateSub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, stateCallback);
    fastSub = nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 1,autoflyCallback);
    rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",1, RC_callback);

    
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    posPub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);


    ros::Rate rate(50.0);
    while (ros::ok())
    {
    
        ros::spinOnce();
        if (!RC_control)
        {
            // wait for FCU connection
            while(ros::ok() && !currentState.connected)
            {
                ROS_INFO("connecting px4...");
                ros::spinOnce();
                rate.sleep();
            }

            set_position(0, 0, 0.5, 0);//note!!!!!!!!
            ros::Timer timer = nh.createTimer(ros::Duration(0.01), timer_callback);

            //set to offboard
            mavros_msgs::SetMode offb_set_mode;
            offb_set_mode.request.custom_mode = "OFFBOARD";
            while(ros::ok() && currentState.mode != "OFFBOARD")
            {
                ROS_INFO("offboarding...");
                if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                }
                ros::spinOnce();
                rate.sleep();
            } 

            // arm
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = true;
            while(ros::ok() && !currentState.armed)
            {
                ROS_INFO("arming...");
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                ros::spinOnce();
                rate.sleep();
            }
            ros::Duration(4).sleep();
            // set_position(0,0,0,0);
            while(ros::ok())
            {
                ros::spinOnce();
                rate.sleep();
            }
        }else{
            std::cout<<"Mananual-RC control!"<<std::endl;
        }
    }
    return 0;
}

