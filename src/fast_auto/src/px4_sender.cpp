#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>  
#include <math.h>
#include <ros/ros.h>
#include "mavros_msgs/PositionTarget.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <quadrotor_msgs/PositionCommand.h>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

//建立一个订阅消息体类型的变量，用于存储订阅的信息
mavros_msgs::State current_state;
mavros_msgs::PositionTarget cur_target_pose;
ros::Publisher posPub;
 
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


void autoflyCallback(const quadrotor_msgs::PositionCommandConstPtr& msg)
{
    cur_target_pose.header.stamp = msg->header.stamp;
    cur_target_pose.coordinate_frame = 1;
    cur_target_pose.type_mask = 0b100111111000;//00b100111111000
    cur_target_pose.position.x = msg->position.x;
    cur_target_pose.position.y = msg->position.y;
    cur_target_pose.position.z = msg->position.z;

    cur_target_pose.yaw = msg->yaw;
    std::cout<<"Pos-->"<<"X: "<<msg->position.x<<"Y: "<<msg->position.y<<"Z: "<<msg->position.z<<"Yaw-->"<<"yaw: "<<msg->yaw<<std::endl;  
}

void timer_callback(const ros::TimerEvent& event)
{
    // posPub.publish(cur_target_pose);
}

void set_position(double x, double y, double z, double yaw)
{
    // cur_target_pose.header.stamp = msg->header.stamp;
    cur_target_pose.coordinate_frame = 1;
    cur_target_pose.type_mask = 0b100111111000;//0b100111111000
    cur_target_pose.position.x = x;
    cur_target_pose.position.y = y;
    cur_target_pose.position.z = z;    
    cur_target_pose.yaw = yaw;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node"); 
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);

    // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    posPub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    // ros::Subscriber fastSub = nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10,autoflyCallback);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    ros::Timer timer = nh.createTimer(ros::Duration(0.02), timer_callback);

    ros::Rate rate(20.0);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("PX4 connected.");
    set_position(0,0,0,0);
    // geometry_msgs::PoseStamped pose;
    // pose.pose.position.x = 0;
    // pose.pose.position.y = 0;
    // pose.pose.position.z = 2;


    for(int i = 10; ros::ok() && i > 0; --i){
        // local_pos_pub.publish(pose);
        posPub.publish(cur_target_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {

        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else 
        {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(1.0)))
            {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // local_pos_pub.publish(pose);
        posPub.publish(cur_target_pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


