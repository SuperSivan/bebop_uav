// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

#include <math_common.h>
#include <cmath>
#include <chrono>
#include <YawQuat.h>
#include "keyboard.h"

// todo should be a common representation
struct XYZYaw
{
    double x;
    double y;
    double z;
    double yaw;
};

struct GPS_struct
{
    double latitude;
    double longitude;
    double altitude;
};

/*****************************获取GPS和Odom*****************************/
class GetGpsOdem
{
public:
    GetGpsOdem(const ros::NodeHandle &nh);
    
    void bebop_odom_cb(const nav_msgs::Odometry& odom_msg);
    void bebop_gps_cb(const sensor_msgs::NavSatFix& gps_msg);
    void update_position_timer_cb(const ros::TimerEvent& event);

private:
    ros::NodeHandle nh_;

    ros::Subscriber bebop_odom_sub_;    
    ros::Subscriber bebop_gps_sub_;
    ros::Timer update_position_timer_;
    
    void set_param();

    nav_msgs::Odometry curr_odom_;
    XYZYaw curr_position_, gps_curr_position_;
    GPS_struct gps_;
    GPS_struct initial_gps_;

    bool has_home_geo_;
    double LAT2M_;
    double LNG2M_;
    double update_time_;
};

 void GetGpsOdem::set_param()
 {
    LAT2M_ = M_PI/180*6371004;//1 degree to 111KM
    LNG2M_ = M_PI/180*6371004;
    update_time_ = 0.1;
 }
GetGpsOdem::GetGpsOdem(const ros::NodeHandle &nh): nh_(nh), has_home_geo_(false)
{
    set_param();
    bebop_odom_sub_ = nh_.subscribe("/bebop/odom", 50, &GetGpsOdem::bebop_odom_cb, this);
    bebop_gps_sub_ = nh_.subscribe("/bebop/fix", 50, &GetGpsOdem::bebop_gps_cb, this);

    // ROS timers
    update_position_timer_ = nh_.createTimer(ros::Duration(update_time_), &GetGpsOdem::update_position_timer_cb, this);
}

void GetGpsOdem::bebop_odom_cb(const nav_msgs::Odometry& odom_msg)
{
    curr_odom_ = odom_msg;
    curr_position_.x = odom_msg.pose.pose.position.x;
    curr_position_.y = odom_msg.pose.pose.position.y;
    curr_position_.z = odom_msg.pose.pose.position.z;
    curr_position_.yaw = YawQuat::get_yaw_from_quat_msg(odom_msg.pose.pose.orientation);

    //ROS_INFO("x: %f, y: %f, z: %f, yaw: %f", curr_position_.x, curr_position_.y, curr_position_.z, curr_position_.yaw);
}

void GetGpsOdem::bebop_gps_cb(const sensor_msgs::NavSatFix& gps_msg)
{
    if(!has_home_geo_)
    {
        has_home_geo_ = true;
        initial_gps_.latitude = gps_msg.latitude;
        initial_gps_.longitude = gps_msg.longitude;
        initial_gps_.altitude = gps_msg.altitude;
        ROS_INFO_STREAM("[GetGpsOdem] GPS reference initializing " << gps_msg.latitude << ", "<< gps_msg.longitude << ", " << gps_msg.altitude);
        return;
    }
    else
    {
        gps_curr_position_.x = (gps_msg.latitude - initial_gps_.latitude) * LAT2M_;
	    gps_curr_position_.y = (gps_msg.longitude - initial_gps_.longitude) * LNG2M_;
        gps_curr_position_.z = gps_msg.altitude - initial_gps_.altitude;
        gps_curr_position_.yaw = 0;
    }
}

void GetGpsOdem::update_position_timer_cb(const ros::TimerEvent& event)
{
    ROS_INFO("odm_x: %f, odm_y: %f, odm_z: %f, yaw: %f", curr_position_.x, curr_position_.y, curr_position_.z, curr_position_.yaw);
    ROS_INFO("gps_x: %f, gps_y: %f, gps_z: %f", gps_curr_position_.x, gps_curr_position_.y, gps_curr_position_.z);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_bebop2_node");
	ros::NodeHandle nh;
    GetGpsOdem ggo(nh);
    std_msgs::Empty emptyMsg;
    ros::NodeHandle nodeHandle;

	ros::Publisher takeOffPublisher = nodeHandle.advertise<std_msgs::Empty>("/bebop/takeoff", 1);
	ros::Publisher landPublisher = nodeHandle.advertise<std_msgs::Empty>("/bebop/land", 1);
	ros::Publisher resetPublisher = nodeHandle.advertise<std_msgs::Empty>("/bebop/reset", 1);
    ros::Publisher flyPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 50);

    geometry_msgs::Twist hoverTwistMsg;
    hoverTwistMsg.linear.x = 0.0;
	hoverTwistMsg.linear.y = 0.0;
	hoverTwistMsg.linear.z = 0.0;
	hoverTwistMsg.angular.x = 0.0;
	hoverTwistMsg.angular.y = 0.0;
	hoverTwistMsg.angular.z = 0.0;

    ros::Rate rate(100);
    char key;
	int exitNeed = 0;
	long int key_cnt = 0;

    init_keyboard();

	while (ros::ok())
	{
		if ( kbhit() )
        {
            key_cnt = 0;

            key = readch();
        }
        else
        {
            key_cnt++;
        }

        if(key_cnt > 5)
        {
            key = 0x20;
        }

		switch (key)
		{
			case 'r':
				exitNeed = 1;
				break;

			case 'v':
				takeOffPublisher.publish(emptyMsg);
				break;

			case 'b':
				landPublisher.publish(emptyMsg);
				break;

			case 'x':
				resetPublisher.publish(emptyMsg);
				break;

            default:
				flyPublisher.publish(hoverTwistMsg);
				break;

		}

		if (exitNeed == 1)
		{
			break;
		}

		ros::spinOnce();

		rate.sleep();
	}

	close_keyboard();
}