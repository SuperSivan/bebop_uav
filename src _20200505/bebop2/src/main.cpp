// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.


#include "ros/ros.h"
#include "gps_controller.h"
#include "keyboard.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Main_Controller_Node");
	ros::NodeHandle nh;
    GpsController ggo(nh);
    std_msgs::Empty emptyMsg;
    ros::NodeHandle nodeHandle;

	ros::Publisher takeOffPublisher = nodeHandle.advertise<std_msgs::Empty>("/bebop/takeoff", 1);
	ros::Publisher landPublisher = nodeHandle.advertise<std_msgs::Empty>("/bebop/land", 1);
	ros::Publisher resetPublisher = nodeHandle.advertise<std_msgs::Empty>("/bebop/reset", 1);
    ros::Publisher flyPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 50);

    //ros::Publisher mission_msg_pub = nodeHandle.advertise<std_msgs::Bool>("/bebop/mission_msg", 1);

    double flyStep = 0.5;
    geometry_msgs::Twist hoverTwistMsg;
    geometry_msgs::Twist forwardTwistMsg;
	geometry_msgs::Twist backwardTwistMsg;
	geometry_msgs::Twist leftTwistMsg;
	geometry_msgs::Twist rightTwistMsg;
	geometry_msgs::Twist upTwistMsg;
	geometry_msgs::Twist downTwistMsg;
	geometry_msgs::Twist rotateLeftTwistMsg;
	geometry_msgs::Twist rotateRightTwistMsg;

    hoverTwistMsg.linear.x = 0.0;
	hoverTwistMsg.linear.y = 0.0;
	hoverTwistMsg.linear.z = 0.0;
	hoverTwistMsg.angular.x = 0.0;
	hoverTwistMsg.angular.y = 0.0;
	hoverTwistMsg.angular.z = 0.0;

    // forward
	forwardTwistMsg.linear.x = flyStep;
	forwardTwistMsg.linear.y = 0.0;
	forwardTwistMsg.linear.z = 0.0;
	forwardTwistMsg.angular.x = 0.0;
	forwardTwistMsg.angular.y = 0.0;
	forwardTwistMsg.angular.z = 0.0;

	// backward
	backwardTwistMsg.linear.x = -1 * flyStep;
	backwardTwistMsg.linear.y = 0.0;
	backwardTwistMsg.linear.z = 0.0;
	backwardTwistMsg.angular.x = 0.0;
	backwardTwistMsg.angular.y = 0.0;
	backwardTwistMsg.angular.z = 0.0;

	// left
	leftTwistMsg.linear.x = 0.0;
	leftTwistMsg.linear.y = flyStep;
	leftTwistMsg.linear.z = 0.0;
	leftTwistMsg.angular.x = 0.0;
	leftTwistMsg.angular.y = 0.0;
	leftTwistMsg.angular.z = 0.0;

	// right
	rightTwistMsg.linear.x = 0.0;
	rightTwistMsg.linear.y = -1 * flyStep;
	rightTwistMsg.linear.z = 0.0;
	rightTwistMsg.angular.x = 0.0;
	rightTwistMsg.angular.y = 0.0;
	rightTwistMsg.angular.z = 0.0;

    // up
	upTwistMsg.linear.x = 0.0;
	upTwistMsg.linear.y = 0.0;
	upTwistMsg.linear.z = flyStep;
	upTwistMsg.angular.x = 0.0;
	upTwistMsg.angular.y = 0.0;
	upTwistMsg.angular.z = 0.0;

	// down
	downTwistMsg.linear.x = 0.0;
	downTwistMsg.linear.y = 0.0;
	downTwistMsg.linear.z = -1 * flyStep;
	downTwistMsg.angular.x = 0.0;
	downTwistMsg.angular.y = 0.0;
	downTwistMsg.angular.z = 0.0;

	// rotate left
	rotateLeftTwistMsg.linear.x = 0.0;
	rotateLeftTwistMsg.linear.y = 0.0;
	rotateLeftTwistMsg.linear.z = 0.0;
	rotateLeftTwistMsg.angular.x = 0.0;
	rotateLeftTwistMsg.angular.y = 0.0;
	rotateLeftTwistMsg.angular.z = flyStep;

	// rotate right
	rotateRightTwistMsg.linear.x = 0.0;
	rotateRightTwistMsg.linear.y = 0.0;
	rotateRightTwistMsg.linear.z = 0.0;
	rotateRightTwistMsg.angular.x = 0.0;
	rotateRightTwistMsg.angular.y = 0.0;
	rotateRightTwistMsg.angular.z = -1 * flyStep;

    ros::Rate rate(100);
    char key;
	int exitNeed = 0;
	long int key_cnt = 0;
    bool mission_flag = false;

    init_keyboard();

	while (ros::ok())
	{
		if (  kbhit() )
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
			case 'q':
				exitNeed = 1;
				break;

            case 'y':
                ggo.initial_gps(true);
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

            case 'g':
                mission_flag = true;
                ggo.set_mission_flag(mission_flag);
				break;
            
            case 'e':
                mission_flag = false;
                ggo.set_mission_flag(mission_flag);
				break;

            case 'a':

				flyPublisher.publish(leftTwistMsg);
				printf("left!\n");
				break;

			case 'd':
				flyPublisher.publish(rightTwistMsg);
				printf("right!\n");
				break;

			case 'w':
				flyPublisher.publish(forwardTwistMsg);
				printf("forward!\n");
				break;

			case 's':
				flyPublisher.publish(backwardTwistMsg);
				printf("backward!\n");
				break;

            case 'j':
				flyPublisher.publish(rotateLeftTwistMsg);
				printf("rotateLeft!\n");
				break;

			case 'l':
				flyPublisher.publish(rotateRightTwistMsg);
				printf("rotateRight!\n");
				break;

			case 'i':
				flyPublisher.publish(upTwistMsg);
				printf("up!\n");
				break;

			case 'k':
				flyPublisher.publish(downTwistMsg);
				printf("down!\n");
				break;

            case 0x20:
				if (!mission_flag)
                {
                    flyPublisher.publish(hoverTwistMsg);
                    //ROS_INFO("longtime--------hover!");
                }
				else
                {
                    //ROS_INFO("longtime--------During-task+++++++");
                }
				break;

            default:
                if (!mission_flag)
                {
                    flyPublisher.publish(hoverTwistMsg);
                    //ROS_INFO("hover!");
                }
				else
                {
                    //ROS_INFO("During-task++++++");
                }               
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