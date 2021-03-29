// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <bebop2/YawCmd.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <math_common.h>
#include <cmath>
#include <chrono>
#include <YawQuat.h>

#define PI 3.14159265359
#define DEPTH_COLS_MAX 640
#define DEPTH_ROWS_MAX 480

using namespace cv;

static const std::string OPENCV_WINDOW = "c_space window";

const double camera_cx = 319.5;
const double camera_cy = 239.5;
const double camera_fx = 269.5;
const double camera_fy = 269.5;
const double uav_x = 0.6;
const double uav_y = 0.1;

// todo should be a common representation
struct XYZYaw
{
    double x;
    double y;
    double z;
    double yaw;
};

/********LookUpTable********/
int N = 100, M1 = 640, M2 = 480, L = 2;
static std::vector<std::vector<std::vector<int>>> xz_table(N, std::vector<std::vector<int>>(M1, std::vector<int>(L))); //¶šÒåÈýÎ¬Î¬¶¯Ì¬Êý×é(100£¬640£¬2)
static std::vector<std::vector<std::vector<int>>> yz_table(N, std::vector<std::vector<int>>(M2, std::vector<int>(L))); //¶šÒåÈýÎ¬Î¬¶¯Ì¬Êý×é(100£¬480£¬2)

/*****************************角度转像素坐标*****************************/
class Angle2uv
{
public:
	int u, v, limit_yaw;
	Angle2uv() {};
	void set(double *angle_x)
	{
		angle = angle_x;
		u = limit(round(tan(*angle_x / 180 * PI) * camera_fx + camera_cx));//ÓÉœÇ¶ÈŒÆËãÍŒÏñ×ø±êÏµÏÂË®Æœ·œÏòÉÏµÄuÖµ
		v = DEPTH_ROWS_MAX / 2;//ÓÉœÇ¶ÈŒÆËãÍŒÏñ×ø±êÏµÏÂË®Æœ·œÏòÉÏµÄvÖµ
	};
	int limit(int x) {
		if (x < 0)
		{
			x = 0;
			*angle = -50;
		}
		if (x >= DEPTH_COLS_MAX)
		{
			x = DEPTH_COLS_MAX - 1;
			*angle = 50;
		}
		return x;
	}
private:
	double *angle;
};

/*****************************深度图像订阅、分析、发布*****************************/
class DepthAnalyse
{
    ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber bebop_odom_sub_;
    ros::Publisher command_pub_;  

public:
    DepthAnalyse(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private): it_(nh), nh_(nh), nh_private_(nh_private), avoidance_out(false),scale(20)
    {
		lookup_table();
		load_target_yaw(nh_);
        image_sub_ = it_.subscribe("/Controller/depth", 1, &DepthAnalyse::image_Cb, this);
		bebop_odom_sub_ = nh_.subscribe("/bebop/odom", 1, &DepthAnalyse::bebop_odom_cb, this);        
        //command_pub_ = nh_.advertise<std_msgs::Float64>("/Controller/command", 1);
		command_pub_ = nh_.advertise<bebop2::YawCmd>("/Controller/command", 1);            
        cv::namedWindow(OPENCV_WINDOW);
    }

    ~DepthAnalyse()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

	bool load_target_yaw(const ros::NodeHandle &nh);

	void image_Cb(const sensor_msgs::ImageConstPtr& msg);
	void bebop_odom_cb(const nav_msgs::Odometry& odom_msg);
    void depth_analyse(cv::Mat depth_c);
    void lookup_table();
    void c_space(cv::Mat image, cv::Mat& dep_new);
	double angle_substract(double target, double current);
    
private:
    bool avoidance_out; 
	bool obstacle_;
    double bias;
	double current_yaw, target_yaw;   
	nav_msgs::Odometry curr_odom_;
    XYZYaw target_position_;
    XYZYaw curr_position_;

	double scale;
};

bool DepthAnalyse::load_target_yaw(const ros::NodeHandle &nh)
{
    bool found = true;
    found = found && nh.getParam("target_yaw", target_yaw);
    return found;
}

/*****************************(测试)图像回调函数*****************************/
/*void DepthAnalyse::image_Cb(const sensor_msgs::ImageConstPtr& msg)
{
	//get image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

	cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
}*/

/*****************************Odometry回调函数*****************************/
void DepthAnalyse::bebop_odom_cb(const nav_msgs::Odometry& odom_msg)
{
	static bool flag = true;
    current_yaw = - math_common::rad2deg(YawQuat::get_yaw_from_quat_msg(odom_msg.pose.pose.orientation));

	if (flag)
	{
		ROS_INFO("current_yaw %f", current_yaw);
		flag = false;
	}
}

/*****************************图像回调函数*****************************/
void DepthAnalyse::image_Cb(const sensor_msgs::ImageConstPtr& msg)
{
	using namespace std::chrono;

    static auto startTime = system_clock::now();
    static auto endTime = system_clock::now();
    startTime = system_clock::now();
        
    static auto lastTime = system_clock::now();
    static auto curTime = system_clock::now();
    static bool first = true;

    //get depth
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    cv::Mat depth_c(DEPTH_ROWS_MAX, DEPTH_COLS_MAX, CV_8UC1, 255);
	c_space(cv_ptr->image, depth_c);
    // calculate bias_yaw
    if (DepthAnalyse::avoidance_out)
    {
        if ( first )
        {
            lastTime = system_clock::now();
            first = false;
        }
        curTime = system_clock::now();
        auto duration = duration_cast<microseconds>(curTime - lastTime);
        double duration_s = double(duration.count()) * microseconds::period::num / microseconds::period::den;
        if (duration_s > 1)
        {
            DepthAnalyse::avoidance_out = false;
            first = true;
        }   
    }
    else
    {
        DepthAnalyse::depth_analyse(depth_c);
    }

	cv::imwrite(cv_ptr->header.frame_id, depth_c);
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, depth_c);
    cv::waitKey(1);
    // 发布消息
    //std_msgs::Float64 bias_msg;
    //bias_msg.data = bias;
	//command_pub_.publish(bias_msg);
	bebop2::YawCmd cmd_msg;
	cmd_msg.yaw_command = bias;
	cmd_msg.avoidance_flag = obstacle_;
	command_pub_.publish(cmd_msg);

    endTime = system_clock::now();
    auto duration = duration_cast<microseconds>(endTime - startTime);
    double duration_s = double(duration.count()) * microseconds::period::num / microseconds::period::den;
}

/*****************************深度分析函数*****************************/  
void DepthAnalyse::depth_analyse(cv::Mat depth_c)
{
	try{
		static int save_count = 1;
		static bool condition = false;
		static bool old_condition = false;
	
		double bias_yaw = angle_substract(target_yaw, current_yaw);
		if (bias_yaw > 50) bias_yaw = 45;
		else if (bias_yaw < -50) bias_yaw = -45;
        
		Angle2uv angtf;
		angtf.set(&bias_yaw);
		double yaw_dis = depth_c.at<uchar>(angtf.v, angtf.u);//Ä¿±ê·œÏòÉÏÕÏ°­ÎïµÄŸàÀë

		old_condition = condition;
		condition = false;
		obstacle_ = false;
		if (yaw_dis < 100)
		{
			ROS_INFO("obstacle!!!!!!!!!!");
			obstacle_ = true;
			condition = true;
			bool left_or_right = true;
			double step_size = 5;
			int step = 1;
			double tmp = bias_yaw;
			bool safe = true, left_ok = true, right_ok = true;
			while (safe)
			{
				if (left_ok)
				{
					bias_yaw = tmp - step * step_size;
					if (bias_yaw < -50) left_ok = false;
					angtf.set(&bias_yaw);
					yaw_dis = depth_c.at<uchar>(angtf.v, angtf.u);
					if (yaw_dis > 100)
					{
						bias_yaw = bias_yaw  - step_size * 4;
						break;
					}
				}
				if (right_ok)
				{
					bias_yaw = tmp + step * step_size;
					if (bias_yaw > 50) right_ok = false;
					angtf.set(&bias_yaw);
					yaw_dis = depth_c.at<uchar>(angtf.v, angtf.u);
					if (yaw_dis > 100)
					{
						bias_yaw = bias_yaw + step_size * 4;
						break;
					}
				}
				if ((left_ok == false) && (right_ok == false))
					safe = false;
				step++;
			}
			if (safe == false)
			{
				std::cout << "Cannot forward!" << std::endl;
			}
		}
		/*if (old_condition && (!condition))
		{
			ROS_INFO("out_avoi!");
			DepthAnalyse::avoidance_out = true;
			if (bias_yaw > 0)
			    bias_yaw = - 9999;
			else
			    bias_yaw = 9999;
		}*/
		cv::line(depth_c, Point(0, 240), Point(640, 240), Scalar(0), 2, CV_AA); 
		cv::line(depth_c, Point(angtf.u, 0), Point(angtf.u, 480), Scalar(0), 2, CV_AA);
		cv::putText(depth_c,std::to_string(bias_yaw),Point(angtf.u,220),FONT_HERSHEY_SIMPLEX,2,Scalar(0),4,8);
		char save_path[256];
		sprintf(save_path, "/home/zhang/airsim_data/depth_target/%06d.jpg", save_count);
		cv::imwrite(save_path, depth_c);
		save_count++;
		DepthAnalyse::bias = bias_yaw;
	}
	catch (std::exception& e) {
		std::cout << "222222 Standard exception: " << std::endl << e.what() << std::endl;
	}
}

/*****************************C_Space函数*****************************/
void DepthAnalyse::c_space(cv::Mat image, cv::Mat& dep_new)
{
	int nr = image.rows; // number of rows
	int nc = image.cols; // number of columns 
	
	int count = 0;
	double alpha, alpha1;

	for (int y = 0; y < nr; y++)//horizontally expanding
	{
		for (int x = 0; x < nc; x++)
		{
			uchar z = image.at<uchar>(y, x);
			if (z >= 100 || z <= 15) continue;
			int u1, u2;
			u1 = xz_table[z][x][0];
			u2 = xz_table[z][x][1];
			
			for (int i = u1; i < u2; i++)
			{
				if (dep_new.at<uchar>(y, i) > z)
					dep_new.at<uchar>(y, i) = z;
			}
		}
	}
	cv::Mat tmp = dep_new.clone();
	for (int y = 0; y < nr; y++)//vertically expanding
	{
		for (int x = 0; x < nc; x++)
		{
			uchar z = dep_new.at<uchar>(y, x);
			if (z >= 100 || z <= 15) continue;
			int u1, u2;
			u1 = yz_table[z][y][0];
			u2 = yz_table[z][y][1];

			for (int i = u1; i < u2; i++)
			{
				if (tmp.at<uchar>(i, x) > z)
					tmp.at<uchar>(i, x) = z;
			}
		}
	}
	dep_new = tmp.clone();
	uchar diff = uchar(uav_x * scale);
	dep_new = dep_new - diff;
}

/*****************************建立LUT*****************************/
void DepthAnalyse::lookup_table()
{
	clock_t begin, end;
	begin = clock();
	for (int i = 0; i < xz_table.size(); i++)//Êä³ö¶þÎ¬¶¯Ì¬Êý×é 
	{
		for (int j = 0; j < xz_table[i].size(); j++)
		{
			double wz = double(i) / scale;
			double wxy;
			wxy = double(j - camera_cx) * wz / camera_fx;
			double dstxy = sqrt(wz * wz + wxy * wxy);
			double alpha = atan((j - camera_cx) / camera_fx);
			double alpha1 = asin(uav_x / dstxy);
			double low_raw = camera_fx * tan(alpha - alpha1) + camera_cx;
			double high_raw = camera_fx * tan(alpha + alpha1) + camera_cx;
			int u1 = (int)round(low_raw);
			int u2 = (int)round(high_raw);
			if (u1 < 0)
			{
				u1 = 0;
			}
			if (u2 > DEPTH_COLS_MAX)
			{
				u2 = DEPTH_COLS_MAX;
			}

			xz_table[i][j][0] = u1;
			xz_table[i][j][1] = u2;
		}
	}
	for (int i = 0; i < yz_table.size(); i++)//Êä³ö¶þÎ¬¶¯Ì¬Êý×é 
	{
		for (int j = 0; j < yz_table[i].size(); j++)
		{
			double wz = double(i) / scale;
			double wxy;
			wxy = double(j - camera_cy) * wz / camera_fy;
			double dstxy = sqrt(wz * wz + wxy * wxy);
			double alpha = atan((j - camera_cy) / camera_fy);
			double alpha1 = asin(uav_y / dstxy);
			double low_raw = camera_fy * tan(alpha - alpha1) + camera_cy;
			double high_raw = camera_fy * tan(alpha + alpha1) + camera_cy;
			int u1 = (int)round(low_raw);
			int u2 = (int)round(high_raw);
			if (u1 < 0)
			{
				u1 = 0;
			}
			if (u2 > DEPTH_ROWS_MAX)
			{
				u2 = DEPTH_ROWS_MAX;
			}

			yz_table[i][j][0] = u1;
			yz_table[i][j][1] = u2;
		}
	}

	end = clock();
	double time_diff = (double)(end - begin) / CLOCKS_PER_SEC;
	ROS_INFO("LUT build: %f", time_diff);
}

double DepthAnalyse::angle_substract(double target, double current)
{
	double result = target - current;

	if (result > 180)
		result -= 360;
	else if (result < - 180)
		result += 360;

	return result;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Depth_Analyse_Node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
    DepthAnalyse da(nh, nh_private);
    ros::spin();
    return 0;
}
