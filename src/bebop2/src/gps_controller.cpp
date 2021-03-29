#include "gps_controller.h"

bool PIDParams::load_from_rosparams(const ros::NodeHandle &nh)
{
    bool found = true;

    found = found && nh.getParam("kp_x", kp_x);
    found = found && nh.getParam("kp_y", kp_y);
    found = found && nh.getParam("kp_z", kp_z);
    found = found && nh.getParam("kp_yaw", kp_yaw);

    found = found && nh.getParam("kd_x", kd_x);
    found = found && nh.getParam("kd_y", kd_y);
    found = found && nh.getParam("kd_z", kd_z);
    found = found && nh.getParam("kd_yaw", kd_yaw);

    found = found && nh.getParam("reached_thresh_xyz", reached_thresh_xyz);
    found = found && nh.getParam("reached_yaw_degrees", reached_yaw_degrees);

    return found;
}

bool DynamicConstraints::load_from_rosparams(const ros::NodeHandle &nh)
{
    bool found = true;

    found = found && nh.getParam("max_vel_horz_abs", max_vel_horz_abs);
    found = found && nh.getParam("max_vel_vert_abs", max_vel_vert_abs);
    found = found && nh.getParam("max_yaw_rate_degree", max_yaw_rate_degree);

    return found;
}

bool GpsController::load_target_postition(const ros::NodeHandle &nh)
{
    bool found = true;
    found = found && nh.getParam("target_x", target_position_.x);
	found = found && nh.getParam("target_y", target_position_.y);
    found = found && nh.getParam("flight_time", flight_time);

    return found;
}

void GpsController::set_param()
 {
    LAT2M_ = M_PI/180*6371004;//1 degree to 111KM
    LNG2M_ = M_PI/180*6371004;
    yaw_command_ = 0;
    update_time_ = 0.07;
    mission_flag_ = false;
    avoidance_flag_ = false;
 }

void GpsController::set_mission_flag(bool flag)
{
    mission_flag_ = flag;
    reached_goal_ = !flag;
    lastTime = std::chrono::_V2::system_clock::now();
}
void GpsController::initial_gps(bool flag)
{
    initial_gps_flag_ = flag;
}

/**********************************类构建函数**********************************/
GpsController::GpsController(const ros::NodeHandle &nh): nh_(nh), has_home_geo_(false), reached_goal_(false), initial_gps_flag_(false), count_sway_(0), txt_open_flag(false)
{
    set_param();
    //params_.load_from_rosparams(nh_private_);
    //constraints_.load_from_rosparams(nh_);
    load_target_postition(nh_);
    ROS_INFO("target_gps_lat: %f, target_gps_lon: %f, target_gps_alt: %f", target_gps_.latitude, target_gps_.longitude, target_gps_.altitude);
    bebop_odom_sub_ = nh_.subscribe("/bebop/odom", 1, &GpsController::bebop_odom_cb, this);
    depth_command_sub_ = nh_.subscribe("/Controller/command", 50, &GpsController::depth_command_cb, this);
    //bebop_gps_sub_ = nh_.subscribe("/bebop/fix", 1, &GpsController::bebop_gps_cb, this);

    flyPublisher_ = nh_.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 2);

    // ROS timers
    //update_position_timer_ = nh_.createTimer(ros::Duration(update_time_), &GpsController::update_position_timer_cb, this);
}

/**********************************depth_command回调函数**********************************/
//void GpsController::depth_command_cb(const std_msgs::Float64& command_msg)
void GpsController::depth_command_cb(const bebop2::YawCmd& command_msg)
{
    using namespace std::chrono;

    static auto lastTime = system_clock::now();
    static auto curTime = system_clock::now();
    static bool first = true;
    old_yaw_command_ = yaw_command_;
    yaw_command_ = command_msg.yaw_command;
    avoidance_flag_= command_msg.avoidance_flag;
    
    /*if (yaw_command_ > 0 && old_yaw_command_ < 0) count_sway_++;
    else if (yaw_command_ < 0 && old_yaw_command_ > 0) count_sway_++;

    if (count_sway_ > 0 && first) 
    {
        lastTime = system_clock::now();
        first = false;
    }
    if (count_sway_ > 2)
    {
        ROS_INFO("out sway!");
        yaw_command_ = old_yaw_command_;
    }
    curTime = system_clock::now();
    auto duration = duration_cast<microseconds>(curTime - lastTime);
    double duration_s = double(duration.count()) * microseconds::period::num / microseconds::period::den;
    if (duration_s > 1)
    {
        count_sway_ = 0;
        first = true;
    }*/

    //ROS_INFO("yaw_command_: %f", yaw_command_);

    if(mission_flag_)
    {
        //ROS_INFO("mission+++++++++++++++++");

        check_reached_goal_only_position();

        if(reached_goal_)
        {
            ROS_INFO_STREAM("[YawController] Reached goal! Hovering at position.");
            mission_flag_ = false;
        }
        else
        {
            //ROS_INFO_STREAM("[YawController] Moving to goal.");
            orientation_control_cmd();
            enforce_dynamic_constraints();
            publish_control_cmd();
        }
    }
    else
    {
        //ROS_INFO("No mission-----------------");
    }

}

/**********************************Odometry回调函数**********************************/
void GpsController::bebop_odom_cb(const nav_msgs::Odometry& odom_msg)
{
    curr_position_.x = odom_msg.pose.pose.position.x;
    curr_position_.y = - odom_msg.pose.pose.position.y;
    curr_position_.z = odom_msg.pose.pose.position.z;
    curr_position_.yaw = - math_common::rad2deg(YawQuat::get_yaw_from_quat_msg(odom_msg.pose.pose.orientation));
    //ROS_INFO("ned_x: %f, ned_y: %f, yaw: %f", curr_position_.x, curr_position_.y, curr_position_.yaw);
    if (mission_flag_)
    {
        if(!txt_open_flag) 
        {
            ned_txt.open("/home/zhang/bebop_mega/ned.txt");
            double initial_yaw = - YawQuat::get_yaw_from_quat_msg(odom_msg.pose.pose.orientation);
            double tmp_x, tmp_y;
            tmp_x = cos(initial_yaw) * target_position_.x - sin(initial_yaw) * target_position_.y;
            tmp_y = sin(initial_yaw) * target_position_.x + cos(initial_yaw) * target_position_.y;
            target_position_.x = tmp_x;
            target_position_.y = tmp_y;
            ned_txt << tmp_x << " " << tmp_y << std::endl;
            txt_open_flag = true;
        }
        ned_txt << curr_position_.x << " " << curr_position_.y << std::endl;
    }
    if (reached_goal_) ned_txt.close();

    //ROS_INFO_STREAM("curr_yaw: " << curr_position_.yaw);
    /*if(initial_gps_flag_)
    {
        target_position_.z = curr_position_.z;
        ROS_INFO_STREAM("target_position_z: " << target_position_.z);
    }*/
}

/**********************************GPS回调函数**********************************/
void GpsController::bebop_gps_cb(const sensor_msgs::NavSatFix& gps_msg)
{
    if(initial_gps_flag_)
    {
        if(!has_home_geo_)
        {
            has_home_geo_ = true;
            initial_gps_.latitude = gps_msg.latitude;
            initial_gps_.longitude = gps_msg.longitude;
            initial_gps_.altitude = gps_msg.altitude;
            //ROS_INFO_STREAM("[GpsController] GPS reference initializing " << gps_msg.latitude << ", "<< gps_msg.longitude << ", " << gps_msg.altitude);
            ROS_INFO("ned_x: %.12f, ned_y: %.12f, ned_z: %.12f", gps_msg.latitude, gps_msg.longitude, gps_msg.altitude);
            target_position_.x = (target_gps_.latitude - initial_gps_.latitude) * LAT2M_;
            target_position_.y = (target_gps_.longitude - initial_gps_.longitude) * LNG2M_;
            target_position_.z = target_gps_.altitude - initial_gps_.altitude;
            target_position_.yaw = 0;
            ROS_INFO_STREAM("[GpsController] GPS target: " << target_position_.x << ", "<< target_position_.y << ", " << target_position_.z);
            return;
        }
        else
        {
            gps_curr_position_.x = (gps_msg.latitude - initial_gps_.latitude) * LAT2M_;
            gps_curr_position_.y = (gps_msg.longitude - initial_gps_.longitude) * LNG2M_;
            gps_curr_position_.z = gps_msg.altitude - initial_gps_.altitude;
            gps_curr_position_.yaw = curr_position_.yaw;

            //ROS_INFO("gps_x: %.10f, gps_y: %.10f, gps_z: %.10f", gps_msg.latitude, gps_msg.longitude, gps_msg.altitude);
            ROS_INFO("ned_x: %f, ned_y: %f, ned_z: %f", gps_curr_position_.x, gps_curr_position_.y, gps_curr_position_.z);
        }
    }
}
/**********************************更新函数**********************************/
/*计时器0.1s循环一次*/
void GpsController::update_position_timer_cb(const ros::TimerEvent& event)
{
    if(mission_flag_)
    {
        //ROS_INFO("mission+++++++++++++++++");

        check_reached_goal_only_position();

        if(reached_goal_)
        {
            ROS_INFO_STREAM("[YawController] Reached goal! Hovering at position.");
            mission_flag_ = false;
        }
        else
        {
            //ROS_INFO_STREAM("[YawController] Moving to goal.");
            orientation_control_cmd();
            enforce_dynamic_constraints();
            publish_control_cmd();
        }
    }
    else
    {
        //ROS_INFO("No mission-----------------");
    }
    
}

/**********************************检查是否到达目的地**********************************/
void GpsController::check_reached_goal_only_position()
{
    using namespace std::chrono;
    curTime = system_clock::now();

    auto duration = duration_cast<microseconds>(curTime - lastTime);
    double duration_s = double(duration.count()) * microseconds::period::num / microseconds::period::den;

    double diff_xyz = sqrt((target_position_.x - curr_position_.x) * (target_position_.x - curr_position_.x) 
                        + (target_position_.y - curr_position_.y) * (target_position_.y - curr_position_.y));

    if ((duration_s > flight_time) || (diff_xyz < params_.reached_thresh_xyz))
        reached_goal_ = true; 
}

/**********************************控制函数**********************************/
void GpsController::orientation_control_cmd()
{
    curr_error_.z = target_position_.z - curr_position_.z;
    curr_error_.yaw = yaw_command_;

    double p_term_z = params_.kp_z * curr_error_.z;
    double p_term_yaw = params_.kp_yaw * curr_error_.yaw;

    double d_term_z = params_.kd_z * (curr_error_.z - prev_error_.z);
    double d_term_yaw = params_.kp_yaw * (curr_error_.yaw - prev_error_.yaw);

    prev_error_ = curr_error_;

    double angular = p_term_yaw + d_term_yaw;
    if (std::fabs(angular) > 20) angular = (angular / std::fabs(angular)) * 20;

    vel_cmd_.Vz = p_term_z + d_term_z;
    vel_cmd_.Ayaw = angular; // todo
}

/**********************************限制函数**********************************/
void GpsController::enforce_dynamic_constraints()
{
    static bool flag_1, flag_2;
    flag_1 = true;
    flag_2 = true;
    //velocity xyz
    if(avoidance_flag_)
    {
        
        if (vel_cmd_.Ayaw >= 8)
        {
            cmd_Msg.linear.x = 0.04;
            if (flag_1)
            {
                flag_1 = false;
                cmd_Msg.linear.y = -0.12;
            }
            else
            {
                cmd_Msg.linear.y = cmd_Msg.linear.y * 0.6;
            }
            
        }
        else if (vel_cmd_.Ayaw <= -8)
        {
            cmd_Msg.linear.x = 0.04;
            if (flag_2)
            {
                flag_2 = false;
                cmd_Msg.linear.y = 0.12;
            }
            else
            {
                cmd_Msg.linear.y = cmd_Msg.linear.y * 0.6;
            }
        }
        else
        {
            flag_1 = true;
            flag_2 = true;
            cmd_Msg.linear.x = 0.08;
            cmd_Msg.linear.y = 0;
        }        
       /*
        if (std::fabs(vel_cmd_.Ayaw) > 8)
        {
            cmd_Msg.linear.x = 0.04;
            double tmp = cmd_Msg.linear.y;
            cmd_Msg.linear.y = - 0.12 * (vel_cmd_.Ayaw / std::fabs(vel_cmd_.Ayaw))
            
            if (std::fabs(cmd_Msg.linear.y - tmp) < 0.001)
            {
                cmd_Msg.linear.y = cmd_Msg.linear.y * 0.8
            }
            
        }
        else
        {
            cmd_Msg.linear.x = 0.08;
            cmd_Msg.linear.y = 0;
        }
        */       
    }
    else
    {
        cmd_Msg.linear.x = 0.08;
        cmd_Msg.linear.y = 0;
    }
    
    
    cmd_Msg.linear.z = 0;
    /*if (std::fabs(vel_cmd_.Vz) > constraints_.max_vel_vert_abs)
    {
        cmd_Msg.linear.z = (vel_cmd_.Vz / std::fabs(vel_cmd_.Vz)) * constraints_.max_vel_vert_abs; 
    }*/
    //angular xyz
    cmd_Msg.angular.x = 0;
    cmd_Msg.angular.y = 0;
    cmd_Msg.angular.z = - math_common::deg2rad(vel_cmd_.Ayaw);
}

/**********************************指令发布函数**********************************/
void GpsController::publish_control_cmd()
{
    if (reached_goal_)
    {
        cmd_Msg.linear.x = 0;
        cmd_Msg.linear.y = 0;
        cmd_Msg.linear.z = 0;
        cmd_Msg.angular.x = 0;
        cmd_Msg.angular.y = 0;
        cmd_Msg.angular.z = 0;
    }

    //ROS_INFO("command yaw: %f, linear_y: %f, angular_yaw: %f", yaw_command_, cmd_Msg.linear.y, cmd_Msg.angular.z);
    ROS_INFO("Bebop2 controll");
    flyPublisher_.publish(cmd_Msg);
}
