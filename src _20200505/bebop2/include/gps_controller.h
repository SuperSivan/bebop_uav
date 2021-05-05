#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <bebop2/YawCmd.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <math_common.h>
#include <cmath>
#include <chrono>
#include <YawQuat.h>
#include <fstream>

/**************************设置参数**************************/
class PIDParams
{
public:
    double kp_x;
    double kp_y;
    double kp_z;
    double kp_yaw;
    double kd_x;
    double kd_y;
    double kd_z;
    double kd_yaw;

    double reached_thresh_xyz;
    double reached_yaw_degrees;

    PIDParams():
        kp_x(0.5),
        kp_y(0.5),
        kp_z(0.5),
        kp_yaw(0.5),
        kd_x(0.1),
        kd_y(0.1),
        kd_z(0.1),
        kd_yaw(0.1),
        reached_thresh_xyz(5),
        reached_yaw_degrees(5.0)
        {}

    bool load_from_rosparams(const ros::NodeHandle& nh);
};

class DynamicConstraints
{
public:
    double max_vel_horz_abs; // meters/sec
    double max_vel_vert_abs;
    double max_yaw_rate_degree;

    DynamicConstraints():
        max_vel_horz_abs(0.2),
        max_vel_vert_abs(0.2),
        max_yaw_rate_degree(20.0)
        {}

    bool load_from_rosparams(const ros::NodeHandle& nh);
};

struct XYZYaw
{
    double x;
    double y;
    double z;
    double yaw;
};

struct VEL_CMD
{
    double Vx;
    double Vy;
    double Vz;
    double Ayaw;
};

struct GPS_struct
{
    double latitude;
    double longitude;
    double altitude;
};

/*****************************获取GPS和Odom，并控制*****************************/
class GpsController
{
public:
    GpsController(const ros::NodeHandle &nh);
    
    void bebop_odom_cb(const nav_msgs::Odometry& odom_msg);
    //void depth_command_cb(const std_msgs::Int8& command_msg);
    void depth_command_cb(const bebop2::YawCmd& command_msg);
    void bebop_gps_cb(const sensor_msgs::NavSatFix& gps_msg);
    void update_position_timer_cb(const ros::TimerEvent& event);
    
    bool load_target_postition(const ros::NodeHandle &nh);

    void set_mission_flag(bool flag);
    void initial_gps(bool flag);
    void set_param();
    double p2p(double delta_x, double delta_y);

    void check_reached_goal_only_position();
    void avoid();
    void orientation_control_cmd();
    void enforce_dynamic_constraints();
    void publish_control_cmd();

private:
    std::ofstream ned_txt;

    ros::NodeHandle nh_;

    ros::Subscriber bebop_odom_sub_;    
    ros::Subscriber bebop_gps_sub_;
    ros::Subscriber depth_command_sub_;
    ros::Timer update_position_timer_;

    ros::Publisher flyPublisher_;

    DynamicConstraints constraints_;  
    PIDParams params_;
    nav_msgs::Odometry curr_odom_;
    geometry_msgs::Twist cmd_Msg;
    XYZYaw curr_position_, gps_curr_position_, target_position_, curr_velocity_, err_position_;
    XYZYaw prev_error_;
    XYZYaw curr_error_;
    VEL_CMD vel_cmd_;
    GPS_struct target_gps_;
    GPS_struct initial_gps_;

    double flight_time;
    double p2p_error_;
    double initial_yaw_;
    bool has_home_geo_;
    bool mission_flag_;

    bool initial_gps_flag_;
    bool reached_goal_;
    bool txt_open_flag;
    bool avoidance_flag_;
    bool last_avoidance_flag_;
    double LAT2M_;
    double LNG2M_;
    double update_time_;


    double yaw_command_;
    double old_yaw_command_;
    int lq;
    int count_sway_;
    
    double forwardSpeed;
    double objectYawThrehold;
    double objectForwardSpeed;
    double objectRotateSpeed;

    std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::nanoseconds> lastTime, curTime;
};