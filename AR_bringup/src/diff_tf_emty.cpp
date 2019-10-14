#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

class Diff_TF{
public:
    Diff_TF();
    void spin();

private:
    ros::NodeHandle nh;

    //ros::Subscriber sub
    ros::Subsrciber l_wheel_sub;
    ros::Subscriber r_wheel_sub;
    //ros::Publisher pub
    ros::Publisher odom_pub;

    tf::TransformBroadcaster odom_broadcaster;

    double encoder_min;
    double encoder_max;

    double encoder_low_wrap;
    double encoder_high_wrap;

    double prev_lencoder;
    double prev_rencoder;

    double lmult;
    double rmult;

    double left;
    double right;

    double rate;
    
    ros::Duration t_delta;
    ros::Time t_next;
    ros::Time then;

    double enc_left;
    double enc_right;
    double ticks_meter;
    double base_width;
    
    double dx;
    double dr;

    double x_final, y_final, theta_final;
    ros::Time current_time, last_time;


    void leftencoderCB(const std_msgs::Int64::ConstrPtr& left_ticks);
    void rightencoderCB(const std_msgs::Int64::ConstrPtr& right_ticks);
    void init_varibles();
    void get_node_params();
    void updateTF();
    void update();
}

Diff_TF::Diff_TF()
{
    init_varibles();

    ROS_INFO("Started odometry computing node(Diff_TF)");

    l_wheel_sub = nh.subscribe("/lwheel", 10, &Odometry_calc::leftencoderCB, this);
    r_wheel_sub = nh.subscribe("/rwheel", 10, &Odometry_calc::rightencoderCB, this);

    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

    get_node_params();
}

void Diff_TF::init_varibles()
{
    prev_lencoder = 0;
    prev_rencoder = 0;
    
    lmult = 0;
    rmult = 0;

    left = 0;
    right = 0;

    encoder_min = -2147483648;
    encoder_max = 2147483648;

    rate = 10;
    ticks_meter = 50;
    base_width = 0.293;

    encoder_low_wrap = ((encoder_max - encoder_min) * 0.3) + encoder_min;
    encoder_high_wrap = ((encoder_max - encoder_min) * 0.7) + encode_max;

    t_delta = ros::Duration(1.0 / rate);
    t_next = ros::Time::now() + t_delta;
    then = ros::Time::now();

    enc_left = 0;
    enc_right = 0;

    dx = 0;
    dr = 0;

    x_final = 0;
    y_final = 0;
    z_final = 0;

    current_time = ros::Time::now();
    last_time = ros::Time::now(); 

}

void Diff_TF::get_node_params()
{
    if(nh.getParam("rate", rate)){
        ROS_INFO_STREAM("Rate from param" << rate);
    }

    if(nh.getParam("encoder_min", encoder_min)){
        ROS_INFO_STREAM("Encoder_MIN from param" << encoder_min);
    }

    if(nh.getParam("encoder_max", encoder_max)){
        ROS_INFO_STREAM("Encoder_MAX from param" << encoder_max);
    }

    if(nh.getParam("wheel_low_wrap", encoder_low_wrap)){
        ROS_INFO_STREAM("wheel_low_wrap from param" << encoder_low_wrap);
    }

    if(nh.getParam("wheel_high_wrap", encoder_high_wrap)){
        ROS_INFO_STREAM("Rate from param" << encoder_high_wrap);
    }

    if(nh.getParam("ticks_meter", ticks_meter)){
        ROS_INFO_STREAM("Ticks from param" << ticks_meter);
    }

    if(nh.getParam("base_width", base_width)){
        ROS_INFO_STREAM("Base Width" << base_width);
    }
}

void Diff_TF::spin()
{
    ros::Rate loop_rate(rate);

    while(ros::ok())
    {
        update();

        loop_rate.sleep();
    }
}

void Diff_TF::update()
{

    
}

void Diff_TF::leftencoderCB(const std_msgs::Int64::ConstrPtr& left_ticks)
{
    double enc = left_ticks->data;

    if((enc < encoder_low_wrap) && (prev_lencoder > encoder_high_wrap))
    {
        lmult = lmult + 1;
    }

    if((enc > encoder_high_wrap) && prev_lencoder < encoder_low_wrap)
    {
        lmult = lmult - 1;
    }

    left = 1.0 * (enc + lmult * (encoder_max - encoder_min));

    prev_lencoder = enc;
}


void Diff_TF::rightencoderCB(const std_msgs::Int64::ConstrPtr& right_ticks);
{
    double enc = right_ticks->data;

    if((enc < encoder_low_wrap) && (prev_lencoder > encoder_high_wrap))
    {
        rmult = rmult + 1;
    }

    if((enc > encoder_high_wrap) && prev_lencoder < encoder_low_wrap)
    {
        rmult = rmult - 1;
    }

    right = 1.0 * (enc + rmult * (encoder_max - encoder_min));

    prev_rencoder = enc;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "diff_tf");
    Diff_TF obj;
    obj.spin();
}