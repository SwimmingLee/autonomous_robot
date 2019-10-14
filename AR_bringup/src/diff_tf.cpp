#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

#define WHEEL_SEPARATION		0.293
#define WHEEL_RADIUS 	 	  	0.045
#define TICK2RAD       	 		0.003846860721

class Odometry_calc{

public:
	Odometry_calc();

	void spin();


private:
	ros::NodeHandle n;
	ros::Subscriber qaunternion_sub;
	ros::Subscriber l_wheel_sub;
	ros::Subscriber r_wheel_sub;
	ros::Publisher odom_pub;

	tf::TransformBroadcaster odom_broadcaster;
	//Encoder related variables
	double encoder_min;
	double encoder_max;

	double encoder_low_wrap;
	double encoder_high_wrap;

	double prev_lencoder;
	double prev_rencoder;

	double lencoder;
	double rencoder;

	double lmult;
	double rmult;

	double left;
	double right;

	double rate;

	ros::Time current_time, last_time;

    float odom_pose[3];
    double odom_vel[3];
    float quat[4];

	void leftencoderCb(const std_msgs::Int64::ConstPtr& left_ticks);
	void rightencoderCb(const std_msgs::Int64::ConstPtr& right_ticks);
    void qautCb(const std_msgs::Float32MultiArray& quat);


	void init_variables();

	void get_node_params();


	void update();
};

Odometry_calc::Odometry_calc(){


	init_variables();
	ROS_INFO("Started odometry computing node");
	l_wheel_sub = n.subscribe("/lwheel",10, &Odometry_calc::leftencoderCb, this);
	r_wheel_sub = n.subscribe("/rwheel",10, &Odometry_calc::rightencoderCb, this);
  	qaunternion_sub = n.subscribe("/quat", 2, &Odometry_calc::qautCb, this);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50); 

	//Retrieving parameters of this node
	get_node_params();
}

void Odometry_calc::init_variables()
{
	prev_lencoder =  lencoder = 0;
	prev_rencoder =  rencoder = 0;

	lmult = 0;
	rmult = 0;

	encoder_min =  -2147483648;
	encoder_max =  2147483648;

	rate = 30;

	encoder_low_wrap = ((encoder_max - encoder_min) * 0.3) + encoder_min ;
	encoder_high_wrap = ((encoder_max - encoder_min) * 0.7) + encoder_min ;
	
	current_time = last_time = ros::Time::now();

	odom_pose[0] = 0.0;
    odom_pose[1] = 0.0;
    odom_pose[2] = 0.0;
}

void Odometry_calc::get_node_params(){

    if(n.getParam("rate", rate)){
	 
		ROS_INFO_STREAM("Rate from param" << rate);	       
	}
}

//Spin function
void Odometry_calc::spin(){

     ros::Rate loop_rate(rate);

     while (ros::ok())
	{
		update();
		loop_rate.sleep();
	
	}


}

//Update function
void Odometry_calc::update(){
	current_time = ros::Time::now();

    double wheel_l, wheel_r;
    double delta_s, theta, delta_theta;

    static double last_theta = 0.0;

    double v, w;
	double step_time;

	step_time = current_time.toSec() - last_time.toSec(); 
		
    //0으로 나누는거 예외처리
    if(step_time == 0) return;

    wheel_l = TICK2RAD * (double)(lencoder -prev_lencoder);
    wheel_r = TICK2RAD * (double)(rencoder -prev_rencoder);

    // if(isnan(wheel_l))
    //     wheel_l = 0.0;

    // if(isnan(wheel_r))
    //     wheel_r = 0.0;
        
    delta_s = WHEEL_RADIUS * (wheel_r + wheel_l) * 0.5;

    theta = atan2(quat[1] + quat[2] + quat[0]*quat[3],
        0.5f - quat[2]*quat[2] - quat[3]*quat[3]);

	//theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;
    
	delta_theta = theta - last_theta;

	v = delta_s / step_time;
    w = delta_theta / step_time;

    //last_velocity[LEFT]  = wheel_l / step_time;
    //last_velocity[RIGHT] = wheel_r / step_time;

    odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta * 0.5));
    odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta * 0.5));
    odom_pose[2] += delta_theta;

    odom_vel[1] = 0.0;
    odom_vel[0] = v;
    odom_vel[2] = w;

    last_theta = theta;

    //next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	geometry_msgs::Quaternion odom_quat;

	odom_quat.x = 0.0;
	odom_quat.y = 0.0;
	odom_quat.z = sin(odom_pose[2] / 2);
	odom_quat.w = cos(odom_pose[2] / 2);

	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";

	//set the position
	odom.pose.pose.position.x = odom_pose[0];
	odom.pose.pose.position.y = odom_pose[1];
	odom.pose.pose.position.z = 0;
	odom.pose.pose.orientation = odom_quat;

	//set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = odom_vel[0];
	odom.twist.twist.linear.y = 0;
	odom.twist.twist.angular.z = odom_vel[2];

	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";

	odom_trans.transform.translation.x = odom.pose.pose.position.x;
	odom_trans.transform.translation.y = odom.pose.pose.position.y;
	odom_trans.transform.translation.z = odom.pose.pose.position.z;
	odom_trans.transform.rotation = odom.pose.pose.orientation;

	//send the transform
	odom_broadcaster.sendTransform(odom_trans);
		 
	//publish the message
	odom_pub.publish(odom);

	last_time = current_time;

    ros::spinOnce();

}





void Odometry_calc::leftencoderCb(const std_msgs::Int64::ConstPtr& left_ticks)
{
// ROS_INFO_STREAM("Left tick" << left_ticks->data);
	double enc = left_ticks->data;

	if((enc < encoder_low_wrap) && (prev_lencoder > encoder_high_wrap)){
		lmult = lmult + 1;
	}
	
	if((enc > encoder_high_wrap) && (prev_lencoder < encoder_low_wrap)){
		lmult = lmult - 1;
	}
    prev_lencoder = lencoder;
	lencoder = 1.0 * (enc + lmult * (encoder_max - encoder_min ));
}


//Right encoder callback

void Odometry_calc::rightencoderCb(const std_msgs::Int64::ConstPtr& right_ticks)
{
	double enc = right_ticks->data;

	if((enc < encoder_low_wrap) && (prev_lencoder > encoder_high_wrap)){	
		rmult = rmult + 1;
	}
	
	if((enc > encoder_high_wrap) && (prev_lencoder < encoder_low_wrap)){	
		rmult = rmult - 1;
	}
    prev_rencoder = rencoder;
	rencoder = 1.0 * (enc + rmult * (encoder_max - encoder_min ));
}


void Odometry_calc::qautCb(const std_msgs::Float32MultiArray& quat_msg)
{
    quat[0] = quat_msg.data[0];
    quat[1] = quat_msg.data[1];
    quat[2] = quat_msg.data[2];
    quat[3] = quat_msg.data[3];
	
	ROS_INFO("%5.5f\t%5.5f\t%5.5f\t%5.5f", quat[0], quat[1], quat[2], quat[3]);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv,"diff_tf");
	Odometry_calc obj;
	obj.spin();


	return 0;

}
