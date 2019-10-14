#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float32.h"
#include <iostream>

using namespace std;

#define ENCODER_MIN         -2147483648
#define ENCODER_MAX         2147483648


class PID_Velocity
{
public:
    PID_Velocity();
    void spin();

private:
    //NodeHandler
    ros::NodeHandle nh;

    //Publisher
    ros::Publisher vel_pub;
    ros::Publisher motor_cmd_pub;

    //Subscriber
    ros::Subscriber wheel_sub;
    ros::Subscriber wheel_vtarget_sub;

    //variables
    std_msgs::Float32 motor_output;
    float target_vel;
    float vel_output;
    float vel_output_min;
    float vel_output_max;
    float vel_threshold;

    std_msgs::Float32 wheel_vel;
    float vel;
    float prev_vel[2]; //moving average

    float vel_error;
    float previous_vel_error;
    float vel_intergral;
    float vel_derivative;
    
    float wheel_prev;
    float wheel_latest;
    float wheel_mult;

    int rolling_pts;

    ros::Time then;
    ros::Time prev_pid_time;
    ros::Duration pid_dt_duration;

    double rate;
    float ticks_per_meter;
    float timeout_ticks;
    float ticks_since_target;

    float Kp;
    float Ki;
    float Kd;

    //encoder 설정 값은 매크로로 설정해도 되지만, 외부 파라미터값을 받을 수 있도록 변수로 설정해둔다.
    long encoder_low_wrap;
    long encoder_high_wrap;
    long prev_encoder;
  

    //Function
    void doPID();
    void calcVelocity();
    void calcRollingVel();
    void appendVel(float val);

    void init_variavles();
    void get_parameters();
    void spinOnce();

    void targetCB(const std_msgs::Float32 &msg);
    void wheelCB(const std_msgs::Int64 &msg);
};

PID_Velocity::PID_Velocity()
{
    init_variavles();

    ROS_INFO("Started pid control node");

    //queue size 는 publisher의 queue size랑 맞춰줘야 한다. 
    wheel_sub = nh.subscribe("wheel", 10, &PID_Velocity::wheelCB, this);
    wheel_vtarget_sub = nh.subscribe("wheel_vtarget", 50, &PID_Velocity::targetCB, this);

    vel_pub = nh.advertise<std_msgs::Float32>("/wheel_vel", 10);
    motor_cmd_pub = nh.advertise<std_msgs::Float32>("/motor_cmd", 10);

    get_parameters();
    encoder_low_wrap = (ENCODER_MAX - ENCODER_MIN) * 0.3f + ENCODER_MIN;
    encoder_low_wrap = (ENCODER_MAX - ENCODER_MIN) * 0.7f + ENCODER_MIN;

    prev_vel[0] = 0.0f;
    prev_vel[1] = 0.0f;

    wheel_mult = 0;

}


void PID_Velocity::init_variavles()
{
    //현재는 쉐프봇에 맞춰있는거라서 수정해야 한다.
    Kp = 400.f;
    Ki = 100.f;
    Kd = 0;


    target_vel = 0;
    vel_output = 0;
    vel_output_min = -255.f;
    vel_output_max = 255.f;
    vel_threshold = 0.001f;

    rolling_pts = 2; // moving average count number

    then = ros::Time::now();
    prev_pid_time = ros::Time::now();

    rate = 50;
    ticks_per_meter = 23106.08660204f;// ChefBot에서는 14860
    timeout_ticks = 4;
    wheel_latest = 0;
    
     
    // wheel_latest = 0;
    // prev_encoder = 0;

}


void PID_Velocity::get_parameters()
{
    if(nh.getParam("Kp", Kp)){
        ROS_INFO_STREAM("Kp from param" << Kp);
    }

    if(nh.getParam("Ki", Ki)){
        ROS_INFO_STREAM("Ki from param" << Ki);
    }

    if(nh.getParam("Kd", Kd)){
        ROS_INFO_STREAM("Kd from param" << Kd);
    }

    if(nh.getParam("rate", rate)){
        ROS_INFO_STREAM("rate from param" << rate);
    }

    if(nh.getParam("timeout_ticks", timeout_ticks)){
        ROS_INFO_STREAM("timeout_ticks Width" << timeout_ticks);
    }

}


void PID_Velocity::spin()
{
    ros::Rate r(rate);
    ros::Rate idle(10);
    //then = ros::Time::now();

    ticks_since_target = timeout_ticks;
    wheel_prev = wheel_latest;
    then = ros::Time::now();
     
    while(ros::ok())
    {	
        spinOnce();
        r.sleep();
    }
	ros::spin();
}

void PID_Velocity::spinOnce()
{
    ros::Rate r(rate);

    previous_vel_error = 0.0f;
    prev_vel[0] = 0.0f;
    prev_vel[1] = 0.0f;
    vel_intergral = 0.0f;
    vel_error = 0.0f;
    vel_derivative = 0.0f;
    vel = 0.0f;
    ros::spinOnce();
    while(ros::ok() && (ticks_since_target < timeout_ticks))
    {
        calcVelocity();
        doPID();
        motor_output.data = vel_output;
        motor_cmd_pub.publish(motor_output);
        
        //버튼으로 움직이는 모델에서 이렇게 하면 않된다.이건 조이스틱에서나 쓸만한 알고리즘이다.
        //timeout_ticks는 버튼으로 입력되는 토픽에서는 쓸모가 없다. 굳이 ticks_since_target을 해주는 이유가 있다면? 
        ticks_since_target += 1;
        // if(ticks_since_target == timeout_ticks) 
        // {
        //     motor_output.data = 0;
        //     motor_cmd_pub.publish(motor_output);
        // }

        ros::spinOnce();
	    r.sleep();
    }
}

void PID_Velocity::calcVelocity()
{
    ros::Duration dt_duration = ros::Time::now() - then;
    float dt = dt_duration.toSec();

    cout << "dt" << dt << endl;
    
    // //모터가 움직이지 않을때 라고 생각했지만, 모터가 느리게 회전하면 같은 값 엔코더값이 여러 개 올 수도 있다.
    // //ticks_per_meter는 미터당 얼마나 틱이 확인됬나의 비율이다.
    // if(wheel_latest == wheel_prev)
    // {
    //     float cur_vel = ( 1 / ticks_per_meter) / dt;

    //     if( abs(cur_vel) < vel_threshold)
    //     {
    //        //cout << "vel_threshold low " << endl;
    //         appendVel(0);
    //         calcRollingVel();
    //     }
    //     else 
    //     {
    //         if(abs(cur_vel) < vel){
    //             cout << "motor stop but AR slow move" << endl;

    //             appendVel(cur_vel);
    //             calcRollingVel();
    //         }//여기에서 else처리를 하지 않으면 모터가 계속 회전해서 위험해 질 수 도 있다.-> 아니다. 어차피 dt값이 증가한다.
            
    //     }
    // }
    // else//모터가 움직일때
    // {
    //     cout << "WHY" << wheel_latest << '\t' << wheel_prev << endl;
    //     float cur_vel = (wheel_latest - wheel_prev) / dt;
    //     appendVel(cur_vel);
    //     calcRollingVel();

    //     cout << "Move AR" << endl;
    //     wheel_prev = wheel_latest;
    //     then = ros::Time::now();
    // }

        cout << "WHY" << wheel_latest << '\t' << wheel_prev << endl;
        float cur_vel = (wheel_latest - wheel_prev) / dt;
        appendVel(cur_vel);
        calcRollingVel();

        cout << "Move AR" << endl;
        wheel_prev = wheel_latest;
        then = ros::Time::now();

    wheel_vel.data = vel;
    vel_pub.publish(wheel_vel);
}

void PID_Velocity::appendVel(float val)
{
    float tmp = prev_vel[1];
    prev_vel[0] = tmp;
    prev_vel[1] = val;
}

void PID_Velocity::calcRollingVel()
{
    vel = (prev_vel[0] + prev_vel[1]) / 2.0f;
}

void PID_Velocity::doPID()
{
    pid_dt_duration = ros::Time::now() - prev_pid_time;
    float pid_dt = pid_dt_duration.toSec();

    prev_pid_time = ros::Time::now();

    vel_error = target_vel - vel;
    vel_intergral = vel_intergral + (vel_error * pid_dt);
    vel_derivative = (vel_error - previous_vel_error) / pid_dt;
    previous_vel_error = vel_error;

    vel_output = Kp * vel_error + Ki * vel_intergral + Kd * vel_derivative;
    

    if(vel_output > vel_output_max)
    {
        vel_output = vel_output_max;
        vel_intergral = vel_intergral - (vel_error * pid_dt);
    }
    if(vel_output < vel_output_min)
    {
        vel_output = vel_output_min;
        vel_intergral = vel_intergral - (vel_error * pid_dt);
    }
    if(target_vel == 0){
        vel_output = 0;
    }
    //cout << sizeof(long) << endl;
    //cout << "out:" << vel_output << " e:" << vel_error << " i:" << vel_intergral << endl;
}

void PID_Velocity::targetCB(const std_msgs::Float32 &msg)
{
    //cout << "targetCB" << endl;
    target_vel = msg.data;
    ticks_since_target = 0;
}

void PID_Velocity::wheelCB(const std_msgs::Int64 &msg)
{
    long enc = msg.data;

    if(enc < encoder_low_wrap && prev_encoder > encoder_high_wrap)
    {
        wheel_mult = wheel_mult + 1;
    }//갑자기 확 감소? 아니다.

    if(enc > encoder_high_wrap && prev_encoder < encoder_low_wrap)
    {
        wheel_mult = wheel_mult - 1;
    }//갑자시 확 증가? 아니다. 오버플로우를 막으려고 한 것
 
    wheel_latest = 1.0f * (enc + wheel_mult * (ENCODER_MAX - ENCODER_MIN)) / ticks_per_meter;
    prev_encoder = enc;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_velocity");
    PID_Velocity obj;

    obj.spin();
}

