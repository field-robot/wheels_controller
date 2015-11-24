#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

double WheelDiameter;									//defining wheel diameter in meters
double WheelSpacing;									//defining wheel spacing in meters
#define PI 3.1415											//defining pi
#define pwmConversion 100
int motorDeadzone;

uint16_t pwmmotor1=1;											//creating uint16_t to send to arduino normally a uint8_t would be sufficient but due to possible overflow of this number a 2 byte int is chosen
uint16_t pwmmotor2=1;
uint16_t pwmmotor3=1;
uint16_t pwmmotor4=1;
uint8_t errorMsg;

uint8_t pwm_m1_prev;
uint8_t pwm_m2_prev;
uint8_t pwm_m3_prev;
uint8_t pwm_m4_prev;

int Pos1;
int Pos2;

ros::Time     lastE1update;
ros::Time     lastE2update;
double lastE1ticks;
double lastE2ticks;
double W1 =0;												//creating Pos1 which is the center of the distance between left front and back wheels
double W2=0;												//creating Pos2 which is the center of the distance between right front and back wheels
double lticksprev;											//creating a variable which will be the previous value of the encoders
double rticksprev;											
double Xprev = 0;												//creating a variable which will be the previous x value
double Yprev = 0;												//creating a variable which will be the previous y value
double X = 0;													//creating a variable which will be the x value
double Y = 0;													//creating a variable which will be the y value
double x_velocity =0;											//creating a variable which will be the velocity in x-direction
double y_velocity =0;											//creating a variable which will be the velocity in y-direction


sensor_msgs::JointState joint_vel_;
double left_back_w =0;
double left_front_w = 0;
double right_back_w = 0;
double right_front_w=0;

double phi = 0;										//creating a variable whih will be the rotation on the z-axis.this will start at 90 degrees
double phi_prev = -0.5*3.1415;										//creating a variable which will be the previous phi value									
double phi_velocity;										//creating a variable which will be the velocity of 
double dt;
bool dir_l = true;
bool dir_r = false;

bool dir_l_prev= false;
bool dir_r_prev= true;

bool autonomeus_drive = false;

float cmdLinX;
float cmdLinY;
float cmdAngZ;





void ticksLeft( const std_msgs::Float32& ticks)
{    
	ros::Time current_time_ = ros::Time::now();
	double dtE1 = (current_time_ - lastE1update).toSec();
	double deltaE1Ticks = ticks.data-lastE1ticks;
    W1 = deltaE1Ticks/39*2*3.1415/dtE1; // [rad/s] based on encoder value
    lastE1update = current_time_;
    lastE1ticks = ticks.data;
    ROS_INFO("Speed left: %f dt: %f, ticks: %f",W1,dtE1,ticks.data);
}

void ticksRight( const std_msgs::Float32& ticks1)
{	
	ros::Time current_time_ = ros::Time::now();
	double dtE2 = (current_time_ - lastE2update).toSec();
	double deltaE2Ticks =ticks1.data - lastE2ticks;
    W2 = deltaE2Ticks/39*2*3.1415/dtE2*-1; // [rad/s] based on encoder value
    lastE2update = current_time_;
        lastE2ticks = ticks1.data;
    ROS_INFO("Speed right: %f ",W2);
}

void arduinoError( const std_msgs::UInt8& error)
{
	errorMsg = error.data;
}



void jointVelocity( const sensor_msgs::JointState& jointstate)
{
	if (sizeof(jointstate.velocity) > 0){
	left_back_w = jointstate.velocity[0];
	ROS_INFO("Left back %f", left_back_w);
	left_front_w = jointstate.velocity[1];
	right_back_w = jointstate.velocity[2];
	right_front_w = jointstate.velocity[3];		
	if (left_back_w<0){ dir_l = true;left_back_w*=-1;}else dir_l = false;
	if (left_back_w > 0.1) left_back_w += 7.63;
	if (left_front_w<0){left_front_w*=-1;}
	if (left_front_w > 0.1) left_front_w += 7.63;
	if (right_back_w<0){ dir_r = false; right_back_w*=-1;}else dir_r = true;
	if (right_back_w > 0.1) right_back_w += 7.63;
	if (right_front_w<0){right_front_w *=-1;}
	if (right_front_w > 0.1) right_front_w += 7.63;
	}
}




int main(int argc, char **argv)
{
	ros::init(argc, argv, "wheels_controller");
	//initializing package
	ros::NodeHandle nh;
	ros::NodeHandle paramHandle("~");
	//starting node
	
	paramHandle.param("wheel_diameter", WheelDiameter, 0.15);
	paramHandle.param("wheel_distance", WheelSpacing, 0.38);
	paramHandle.param("motor_deadzone", motorDeadzone, 80);


	ros::Publisher send_pwm_motor1 = nh.advertise<std_msgs::UInt8>("sub_pwm_value_motor1",1);
	ros::Publisher send_pwm_motor2 = nh.advertise<std_msgs::UInt8>("sub_pwm_value_motor2",1);
	ros::Publisher send_dir_motor1 = nh.advertise<std_msgs::Bool>("direction_motor1",1);
	ros::Publisher send_dir_motor2 = nh.advertise<std_msgs::Bool>("direction_motor2",1);
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom",100);		//odometry publisher init
	
	tf::TransformBroadcaster   odom_broadcaster;													// tf transform broadcaster init
	

	ros::Subscriber errorArduino = nh.subscribe("arduino_error",1, &arduinoError);
	ros::Subscriber jointstates = nh.subscribe("joint_states",1,&jointVelocity);
	ros::Subscriber encoder_left = nh.subscribe("pub_speed_left",1, &ticksLeft);
    	ros::Subscriber encoder_right = nh.subscribe("pub_speed_right",1, &ticksRight);
	

	ros::Time current_time, last_time;
	last_time = ros::Time::now();
	ROS_INFO("%s", "Wheels Controller is running...");
	ros::Rate loop_rate(10);
	//setting loop frequency to 10Hz
	
    bool direct = true;
	while (ros::ok())
	{
       		 	current_time = ros::Time::now();
        dt = (current_time-last_time).toSec();
	double vrobot,wrobot;
	
	vrobot = (WheelDiameter/2)*(W1+W2)/2;

	wrobot = (WheelDiameter/2)*(W1-W2)/(WheelSpacing);
	phi = wrobot*dt + phi_prev;
	x_velocity = vrobot*cos(phi);
	y_velocity= vrobot*sin(phi);
	phi_prev = wrobot*dt + phi_prev;
	X = x_velocity*dt + Xprev;
	Xprev = X;
	Y = y_velocity*dt + Yprev;
	Yprev = Y;
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(phi+0.5*PI);    //-0.5*PI+phi
		
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";
	odom_trans.transform.translation.x = X;
	odom_trans.transform.translation.y = Y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;
	
	odom_broadcaster.sendTransform(odom_trans);
		
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";
		
	odom.pose.pose.position.x = X;
	odom.pose.pose.position.y = Y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;
	
	odom.twist.twist.linear.x = x_velocity;
	odom.twist.twist.linear.y = y_velocity;
	odom.twist.twist.angular.z = phi_velocity;
	
	odom_pub.publish(odom);

        //
		//end calculations
		//creating variables to send/receive
		
		
		
		
		//std_msgs::UInt8 error_message;
        //error_message.data = errorMsg;(
		
		


		
		//end of publishing statements
		
		ros::spinOnce();
		
		last_time = current_time;
		loop_rate.sleep();
		
	}

	return 0;
}
	



