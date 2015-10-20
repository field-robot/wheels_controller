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

#define WheelDiameter 0.150									//defining wheel diameter in meters
#define WheelSpacing 0.3302									//defining wheel spacing in meters
#define PI 3.1415											//defining pi
#define pwmConversion 100
#define motorDeadzone 50

uint8_t pwmmotor1=1;											//creating uint8_t to send to arduino
uint8_t	pwmmotor2=1;
uint8_t pwmmotor3=1;
uint8_t pwmmotor4=1;
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



void pwm_motor1( const std_msgs::UInt8& pwmvalue)
{
	pwmmotor1 = pwmvalue.data;
	pwmmotor3 = pwmvalue.data;
	//Pos1 = (pwmvalue.data/2)*(WheelDiameter*PI)/20; //based on pwm value
    
	
}
	
void pwm_motor2( const std_msgs::UInt8& pwmvalue)
{
	pwmmotor2 = pwmvalue.data;
	pwmmotor4 = pwmvalue.data;
	//Pos2 = (pwmvalue.data/2)*(WheelDiameter*PI)/20; //#pwm value
    

}

void pwm_motor3( const std_msgs::UInt8& pwmvalue)
{
	pwmmotor3 = pwmvalue.data;
}

void pwm_motor4( const std_msgs::UInt8& pwmvalue)
{
	pwmmotor4 = pwmvalue.data;
}

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

void directionLeft( const std_msgs::Bool& dir)
{
	dir_l = dir.data;
	if (dir.data == true) W1*=-1;
}

void directionRight( const std_msgs::Bool& dir)
{
	dir_r = dir.data;
	if (dir.data == true) W2*=-1;
}

/*void cmdVel( const geometry_msgs::Twist& twist)
{
	cmdLinX = twist.linear.x;
	cmdAngZ = twist.angular.z;
	
}*/

void jointVelocity( const sensor_msgs::JointState& jointstate)
{
	left_back_w = jointstate.velocity[0];
	if (jointstate.velocity[0]<0){ dir_l = true;left_back_w*=-1;}else dir_l = false;
	ROS_INFO("Direction Left: %b",dir_l);
	left_front_w = jointstate.velocity[1];
	if (jointstate.velocity[1]<0){left_front_w*=-1;}
	right_back_w = jointstate.velocity[2];
	if (jointstate.velocity[2]<0){ dir_r = false; right_back_w*=-1;}else dir_r = true;
	right_front_w = jointstate.velocity[3];
	if (jointstate.velocity[3]<0){right_front_w *=-1;}
	ROS_INFO("Speed left_back: %f",left_back_w);
}

void SwitchState(const std_msgs::UInt8& button)
{
	if (button.data == 1)
	{
	autonomeus_drive = true;
	}
	if (button.data == 0)
	{
	autonomeus_drive = false;
	}
	ROS_INFO("Button State: %i ",button.data);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "wheels_controller");
	//initializing package
	ros::NodeHandle nh;
	//starting node
	//ros::Publisher send_error = nh.advertise<std_msgs::UInt8>("arduino_error",1);
	ros::Publisher send_pwm_motor1 = nh.advertise<std_msgs::UInt8>("sub_pwm_value_motor1",1);
	ros::Publisher send_pwm_motor2 = nh.advertise<std_msgs::UInt8>("sub_pwm_value_motor2",1);
	ros::Publisher send_pwm_motor3 = nh.advertise<std_msgs::UInt8>("sub_pwm_value_motor3",1);
	ros::Publisher send_pwm_motor4 = nh.advertise<std_msgs::UInt8>("sub_pwm_value_motor4",1);
	ros::Publisher send_dir_motor1 = nh.advertise<std_msgs::Bool>("direction_motor1",1);
	ros::Publisher send_dir_motor2 = nh.advertise<std_msgs::Bool>("direction_motor2",1);
	ros::Publisher send_dir_motor3 = nh.advertise<std_msgs::Bool>("direction_motor3",1);
	ros::Publisher send_dir_motor4 = nh.advertise<std_msgs::Bool>("direction_motor4",1);
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);					//odometry publisher init
	
	tf::TransformBroadcaster odom_broadcaster;													// tf transform broadcaster init
	

	ros::Subscriber errorArduino = nh.subscribe("arduino_error",1, &arduinoError);
	ros::Subscriber jointstates = nh.subscribe("joint_states",1,&jointVelocity);
	
    ros::Subscriber encoder_left = nh.subscribe("pub_speed_left",1, &ticksLeft);
    ros::Subscriber encoder_right = nh.subscribe("pub_speed_right",1, &ticksRight);
	
	ros::Subscriber dirLeft = nh.subscribe("dir_L",1, &directionLeft);
	ros::Subscriber dirRight = nh.subscribe("dir_R",1, &directionRight);
	
	//ros::Subscriber cmdvel = nh.subscribe("cmd_vel",1,&cmdVel);
	
	ros::Subscriber autonomeus = nh.subscribe("switching",1,&SwitchState);

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
	//ROS_INFO("X: %f x_vel: %f x_prev: %f",X,x_velocity,Xprev);
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
		
		
		pwmmotor1 = left_front_w/25*255;
		ROS_INFO("PWM motor 1: %i", pwmmotor1);
		pwmmotor2 = right_front_w/25*255;
		ROS_INFO("PWM motor 2: %i", pwmmotor2);
		pwmmotor3 = left_back_w/25*255;
		ROS_INFO("PWM motor 3: %i", pwmmotor3);
		pwmmotor4 = right_back_w/25*255;
		ROS_INFO("PWM motor 4: %i", pwmmotor4);
	
		
         
		//end of variables declaration
		//publishing statements
		//send_error.publish(error_message);
		if (pwmmotor1 != pwm_m1_prev)
		{
			std_msgs::UInt8 valuemotor1;
			valuemotor1.data = pwmmotor1;
			send_pwm_motor1.publish(valuemotor1);
			pwm_m1_prev = pwmmotor1;
		}
		if (pwmmotor2 != pwm_m2_prev)
		{
			std_msgs::UInt8 valuemotor2;
			valuemotor2.data = pwmmotor2;
			send_pwm_motor2.publish(valuemotor2);
			pwm_m2_prev = pwmmotor2;
		}
		if (pwmmotor3 != pwm_m3_prev)
		{
			std_msgs::UInt8 valuemotor3;
			valuemotor3.data = pwmmotor3;
			send_pwm_motor3.publish(valuemotor3);
			pwm_m3_prev = pwmmotor3;
		}
		if (pwmmotor4 != pwm_m4_prev)
		{
			std_msgs::UInt8 valuemotor4;
			valuemotor4.data = pwmmotor4;
			send_pwm_motor4.publish(valuemotor4);
			pwm_m4_prev = pwmmotor4;
		}
		
			std_msgs::Bool  dir;
			dir.data = dir_l;
			send_dir_motor1.publish(dir);
            send_dir_motor3.publish(dir);
			dir_l_prev = dir_l;
		
		
			std_msgs::Bool  dir1;
			dir1.data = dir_r;
			send_dir_motor2.publish(dir1);
            send_dir_motor4.publish(dir1);
			dir_r_prev = dir_r;
		

		
		//end of publishing statements
		
		ros::spinOnce();
		
		last_time = current_time;
		loop_rate.sleep();
		
	}

	return 0;
}
	



