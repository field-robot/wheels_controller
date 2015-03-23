#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
//#include "math.h"
//#include <sstream>

#define WheelDiameter 0.150
#define WheelSpacing 0.3302
#define PI 3.1415

uint8_t pwmmotor1;
uint8_t	pwmmotor2;
uint8_t pwmmotor3;
uint8_t pwmmotor4;

int16_t oldticksleft;
int16_t oldticksright;

double Pos1;
double Pos2;
double lticksprev;
double rticksprev;
double Xprev;
double Yprev;
double X;
double Y;
float phi = 0.5*PI;
float phi_prev = 0.5*PI;




//bool dir1;
//bool dir2;
//bool dir3;
//bool dir4;


void pwm_motor1( const std_msgs::UInt8& pwmvalue)
{
	pwmmotor1 = pwmvalue.data;
	pwmmotor3 = pwmvalue.data;
}
	
void pwm_motor2( const std_msgs::UInt8& pwmvalue)
{
	pwmmotor2 = pwmvalue.data;
	pwmmotor4 = pwmvalue.data;
}

void pwm_motor3( const std_msgs::UInt8& pwmvalue)
{
	pwmmotor3 = pwmvalue.data;
}

void pwm_motor4( const std_msgs::UInt8& pwmvalue)
{
	pwmmotor4 = pwmvalue.data;
}

void ticksLeft( const std_msgs::Int16& ticks)
{
	if (ticks.data>lticksprev)
	{
		Pos1 = ticks.data/WheelDiameter;
		lticksprev=ticks.data;
	}
	
}

void ticksRight( const std_msgs::Int16& ticks)
{
	if (ticks.data>rticksprev)
	{
		Pos2 = ticks.data/WheelDiameter;
		rticksprev=ticks.data;
	}
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "wheels_controller");
	//initializing package
	ros::NodeHandle nh;
	//starting node
	ros::Publisher send_pwm_motor1 = nh.advertise<std_msgs::UInt8>("sub_pwm_value_motor1",100);
	ros::Publisher send_pwm_motor2 = nh.advertise<std_msgs::UInt8>("sub_pwm_value_motor2",100);
	ros::Publisher send_pwm_motor3 = nh.advertise<std_msgs::UInt8>("sub_pwm_value_motor3",100);
	ros::Publisher send_pwm_motor4 = nh.advertise<std_msgs::UInt8>("sub_pwm_value_motor4",100);
	ros::Publisher send_dir_motor1 = nh.advertise<std_msgs::Bool>("direction_motor1",100);
	ros::Publisher send_dir_motor2 = nh.advertise<std_msgs::Bool>("direction_motor2",100);
	ros::Publisher send_dir_motor3 = nh.advertise<std_msgs::Bool>("direction_motor3",100);
	ros::Publisher send_dir_motor4 = nh.advertise<std_msgs::Bool>("direction_motor4",100);
	
	ros::Subscriber sub = nh.subscribe("key_speed_LF", 1, &pwm_motor1);
	ros::Subscriber sub1 = nh.subscribe("key_speed_RF", 1, &pwm_motor2);
	ros::Subscriber sub2 = nh.subscribe("speed_LB", 1, &pwm_motor3);
	ros::Subscriber sub3 = nh.subscribe("speed_RB", 1, &pwm_motor4);
	
	ros::Subscriber encoder_left = nh.subscribe("sendTicksLeft",1, &ticksLeft);
	ros::Subscriber encoder_right = nh.subscribe("sendTicksRight",1, &ticksRight);
	//ros::Subscriber zRotGyro = nh.subscribe("zRotation",1, &zRot);
	//ros::Subscriber xAccGyro = nh.subscribe("xAccelaration",1, &xAcc);
	//ros::Subscriber yAccGyro = nh.subscribe("yAccelaration",1, &yAcc);
	
	//creating publishers for pwm value and dir value
	ros::Rate loop_rate(100);
	//setting loop frequency to 10Hz
	//int count = 0;
	bool direct = true;
	while (ros::ok())
	{
		if (abs((Pos1-Pos2))/WheelSpacing != phi_prev)
		{
			phi = abs(Pos1-Pos2)/WheelSpacing + phi_prev;
			phi_prev = abs(Pos1-Pos2)/WheelSpacing + phi_prev;
		}
		
		if ((0.5*WheelSpacing - cos(phi)*0.5*WheelSpacing) != Xprev)
		{
			X = 0.5*WheelSpacing - cos(phi)*0.5*WheelSpacing + Xprev;
			Xprev = 0.5*WheelSpacing - cos(phi)*0.5*WheelSpacing + Xprev;
		}
		
		if ((sin(phi)*0.5*WheelSpacing)!= Yprev)
		{
			Y= sin(phi)*0.5*WheelSpacing + Yprev;
			Yprev = sin(phi)*0.5*WheelSpacing + Yprev;
		}
		
		
		
		std_msgs::UInt8 valuemotor1;
		std_msgs::UInt8 valuemotor2;
		std_msgs::UInt8 valuemotor3;
		std_msgs::UInt8 valuemotor4;

		std_msgs::Bool  dir;
		
		
		valuemotor1.data = pwmmotor1;
		valuemotor2.data = pwmmotor2;
		valuemotor3.data = pwmmotor3;
		valuemotor4.data = pwmmotor4;

		dir.data = direct;
		send_pwm_motor1.publish(valuemotor1);
		send_pwm_motor2.publish(valuemotor2);
		send_pwm_motor3.publish(valuemotor3);
		send_pwm_motor4.publish(valuemotor4);
		send_dir_motor1.publish(dir);
		send_dir_motor2.publish(dir);
		send_dir_motor3.publish(dir);
		send_dir_motor4.publish(dir);
		//ROS_INFO("%s", value.data);
		ros::spinOnce();
		loop_rate.sleep();
		//count++;
	}

	return 0;
}
	



