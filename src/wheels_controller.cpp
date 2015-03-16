#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"

#include <sstream>

uint8_t pwmmotor1;
uint8_t	pwmmotor2;
uint8_t pwmmotor3;
uint8_t pwmmotor4;

//bool dir1;
//bool dir2;
//bool dir3;
//bool dir4;


void pwm_motor1( const std_msgs::UInt8& pwmvalue)
{
	pwmmotor1 = pwmvalue.data;
}
	
void pwm_motor2( const std_msgs::UInt8& pwmvalue)
{
	pwmmotor2 = pwmvalue.data;
}

void pwm_motor3( const std_msgs::UInt8& pwmvalue)
{
	pwmmotor3 = pwmvalue.data;
}

void pwm_motor4( const std_msgs::UInt8& pwmvalue)
{
	pwmmotor4 = pwmvalue.data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wheels_controller");
	//initializing package
	ros::NodeHandle nh;
	//starting node
	ros::Publisher send_pwm_motor1 = nh.advertise<std_msgs::UInt8>("pwm_value",100);
	ros::Publisher send_pwm_motor2 = nh.advertise<std_msgs::UInt8>("pwm_value_motor2",100);
	ros::Publisher send_pwm_motor3 = nh.advertise<std_msgs::UInt8>("pwm_value_motor3",100);
	ros::Publisher send_pwm_motor4 = nh.advertise<std_msgs::UInt8>("pwm_value_motor4",100);
	ros::Publisher send_dir_motor1 = nh.advertise<std_msgs::Bool>("direction_motor1",100);
	ros::Publisher send_dir_motor2 = nh.advertise<std_msgs::Bool>("direction_motor2",100);
	ros::Publisher send_dir_motor3 = nh.advertise<std_msgs::Bool>("direction_motor3",100);
	ros::Publisher send_dir_motor4 = nh.advertise<std_msgs::Bool>("direction_motor4",100);
	
	ros::Subscriber sub = nh.subscribe("speed_LF", 1, &pwm_motor1);
	ros::Subscriber sub1 = nh.subscribe("speed_RF", 1, &pwm_motor2);
	ros::Subscriber sub2 = nh.subscribe("speed_LB", 1, &pwm_motor3);
	ros::Subscriber sub3 = nh.subscribe("speed_RB", 1, &pwm_motor4);
	
	
	//creating publishers for pwm value and dir value
	ros::Rate loop_rate(10);
	//setting loop frequency to 10Hz
	//int count = 0;
	bool direct = true;
	while (ros::ok())
	{
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
	



