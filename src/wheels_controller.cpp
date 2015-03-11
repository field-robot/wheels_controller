#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"

#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wheels_controller");
	//initializing package
	ros::NodeHandle nh;
	//starting node
	ros::Publisher send_pwm_motor1 = nh.advertise<std_msgs::UInt8>("pwm_value_motor1",100);
	ros::Publisher send_pwm_motor2 = nh.advertise<std_msgs::UInt8>("pwm_value_motor2",100);
	ros::Publisher send_pwm_motor3 = nh.advertise<std_msgs::UInt8>("pwm_value_motor3",100);
	ros::Publisher send_pwm_motor4 = nh.advertise<std_msgs::UInt8>("pwm_value_motor4",100);
	ros::Publisher send_dir_motor1 = nh.advertise<std_msgs::Bool>("direction_motor1",100);
	ros::Publisher send_dir_motor2 = nh.advertise<std_msgs::Bool>("direction_motor2",100);
	ros::Publisher send_dir_motor3 = nh.advertise<std_msgs::Bool>("direction_motor3",100);
	ros::Publisher send_dir_motor4 = nh.advertise<std_msgs::Bool>("direction_motor4",100);
	//creating publishers for pwm value and dir value
	ros::Rate loop_rate(10);
	//setting loop frequency to 10Hz
	int count = 0;
	bool direct = true;
	while (ros::ok())
	{
		std_msgs::UInt8 value;
		std_msgs::Bool  dir;
		if (count == 256)
		{	
			count=0;
			if (direct==true){direct = false;}
			else {direct = true;}
		}
		
		value.data = count;
		dir.data = direct;
		send_pwm_motor1.publish(value);
		send_pwm_motor2.publish(value);
		send_pwm_motor3.publish(value);
		send_pwm_motor4.publish(value);
		send_dir_motor1.publish(dir);
		send_dir_motor2.publish(dir);
		send_dir_motor3.publish(dir);
		send_dir_motor4.publish(dir);
		//ROS_INFO("%s", value.data);
		ros::spinOnce();
		loop_rate.sleep();
		count++;
	}

	return 0;
}
	



