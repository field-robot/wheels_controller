#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"

#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wheels_controller");

	ros::NodeHandle nh;

	ros::Publisher send_pwm = nh.advertise<std_msgs::UInt8>("pwm_value",100);
	ros::Publisher send_dir = nh.advertise<std_msgs::Bool>("direction",100);

	ros::Rate loop_rate(10);
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
		send_pwm.publish(value);
		send_dir.publish(dir);
		//ROS_INFO("%s", value.data);
		ros::spinOnce();
		loop_rate.sleep();
		count++;
	}

	return 0;
}
	



