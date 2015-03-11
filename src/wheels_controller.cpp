#include "ros/ros.h"
#include "std_msgs/UInt8.h"

#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wheels_controller");

	ros::NodeHandle nh;

	ros::Publisher send_pwm = nh.advertise<std_msgs::UInt8>("pwm_value",100);

	ros::Rate loop_rate(10);
	int count = 0;

	while (ros::ok())
	{
		if (count == 256)
		{	
			count=0;
		}
		std_msgs::UInt8 value;
		value.data = count;

		send_pwm.publish(value);
		//ROS_INFO("%s", value.data);
		ros::spinOnce();
		loop_rate.sleep();
		count++;
	}

	return 0;
}
	



