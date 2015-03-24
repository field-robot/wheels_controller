#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define WheelDiameter 0.150									//defining wheel diameter in meters
#define WheelSpacing 0.3302									//defining wheel spacing in meters
#define PI 3.1415								//defining pi

uint8_t pwmmotor1;											//creating uint8_t to send to arduini
uint8_t	pwmmotor2;
uint8_t pwmmotor3;
uint8_t pwmmotor4;
uint8_t errorMsg;

double Pos1;												//creating Pos1 which is the center of the distance between left front and back wheels
double Pos2;												//creating Pos2 which is the center of the distance between right front and back wheels
double lticksprev;											//creating a variable which will be the previous value of the encoders
double rticksprev;											
double Xprev;												//creating a variable which will be the previous x value
double Yprev;												//creating a variable which will be the previous y value
double X;													//creating a variable which will be the x value
double Y;													//creating a variable which will be the y value
double x_velocity;											//creating a variable which will be the velocity in x-direction
double y_velocity;											//creating a variable which will be the velocity in y-direction


double phi = 0.5*PI;										//creating a variable whih will be the rotation on the z-axis.this will start at 90 degrees
double phi_prev = 0;										//creating a variable which will be the previous phi value									
double phi_velocity;										//creating a variable which will be the velocity of 
double dt;
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

void ticksLeft( const std_msgs::Int32& ticks)
{
	if (ticks.data>lticksprev)
	{
		Pos1 = ticks.data/WheelDiameter;
		lticksprev=ticks.data;
	}
	
}

void ticksRight( const std_msgs::Int32& ticks)
{
	if (ticks.data>rticksprev)
	{
		Pos2 = ticks.data/WheelDiameter;
		rticksprev=ticks.data;
	}
}

void arduinoError( const std_msgs::UInt8& error)
{
	errorMsg = error.data;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "wheels_controller");
	//initializing package
	ros::NodeHandle nh;
	//starting node
	ros::Publisher send_error = nh.advertise<std_msgs::UInt8>("arduino_error",100);
	ros::Publisher send_pwm_motor1 = nh.advertise<std_msgs::UInt8>("sub_pwm_value_motor1",100);
	ros::Publisher send_pwm_motor2 = nh.advertise<std_msgs::UInt8>("sub_pwm_value_motor2",100);
	ros::Publisher send_pwm_motor3 = nh.advertise<std_msgs::UInt8>("sub_pwm_value_motor3",100);
	ros::Publisher send_pwm_motor4 = nh.advertise<std_msgs::UInt8>("sub_pwm_value_motor4",100);
	ros::Publisher send_dir_motor1 = nh.advertise<std_msgs::Bool>("direction_motor1",100);
	ros::Publisher send_dir_motor2 = nh.advertise<std_msgs::Bool>("direction_motor2",100);
	ros::Publisher send_dir_motor3 = nh.advertise<std_msgs::Bool>("direction_motor3",100);
	ros::Publisher send_dir_motor4 = nh.advertise<std_msgs::Bool>("direction_motor4",100);
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);							//odometry publisher init
	
	tf::TransformBroadcaster odom_broadcaster;													// tf transform broadcaster init
	
	ros::Subscriber sub = nh.subscribe("key_speed_LF", 1, &pwm_motor1);
	ros::Subscriber sub1 = nh.subscribe("key_speed_RF", 1, &pwm_motor2);
	ros::Subscriber sub2 = nh.subscribe("speed_LB", 1, &pwm_motor3);
	ros::Subscriber sub3 = nh.subscribe("speed_RB", 1, &pwm_motor4);
	ros::Subscriber errorArduino = nh.subscribe("arduino_error",1, &arduinoError);
	
	ros::Subscriber encoder_left = nh.subscribe("sendTicksLeft",1, &ticksLeft);
	ros::Subscriber encoder_right = nh.subscribe("sendTicksRight",1, &ticksRight);
	//ros::Subscriber zRotGyro = nh.subscribe("zRotation",1, &zRot);
	//ros::Subscriber xAccGyro = nh.subscribe("xAccelaration",1, &xAcc);
	//ros::Subscriber yAccGyro = nh.subscribe("yAccelaration",1, &yAcc);
	
	//creating publishers for pwm value and dir value
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	
	ros::Rate loop_rate(100);
	//setting loop frequency to 100Hz
	
	bool direct = true;
	while (ros::ok())
	{
		current_time = ros::Time::now();
		dt = (current_time-last_time).toSec();
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
		x_velocity = (X-Xprev)/dt;
		y_velocity = (Y-Yprev)/dt;
		phi_velocity = (phi-phi_prev)/dt;
		
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(phi);    
		
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
		odom.child_frame_id = "base_link";
		
		odom.pose.pose.position.x = X;
		odom.pose.pose.position.y = Y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		
		odom.twist.twist.linear.x = x_velocity;
		odom.twist.twist.linear.y = y_velocity;
		odom.twist.twist.angular.z = phi_velocity;
		
		odom_pub.publish(odom);
		
		std_msgs::UInt8 valuemotor1;
		std_msgs::UInt8 valuemotor2;
		std_msgs::UInt8 valuemotor3;
		std_msgs::UInt8 valuemotor4;
		std_msgs::UInt8 error_message;

		std_msgs::Bool  dir;
		
		error_message.data = errorMsg;
		valuemotor1.data = pwmmotor1;
		valuemotor2.data = pwmmotor2;
		valuemotor3.data = pwmmotor3;
		valuemotor4.data = pwmmotor4;

		dir.data = direct;
		send_error.publish(error_message);
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
		
		last_time = current_time;
		loop_rate.sleep();
		//count++;
	}

	return 0;
}
	



