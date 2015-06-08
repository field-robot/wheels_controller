#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define WheelDiameter 0.150									//defining wheel diameter in meters
#define WheelSpacing 0.3302									//defining wheel spacing in meters
#define PI 3.1415								//defining pi
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


double Pos1 =0;												//creating Pos1 which is the center of the distance between left front and back wheels
double Pos2=0;												//creating Pos2 which is the center of the distance between right front and back wheels
double lticksprev;											//creating a variable which will be the previous value of the encoders
double rticksprev;											
double Xprev;												//creating a variable which will be the previous x value
double Yprev;												//creating a variable which will be the previous y value
double X = 0;													//creating a variable which will be the x value
double Y = 0;													//creating a variable which will be the y value
double x_velocity =0;											//creating a variable which will be the velocity in x-direction
double y_velocity =0;											//creating a variable which will be the velocity in y-direction


double phi = 0;										//creating a variable whih will be the rotation on the z-axis.this will start at 90 degrees
double phi_prev = 0;										//creating a variable which will be the previous phi value									
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
    //Pos1 = (ticks.data*(WheelDiameter*PI)/39*20); //based on encoder value
   
    ROS_INFO("Speed left: %f ",ticks.data);
    
}

void ticksRight( const std_msgs::Float32& ticks1)
{	
   //Pos2 = (ticks1.data*(WheelDiameter*PI)/39*20); //based on encoder value
   ROS_INFO("Speed right: %f ",ticks1.data);
}

void arduinoError( const std_msgs::UInt8& error)
{
	errorMsg = error.data;
}

void directionLeft( const std_msgs::Bool& dir)
{
	dir_l = dir.data;
	if (dir.data == true) Pos1*=-1;
}

void directionRight( const std_msgs::Bool& dir)
{
	dir_r = dir.data;
	if (dir.data == true) Pos2*=-1;
}

void cmdVel( const geometry_msgs::Twist& twist)
{
	cmdLinX = twist.linear.x;
	cmdLinY = twist.linear.y;
	cmdAngZ = twist.angular.z;
	
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
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 200);							//odometry publisher init
	
	tf::TransformBroadcaster odom_broadcaster;													// tf transform broadcaster init
	
	ros::Subscriber sub = nh.subscribe("key_speed_LF", 1, &pwm_motor1);
	ros::Subscriber sub1 = nh.subscribe("key_speed_RF", 1, &pwm_motor2);
	ros::Subscriber sub2 = nh.subscribe("speed_LB", 1, &pwm_motor3);
	ros::Subscriber sub3 = nh.subscribe("speed_RB", 1, &pwm_motor4);
	ros::Subscriber errorArduino = nh.subscribe("arduino_error",1, &arduinoError);
	
    ros::Subscriber encoder_left = nh.subscribe("pub_speed_left",1, &ticksLeft);
    ros::Subscriber encoder_right = nh.subscribe("pub_speed_right",1, &ticksRight);
	
	ros::Subscriber dirLeft = nh.subscribe("dir_L",1, &directionLeft);
	ros::Subscriber dirRight = nh.subscribe("dir_R",1, &directionRight);
	
	ros::Subscriber cmdvel = nh.subscribe("cmd_vel",1,&cmdVel);
	
	ros::Subscriber autonomeus = nh.subscribe("switching",1,&SwitchState);


	//ros::Subscriber zRotGyro = nh.subscribe("zRotation",1, &zRot);
	//ros::Subscriber xAccGyro = nh.subscribe("xAccelaration",1, &xAcc);
	//ros::Subscriber yAccGyro = nh.subscribe("yAccelaration",1, &yAcc);
	
	//creating publishers for pwm value and dir value
	
	ros::Time current_time, last_time;
	last_time = ros::Time::now();
	ROS_INFO("%s", "Wheels Controller is running...");
	ros::Rate loop_rate(10);
	//setting loop frequency to 10Hz
	
    bool direct = true;
	while (ros::ok())
	{
        current_time = ros::Time::now();

		//calculating odometry of the robot
		
		//if ((Pos1-Pos2)/(2*WheelSpacing) != phi_prev)
		//{
			phi = (Pos1-Pos2)/(2*WheelSpacing)+phi_prev;
			phi_prev = (Pos1-Pos2)/(2*WheelSpacing)+phi_prev;
		//}
		
		//if ((Pos1+Pos2)/2*sin(phi) != Xprev)
		//{
			X = (Pos1+Pos2)/2*sin(phi)+Xprev;
			Xprev = (Pos1+Pos2)/2*sin(phi)+Xprev;
		//}
		
		//if ((-(Pos1+Pos2)/2)*cos(phi)!= Yprev)
		//{
			Y= -((Pos1+Pos2)/2)*cos(phi)+Yprev;
			Yprev = -((Pos1+Pos2)/2)*cos(phi)+Yprev;
		//}
		x_velocity = (Pos1+Pos2)/2*cos(phi);
		y_velocity = (Pos1+Pos2)/2*sin(phi);
		phi_velocity = (Pos1-Pos2)/(2*WheelSpacing);
		//end of calculations of odometry
		//creating an odometry message
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(-0.5*PI+phi);    //-0.5*PI+phi
		
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
		//end of odometry message
		//kinematic calculations 
       // if (autonomeus_drive == true)
        //{
        
		pwmmotor2 = (sqrt(cmdLinX*cmdLinX+cmdLinY*cmdLinY)+0.5*WheelSpacing*cmdAngZ)*pwmConversion;
		Pos2 = pwmmotor2/pwmConversion;
		ROS_INFO("Pos2: %f ",Pos2);
		ROS_INFO("Speed right: %f ",Pos2);
		if (cmdLinX <0){
		
		pwmmotor2 += motorDeadzone;
		dir_r = true;
		}else dir_r = false;
		pwmmotor2 += motorDeadzone;
		pwmmotor4 = pwmmotor2;
		
		pwmmotor1 = (sqrt(cmdLinX*cmdLinX+cmdLinY*cmdLinY)-0.5*WheelSpacing*cmdAngZ)*pwmConversion;
		Pos1 = pwmmotor1/pwmConversion;
		ROS_INFO("Speed left: %f ",Pos1);
		if (cmdLinX <0){
		
		pwmmotor1 += motorDeadzone;
		dir_l = true;
		}else dir_l = false;
		pwmmotor1 += motorDeadzone;
        pwmmotor3 = pwmmotor1;
        if (cmdLinX == 0 && cmdLinY == 0 && cmdAngZ == 0)
		{
			pwmmotor1 = 0;
			pwmmotor2 = 0;
			pwmmotor3 = 0;
			pwmmotor4 = 0;
		}
        //}
		//end calculations
		//creating variables to send/receive
		
		
		
		
		//std_msgs::UInt8 error_message;
        //error_message.data = errorMsg;(
		
		
		
	
		
         
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
			dir1.data = !dir_r;
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
	



