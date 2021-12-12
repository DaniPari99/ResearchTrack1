#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "assignment2/User.h" //nome del pacchetto dove si trova il service User
#include "std_srvs/Empty.h"


#define RANGE_SIDE 40 //size of the two arrays of side distances
#define RANGE_FRONT 160 //size of the array of frontal distances
#define DTH 1 // min frontal distance threshold

#define MAX_VEL 5 //max velocity
#define MIN_VEL 0 //min velocity

//Initialize a publisher
ros::Publisher pub;

//Initialize a client
ros::ServiceClient client2;

//Initialize a global variable for storing the increasing/decreasing of velocity due to successive accelerations or decelerations
float vel_change = 0;

//Initialize a global variable for checking MAX/MIN vel
float velocity_realtime = 0;

//Global variable of type array for storing the values of the 720 elements of the msg 'ranges'
float myArraySide[RANGE_SIDE]; //for storing side distances
float myArrayFront[RANGE_FRONT]; //for storing front distances

//Function to compute the min distance above all the distances belonging to the left/right arrays
float calculate_min_side_dist(float *myArraySide)
{	
	float min_side_distance;
	 	
  	for(int i = 0; i < RANGE_SIDE; i++)
  	{
  		if(i == 0)
  		{
  			min_side_distance = myArraySide[i];
  		}
  		
  		if(myArraySide[i] < min_side_distance)
  		{
  			min_side_distance = myArraySide[i];
  		}
  	}
  	return min_side_distance;
}

//Function to compute the min distance above all the distances belonging to the frontal array
float calculate_min_front_dist(float *myArrayFront)
{	
	float min_front_distance;
  	
  	for(int i = 0; i < RANGE_FRONT; i++)
  	{
  		if(i == 0)
  		{
  			min_front_distance = myArrayFront[i];
  		}
  		
  		if(myArrayFront[i] < min_front_distance)
  		{
  			min_front_distance = myArrayFront[i]; 
  		}
  	}
  	return min_front_distance;
}

//Callback function that must be executed everytime that changes a value from the array 'ranges'
void controllerCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{	
	//Arrays for storing rispectively all the right, left, front distances
	float dist_rt_ranges[RANGE_SIDE];
	float dist_lf_ranges[RANGE_SIDE];
	float dist_front_ranges[RANGE_FRONT];
	
	//Local variables for storing the current min distance fr rispectively right, left and in front
	float min_rt_obstacle;
	float min_lf_obstacle;
	float min_front_obstacle;
	
	//I put the first 40 elements of the  array 'ranges' (that corresponds to the right distances) in an array called 'dist_rt_ranges'
	for(int i = 0; i < RANGE_SIDE; i++)	
  	{
  		dist_rt_ranges[i] = msg->ranges[i];  		
  	}	
	min_rt_obstacle = calculate_min_side_dist(dist_rt_ranges); //I calculate the min distance from right
	
	//I put the last 40 elements of the  array 'ranges' (that corresponds to the left distances) in an array called 'dist_rt_ranges'	
	for(int i = 0; i < RANGE_SIDE; i++)
  	{
  		dist_lf_ranges[i] = msg->ranges[i + 680];  		
  	}
	min_lf_obstacle = calculate_min_side_dist(dist_lf_ranges); //I calculate the min distance from left
	
	//I put the 160 central values of the  array 'ranges' (that corresponds to the front distances) in an array called 'dist_rt_ranges'
	for(int i = 0; i < RANGE_FRONT; i++)
  	{
  		dist_front_ranges[i] = msg->ranges[i + 280];		
  	}  	
	min_front_obstacle = calculate_min_front_dist(dist_front_ranges); //I calculate the min distance from in front
  	  	
/*
-------------------------------------------------------------------------------------------------------control unit--------------------------------------------------------------------------------------------
*/  	

	geometry_msgs::Twist my_vel; //Define a message of type geometry_msgs accordig to pubish velocity
	
	//Initialize the robot velocity at 2, but when the user increments or decrements it the velocity may change because of the change of the variable 'vel_change'
	my_vel.linear.x = 2 + vel_change;
	
	//Update the variable to check MAX/MIN vel
	velocity_realtime = my_vel.linear.x;
	
	//If the front distance threshold is reached
  	if(min_front_obstacle < DTH) 
  		{	
  			//make the robot to go where there is more space avoiding the obstacles
  			if(min_rt_obstacle > min_lf_obstacle)
  				{	
  					my_vel.linear.x = 0.3;
  					velocity_realtime = my_vel.linear.x;
  					my_vel.angular.z = -2;
  				}
  			else
  				{
  					my_vel.linear.x = 0.3;
  					velocity_realtime = my_vel.linear.x;
  					my_vel.angular.z = 2;
  				}
  		}
	
	//Publication of velocity
	pub.publish(my_vel);
}

//Callback function that must be called everytime that the client node (user_interface) send a request
bool serviceCallback(assignment2::User::Request &req, assignment2::User::Response &res)
{	
	//Initialize the node handler
	ros::NodeHandle nh;
	
	//Declare the client subscribed to the 'Empty' service
	client2 = nh.serviceClient<std_srvs::Empty>("/empty");
	
	geometry_msgs::Twist my_vel;
	
	//Declare my name of the Empty service
	std_srvs::Empty reset_srv;
	
	//If the command received from the user is 'a/A' --> increase the velocity
	if(req.command == 'a' || req.command == 'A')
	{	
		std::cout <<"Accelerate" << std::endl;
		
		if (velocity_realtime < MAX_VEL) //control for avoiding to go too fast
		vel_change += 0.1;
		res.feedback = true;		
	}
	//Else If the command received from the user is 'd/D' --> decrease the velocity
	else if(req.command == 'd' || req.command == 'D')
	{
		std::cout << "Decelerate" << std::endl;
		
		if (velocity_realtime > MIN_VEL) //control for avoiding to go in reverse
		vel_change -= 0.1;
		res.feedback = true;
	}
	//Else If the command received from the user is 'r/R' --> reset the robot position
	else if(req.command == 'r' || req.command == 'R')
	{
		std::cout << "Reset position" << std::endl;
		ros::service::call("/reset_positions", reset_srv);
		res.feedback = true;
	}
	//If something is wrong the feedback must be negative 
	else
	{
		res.feedback = false;
	}
	//Publish the increased / decreased velocity
	pub.publish(my_vel);
	
	return res.feedback;
	
}

//main robot_controller function
int main (int argc, char **argv)
{
	//Initialize the node, setup the NodeHandle for handling the communication with the ROS system  
	ros::init(argc, argv, "robot_subscriber");  
	ros::NodeHandle nh;
	
	//Define the publisher to the cmd_topic
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	//Define the subscriber to base_scan topic  
	ros::Subscriber sub = nh.subscribe("/base_scan", 1, controllerCallback);
	
	//Define a service 
	ros::ServiceServer service = nh.advertiseService("/user", serviceCallback);
	
	ros::spin();
	return 0;
}
