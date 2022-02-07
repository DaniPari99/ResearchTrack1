#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "final_assignment/User.h"
#include "std_srvs/Empty.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "actionlib_msgs/GoalID.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"


#define DTH 1
#define GOAL_TH 0.5


//Initialize a publishers
ros::Publisher pub_goal;
ros::Publisher pub_vel;
ros::Publisher pub_cancel;

//Initialize a client
ros::ServiceClient client;


bool manDrive = false; //bool variable for enabling 2nd modality
bool collisionAvoidence = false; //bool variable for enabling 3rd modality
bool goal = false; //bool variable to store the status of the goal

//global variables for storing the goal chosen by the user
float target_x;
float target_y;

std::string id = ""; // Goal ID

geometry_msgs::Twist my_vel; // Velocity to be checked by the collision avoidence before being published

//message for cancelling a goal when it is reached
actionlib_msgs::GoalID canc_goal;

//function to cancell a goal if it is reached
bool cancelGoal()
{
	if(!goal)
		return false;
	canc_goal.id = id;
	pub_cancel.publish(canc_goal);
	goal = false;
	return true;
	
}

//function to compute the distance from the goal
void currentPositionCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg)
{

    // Update the goal ID if there is a new goal
    if (id != msg->status.goal_id.id) 
    {
        id = msg->status.goal_id.id;
    }
    
    if(goal)
    {
    	//take the current robot position
        float dist_x;
	float dist_y;
	float robot_x = msg->feedback.base_position.pose.position.x;
	float robot_y = msg->feedback.base_position.pose.position.y;
    
	//calculate the error from the actual position and the goal position
	dist_x = robot_x - target_x;
	dist_y = robot_y - target_y;

	//if the robot is on the goal position
	if (abs(dist_x) <= GOAL_TH && abs(dist_y) <= GOAL_TH)
	{
	     printf("Goal reached\n");
	     cancelGoal();
	}

     }
}

/*
Callback function called whenever the teleop twist keyboard want to change the robot velocity
If the robot isn't neither in manual driving nor in collision avoidence modality the message is ignored
If the robot is in manual driving modality the message is sent to the cmd_vel topic
If the robot is in collision avoidence modality the message is saved in a global variable (my_vel)
and it will be corrected by the collision avoidence function before being published
*/
void getVelCallback(const geometry_msgs::Twist::ConstPtr& msg) 
{
    if(!manDrive && !collisionAvoidence)
    	return;
    if(manDrive && !collisionAvoidence)
    {
    	pub_vel.publish(msg);
    	return;
    }
    if(!manDrive && collisionAvoidence)
    {
    	my_vel.linear.x = msg->linear.x;
    	my_vel.angular.z = msg->angular.z;
    }
}

//function to compute the min element of an array
double computeMinDist(double scan[], int size)
{
	double min_dist = 100;
	for(int i=0; i < size; i++)
	{
		if(scan[i] < min_dist)
		{
			min_dist = scan[i];
		}
	}
	return min_dist;
}


//This function is called whenever a message from LaserScan topic arrives.
void collisionAvoidenceCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	if(!collisionAvoidence)
		return;
	//get array ranges by laserscan
	float scan[msg->ranges.size()];
	
	for(int j = 0; j < msg->ranges.size(); j++)
	{
		scan[j] = msg->ranges[j];
	}
	
	//divide the array ranges values in 5 sectors
	int sector_size = (msg->ranges.size())/5;
	double left[sector_size];
	double front_left[sector_size];
	double front[sector_size];
	double front_right[sector_size];
	double right[sector_size];
	
	//fill sector arrays
	for(int i = 0; i < sector_size; i++)
	{
		right[i] = scan[i];
	}
	
	for(int i = 0; i < sector_size; i++)
	{
		front_right[i] = scan[i + sector_size];
	}
	
	for(int i = 0; i < sector_size; i++)
	{
		front[i] = scan[i + 2*sector_size];
	}
	
	for(int i = 0; i < sector_size; i++)
	{
		front_left[i] = scan[i + 3*sector_size];
	}
	
	for(int i = 0; i < sector_size; i++)
	{
		left[i] = scan[i + 4*sector_size];
	}
	
		
	//check if the robot is going to crash to obstacles and in case stop it
	if(computeMinDist(front, sector_size) < DTH) 
	{
		if(my_vel.linear.x > 0 && my_vel.angular.z == 0) 
		{
			my_vel.linear.x = 0;
			printf("Attenction: there is an obstacle in front\n"); 
		}
	}
		
	if(computeMinDist(front_left, sector_size) < DTH)
	{
		if(my_vel.linear.x > 0 && my_vel.angular.z > 0)
		{
			my_vel.linear.x = 0;
			my_vel.angular.z = 0;
			printf("Attenction: there is an obstacle on the front-left\n");
		}
	}
	
	if(computeMinDist(front_right, sector_size) < DTH)
	{
		if(my_vel.linear.x > 0 && my_vel.angular.z < 0)
		{
			my_vel.linear.x = 0;
			my_vel.angular.z = 0;
			printf("Attenction: there is an obstacle on the front-right\n");
		}
	}
		
	if(computeMinDist(left, sector_size) < DTH)
	{
		if(my_vel.linear.x == 0 && my_vel.angular.z > 0)
		{
			my_vel.angular.z = 0;
			printf("Attenction: there is an obstacle on the left\n");
		}
	}
		
	if(computeMinDist(right, sector_size) < DTH)
	{
		if(my_vel.linear.x == 0 && my_vel.angular.z < 0)
		{
			my_vel.angular.z = 0;
			printf("Attenction: there is an obstacle on the right\n");
		}
	}

    	pub_vel.publish(my_vel); //publish the corrected velocity for avoiding collisions 
		
}



//Callback function that must be called everytime that the client node sends a request
bool serviceCallback(final_assignment::User::Request &req, final_assignment::User::Response &res)
{	
	//Initialize the node handler
	ros::NodeHandle nh;
	
	//if 1st modality is chosen
	if(req.command == 1)
	{	
		std::cout <<"1st modality chosen" << std::endl;
		
		
		
		move_base_msgs::MoveBaseActionGoal goal_msg;
		
		std::cout <<"What target do you want to achieve?\n" << std::endl;
		
		//get the user's position goal chosen
		std::cout <<"Type a x" << std::endl;
		std::cin >> target_x;		
		std::cout <<"Type a y" << std::endl;
		std::cin >> target_y;
		
		//set the goal status at true
		goal = true;
		
		goal_msg.goal.target_pose.header.frame_id = "map";
		goal_msg.goal.target_pose.pose.orientation.w = 1;
		
		//fill the goal position fields
		goal_msg.goal.target_pose.pose.position.x = target_x;
		goal_msg.goal.target_pose.pose.position.y = target_y;
		
		//publish the goal chosen 
		pub_goal.publish(goal_msg);
		
		
		res.feedback = true;		
	}
	//if 2nd modality is chosen
	else if(req.command == 2)
	{
		std::cout << "2nd modality chosen" << std::endl;
		
		manDrive = true;
		collisionAvoidence = false;

		res.feedback = true;
	}
	//if 3rd modality is chosen
	else if(req.command == 3)
	{
		std::cout << "3rd modality chosen" << std::endl;
		
		collisionAvoidence = true;
		manDrive = false;

		res.feedback = true;
	}
	//If something is wrong the feedback must be negative 
	else
	{
		res.feedback = false;
	}
	
	return res.feedback;
	
}

//main server function
int main (int argc, char **argv)
{
	//Initialize the node, setup the NodeHandle for handling the communication with the ROS system   
	ros::init(argc, argv, "server"); 
	ros::NodeHandle nh;
	
	//publish to some topics
	pub_goal = nh.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 1);
	pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	pub_cancel = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1); // Cancel actual goal
	
	//subsrcribe to some topics
	ros::Subscriber sub_keyBoard = nh.subscribe("/new_cmd_vel", 1, getVelCallback); // Get velocity from teleop twist keyboard
	ros::Subscriber sub_laser = nh.subscribe("/scan", 1, collisionAvoidenceCallback); // Laser scanner
	ros::Subscriber sub_position = nh.subscribe("/move_base/feedback", 1, currentPositionCallback); // Current  Status
	
	//Define a service 
	ros::ServiceServer service = nh.advertiseService("/user", serviceCallback);
	
	ros::spin();
	return 0;
}
