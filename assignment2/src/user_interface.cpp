#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "assignment2/User.h" // this header should be both on the client side and on the server side

//Initialize the client that send the request to the server (robot_controller)
ros::ServiceClient client;

//Initialize a global variable for storing the choice pressed by the user
char choice;
bool resp;

//main user_interface function
int main (int argc, char **argv)
{	
	//Initialize the node, setup the NodeHandle for handling the communication with the ROS system  
	ros::init(argc, argv, "user_interface");  
	ros::NodeHandle nh;
	
	//Declare the client subscribed to the 'Empty' service
	client = nh.serviceClient<assignment2::User>("/user");
	
	//Declare my name of the User service
	assignment2::User usr_srv;
	
	while(1)
	{	
		//Menu to tell the user which are the possible choices
		std::cout << "Menu: you can type: \n 1) a: for accelerate \n 2) d: for decelerate \n 3) r: for resetting the robot position" << std::endl;
		
		//Get the command pressed from the stdin and store it in a variable
		std::cin >> choice;
		
		//Check if the command pressed from the user is correct
		if(choice == 'a' || choice == 'd' || choice == 'r' || choice == 'A' || choice == 'D' || choice == 'R')
		{
			std::cout << "the character typed is: " << choice << std::endl;

			usr_srv.request.command = choice;
			client.waitForExistence();
			
			//Call a service
			client.call(usr_srv);
			
			//Put into 'resp' variable the response by the server. It tells me if the request is satisfied or not
			resp = usr_srv.response.feedback;
			
			//Print the feedback: it returns '1' if the request is satisfied, otherwise '0'
			std::cout << "feedback: " << resp << std::endl;
		}
		//If the user choice is invalid, it is not sent to the server
		else
		{
			std::cout << "Invalid command, retype" << std::endl; //Advise the user to retype beacuse the choice is invalid
		}		
		ros::spinOnce();	
	}	
	return 0;

}
