#Research Track I - second assignment

------------------------------------------

Parisi Daniele Martino 4670964

The second assignment requires to write the c++ code for moving a robot autonomously inside a circuit.
Here however:

* we can use ROS for controlling the robot
* the robot is endowed with laser scanners
* we want to have a node for the control of the robot and an additional node which interacts with the user to:
   - increase / decrease the speed
   - reset the robot position

This is the second assignment's arena:

(...) 

##How to run the simulation

In order to run the simulation there is a launch file that can be launched with the following command:

```bash
$ rosrun stage_ros stageros $(rospack find assignment2)/world/my_world.world
```

##Robot behaviours

When the simulation is launched the robot is spawned in the pre-built environment. It starts to move around the circuit only when you will have launched in another shell the following command:

```bash
$ rosrun assignment2 robot_controller
```
I you want, in another shell, you can launch the following command:

```bash
$ rosrun assignment2 user_interface
```
 according to increase / decrease the velocity of the robot or to reset its position.
 The 'rosrun' command is composed by 2 parts: one to specify the package (assignment2) in which is contained the executable code of the node (robot_controller, user_interface).

##About software architecture

First of all I implemented the **robot_controller** node that is the code responsible for the robot motion. Once pleased of the robot behaviour alongside the circuit I implemented the **user_interface** node that is capable to get, read and then send to the robot_controller the command chosen by the user:
* 'a' to increase the velocity of the robot
* 'd' to decrease the velocity of the robot
* 'r' to reset the robot position
For the communication I implemented a client-server interface: the robot_controller node is my server and the user_interface is my client. For doing this I made a .srv file which contains a char as request and a bool as response. The request corresponds to the user choice and the response tells the user about the succes ('1') or not ('0') of the operation.
Before writing the final code I did for each node the so called pseudo-code.
###robot controller pseudo-code
```
CONTROLLER CALLBACK:

Subdivide the distances array 'ranges' into 3 parts for the left / right / frontal distances

Calculate the min distance value for each subarray

set a std velocity, for example equal to 2

if the min distance of the frontal array become less than a certain threshold

	if the min distance from right obstacles is greater than min distance from left obstacles
		
		turn right 
		
	else
	
		turn left
		
SERVICE CALLBACK:

if the command pressed by the user corresponds to 'a'

	increase the velocity
	
else if the command pressed by the user corresponds to 'd'

	decrease the velocity
	
else if the command pressed by the user corresponds to 'r'

	reset robot position
	
```
###user interface pseudo-code
```
while true

	print the menu to show the user all the possible choices
	
	put in a variable the choice by the user
	
	if the command pressed by the user is valid
	
		put the char pressed in the request variable
		
		call the service
		
		Put into 'resp' variable the response by the server
		
		Print the feedback (response)
		
	else (if the command pressed is not valid)
	
		print: "invalid command"
		
	        send nothing to the server
```
After that I started to write the code. In the robot_controller node, according to avoid the robot have a negative velocity and so to avoid the robot go on reverse I set a min velocity equal to zero. In a  similar way, to avoid the robot go too fast and so to avoid the crash of the robot I set a max velocity that can be modified by the developer according to his needs.

For calculate the min distance from the right / left side I implemented a method called **calculate_min_side_dist** that returns the min element of an array given as entry parameter.
In the same way, but with a different array size I implemented a method called **calculate_min_front_dist**. 

##System limitations and possible improvements

###Limitations:

- The more you increase the velocity, the more increases the probability of crash
- There are 3 nodes (the world node included) and for launching them I have to use the command 'rosrun' three times in three different shels

###Possible improvements:

- I could improve my code according to make the robot goes at high-speed without crashing
- I could use roslaunch tool for launching all my nodes with one command



