# Research Track I - final assignment

------------------------------------------

Parisi Daniele Martino 4670964

The final assignment requires to write the code for moving a robot in the environment initially unknown. Three modalities for moving the robot are required:
* the robot has to reach a goal chosen by the user
* the robot is driven by the user through the keyboard
* the robot is driven by the user with the assistence of a collision avoidence architecture

This is the final assignment's environment visualized with Gazebo: 
![MicrosoftTeams-image(4)](https://user-images.githubusercontent.com/62515616/152884108-da0032b9-8a4f-4565-8503-6839d839a8f2.png)

Initially the robot does not have ha map of the environment, but it can build it through the laser scanner and thanks to the **gmapping** package. The final map built by the robot is visualized in Rviz as follows:

![MicrosoftTeams-image(3)](https://user-images.githubusercontent.com/62515616/152884728-46a1fe86-923b-4e8d-9b25-a15c8540d695.png)


## How to run the simulation
In order to launch different nodes with only a command I implemented a launch file called **final.launch** that is the following:
```bash
$ <?xml version="1.0"?>

<launch>
    <include file="$(find final_assignment)/launch/simulation_gmapping.launch" />
    <include file="$(find final_assignment)/launch/move_base.launch" />
    <node pkg="final_assignment" type="user_interface" name="user_interface" output="screen" required="true" launch-prefix="xterm -e"/>
    <node pkg="final_assignment" type="server" name="server" output="screen" required="true" launch-prefix="xterm -e"/>
     <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="teleop" output="screen" launch-prefix="xterm -e" respawn="true">
    <remap from="cmd_vel" to="new_cmd_vel"/>
   </node>
</launch>
```
it contains 2 nested launch file: simulation_gmapping.launch and move_base.launch which respectively launch the simulation environment and provide some tools for moving the robot in the environment 
it can be launched with the command:

```bash
$ roslaunch final_assignment final.launch
```
## Project structure








