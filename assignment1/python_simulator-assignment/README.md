#Research Track I - first assignment

------------------------------------------

Parisi Daniele Martino 4670964

This first assignment requires to write a Python script for achieving this robot's behaviour:

* Constantly drive the robot around the circuit in the counter-clockwise direction
* Avoid the golden token
* When the robot is closer to a silver token, it should grab it, and move it behind itself

This is the first assignment's arena:
(...) 
![arena](https://user-images.githubusercontent.com/62515616/140643120-618d7a83-29bf-4a80-9c9d-f05e76aa9e0a.png)

##How to run the simulation

In order to run the simulation there is a launch file that can be launched with the following command:

```bash
$ python run.py assignment1.py
```

##Robot behaviours

When the simulation is launched the robot is spawned in the pre-built environment and it starts suddenly to move around the circuit.
Once the robot is closer to a silver token, it should grab it turn of 180 degrees, release it and go back for its way; this behaviour is respected in most cases, but in a few cases not. For example sometimes the robot is not able to grab a token and especially this token is always the final token. In the same way, while the robot is looking for silver tokens, it should avoid the wall formed by golden tokens.
In the end, I can be pleased with the final behaviour despite of the bug that explained previously.

##About software architecture

Before beginning to write the code I wrote the pseudo-code that is the following:

```
While true

	update the coordinates
	
	if the distance from silver token is less than or equal to a certain threshold grabbing distance
	
		print "found it"
		
		if Robot grab the token
		
			print "gotcha"
			
			move the token behind itself
			
		else 
		
			print "Aww, I'm not close enough"
			
	else
	
		if distance from golden token is less than a threshold distance
		
			if the distance from right golden wall is greater than the distance from left golden wall 
			
				print "turn right because there is an obstacle"
				
				turn right a bit
				
			else
				print "turn left because there is an obstacle"
				
				turn left a bit
				
		else
			
			if the robot is well aligned with silver token 
				
				print "that'll do!"
				
				drive a bit
				
			else if the robot is disaligned from the left
			
				print "left a bit"
				
				turn left a bit
				
			else if the robot is disaligned from the right
			
				print "right a bit"
				
				turn right a bit
```


After that, for the implementation of this pseudo-code I decided to write the following methods:

* **drive**: it is the function that permit the robot to go straight on. It has got 2 paramethers: linear velocity and time

* **turn**: it is the function that permit the robot to turn. It has got 2 paramethers: angular volicity and time

* **find_silver_token**: it is the function that computes the coordinates (dist_s: distance from silver token; rot_y_s: angle between the robot and the silver token) of the closest silver token contained in a certain range. 

* **find_golden_token**: it is the function that computes the coordinates (dist_g: distance from silver token; rot_y_g: angle between the robot and the golden token) of the closest golden token contained in a certain range. In my code I choose g_th = 50 as the angular threshold for avoiding golden token

* **compute_rt_distance**: it is the function that computes the distance from right side.

* **compute_lf_distance**: it is the function that computes the distance from left side

* **point_to_silver_token**: it is the function that make the robot pointing to the closest silver token

* **avoid_golden_token**: it is the function that make the robot avoiding golden tokens. How it works? The basic idea is to compute the distance from left and right tokens and then make the robot turn left or right according to the biggest distance

The basic idea of my solution expressed by pseudo-code and then written in python language is traveling all the circuit avoiding golden tokens and then when a silver token is closer enough, pointing to it and then release it. 
The basic idea for avoiding golden tokens is to drive the robot where there is the greatest distance from golden wall. Everytime we are too much closer with a golden token, we must turn where the distance is greater. As it is showed by the following figure:

(...)
![alt text](http://url/to/Curve.png)

Regarding the thresholds I used global variables and after some attempts I finally set up them as following:

* **a_th = 4.0** is the threshold that controls the alignement. rot_y_s must be in absolute value less than or equal to a_th. I used this condition in the function 'point_to_silver_token'

* **s_th = 70** is the angular threshold for avoiding that the robot can see and point to previous tokens. token.rot_y must be in absolute value less than or equal to s_th. I used this condition in the function 'find_silver_token'

* **g_th = 50** is the angular threshold to fix the range of golden token visibility. token.rot_y must be in absolute value less than or equal to g_th. I used this condition in the function 'find_golden_token'

* **d_th = 0.4** is the threshold for the control of the linear distance: When dist_s is less than d_th the robot can grab silver tokens. I used this condition in the first 'if' of while true loop

* **d_g_th = 0.8** ist the Minimum safety distance from golden token. Initially I chose for this variable the value '1', but with this value it might occur the following extreme situation:

(...) figure of the situation

In this extremely situation the robot tend to go back in the clockwise direction.
reducing the value of **d_g_th** from 1 until 0.8 I noticed that this extreme situation is minimized.

##System limitations and possible improvements

###Limitations:

- Sometimes the robot goes back in clockwise direction as I mentioned previously. This obviously is a wrong behaviour but it happens because of a limit of the system due to the fact that the robot hasn't got any internal odometry

- Sometimes the robot is not able to grab the last silver token. This is also a wrong behaviour maybe due to the simulator

- The algorithm is not well optimized, in fact the robot does not accomplish the fastest path, it does not take a linear way, but it goes a little zig zag and when there is a curve it notice that there is only at the end

###Possible improvements:

- The algorithm could be better optimized as I wrote previously
























