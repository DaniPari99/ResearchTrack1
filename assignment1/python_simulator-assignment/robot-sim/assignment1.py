from __future__ import print_function

import time
from sr.robot import *

"""
Assignment 1 python script



	When done, run with:
	$ python run.py solutions/exercise3_solution.py

"""


a_th = 4.0
""" float: Threshold for the control of the orientation"""
s_th = 70
""" float: Threshold to avoid that the robot goes back to a silver token already token"""

g_th = 50
""" float: Threshold to fix the range of golden token visibility"""

d_th = 0.4
""" float: Threshold for the control of the linear distance"""

d_g_th = 0.8
""" float: Minimum safety distance from golden token"""

R = Robot()
""" instance of the class Robot"""

def drive(speed, seconds):
    """
    Function for setting a linear velocity
    
    Args: speed (int): the speed of the wheels
	  seconds (int): the time interval
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

def turn(speed, seconds):
    """
    Function for setting an angular velocity
    
    Args: speed (int): the speed of the wheels
	  seconds (int): the time interval
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = -speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

def find_silver_token():
    """
    Function to find the closest silver token

    Returns:
	dist (float): distance of the closest silver token (-1 if no silver token is detected)
	rot_y (float): angle between the robot and the silver token (-1 if no silver token is detected)
    """
    dist=2
    for token in R.see():
        if token.dist < dist and token.info.marker_type is MARKER_TOKEN_SILVER and -s_th <= token.rot_y <= s_th:
            dist=token.dist
	    rot_y=token.rot_y
    if dist==2:
	return -1, -1
    else:
   	return dist, rot_y

def find_golden_token():
    """
    Function to find the closest golden token

    Returns:
	dist (float): distance of the closest golden token (-1 if no golden token is detected)
	rot_y (float): angle between the robot and the golden token (-1 if no golden token is detected)
    """
    dist=100
    for token in R.see():
        if token.dist < dist and token.info.marker_type is MARKER_TOKEN_GOLD and -g_th <= token.rot_y <= g_th:
            dist=token.dist
	    rot_y =token.rot_y
    if dist==100:
	return -1, -1
    else:
   	return dist, rot_y
   	
def compute_rt_distance():
    """
    Function to compute the distance from right golden token

    Returns:
    dist (float): distance from golden token on the right (-1 if no golden 	token is detected)
	
    """

    dist = 100
    for token in R.see():
    	if token.dist < dist and token.info.marker_type is MARKER_TOKEN_GOLD and 75 <= token.rot_y <= 105: #Check golden token only in the range included from 80 and 100 degrees that corrispond to 														     #the right	
		dist = token.dist
    if dist==100:
    	return -1
    else:	
	return dist
			
def compute_lf_distance():

     """
     Function to compute the distance from golden token on the right

     Returns:
     dist (float): distance from golden token on the left (-1 if no golden 	token is detected)
	
     """

     dist = 100
     for token in R.see():
     	if token.dist < dist and token.info.marker_type is MARKER_TOKEN_GOLD and -105 <= token.rot_y <= -75: #Check golden token only in the range included from -100 and -80 degrees that  													               #corresponds to the right
		dist = token.dist
     if dist==100:
     	return -1
     else:	
	return dist
		
def point_to_silver_token():
    """
    Function to drive the robot for pointing to silver token

    Returns:
	nothing
	
    """
    
    if -a_th <= rot_y_s <= a_th and rot_y_s != -1: #if the robot's orientation is into the range of a threshold so if the robot is aligned with the nearest silver token
    	print("Ah, that'll do")
    	drive(25, 0.5)		  #go straight on
    		  
    elif rot_y_s < -a_th:	  #if the orientation is out of the range from right
    	print("left a bit")
    	turn(-6, 0.5)		  #turn left
    	
    elif rot_y_s > a_th:	  #if the orientation is out of the range from left
    	print("right a bit")
    	turn(6, 0.5)		  #turn right
    	
def avoid_golden_token():
    """
    Function to avoid golden token

    Returns:
    nothing
	
    """
    if (dist_g_rt > dist_g_lf):	#if the distance from golden token on the right is greater than the distance from the golden token on the left
    	print("turn right because there is an obstacle")
    	turn(5, 1)			#turn right
    	
    else:				#if the distance from golden token on the left is greater than the distance from the golden token on the right
      	print("turn left because there is an obstacle")
      	turn(-5, 1)			#turn left		
			

while 1:

    	dist_g, rot_y_g = find_golden_token() #updating the closest golden token's coordinates
    	dist_s, rot_y_s = find_silver_token() #updating the closest silver token's coordinates
    	
  	if dist_s < d_th and dist_s != -1: #if we are close to the token, we try grab it.
        	print("Found it!")
        	
        	if R.grab():		    #if the robot grabs the token
        		print("Gotcha!")
			turn(20, 3) 	    #180 degrees turn
			R.release() 	    #release the token
			turn(-20, 3) 	    #go back of 180 degrees 
			
		else: 			    #if the robot doesn't grab the token
        		print("Aww, I'm not close enough")  
        		
        else:							#if the distance from silver token is greater the a threashold d_th or dist_s==-1
        	if (dist_g < d_g_th and dist_g != -1):		#if the distance from golden token dist_g is less than the minimum safety distance d_g_th and dist_g != -1
        		
    			dist_g_rt = compute_rt_distance()	#updating the distances from left and right
    			dist_g_lf = compute_lf_distance()
    			
    			avoid_golden_token()			#function call to avoid golden token
    			
    		elif (dist_g >= d_g_th or dist_g == -1):
    			print("Go straight on")		#if distance from golden token is greater than or equal to one or dist_g==-1 so golden token is to far
    			drive(25, 0.5)				#go straight on
    			
    			dist_s, rot_y_s = find_silver_token()	#updating silver's coordinates
    			point_to_silver_token()		#function call to point to silver token
    				
    			
    			
    			
    			
    			
    		
    			
    		
    			
		
    		
    
    			
    			
    			
    
