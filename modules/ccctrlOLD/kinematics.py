"""
MIT License

Copyright (c) 2016 hjalte33

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

"""
This high-level library contains a robot class with the nessesary tools to create
PTP and Lin movements of the crustcrawler. it also provides tools to controll the gripper. 
calling a movement before the current running movement has finished might break the code 
"""

import math
import time
#import the low-level dynamixel library 
from .dynamixel_mx_driver import dynamixel_mx
#import the PID library 
from ..ivPID import PID


class robot(dynamixel_mx):
	def __init__(self, port, linkDimentions=[30,30,30],baudrate=1000000, timeout=0.02):
		dynamixel_mx.__init__(self, port, baudrate, timeout)
		
		#check if the link dimentions makes sense
		if len(linkDimentions) != 3:
			self.linkDim = [30,30,30]
		else:
			self.linkDim = linkDimentions
	
	def ikine (self, x, y, z):
		#set the right world coordinate system
		z -= self.linkDim[0] 
		#prepare list of angles
		angles = []
			
		#some initial calculation
		dx = math.sqrt((x**2)+(y**2)+(z**2))
		
		angles.append(math.atan2(y,x))
		
		angles.append((math.pi/2) - math.atan2(z,math.sqrt(x**2+y**2)) - math.acos(((self.linkDim[1]**2)+(dx**2)-(self.linkDim[2]**2))/(2*self.linkDim[1]*dx)))
		
		angles.append(math.pi - math.acos(((self.linkDim[1]**2)+(self.linkDim[2]**2)-(dx**2))/(2*self.linkDim[1]*self.linkDim[2])))
		return angles

	def fkine (self, th0, th1, th2):
		#prepare list of position data
		position = []
		
		position.append(self.linkDim[2]*(math.cos(th0)*math.sin(th2)*math.sin(th1 + math.pi/2) - math.cos(th0)*math.cos(th2)*math.cos(th1 + math.pi/2)) - self.linkDim[1]*math.cos(th0)*math.cos(th1 + math.pi/2))
		
		position.append( - self.linkDim[2]*(math.cos(th2)*math.cos(th1 + math.pi/2)*math.sin(th0) - math.sin(th0)*math.sin(th2)*math.sin(th1 + math.pi/2)) - self.linkDim[1]*math.cos(th1 + math.pi/2)*math.sin(th0))
		
		position.append(self.linkDim[0] + self.linkDim[2]*(math.cos(th2)*math.sin(th1 + math.pi/2) + math.cos(th1 + math.pi/2)*math.sin(th2)) + self.linkDim[1]*math.sin(th1 + math.pi/2))
		
		return position	

		
	def mvLin(self, goal = [0,0,0], velocity=0.1):
		""" Velocity is in m/s """
		
		#first calculate where we are.
		startPoint = self.fkine(*super().get_angles())
		
		#calculate vector to follow in cartesian space
		#this piece of code is basicaly vector = goal - startPoint
		vector = [i - j for i, j in zip(goal, startPoint)] 
		
		#calculate length of the vector. this is basicaly sqrt(x^2+y^2+z^2)
		length = math.sqrt(((goal[0]-startPoint[0])**2)+((goal[1]-startPoint[1])**2)+((goal[2]-startPoint[2])**2))
		
		#set up a list of  pid controllers for each motor speed
		linPID = [PID(800,100,10), PID(800,100,10), PID(800,100,10)]
		
		for item in linPID:
			item.output = 1
		
		#what time is it when we start
		startTime = time.time()
		
		#initial speeds. Never let these be 0 
		speeds = [1,1,1]
		
		#from 0 to 1 how far on the trajectory shold we be
		timeFrac = 0
		
		while timeFrac <= 1  : 
			"""
			run as long as we still have time
			calculate how much time we have left to do the movement.
			timeFrac is between 0 and 1 
			length in cm ; velocity in m/s
			"""
			timeFrac = (time.time()-startTime)/((length/100)/velocity)
			
			#calculate trajectory
			#traj = startPoint + (timeFrac * vector) 
			#it gives the point where whe should be at the given time. 
			#newVector = [q * timeFrac for q in vector]
			tradj = [i + (j * timeFrac) for i, j in zip(startPoint,vector)]
			
			#pass the tradj to the ikine to get angles. 
			tradjAng = self.ikine(*tradj)
			
			
			"""the angles right now .
			now controll the motor speeds so we will 
			finish in time. This is done by having a
			PID controller always checking if we the
			motors are behind the position it should
			have. 			
			"""
			
			#get PID output and put into speeds
			#we add one to ensure speed stays above 1
			#move moters to tradjectory angles 
			for x, item in enumerate(linPID):
				super().set_angle(x+1,tradjAng[x],int(math.fabs(item.output))+1)
			
			#the angles right now and calculate error and update PID 
			for index, item in enumerate(linPID):
				item.update(math.fabs(super().get_angle(index+1)- tradjAng[index]))
		
	def mvPTP(self, goal=[0,0,0],velocity=10): 
		"""point should be in the cartesian space """
		angles = self.ikine(*goal) # calculate joint angles
		for n in range(0,3):        # set the goal position of each joint
			super().set_angle(n+1,angles[n],velocity) # uses the parrent method 
		pass
		
	def closeGrip(self, speed=10, strength=400 , amount=None):
		#defaulst close position 
		if amount != None:
			super().set_angle(4, super().get_angle(4,True) - amount, speed,True)
			super().set_angle(5, super().get_angle(5,True) + amount, speed,True)
		#close the abount given
		else :
			super().set_angle(4, -4, speed,True)
			super().set_angle(5, 4, speed,True)
		time.sleep(0.05)
		#see if we exceed the strength
		while super().get_torque(4) < strength and super().get_torque(5) < 1024 + strength and super().is_moving(4) == True : 
			#the while loop might check the three conditions too quickly 
			#so the code migt have to be slowed down.
			pass
		#give the motors time to respond	
		time.sleep(0.05)
		super().stop(4)
		super().stop(5)
	
	def openGrip(self, speed=10, amount=30):
		if amount > self.get_angle(4,True) and amount > self.get_angle(5,True):
			super().set_angle(4,   amount, speed,True)
			super().set_angle(5, - amount, speed,True)
