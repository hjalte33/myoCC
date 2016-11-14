import math
import time
from .dynamixel_mx_driver import dynamixel_mx
from ..ivPID import PID


class robot(dynamixel_mx):
	def __init__(self, port, linkDimentions=[30,30,30]):
		dynamixel_mx.__init__(self,port)
		
		#check if the link dimentions makes sense
		if len(linkDimentions) != 3:
			self.linkDim = [30,30,30]
		else:
			self.linkDim = linkDimentions
	
	def getAngles(self):
		ang = []
		for n in range(3) :
			ang.append(super().get_angle(n+1))
		
		return ang
	
	def ikine (self, x, y, z):
		z -= self.linkDim[0] #set the right world coordinate system
		
		angles = []
		tmp = ((x**2)+(y**2)+(z**2))
		
		#some initial calculation
		dx = math.sqrt(tmp)
		
		angles.append(math.atan2(y,x))
		
		angles.append((math.pi/2) - math.atan2(z,math.sqrt(x**2+y**2)) - math.acos(((self.linkDim[1]**2)+(dx**2)-(self.linkDim[2]**2))/(2*self.linkDim[1]*dx)))
		
		angles.append(math.pi - math.acos(((self.linkDim[1]**2)+(self.linkDim[2]**2)-(dx**2))/(2*self.linkDim[1]*self.linkDim[2])))
		return angles

	def fkine (self, th0, th1, th2):
		position = []
		
		position.append(self.linkDim[2]*(math.cos(th0)*math.sin(th2)*math.sin(th1 + math.pi/2) - math.cos(th0)*math.cos(th2)*math.cos(th1 + math.pi/2)) - self.linkDim[1]*math.cos(th0)*math.cos(th1 + math.pi/2))
		
		position.append( - self.linkDim[2]*(math.cos(th2)*math.cos(th1 + math.pi/2)*math.sin(th0) - math.sin(th0)*math.sin(th2)*math.sin(th1 + math.pi/2)) - self.linkDim[1]*math.cos(th1 + math.pi/2)*math.sin(th0))
		
		position.append(self.linkDim[0] + self.linkDim[2]*(math.cos(th2)*math.sin(th1 + math.pi/2) + math.cos(th1 + math.pi/2)*math.sin(th2)) + self.linkDim[1]*math.sin(th1 + math.pi/2))
		
		return position	
	
	def mvLinB(self, goal=[0,0,0], velocity=0.01):
		"""
		velocity is in m/s
		"""
		
		#get the goal angles
		goalAngles = self.ikine(*goal)
		
		#first calculate where we are.
		startPoint = self.fkine(*self.getAngles())
		
		#calculate vector to follow
		#this piece of code is basicaly vector = point - nowPoints
		vector = [i - j for i, j in zip(goal, startPoint)] 
		
		length = math.sqrt((math.fabs(goal[0]-startPoint[0])**2)+(math.fabs(goal[1]-startPoint[1])**2)+(math.fabs(goal[2]-startPoint[2])**2))
		
		linPID = [PID(1000,200,10), PID(1000,200,10), PID(1000,200,10)]
		
		for item in linPID:
			item.sample_time = 0.05
		
		#activate the motors by setting the goal 
		#move super slow in the beginning
		startTime = time.time()
		
		for n in range(0,3):
			super().set_angle(n+1,goalAngles[n],1)
			pass
		
		#now quickly set speeds so the motors will move the arm linearyly
		#messuer how far away we are from the tradjectory and set motor speeds accordingly 
		#uses the pid controller 	
		speeds = [1,1,1]
		while self.getAngles() != goalAngles:
			
			timeFrac = (time.time()-startTime)/(length/velocity)
			#calculate the tradjectory
			#this below does the following 
			#traj = startPoint + n * vector 
			tmpVector = [q * timeFrac for q in vector]
			tradj = [i + j for i, j in zip(startPoint,tmpVector)]
			tradjAng = self.ikine(*tradj)
			
			nowAngles = self.getAngles()
			
			for index, item in enumerate(linPID):
				item.SetPoint = tradjAng[index]
			
			for index, item in enumerate(linPID):
				item.update(nowAngles[index])
			
			for index, item in enumerate(linPID):
				speeds[index] = item.output
			
			
			for n in range(3):
				if speeds[n] < 1:
					speeds[n] = math.fabs(speeds[n])+1
			
			

			for n in range(3):
				data = []
				data.append(int(speeds[n]) & 0xFF)
				data.append((int(speeds[n]) & 0xFF00) >> 8)
				super().write_data(n+1,32,data)
				print(data)
					
				
			
	
	def mvLin(self, point, velocity=10):
		
		#first get the angles we have now.
		nowAngles = []
		for n in range(0,3):
			nowAngles.append(super().get_angle(n+1))
			pass
	
		#then calculate where we are.
		nowPoint = self.fkine(*nowAngles)
		
		#calculate vector to follow
		#this piece of code is basicaly vector = point - nowPoints
		vector = [i - j for i, j in zip(point, nowPoint)] 
	
		#calculate and follow the trajectory points
			
		for n in range(0,100,2):
			
			#this below does the following 
			#traj = nowPoint + n * vector 
			newVector = [q * (n/float(100)) for q in vector]
			tradj = [i + j for i, j in zip(nowPoint,newVector)]
			
			#pass the tradj to the ikine to get angles. 
			tradjAng = self.ikine(*tradj)
		
			for x in range (0,3):
				super().set_angle(x+1,tradjAng[x],velocity)
			
			
			#check if we are still moving
			while True :
				time.sleep(0.01)
				if super().is_moving(1) == True	:
					continue
				elif super().is_moving(2) == True:
					continue
				elif super().is_moving(3) == True:
					continue
				break 
				#the loop breaks only if it passes throug all if statements
			
		
	def mvPTP(self, point,velocity=10): 
		"""point should be in the cartesian space """
		angles = self.ikine(*point) # calculate joint angles
		for n in range(0,3):        # set the goal position of each joint
			super().set_angle(n+1,angles[n],velocity) # uses the parrent method 
		pass
		
	def mvXYZ(self, direction, distance):
		#some code 
		pass
		
	def mvJoint(jointNum):
		#some code 
		pass 
		
	def mvPath(path=[]):
		#some code (rip writing this)
		pass