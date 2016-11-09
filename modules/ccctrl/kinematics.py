import math
from .dynamixel_mx_driver import dynamixel_mx


class robot(dynamixel_mx):
	def __init__(self, port, linkDimentions=[30,30,30]):
		dynamixel_mx.__init__(self,port)
		
		#check if the link dimentions makes sense
		if len(linkDimentions) != 3:
			self.linkDim[30,30,30]
		else:
			self.linkDim[linkDim]
	
	def ikine (self, x, y, z):
		z = z-self.linkDim[0] #this should be changed so it depends on the robot 
		
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
		
	def mvLin(self, point):
		#some code
		pass
		
	def mvPtp(self, point=[],velocity):
		angles = self.ikine(*point) # calculate joint angles
		for n in range(0,3):        # set the goal position of each joint
		super().set_angle(n+1,angles[n],velocity) # uses the parrent method 
		pass
		
	def mvXYZ(direction):
		#some code 
		pass
		
	def mvJoint(jointNum):
		#some code 
		pass 
		
	def mvPath(path=[]):
		#some code (rip writing this)
		pass