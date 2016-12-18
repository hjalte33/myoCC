"""
The trajectory planner and the controller are separate entities, which
makes future expansions to system possible. One can imagine having
many different trajectory planners instead of just the PTP and Linear,
e.g. circular and other much more complex curves and paths. For this
project there was, as a start, only focused on the execution of linear
movements.

This library contains a trajectory planner but its not yet fully
implemented into the controller. The Trajectory class needs 
implementation of the dynamixel_mx class so it can read the start 
joint angles. It also still needs to pass the calculated data to 
the robot. Executing the mvLin method will calculate trajectory
point between the Pseudo_read_angles and the [x y z] goal  
"""
import math
import time

class Trajectory(object):
	def __init__(self):
		self.linkDim = [30,30,30]
		self.Pseudo_read_angles = [0, 0.1, 0]
	
	def ikine (self, x, y, z):
		
		z -= self.linkDim[0] #set the right world coordinate system
		
		angles = []
			
		#some initial calculation
		dx = math.sqrt((x**2)+(y**2)+(z**2))
		
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
		
	def mvLin(self, goal, cartesian_Velocity, sampleSpeed): #Units: cm and m/s
		# Get the goal point in cartesian space
		# goal should be a list of [x, y, z]
		self.goal = goal

		# Set the time between samples
		self.sampleSpeed = sampleSpeed

		# Store the desired velocity as a member variable
		self.velocity = cartesian_Velocity # m/s

		# Get the start point in cartesian space as a list
		start_point = self.fkine(*self.Pseudo_read_angles) # Insert: *robot.get_angles().  #Output is in cm

		# Subtract the start length from the goal length: goal - start = 
		vector = [i-j for i, j in zip(self.goal, start_point)] # Unit: cm

		# Calculate the vector length between the start point and end point
		length = math.sqrt((vector[0]**2)+(vector[1]**2)+(vector[2]**2)) #Unit: cm

		# Calculate the time available for the motion
		time_available = ((length/100) / self.velocity) # Divide length by 100 to match units (m/(m/s))

		#  Get servo angles in radians at start position as a list
		tradj0 = self.ikine(*start_point)
		
		# Register time of movement start: time.time() returns time in seconds since epoch as a floating point number 
		startTime=time.time()
		tempStartTime = time.time()

		""" The while loop calculates new points along the vector of which we want to move linearly based on the fraction that has past of the total time available.
		When timeFrac is equal to 1 the goal point will be calculated whereafter the loop will be exited """
		while time.time()-startTime <= time_available:

			# This loop controls the rate of samples. Execution of the code will only continue once samplespeed has passed
			while time.time() < tempStartTime + self.sampleSpeed:
				# Do Nothing
				pass

			# A new loop specific start time is assigned to make sure that we wait before continuing the calculation of the next point 
			tempStartTime = time.time()
			
			# Calculate how much time of the total available time that has past as a fraction
			timeFrac = (time.time()-startTime)/(time_available)
			# Calculate the new point on the vector to follow. 
			tradj1 = [i + (j * timeFrac) for i, j in zip(start_point, vector)]
			
			#pass the tradj to the ikine to get angles of next point. 
			tradj1 = self.ikine(*tradj1)

			# The travel distance to the new point for each servo is calculated
			angDif = [i-j for i,j in zip(tradj1,tradj0)]

			# The servos should travel the distance of angdif in the time span of samplespeed. Thus, the goal velocity to the next point can be calculated.
			velocity = [i/sampleSpeed for i in angDif]

			# The next angles now becomes current angles prior to next calculation 
			tradj0 = tradj1
			
			print("\nangles = ", tradj0, "\nvelocity = ", velocity,) 

		tradj1 = self.ikine(*self.goal)
		velocity = [0.0, 0.0, 0.0]
		print("\nangles = ", tradj1, "\nvelocity = ", velocity) 
		
		

	def mvPTP(self, goal=[0,0,0],velocity=10): 
		"""point should be in the cartesian space """
		angles = self.ikine(*goal) # calculate joint angles
		for n in range(0,3):        # set the goal position of each joint
			super().set_angle(n+1,angles[n],velocity) # uses the parrent method 
		pass
		
		
	def closeGrip(self, speed=10, strength=400 , amount=None):
		if amount != None:
			super().set_angle(4, super().get_angle(4,True) - amount, speed,True)
			super().set_angle(5, super().get_angle(5,True) + amount, speed,True)
		else :
			super().set_angle(4, -4, speed,True)
			super().set_angle(5, 4, speed,True)
		time.sleep(0.05)
		#see if we exceed the strength
		while super().get_torque(4) < strength and super().get_torque(5) < 1024 + strength and super().is_moving(4) == True : 
			#the while loop might check the three conditions too quickly 
			#so the code migt have to be slowed down.
			pass
		#	
		time.sleep(0.1)
		super().stop(4)
		super().stop(5)
	
	def openGrip(self, speed=10, amount=30):
		if amount > self.get_angle(4,True) and amount > self.get_angle(5,True):
			super().set_angle(4,   amount, speed,True)
			super().set_angle(5, - amount, speed,True)


