"""Python tools for working with the Crust Crawler robots.

The new controller controls the motors by first turning off 
the internal PID controller of each of the Dynamixel motors.
 The angle and joint velocities are now solely taken care of
 by the new controller. Instead of sending goal angles to
 the motors, the new controller turns the motors by sending
 torque values only. The controller then reads the angles
 and angular velocities of all the motors as feedback
 allowing the controller to constantly adjust the torques.
 The calculation of desired angles and angular velocities is
 carried out by a trajectory planner that is able to pass
 this data on to the controller. 

"""

from .dynamixel_mx_driver import dynamixel_mx
from ..ivPID import PID


import threading
import profile
import time

class motor(object):
	"""
	This class contains all the nessesary informaiton about the motors.
	Arguments:
		id(int): The motor id of the corrosponding dynamixel motor.
		robot (obj): Object of the dynamixel class containing 
			low-level serial comunication
		PID_ang(list): List of values for the angular PID controller
		PID_vel(list): list of values for the velocity PID controller 
	
	Methods
		set_taget_ang_vel(ang_vel:float): Sets the target velocity
		set_target_ang(ang): Sets the target angle 
		Update(): This reads the current motor values, update the 
			two PID controllers	and writes back a torque to the 
			motor. This method should be called at the frequenzy 
			at which the motors are wanted updated. 
		lock (is_locked:bool): locks or unlocks the motors position
			and turn the internal PID controllers of the dynamixel 
			motors on or off depending on the bool. 
	"""
	def __init__ (self, id, robot, PID_ang = [80,1.7,0.41], PID_vel = [1,0,0]):
		# set up attributes 
		self.id = id
		print (self.id)
		self.robot = robot
		# should be an object in the dynamixel_mx class 
		
		self.joint_limit_ccw = self.robot.get_joint_limits(self.id, direction = 'ccw')
		time.sleep(0.1)
		self.joint_limit_cw = self.robot.get_joint_limits(self.id, direction = 'cw')
		#print ("joint limits - id =  ",self.id," - " , self.joint_limit_ccw, " ", self.joint_limit_cw)
		self.PID_ang = PID(*PID_ang)
		self.PID_vel = PID(*PID_vel)
	
		self.target_ang_vel = 0
		self.target_ang = 0		
		self.target_torque = 0
		self.is_locked = None
		
		self.read_data = robot.read_data
		self.set_torque = robot.set_torque

	def set_target_ang_vel(self, ang_vel):
		self.target_ang_vel = ang_vel
	
	def set_target_ang (self, ang):
		self.target_ang = ang
	
	def update (self):
		#read raw motor data and extract current angle and velocity
		data = self.read_data(self.id,start_address = 36, length=4)
		angle    = ((data[0]+data[1]*256)-2048)*0.0015339807878856412 #((2*math.pi)/4096) # to speed up calculation
		velocity = data[2]+data[3]*256
		if velocity <1024:
			velocity = velocity * 0.11
		else:
			velocity = (velocity-1024)*(-0.11)
		
		#check for joint limit and stop the motor if exeeded
		#else update the two PID controllers
		#lastly write the torqu value 
		if self.joint_limit_ccw <= angle <= self.joint_limit_cw:
			self.robot.stop(self.id)
			time.sleep(0.05)
			print("motor ", self.id, " has stopped due to joint limit")
		else:
			self.PID_ang.SetPoint = self.target_ang
			self.PID_ang.update(angle)
			
			self.PID_vel.SetPoint = self.target_ang_vel + self.PID_ang.output
			self.PID_vel.update(velocity)
			
			target_torque = self.set_torque(self.id, self.PID_vel.output)
			#print("motor id ", self.id, " has error ", self.PID_ang.error," on ang, and ", self.PID_vel.error, "on vel") 

	def lock (self, is_locked):
		#turn on or off the internal PID controllers of the dynamixel motors
		self.is_locked = is_locked
		if is_locked == True:
			self.robot.set_torque_mode(self.id,0)
			self.robot.stop(self.id)
			print("motor ", self.id, " is locked")
		else:
			self.robot.set_torque_mode(self.id,1)
			print("motor ", self.id, " is unlocked")
		
class robot(object):
	"""
	The Robot class is an abstract class that contains one object of the low-level
	Dynamixel_mx class, and many objects from the motor class. The purpose of the 
	Robot class is to run a background process that constantly writes torque values 
	and reads joint angles and joint velocities back. It receives the desired angles
	and joint velocities from the trajectory planner, and makes sure the CrustCrawler
	acts accordingly.
	
	The Robot class contains exactly one object of the Dynamixel_mx class. There can
	only be one instance of the Dynamixel_mx because only one serial communication 
	can be established at a time.
	
	Arguments:
		num_motors (int): numbers of motor objects 
		interval (float): how often the motor objects are updated 
			(this is currently not in use)
		port  (string)  : the port of the serial comunication. 
	
	Methods:
		run (): initialises the worker 
		worker (): A seperat thread that updates the motor objects
		shutdown (): locks motors and shut down the worker thread 
			(not fully working yet.)
		set_target_ang_vel (id:int, ang_vel:float): finds the motor 
			with the specified id and sets the angle velocity
		set_target_ang(id:int, ang:float): finds the motor with the
			specified id and sets the angle 
		set_PID(id:int, PID_ang:list, PID_vel:list): finds the motor
			with the specified id and sets the two PID controllers
	"""
	
	def __init__(self, num_motors = 5, interval=0.2, port = 'COM9'):
		
		#create a serial low level comunication. 
		self.robot = dynamixel_mx(port)
			
		self.num_motors = num_motors	
		self.motors = []
		#make a list of motors to controll.
		for n in range(num_motors):
			self.motors.append(motor(id = n+1, robot = self.robot))
		""" Constructor
		:type interval: int
		:param interval: Check interval, in seconds
		"""
		self.interval = interval

			

	def run(self):
		thread = threading.Thread(target=self.worker, args=())
		thread.daemon = True	# Daemonize thread
		thread.start()
		
	def worker(self):
		for motor in self.motors :
			motor.lock(False)			
			
		while True:
			for motor in self.motors :
				motor.update() 									

	def	shutdown (self):
		#code for shutting down worker thread missing !
		for motor in self.motors:
			motor.lock(True)	
							
	def set_target_ang_vel (self, id, ang_vel):
		for motor in self.motors:
			if motor.id != id:
				continue
			else:
				motor.target_ang_vel = ang_vel
				break					
	
	def set_target_ang (self, id, ang):
		for motor in self.motors:
			if motor.id != id:
				continue
			else:
				motor.target_ang = ang
				break

					
	def set_pid(self, id, PID_ang=[10,0,0], PID_vel=[10,0,0]):
		for motor in self.motors:
			if motor.id != id:
				continue
			else:
				motor.PID_ang.setKp(PID_ang[0]) 
				motor.PID_ang.setKi(PID_ang[1]) 
				motor.PID_ang.setKd(PID_ang[2])  
				motor.PID_vel.setKp(PID_vel[0])
				motor.PID_vel.setKi(PID_vel[1]) 
				motor.PID_vel.setKd(PID_vel[2]) 
				break
		
		
	
	