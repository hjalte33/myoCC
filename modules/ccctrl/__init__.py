"""Python tools for working with the Crust Crawler robots.

This module has some tools to start using the Crust Crawler pro-Series arms
found in the Automation and Control labs at Aalborg University.

These tools are made to be used with a RS485 adapter or a UartSBee connected
to a RS485 driver in one end and the pc in the other (not with an XBee).

The DTR pin on the UartSBee works as the driver/receiver enable, so be sure
to hook this up too when you are hooking rx, tx, vcc, and gnd up.

"""

#from .dynamixel_mx_driver import dynamixel_mx
#from .kinematics import robot
from .dynamixel_mx_driver import dynamixel_mx
from ..ivPID import PID


import threading
import profile
import time

class motor(object):
	def __init__ (self, id, robot, PID_ang = [80,1.7,0.41], PID_vel = [1,0,0]):
		self.id = id
		print (self.id)
		self.robot = robot
		# should be an object in the dynamixel_mx class 
		
		self.joint_limit_ccw = self.robot.get_joint_limits(self.id, direction = 'ccw')
		time.sleep(0.1)
		self.joint_limit_cw = self.robot.get_joint_limits(self.id, direction = 'cw')
		print ("joint limits - id =  ",self.id," - " , self.joint_limit_ccw, " ", self.joint_limit_cw)
		self.PID_ang = PID(*PID_ang)
		self.PID_vel = PID(*PID_vel)
		#self._ang = 1
		#self._ang_vel = 0
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
		data = self.read_data(self.id,start_address = 36, length=4)
		angle    = ((data[0]+data[1]*256)-2048)*0.0015339807878856412 #((2*math.pi)/4096)
		velocity = data[2]+data[3]*256
		if velocity <1024:
			velocity = velocity * 0.11
		else:
			velocity = (velocity-1024)*(-0.11)
			
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
			print("motor id ", self.id, " has error ", self.PID_ang.error," on ang, and ", self.PID_vel.error, "on vel") 

	def lock (self, is_locked):
		self.is_locked = is_locked
		if is_locked == True:
			self.robot.set_torque_mode(self.id,0)
			self.robot.stop(self.id)
			print("motor ", self.id, " is locked")
		else:
			self.robot.set_torque_mode(self.id,1)
			print("motor ", self.id, " is unlocked")
		
class robot(object):

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
			time.sleep(0.1)			
			
		while True:
			for motor in self.motors :
				motor.update() 								

	def	shutdown (self):
		for motor in self.motors:
			motor.lock(True)	
							
	def set_ang_vel (self, id, ang_vel):
		pass					
	
					
	def set_pid(self, PID_ang=[10,0,0], PID_vel=[10,0,0]):
		pass
		
	
	