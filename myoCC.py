import time

from modules import ccctrl
driver=ccctrl.dynamixel_mx('/dev/ttyUSB3')
#on windows write the com port as 'COM9' if its port 9.

#lock all motors so it's not just a spagetti robot
driver.torque_enable(254,True)

#get the robot slowly to it's home position.
homePosition=[2048,2048,2048,2048,2048]
poseOne=[2048,1024,3000,1500,700]

#slowly move the moters to the home position 
for n in xrange(0,5):
	driver.set_position(n+1, homePosition[n], 10)

#check if we are at the wanted pose


