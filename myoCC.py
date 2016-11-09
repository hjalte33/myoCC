import time

from modules import ccctrl
driver=ccctrl.dynamixel_mx('COM8')
#on windows write the com port as 'COM9' if its port 9.

#lock all motors so it's not just a coocked spagetti robot
driver.torque_enable(254,True)

#get the robot slowly to it's home position.
homePosition=[2048,2048,2048,2048,2048]
poseOne=[2048,1800,3000,1500,700]

#slowly move the moters to the home position 
for n in range(0,5):
	driver.set_position(n+1, homePosition[n], 20)

#check if we are still moving
while True :
	if driver.is_moving(1) == True	:
		continue
	elif driver.is_moving(2) == True:
		continue
	elif driver.is_moving(3) == True:
		continue
	break 
	#the loop breaks only if it passes throug all if statements

#move linearely down
#set the home xyz joint angles
tradj = [0,0,90]

while tradj[2] > 65:
	angles = ccctrl.ikine(*tradj)
	for n in range(0,3):
		driver.set_angle(n+1,angles[n],50)
	tradj[2] -= 0.3
	
	while True :
		if driver.is_moving(1) == True:	
			continue
		elif driver.is_moving(2) == True:
			continue
		elif driver.is_moving(3) == True:
			continue
		break 
		#the loop breaks only if it passes throug all if statements

tradj = [0,0,65]	
		
