import time

from modules import ccctrl
driver=ccctrl.dynamixel_mx('/dev/ttyUSB0') 
#on windows write the com port as 'COM9' if its port 9.

#lock all motors so it's not just a spagetti robot
driver.torque_enable(254)

#get the robot slowly to it's home position.
homePosition=[2048,1024,2048,2048,2048]
howFarGoal =[]

#how far are we from home?
for n in xrange(0,4):
	howFarGoal.append(driver.get_position(n+1)-homePosition[n])

# Send trajectory 	
isHome = False
while isHome != True:
	for x in xrange(0,4):
		setPoint = homePosition[x] + howFarGoal[x]
		if howFarGoal[x] > 0:
			howFarGoal -= 1 
		elif howFarGoal < 0:
			howFarGoal += 1
		driver.set_position(x+1, setPoint)
		
	isHome = True
	#check if we are done
	for x in xrange(0,4):
		if howFarGoal[x] != 0:
			isHome = False
			break
	time.sleep(5)	
	
	
	

