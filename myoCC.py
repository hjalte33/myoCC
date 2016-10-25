import time

from modules import ccctrl
driver=ccctrl.dynamixel_mx('/dev/ttyUSB1') 
#on windows write the com port as 'COM9' if its port 9.

#lock all motors so it's not just a spagetti robot
driver.torque_enable(254,True)

#get the robot slowly to it's home position.
homePosition=[2046,2048,2048,2048,2048]
howFarGoal =[]

#how far are we from home?
for n in xrange(0,5):
	howFarGoal.append(driver.get_position(n+1)-homePosition[n])
	print(howFarGoal)

# Send trajectory 	
isHome = False
while isHome != True:
	for x in xrange(0,5):
		setPoint = homePosition[x] + howFarGoal[x]
		if howFarGoal[x] > 0:
			howFarGoal[x] -= 4 
		elif howFarGoal[x] < 0:
			howFarGoal[x] += 4
		driver.set_position(1+x, setPoint)
		
	isHome = True
	#check if we are done
	for x in xrange(0,5):
		if howFarGoal[x] not in range(-5,5):
			isHome = False
			break

driver.torque_enable(254,False)


	
	
	

