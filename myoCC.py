from modules import ccctrl
driver=ccctrl.dynamixel_mx(ttyUSB0)

#lock all motors so it's not just spagetti
driver.torque_enable(254)

#get the robot slowly to it's home position.
homePosition=[2048,2048,2048,2048,2048]
bool ishome = False
howFarGoal =[]
for n in xrange(0,4):
	howFarGoal.append(n)=driver.get_position(n+1)-homePosition[n]
	pass
	
while ishome != True:
	for x in xrange(0,4):
		
		pass
	pass

