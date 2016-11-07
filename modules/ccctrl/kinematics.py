import math

d0 = 30 #from ground and up
d1 = 30 #first link 
d2 = 30
a1 = d1
a2 = d2



def ikine (x, y, z):
	z = z-d0 #this should be changed so it depends on the robot 
	
	angles = []
	tmp = ((x**2)+(y**2)+(z**2))
	
	dx = math.sqrt(tmp)
	
	
	angles.append(math.atan2(y,x))
	
	angles.append((math.pi/2) - math.atan2(z,math.sqrt(x**2+y**2)) - math.acos(((d1**2)+(dx**2)-(d2**2))/(2*d1*dx)))
	
	angles.append(math.pi - math.acos(((d1**2)+(d2**2)-(dx**2))/(2*d1*d2)))
	return angles

def fkine ( th0, th1, th2):
	position = []
	
	position.append(a2*(math.cos(th0)*math.sin(th2)*math.sin(th1 + math.pi/2) - math.cos(th0)*math.cos(th2)*math.cos(th1 + math.pi/2)) - a1*math.cos(th0)*math.cos(th1 + math.pi/2))
	
	position.append( - a2*(math.cos(th2)*math.cos(th1 + math.pi/2)*math.sin(th0) - math.sin(th0)*math.sin(th2)*math.sin(th1 + math.pi/2)) - a1*math.cos(th1 + math.pi/2)*math.sin(th0))
	
	position.append(d0 + a2*(math.cos(th2)*math.sin(th1 + math.pi/2) + math.cos(th1 + math.pi/2)*math.sin(th2)) + a1*math.sin(th1 + math.pi/2))
	
	return position	