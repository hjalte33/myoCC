import math

d0 = 30
d1 = 30
d2 = 30


def ikine (x, y, z):
	z = z-d0 #this should be changed so it depends on the robot 
	
	angles = []
	tmp = ((x**2)+(y**2)+(z**2))
	
	dx = math.sqrt(tmp)
	
	
	angles.append(math.atan2(y,x))
	
	angles.append((math.pi/-2) + math.atan2(z,math.sqrt(x**2+y**2)) + math.acos(((d1**2)+(dx**2)-(d2**2))/(2*d1*dx)))
	
	angles.append(math.pi - math.acos(((d1**2)+(d2**2)-(dx**2))/(2*d1*d2)))
	print(angles)
	return angles

	
	
def fkine (self, th0, th1, th2):
	position = []
	
	position.append(math.cos(th0))
	
	return position	