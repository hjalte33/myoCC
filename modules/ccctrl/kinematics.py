import math

d0 = 25
d1 = 25
d2 = 25


def ikine (self, x, y, z):
	z = z-d0 #this should be changed so it depends on the robot 
	
	dx = math.sqrt(x^2+y^2+z^2)
	th0 = math.atan2(y,x);
	th1 = (math.pi/2) - math.atan2(z,sqrt(x^2+y^2))-math.acos((d1^2+dx^2-d2^2)/(2*d1*dx));
	th2 = pi - math.acos((d1^2+d2^2-dx^2)/(2*d1*d2));
	
	angles = [th0, th1, th2]
	return angles
	
def fkine (self, th0, th1, th2):
	position = []
	
	position.append(math.cos(th0))
	
		