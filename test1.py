"""
MIT License

Copyright (c) 2016 hjalte33

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

"""
This code is made to test the second version of the CrustCrawler controllre 
the controller will go to a surtain set point and stay there even if the 
forces ond the robot chance. At the moment the sample time is too slow for 
this to work proporly so the robot will make some overshoot 

When the robot is started the terminal window listens for input. Type thre
numbres seperated by a space. The numbers are angles in radians 

There is currently a bug in python that makes it impossible to terminate
the program with a keyboard interrupt (Ctrl + c) while listening for raw keyboard
input. The test program can be terminated if there is given any other input than 
a list with 3 numbers. 
"""


from modules import ccctrl
from math import pi
from time import sleep
import sys


try:
	#create a robot obejct with 3 motors
	myCC = ccctrl.robot(num_motors = 3)
	#set the PID controllers for each motors
	#the values are found by trial and error
	myCC.set_pid(1,[120,2,0],[1,0,0])
	myCC.set_pid(2,[800,50,5],[1,0,0])
	myCC.set_pid(3,[300,50,5],[1,0,0])
	#set goal angles for each of the motors
	myCC.set_target_ang(1,0)
	myCC.set_target_ang(2, pi/6)
	myCC.set_target_ang(3, pi/3)
	#start the robot controller 
	#beware of sudden movement !!!!!!
	myCC.run()
	sleep(1)
	
	while True:
		#this loop wil listen for imput.
		#if the imput is a list of length 3 
		#the valuese are interpreted as 
		#angles in radans. any other input 
		#will terminate the controller and
		#lock the motors.
		string_input = input('Write 3 new set angles\n')
		newGoal = string_input.split()
		newGoal = [float(a) for a in newGoal]
		if len(newGoal) == 3:
			myCC.set_target_ang(1, newGoal[0])
			myCC.set_target_ang(2, newGoal[1])
			myCC.set_target_ang(3, newGoal[2])
		else: 
			print('That\'s not a list')
			break
		sleep(0.1)

except KeyboardInterrupt:
	print("\nquitting")
finally:
	myCC.shutdown()
