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
import time
import sys
from modules import ccctrlOLD
#from modules import readCsv
from modules import myo as libmyo ; libmyo.init('./myo-sdk-win-0.9.0/bin')

cc=ccctrl.robot('COM9')
#on windows write the com port as 'COM9' if its port 9.

		
#lock all motors so it's not just a coocked spagetti robot
cc.torque_enable(254,True)
cc.set_acceleration(4,5)
cc.set_acceleration(254,10)
time.sleep(0.05)

def wait():
	var = 0
	while var !=5:
		if cc.is_moving(var+1) == True:
			var = 0
		else:
			var += 1
	time.sleep(0.05)		
			

			

hub = libmyo.Hub()

feed = libmyo.device_listener.Feed()
hub.run(3000, feed)
print("\nconnected")

		# Listen to keyboard interrupts and stop the hub in that case.
try:
	print("Waiting for a Myo to connect ...")
	myo = feed.wait_for_single_device(2)
	if not myo:
		print("No Myo connected after 2 seconds.")
	while True:
		cc.mvPTP([20,0,60],velocity = 50)
		wait()
		#try close the gripper
		cc.closeGrip()
		wait()
		#are we holding the ball?
		#if not open the gripper again
		if cc.get_angle(4,True) < 5 : 
			cc.openGrip(amount=20,speed=200)
			
		cc.mvPTP([0,40,30],velocity = 50)
		wait()
		cc.mvLin([0,40,18])
		wait()
		
		
		releaseTime = time.time() + 3
		myo.vibrate('medium')
		while time.time() < releaseTime:
			
			if myo._pose == libmyo.Pose.fingers_spread:
				cc.openGrip(speed = 100)
				myo.vibrate('short')
				myo.vibrate('short')
				myo.vibrate('short')
				wait
				break
			elif myo._pose == libmyo.Pose.fist:
				cc.closeGrip(speed = 100, strength = 350)
				myo.vibrate('short')
				myo.vibrate('short')
				wait	
				break
				
		wait()
		cc.mvLin([0,40,30])
		wait()
		cc.mvPTP([20,0,60],velocity = 50)
		wait()
		cc.mvPTP([0,-40,30],velocity = 50)
		wait()
		cc.mvLin([0,-40,11])
		wait()
		
		releaseTime = time.time() + 3
		myo.vibrate('medium')
		while time.time() < releaseTime:
			
			if myo._pose == libmyo.Pose.fingers_spread:
				cc.openGrip(speed = 100)
				myo.vibrate('short')
				myo.vibrate('short')
				myo.vibrate('short')
				wait
				break
			elif myo._pose == libmyo.Pose.fist:
				cc.closeGrip(speed = 100, strength = 350)
				myo.vibrate('short')
				myo.vibrate('short')
				wait	
				break
		wait()
		cc.mvLin([0,-30,30])
		
except KeyboardInterrupt:
		print("\nQuitting ...")
finally:
	print("Shutting down hub...")
	hub.shutdown()		

	