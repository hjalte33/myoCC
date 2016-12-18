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
import os
import sys
from modules import ccctrl
#from modules import readCsv
from modules import myo as libmyo ; libmyo.init('./myo-sdk-win-0.9.0/bin')
import tkinter as tk

#some callbacks 
def callback():
	runProgram()
	root.after(10)
def callback2():
	sys.exit()

root=tk.Tk()
e = tk.Entry(root)
e.pack()

e.focus_set()


#some gui setup 
def pGet():
    e.configure(state='disabled')
    bGet.configure(state='disabled')
    
    try:
         new = float(e.get())
         return (new)
    except: 
         e.configure(state='normal')
         bGet.configure(state='normal')
         print("fgt")

bGet = tk.Button(root, text="Set value", width=10, command=pGet)
bGet.pack()
root.title("User interface")
root.geometry("400x400")
root.after(10)
b = tk.Button(root, text="Start", fg="green",command=callback)
b.pack()

quit = tk.Button(root, text="Stop", fg="red", command=callback2)
quit.pack()



#on windows write the com port as 'COM9' if its port 9.
#use the old ccctl library
cc=ccctrlOLD.robot('COM9')

#lock all motors so it's not just a coocked spagetti robot
cc.torque_enable(254,True)
cc.set_acceleration(4,5)
cc.set_acceleration(254,10)
time.sleep(0.05)

#defines a wait loop that waits untill all motors are in position
def wait():
	var = 0
	while var !=5:
		if cc.is_moving(var+1) == True:
			var = 0
		else:
			var += 1
	time.sleep(0.05)		

	
#set up the myo hub
hub = libmyo.Hub()
feed = libmyo.device_listener.Feed()
hub.run(3000, feed)
print("\nconnected")


#the prototype
def runProgram():
	# Listen to keyboard interrupts and stop the hub in that case.
	try:
		print("Waiting for a Myo to connect ...")
		myo = feed.wait_for_single_device(2)
		centre = libmyo.Quaternion(myo.orientation[0], myo.orientation[1], myo.orientation[2], myo.orientation[3]).normalized().conjugate()
		#startAcc = myo.acceleration
		#print(startAcc[0])
		if not myo:
			print("No Myo connected after 2 seconds.")
		
		while True:
			#move to start position 
			cc.mvPTP([0,26,50],velocity = 50)
			releaseTime = time.time() + 30000
			vibratefrequency = 0
			print("\nWave out")
			
			while time.time() < releaseTime:
				#are we holding the ball?
				if cc.get_angle(4,True) < 5 : 
					cc.openGrip(amount=20,speed=200)
				#are we waveing out ? 	
				if myo._pose == libmyo.Pose.wave_out:
					cc.mvPTP([-23,26,40],velocity = 50)
					wait()
					print("\nPull down")	
					break
				#vibrate two times every second 	
				elif time.time() > vibratefrequency:
					myo.vibrate('short')
					myo.vibrate('short')
					vibratefrequency=time.time()+ 1			
			
			while time.time() < releaseTime:
				#at pick up position 
				#look ot the orientation of the arm 
				#if arm points down move the robot down 
				obj = libmyo.Quaternion(myo.orientation[0], myo.orientation[1], myo.orientation[2], myo.orientation[3])
				frame =  centre.__mul__(obj)
				if frame[1]<(pGet()-1.5*pGet()):
					cc.mvLin([-23,26,14])
					wait()
					print("\nFist")
					break
				#vibrate two times every second 
				elif time.time() > vibratefrequency:
					myo.vibrate('short')
					myo.vibrate('short')
					vibratefrequency=time.time()+ 1
			
			releaseTime = time.time() + 30
			vibratefrequency = 0
			
			while time.time() < releaseTime:
				#wait until fist is made 
				if myo._pose == libmyo.Pose.fist:
					cc.closeGrip(speed = 100)
					print("\nPull up")	
					break
				#vibrate two times every second 	
				elif time.time() > vibratefrequency:	
					myo.vibrate('short')
					myo.vibrate('short')
					vibratefrequency=time.time()+ 1
					
			while time.time() < releaseTime:
				#at pick up position 
				#look ot the orientation of the arm 
				#if arm points up move the robot up
				obj = libmyo.Quaternion(myo.orientation[0], myo.orientation[1], myo.orientation[2], myo.orientation[3])
				frame =  centre.__mul__(obj)
				if frame[1]>pGet():
					cc.mvLin([-23,26,40])
					wait()
					print("\nWave in")	
					break
				#vibrate two times every second 
				elif time.time() > vibratefrequency:
					myo.vibrate('short')
					myo.vibrate('short')
					vibratefrequency=time.time()+ 1
		
			wait()
			cc.mvPTP([0,26,50],velocity = 50)
			wait()
			releaseTime = time.time() + 30
			vibratefrequency = 0
			
			while time.time() < releaseTime:
				#wait until a wave in is made 
				if myo._pose == libmyo.Pose.wave_in:
					cc.mvPTP([24,22,50],velocity = 50)
					wait()
					print("\nPull down")
					break	
				#vibrate two times every second 
				elif time.time() > vibratefrequency:
					myo.vibrate('short')
					myo.vibrate('short')
					vibratefrequency=time.time()+ 1
					
			while time.time() < releaseTime:
				#at drop off position 
				#look ot the orientation of the arm 
				#if arm points down move the robot down
				obj = libmyo.Quaternion(myo.orientation[0], myo.orientation[1], myo.orientation[2], myo.orientation[3])
				frame =  centre.__mul__(obj)
				if frame[1]<(pGet()-1.5*pGet()):
					cc.mvLin([24,22,23])
					print("\nSpread")
					wait()			
					break	
				#vibrate two times every second 
				elif time.time() > vibratefrequency:
					myo.vibrate('short')
					myo.vibrate('short')
					vibratefrequency=time.time()+ 1 
					
			while time.time() < releaseTime:
				#wait for a fingers spread 
				if myo._pose == libmyo.Pose.fingers_spread:
					cc.openGrip(speed = 100)
					print("\nPull up")					
					break	
				#vibrate two times every second 
				elif time.time() > vibratefrequency:
					myo.vibrate('short')
					myo.vibrate('short')
					vibratefrequency=time.time()+ 1
		
			while time.time() < releaseTime:
				#at drop off position 
				#look ot the orientation of the arm 
				#if arm points up move the robot up
				obj = libmyo.Quaternion(myo.orientation[0], myo.orientation[1], myo.orientation[2], myo.orientation[3])
				frame =  centre.__mul__(obj)
				if frame[1]>pGet():
					cc.mvLin([24,22,50])
					wait()
					break
				elif time.time() > vibratefrequency:	
					myo.vibrate('short')
					myo.vibrate('short')
					vibratefrequency=time.time()+ 1 
		#while loop 			
		
	except KeyboardInterrupt:
			print("\nQuitting ...")
	finally:
		print("Shutting down hub...")
		hub.shutdown()		
		
		
root.mainloop()