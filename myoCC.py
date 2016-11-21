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

from modules import ccctrl
#from modules import readCsv
from modules import myo as libmyo ; libmyo.init('./myo-sdk-win-0.9.0/bin')
cc=ccctrl.robot('COM9')
#on windows write the com port as 'COM9' if its port 9.

class Listener(libmyo.DeviceListener):
	"""
	Listener implementation. Return False from any function to
	stop the Hub.
	"""

	interval = 0.05  # Output only 0.05 seconds

	def __init__(self):
		super(Listener, self).__init__()
		self.orientation = None
		self.pose = libmyo.Pose.rest
		self.emg_enabled = False
		self.locked = False
		self.rssi = None
		self.emg = None
		self.last_time = 0

	def output(self):
		ctime = time.time()
		if (ctime - self.last_time) < self.interval:
			return
		self.last_time = ctime
		parts = []
		if self.orientation:
			parts.append(str(self.pose).ljust(10))
		print('\r' + ''.join('[{0}]'.format(p) for p in parts), end='')
		sys.stdout.flush()

	def on_connect(self, myo, timestamp, firmware_version):
		myo.vibrate('short')
		myo.vibrate('short')
		myo.request_rssi()
		myo.request_battery_level()

	def on_rssi(self, myo, timestamp, rssi):
		self.rssi = rssi
		self.output()

	def on_pose(self, myo, timestamp, pose):
		if pose == libmyo.Pose.double_tap:
			myo.set_stream_emg(libmyo.StreamEmg.enabled)
			self.emg_enabled = True
		elif pose == libmyo.Pose.fingers_spread:
			super().openGrip(speed = 50)
		elif pose == libmyo.Pose.fist:
			super().closeGrip(speed = 50)
		self.pose = pose
		self.output()

	def on_orientation_data(self, myo, timestamp, orientation):
		self.orientation = orientation
		self.output()

	def on_accelerometor_data(self, myo, timestamp, acceleration):
		pass

	def on_gyroscope_data(self, myo, timestamp, gyroscope):
		pass

	def on_emg_data(self, myo, timestamp, emg):
		self.emg = emg
		self.output()

	def on_unlock(self, myo, timestamp):
		self.locked = False
		self.output()

	def on_lock(self, myo, timestamp):
		self.locked = True
		self.output()

	def on_event(self, kind, event):
		"""
		Called before any of the event callbacks.
		"""

	def on_event_finished(self, kind, event):
		"""
		Called after the respective event callbacks have been
		invoked. This method is *always* triggered, even if one of
		the callbacks requested the stop of the Hub.
		"""

	def on_pair(self, myo, timestamp, firmware_version):
		"""
		Called when a Myo armband is paired.
		"""

	def on_unpair(self, myo, timestamp):
		"""
		Called when a Myo armband is unpaired.
		"""

	def on_disconnect(self, myo, timestamp):
		"""
		Called when a Myo is disconnected.
		"""

	def on_arm_sync(self, myo, timestamp, arm, x_direction, rotation,
					warmup_state):
		"""
		Called when a Myo armband and an arm is synced.
		"""

	def on_arm_unsync(self, myo, timestamp):
		"""
		Called when a Myo armband and an arm is unsynced.
		"""

	def on_battery_level_received(self, myo, timestamp, level):
		"""
		Called when the requested battery level received.
		"""

	def on_warmup_completed(self, myo, timestamp, warmup_result):
		"""
		Called when the warmup completed.
		"""





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
			
print("Connecting to Myo ... Use CTRL^C to exit.")
try:
	hub = libmyo.Hub()
except MemoryError:
	print("Myo Hub could not be created. Make sure Myo Connect is running.")
	return
	
hub.set_locking_policy(libmyo.LockingPolicy.none)
hub.run(1000, Listener())

# Listen to keyboard interrupts and stop the hub in that case.
try:
	while hub.running :
		cc.mvPTP([20,0,60],50)
		wait()
		#cc.closeGrip(strength=200)
		if cc.get_angle(4,True) < 5 : #are we holding the ball?
			cc.openGrip(amount=20,speed=200)
		cc.openGrip(speed = 50,amount = 20)	
		cc.mvPTP([0,40,30],50)
		wait()
		cc.mvLin([0,40,18])
		wait()
		#if True : # check if theres is an input from myo
		#	cc.closeGrip(strength=350)
		
		wait()
		cc.mvLin([0,40,30])
		wait()
		cc.mvPTP([20,0,60],50)
		wait()
		cc.mvPTP([0,-40,30],50)
		wait()
		cc.mvLin([0,-40,10])
		wait()
		#if True:
		#	cc.openGrip(amount=20, speed = 100)
			
		cc.mvLin([0,-30,30])
except KeyboardInterrupt:
		print("\nQuitting ...")
finally:
	print("Shutting down hub...")
	hub.shutdown()		