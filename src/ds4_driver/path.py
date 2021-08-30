import rospy

class SpeedSpecifier:
	def __init__(self, velocity, turn):
		# Keep values within acceptable range
		assert -1 <= velocity <= 1
		assert -1 <= turn <= 1
		self.velocity = velocity
		self.turn = turn

	def equals(self, value, value2 = None):
		if value2 is None:
			return (self.velocity, self.turn) == value
		return (self.velocity, self.turn) == (value, value2)

	def __getitem__(self, i):
		if i == 0:
			return self.velocity
		if i == 1:
			return self.turn
		raise NotImplementedError

	def __repr__(self):
		return "speed: {:.3f}, turn: {:.3f}".format(self.velocity, self.turn)


def getMultiCheckpointPath(checkpoints):
	if (checkpoints[0][0] not in [0, None]):
		checkpoints.insert(0, (0,0,0))
 	assert checkpoints[-1][1] == checkpoints[-1][2] == 0
	# Hard to do this any other way
	checkpoints = {x[0]: SpeedSpecifier(x[1], x[2]) for x in checkpoints}
	return Path(checkpoints)

def getPath(pathDuration, linearSpeed, turnRate = 0):
	if pathDuration == 0 or pathDuration is None:
		return Path({0: SpeedSpecifier(0, 0)})
	p = Path({0: SpeedSpecifier(linearSpeed, turnRate), pathDuration: SpeedSpecifier(0, 0)})
	# print(p.path)
	return p

class Path:
	SPEED = 0
	ROTATION = 1

	def __init__(self, timeValueDict):
		self.path = sorted(timeValueDict.items())
		for i in range(len(self.path)):
			self.path[i] = rospy.Time(self.path[i][0]), self.path[i][1]
	# First key must be zero, else we have invalid time periods
		assert self.path[0][0]  == rospy.Time(0)
		# Last speed value must be zero, else we never stop
		assert self.path[-1][1].equals((0, 0))
	def getCurrentValue(self, currentTime, type):
		#print(isinstance(currentTime, rospy.Time))
		assert currentTime >= rospy.Time(0)
		prevVal = None
		for key,val in self.path:
			#print(key)
			#print(currentTime)
			if key > currentTime:
				# This can't be true for first item, so the only instance where we would return None is eliminated.
				assert prevVal is not None # But for safety, we'll assert this anyway
				if type is Path.SPEED:
					return prevVal[0]
				elif type is Path.ROTATION:
					return prevVal[1]
				else:
					raise RuntimeError("This should be unreachable(non-speed, non-rotation control type)")
			prevVal = val
		#print(currentTime)
		#print(self.path)
		#print(self.isDone(currentTime))
		return 0
		raise RuntimeError("This should be unreachable.")

	def isDone(self, currentTime):
		return currentTime > self.getEndTime()

	def getEndTime(self):
		return self.path[-1][0]
	def size(self):
		return len(self.path)
