
class SpeedSpecifier:
    def __init__(self, velocity, turn):
        # Keep values within acceptable range
        assert -1 < velocity < 1
        assert -1 < turn < 1
        self.velocity = velocity
        self.turn = turn
        

def getPath(pathDuration, linearSpeed, turnRate = 0):
    if pathDuration == 0 or pathDuration is None:
        return Path({0: SpeedSpecifier(0, 0)})
    return Path({0: SpeedSpecifier(linearSpeed, turnRate), pathDuration: SpeedSpecifier(0, 0)})

class Path:
    SPEED = 0
    ROTATION = 1
    
    def __init__(self, timeValueDict):
        # First key must be zero, else we have invalid time periods
        assert sorted(self.path.items())[0][0]  == 0
        # Last speed value must be zero, else we never stop
        assert sorted(self.path.items())[-1][1] == (0, 0)
        self.path = sorted(timeValueDict.items())
        
    def getCurrentValue(self, currentTime):
        assert currentTime >= 0
        prevVal = None
        for key,val in self.path:
            if key > currentTime:
                # This can't be true for first item, so the only instance where we would return None is eliminated.
                assert prevVal is not None # But for safety, we'll assert this anyway
                return prevVal
            prevVal = val
        raise RuntimeError("This should be unreachable.")
    
    def isDone(self, currentTime):
        return currentTime > self.getEndTime()
    
    def getEndTime(self):
        return self.path[-1][0]