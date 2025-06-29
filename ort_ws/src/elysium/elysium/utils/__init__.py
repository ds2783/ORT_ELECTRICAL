import numpy as np

class ApproxIntegration:
    def __init__(self, previous_y):
        self.prev_y = previous_y

    def integ_trap(self, y, dt):  # Trapezoidal rule approximate integration
        ret = ((self.prev_y + y) / 2) * dt
        self.prev_y = y
        return ret

class RollingAverage:
    def __init__(self, length):
        self.items = 0
        self.length = length
        self._queue = np.zeros(length, dtype=np.float32)
    
    def add(self, value):
        if self.items < self.length:
            self.items += 1
    
        self._queue[:-1] = self._queue[1:]
        self._queue[-1] = value

    @property
    def average(self):
        _items = self.items if self.items else 1  # Avoid dividing by zero.
        
        return np.sum(self._queue)/_items