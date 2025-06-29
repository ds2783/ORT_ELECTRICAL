import numpy as np

class Integration:
    def __init__(self, previous_y=0):
        self.prev_y = previous_y

    def single(self, y, dx):  # Trapezoidal rule approximate integration
        ret = ((self.prev_y + y) / 2) * dx
        self.prev_y = y
        return ret
    
    def rollingIntegration(self, x_vals, y_vals, c=0):
        rolling_val = c
        self.prev_y = y_vals[0]
        integ_vals = [c]
        for index in range(len(x_vals - 1)):
            x_1 = x_vals[index]
            x_2 = x_vals[index + 1]
            dx = x_2 - x_1
            rolling_val += self.single(y_vals[index + 1], dx)
            integ_vals.append(rolling_val)
        return integ_vals

    def integrate(self, x_vals, y_vals, c=0):
        final = c
        self.prev_y = y_vals[0]
        for index in range(len(x_vals - 1)):
            x_1 = x_vals[index]
            x_2 = x_vals[index + 1]
            dx = x_2 - x_1
            final += self.single(y_vals[index + 1], dx)
        return final

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
