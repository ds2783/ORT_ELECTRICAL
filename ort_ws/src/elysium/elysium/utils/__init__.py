import numpy as np

class Integration:
    def __init__(self, previous_y=.0):
        self.prev_y = previous_y

    def single(self, y, dx):  # Trapezoidal rule approximate integration
        ret = ((self.prev_y + y) / 2) * dx
        self.prev_y = y
        return ret
    
    def rollingIntegration(self, x_vals, y_vals, c=0):
        rolling_val = c
        self.prev_y = y_vals[0]
        integ_vals = [c]
        for index in range(len(x_vals) - 1):
            x_1 = x_vals[index]
            x_2 = x_vals[index + 1]
            dx = x_2 - x_1
            rolling_val += self.single(y_vals[index + 1], dx)
            integ_vals.append(rolling_val)
        return integ_vals

    def integrate(self, x_vals, y_vals, c=0):
        final = c
        self.prev_y = y_vals[0]
        for index in range(len(x_vals) - 1):
            x_1 = x_vals[index]
            x_2 = x_vals[index + 1]
            dx = x_2 - x_1
            final += self.single(y_vals[index + 1], dx)
        return final

class RollingAverage:
    def __init__(self, dim):
        """Rolling average for a ndarray or a 2D ndarray.

        :param dim: shape of numpy array, format of (Row, Column). The row will dictate how many samples to average over.
        :type dim: Iterable, Tuple, List
        """

        self.items = 0
        self.dim = dim
        
        if isinstance(dim, int):
            self.dim = (dim, 1)

        self._queue = np.zeros(dim, dtype=np.float32)
    
    def add(self, value):
        """Add a value or set of values to the rolling average. If the shape of the 
        rolling average is not (row, 1), make sure the row vector passed is the same shape as 
        (1, col). (Basically make sure the row vector is the correct shape.)

        :param value: Single value or row vector
        :type value: int, np.ndarray
        """
        if self.items < self.dim[0]:
            self.items += 1
    
        self._queue[:-1, :] = self._queue[1:, :]
        self._queue[-1, :] = value  # make sure this is a row vector if its 2D! 

    @property
    def average(self):
        _items = self.items if self.items else 1  # Avoid dividing by zero.
        
        return np.sum(self._queue)/_items
