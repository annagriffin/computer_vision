

class ContourPoint:
    def __init__(self, x=0, y=0, N=0, H=0):
        self.x = x
        self.y = y
        self.N = N
        self.H = H
        
    def __str__(self):
        return("x: {}, y: {}, N: {}, H: {}".format(self.x, self.y, self.N, self.H))

