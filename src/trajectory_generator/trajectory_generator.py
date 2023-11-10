import math
#Create circular trajectory 
def get_trajectory(origin, radius, period):
    '''Return a function maps t => pose(x, y, theta) 
    in 2D euleur coordinate system 

    origin is in (x, y) form
    period: the time needed to move around the full circle (s)
    '''

    f = 1/period 
    return lambda t: (origin[0] + radius*math.cos(2*math.pi * f * t), 
                      origin[1] + radius * math.sin(2*math.pi * f * t), 
                      2*math.pi * f * t + math.pi / 2)

    

