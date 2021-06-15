import math


class Rover:
    def __init__(self, length, width, x_pos, y_pos, pivot_length, angle):
        self.length = length
        self.width = width
        self.x_pos = x_pos # x value of pivot
        self.y_pos = y_pos # y value of pivot
        self.angle = angle
        self.pivot_length = pivot_length

        self.dist_centre_topcorners = math.sqrt((self.length-self.pivot_length)*(self.length-self.pivot_length)+(self.width/2)*(self.width/2))

        self.dist_centre_bottomcorners = math.sqrt((self.pivot_length)*(self.pivot_length)+(self.width/2)*(self.width/2))

        self.ang_topcorner_bottomcorner = math.degrees(math.atan((self.width/2) / (self.pivot_length)))

        self.ang_front_topleft = math.degrees(math.atan((self.width/2) / (self.length-self.pivot_length)))

        self.ang_back_to_bottomleft = math.degrees(math.atan((self.width/2) / (self.pivot_length)))

        self.ang_topleft_bottomleft = 180 - self.ang_front_topleft - self.ang_back_to_bottomleft
