
class Car:
    def __init__(self, speed_input, angle_input, direction, radius, target,brake):
        self.speed = speed_input        #float, speed input
        self.angle = angle_input    #integer, servomotor input
        self.direction = direction      #1 if right, -1 if left
        self.radius = radius            #curvature radius
        self.target = target            #[xt,yt], coordinates of the targeted point
        self.brake = brake              #booléen (1 si décélération)

class Lidar:
    def __init__(self, data, temp, vrel):
        self.data = data        #array, data collected at t=k*delta_t
        self.prev_data = temp   #array, data collected at t=(k-1)*delta_t
        self.vrel = vrel        #detected points relative speed for each Lidar angle index