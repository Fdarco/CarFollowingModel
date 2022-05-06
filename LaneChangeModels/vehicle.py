import math
import queue


class vehicle:
    def __init__(self, id, type, xPos, yPos, velocity):
        self.id = id
        # type: virtual / real
        self.type = type
        self.V_max = 33.3
        self.V_des = 33.3
        # tau: reation time for CAVs -> safety distance
        # tau should be 3 if the vehicle is HV
        self.tau = 1.5
        self.c_v = 0.05
        self.xPos = xPos
        self.yPos = yPos
        self.velocity = velocity
        self.acce = 2.778
        # C: tolerance for LC
        self.C = 0
        self.C_thr = 4
        # theta: Angle between driving direction and lane line
        self.theta = 0
        self.length = 2
        self.width = 4
        self.dt = 0.1

    # for obs_veh
    def run(self):
        self.xPos += self.velocity * self.dt