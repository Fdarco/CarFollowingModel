# ****************************************************************************
# created at: [2022-04-05 21:09]
# fudaocheng@pjlab.org.cn
# ****************************************************************************
# SPF: safe potential field
# cite: Yanfeng Jia, et al. Car-following characteristics 
# and model of connected autonomous vehicles based on safe potential field


import math
from re import U
from vehicle import vehicle


class laneMarking:
    def __init__(self, yPos, P) -> None:
        self.yPos = yPos
        # P 根据 laneMarking 的类型变化产生不同
        # P = 2 表示白色虚线
        # P = 8 表示双黄线
        self.P = P


class road:
    def __init__(
        self, id, leftBoundary, rightBoundary, 
        lmList: list[laneMarking]
    ) -> None:
        self.id = id
        self.boundaries = [leftBoundary, rightBoundary]
        self.lmList = lmList


    def getRes(self, veh: vehicle):
        res = 0
        for boundary in self.boundaries:
            x_r = veh.yPos - boundary
            res += math.pow(1/abs(x_r), 2) * (x_r / abs(x_r))
        return res


class SPF:
    def __init__(
        self, delta = 1.22, lambdaPara = 3,
        t_0 = 1.5, c_v = 0.05, c_a = 0, 
        u = 0.713, v = 1.648, eta_0 = 28.535, 
        eta_1 = 43.531, gamma = 0.032, s_0 = 2.11
    ) -> None:
        self.delta = delta
        self.lambdaPara = lambdaPara
        self.t_0 = t_0
        self.c_v = c_v
        self.c_a = c_a
        self.u = u
        self.v = v
        self.eta_0 = eta_0
        self.eta_1 = eta_1


    def getU_L(self, nowVeh: vehicle, rd: road):
        sumField = 0
        for lm in rd.lmList:
            mid1_up = -math.pow((nowVeh.yPos - lm.yPos), 2)
            mid1_down = 2 * math.pow(self.delta, 2)
            mid1 = math.exp(mid1_up / mid1_down)
            mid2 = (nowVeh.yPos - lm.yPos) / abs(nowVeh.yPos - lm.yPos)
            Field = lm.P * mid1 * mid2
            sumField += Field
        return sumField


    def getU_R(self, nowVeh: vehicle, rd: road):
        return 0.5 * self.lambdaPara * rd.getRes(nowVeh)


    def getX_r(self, nowVeh: vehicle):
        v_r = nowVeh.precedVeh.velocity - nowVeh.velocity
        timeres = self.t_0 - self.c_v * v_r - self.c_a * nowVeh.precedVeh.acceleration
        return timeres * nowVeh.velocity


    def getXAcceleration(self, nowVeh: vehicle):
        L = nowVeh.precedVeh.xPos - nowVeh.xPos
        X_r = self.getX_r(nowVeh)
        res1 = math.pow(X_r, self.u) / math.pow(L, self.u + 1)
        res2 = math.pow(X_r, self.v) / math.pow(L, self.v + 1)
        return self.eta_1 * (res1 - res2)