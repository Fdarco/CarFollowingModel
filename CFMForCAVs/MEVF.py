# ****************************************************************************
# created at: [2022-04-03 08:48]
# fudaocheng@pjlab.org.cn 
# ****************************************************************************
# MEVF: mean expected velocity field
# key parameter: sensitivity, strength factor, mean expected velocity
# cite: Zhu Wen-Xing, et al. A new car-following model for autonomous vehicles
# flow with mean expected velocity field


import math
from vehicle import vehicle

# alpha in [0.8, 1.2, 1.6, 2.0] 敏感系数，值越大，越稳定
# 0 < beta < 0.5
# beta in [0.1, 0.2, 0.3, 0.4] 强度系数，表示当前车对前面车辆速度信息的敏感程度
# gamma in [1, 2, 3, 4] 速度域参数，表示获取前面 gamma 辆车的信息
class MEVF:
    def __init__(self, alpha = 1.2, beta = 0.2, gamma = 2) -> None:
        self.name = 'MEVF'
        self.alpha = alpha 
        self.beta = beta
        self.gamma = gamma


    def getOptimalVelocity(self, nowVeh: vehicle) -> float:
        if nowVeh.precedVeh:
            deltax = nowVeh.precedVeh.xPos - nowVeh.xPos
            midres1 = math.tanh(0.13 * (deltax - nowVeh.length) - 1.57)
            midres2 = math.tanh(0.13 * nowVeh.length + 1.57)
            return (nowVeh.maxVelocity / 2) * (midres1 + midres2)
        else:
            return nowVeh.desireVelocity


    def getVmf(self, precedList: list[vehicle]) -> float:
        gamma = len(precedList)
        # 这里按照论文里做的，获取前面几辆车的最优速度
        # 实际上和前车进行信息交互可能会更好一些
        # 当然进行信息交互会有频率的约束，仿真的 timestep 也需要设置合理
        sumOptVel = 0
        for veh in precedList:
            sumOptVel += self.getOptimalVelocity(veh)
        
        return sumOptVel / gamma


    def getAcceleration(self, nowVeh: vehicle) -> float:
        cnt = 0
        precedList = []
        root = nowVeh
        while root.precedVeh and cnt <= self.gamma:
            rp = root.precedVeh
            precedList.append(rp)
            root = rp
            cnt += 1

        Vmf = self.getVmf(precedList)
        Vo = self.getOptimalVelocity(nowVeh)
        acceleration = self.alpha * (Vo + self.beta * (Vmf - Vo) - nowVeh.velocity)
        return acceleration


    def __str__(self) -> str:
        return '{}: alpha: {}, beta: {}, gamma: {}'.format(
            self.name, self.alpha, self.beta, self.gamma
            )


if __name__ == '__main__':
    pass