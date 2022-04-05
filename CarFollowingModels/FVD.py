# crated at: [2022-03-21 15:32]
# fudaocheng@pjlab.org.cn


import math
from vehicle import vehicle

# 这里可以考虑使用张俊杰师兄的论文的方法标定一下参数
# 构建不同类型的驾驶人模型
class FVD:
    def __init__(self, alpha, beta, h_pc) -> None:
        self.name = 'FVD'
        # alpha 是驾驶员敏感系数
        self.alpha = alpha
        # beta 是速度差反馈系数
        self.beta = beta
        # h_pc 是安全车头间距
        self.h_pc = h_pc


    def getOptimalVelocity(self, nowVeh, precedVeh):
        headway = precedVeh.xPos - nowVeh.xPos
        return (1/2)*nowVeh.maxVelocity*(math.tanh(headway-self.h_pc)+math.tanh(self.h_pc))
        

    def getAcceleration(self, nowVeh, precedVeh):
        headway = precedVeh.xPos - nowVeh.xPos
        optimalVelocity = self.getOptimalVelocity(nowVeh, precedVeh)
        acceleration = self.alpha*(optimalVelocity-nowVeh.velocity) + self.beta*headway
        return acceleration


if __name__ == '__main__':
    from matplotlib import pyplot as plt

    FVD1 = FVD(0.41, 0.5, 7.02)
    veh1 = vehicle('veh1', 10, 10, FVD1)
    veh2 = vehicle('veh2', 0, 15, FVD1)
    veh2.precedVeh = veh1
    xPosList_1 = []
    xPosList_2 = []
    velocityList_1 = []
    velocityList_2 = []

    for i in range(int(3/veh1.dt)):
        veh1.FVDStep()
        veh2.FVDStep()
        xPosList_1.append(veh1.xPos)
        xPosList_2.append(veh2.xPos)
        velocityList_1.append(veh1.velocity)
        velocityList_2.append(veh2.velocity)

    ax1 = plt.subplot(2, 1, 1)
    ax1.set_ylabel('x position (m)')
    ax1.plot(list(range(len(xPosList_1))), xPosList_1, color='r', label='preceding vehicle')
    ax1.plot(list(range(len(xPosList_2))), xPosList_2, color='b', label='following vehicle')

    ax2 = plt.subplot(2, 1, 2)
    ax2.set_xlabel('time step (0.01s)')
    ax2.set_ylabel('velocity (m/s)')
    ax2.plot(list(range(len(velocityList_1))), velocityList_1, color='r', label='preceding vehicle')
    ax2.plot(list(range(len(velocityList_2))), velocityList_2, color='b', label='following vehicle')

    ax1.legend()
    ax2.legend()
    plt.show()