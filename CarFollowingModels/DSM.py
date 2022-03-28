# created at: [2022-03-23 09:44]
# fudaocheng@pjlab.org.cn


from vehicle import vehicle
import math

# 驾驶人反应时间 tau_1
# 车辆制动器响应时间 tau_2 = 0.15s
# tau = tau_1 + tau_2
# alpha_1 加速相关系数
# alpha_2 减速相关系数
# SM_DL DSM 下限
# SM_DH DSM 上限
class DSM:
    def __init__(self, tau_2=0.15, alpha_1=6.43, alpha_2=12.22, SM_DL=0.75, SM_DH=0.94) -> None:
        self.name = 'DSM'
        self.tau_2 = tau_2
        self.alpha_1 = alpha_1
        self.alpha_2 = alpha_2
        self.SM_DL = SM_DL
        self.SM_DH = SM_DH


    # 两车相对间距
    @staticmethod
    def getDn(nowVeh: vehicle, precedVeh: vehicle) -> float:
        return precedVeh.xPos - nowVeh.xPos - precedVeh.length


    def getSM(self, nowVeh: vehicle, precedVeh: vehicle) -> float:
        Dn = DSM.getDn(nowVeh, precedVeh)
        measure = nowVeh.velocity * self.tau_2 / Dn
        velocity_diff = nowVeh.velocity - precedVeh.velocity
        velocity_sum = nowVeh.velocity + precedVeh.velocity
        return 1 - (measure + velocity_diff*velocity_sum/(2*nowVeh.deceleration*Dn))


    def getAcceleration(self, nowVeh: vehicle, precedVeh: vehicle):
        SM = self.getSM(nowVeh, precedVeh)
        accetau = 0
        if SM > self.SM_DH:
            accetau = self.alpha_1 * (SM - self.SM_DH)
        elif SM < self.SM_DL:
            accetau = self.alpha_2 * (SM - self.SM_DL)

        Dn = DSM.getDn(nowVeh, precedVeh)
        if Dn < 3 and nowVeh.velocity > 0 and  nowVeh.velocity > precedVeh.velocity:
            S0 = 1.9
            brakeConstrant = - math.pow(nowVeh.velocity, 2) / (2 * max(Dn - S0, 0.01))
            accetau = max(brakeConstrant, -8)

        return accetau


    def __str__(self) -> str:
        return 'alpha_1: {}; alpha_2: {}, SM_DH: {}, SM_DL: {}'.format(
                    self.alpha_1, self.alpha_2, self.SM_DH, self.SM_DL
                )


if __name__ == '__main__':
    from matplotlib import pyplot as plt


    DSM1 = DSM()
    veh1 = vehicle('veh1', 10, 10, DSM1)
    veh2 = vehicle('veh2', 0, 15, DSM1)
    veh2.precedVeh = veh1
    xPosList_1 = []
    xPosList_2 = []
    velocityList_1 = []
    velocityList_2 = []


    for i in range(int(10/veh1.dt)):
        print('-'*20)
        print(i)
        print(veh1)
        print(veh2)

        veh1.DSMStep()
        veh2.DSMStep()
        xPosList_1.append(veh1.xPos)
        xPosList_2.append(veh2.xPos)
        velocityList_1.append(veh1.velocity)
        velocityList_2.append(veh2.velocity)

    ax1 = plt.subplot(2, 1, 1)
    ax1.plot(list(range(len(xPosList_1))), xPosList_1, color='r')
    ax1.plot(list(range(len(xPosList_2))), xPosList_2, color='b')

    ax2 = plt.subplot(2, 1, 2)
    ax2.plot(list(range(len(velocityList_1))), velocityList_1, color='r')
    ax2.plot(list(range(len(velocityList_2))), velocityList_2, color='b')

    plt.show()
    