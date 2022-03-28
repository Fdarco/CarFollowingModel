# created at: [2022-03-25 21:12]
# fudaocheng@pjlab.org.cn


from vehicle import vehicle
from DSM import DSM


import math
import numpy as np
from scipy.stats import johnsonsb
from matplotlib import markers, pyplot as plt


# 生成前车
def genAheadVeh() -> vehicle:
    DSM1 = DSM()
    vehAhead = vehicle('veh1', 20, 10, DSM1)
    return vehAhead


def printData(data):
    print('='*20)
    print('data: ', data)
    print('average: {}, std: {}, median: {}, maximum: {}, minimum: {}'.format(
        np.average(data), np.std(data), np.median(data), max(data), min(data)
    ))


class generalVeh:
    def __init__(self) -> None:
        pass

    # 根据 Johnson SB 分布生成随机车辆
    @staticmethod
    def generalDriver(vehID, xPos, oriVelocity) -> vehicle:
        randAlpha_1 = johnsonsb.rvs(1.662, 0.45828, 5.6776, 31.873, size = 1)[0]
        randAlpha_2 = johnsonsb.rvs(0.17437, 0.51139, 3.36, 26.447, size = 1)[0]
        randTau = johnsonsb.rvs(1.3301, 1.1102, 0.21031, 1.7804, size = 1)[0]
        randSM_DH = johnsonsb.rvs(-1.3728, 1.0014, 0.28035, 0.73536, size = 1)[0]
        randSM_DL = johnsonsb.rvs(-0.66092, 0.96002, 0.36268, 0.62005, size = 1)[0]

        DSMins = DSM(
            alpha_1 = randAlpha_1, 
            alpha_2 = randAlpha_2, 
            SM_DH=randSM_DH, 
            SM_DL = randSM_DL
            )
        vehins = vehicle(vehID, xPos, oriVelocity, DSMins, tau = randTau)
        return vehins

    # 生成测试车辆的数据
    @staticmethod
    def generalDriverTest(vehTest: vehicle):
        vehAhead = genAheadVeh()
        vehTest.precedVeh = vehAhead
        xPosList_1 = []
        xPosList_2 = []
        velocityList_1 = []
        velocityList_2 = []
        for _ in range(int(3/vehAhead.dt)):
            vehAhead.DSMStep()
            vehTest.DSMStep()
            xPosList_1.append(vehAhead.xPos)
            xPosList_2.append(vehTest.xPos)
            velocityList_1.append(vehAhead.velocity)
            velocityList_2.append(vehTest.velocity)

        return xPosList_2, velocityList_2

    @staticmethod
    def generalDriverPlot(iter) -> None:
        ax1 = plt.subplot(2, 1, 1)
        ax1.set_ylabel('x position (m)')
        ax2 = plt.subplot(2, 1, 2)
        ax2.set_xlabel('timestep (0.01s)')
        ax2.set_ylabel('velocity (m/s)')

        vehAhead = genAheadVeh()
        apl = []
        avl = []
        for _ in range(int(3/vehAhead.dt)):
            vehAhead.DSMStep()
            apl.append(vehAhead.xPos)
            avl.append(vehAhead.velocity)

        ax1.plot(
            list(range(len(apl))), 
            apl, color='black', linestyle='--', label='preceding vehicle'
        )
        ax2.plot(
            list(range(len(avl))),
            avl, color='black', linestyle='--', label='preceding vehicle'
        )

        for i in range(iter):
            vehinstance = generalVeh.generalDriver(str(i), 0, 15)
            viPara = (
                round(vehinstance.tau, 2),
                round(vehinstance.CFModel.SM_DL, 2),
                round(vehinstance.CFModel.SM_DH, 2),
                round(vehinstance.CFModel.alpha_1, 2),
                round(vehinstance.CFModel.alpha_2, 2)
            )
            pl, vl= generalVeh.generalDriverTest(vehinstance)
            ax1.plot(list(range(len(pl))), pl, label=viPara)
            ax2.plot(list(range(len(vl))), vl, label=viPara)

        ax1.legend()
        ax2.legend()
        plt.show()


class categoryVeh:
    def __init__(self) -> None:
        pass 

    @staticmethod
    def categoryDriver(vehID, xPos, oriVelocity, a, b, c) -> vehicle:
        # 0: 不敏感型; 1: 适中型; 2: 敏感型
        # alpha_1, alpha_2
        # average, std, median
        sensitivePara = {
            0: [(5.92, 0.909, 5.60), (5.57, 1.173, 5.28)],
            1: [(7.27, 2.350, 6.27), (13.59, 5.418, 12.18)],
            2: [(12.56, 7.668, 10.24), (23.09, 4.489, 23.64)]
        }
        # 0: 冒险型; 1: 重力型; 2: 保守型
        # SM_DL, SM_DH
        # average, std, median
        characterPara = {
            0: [(0.53, 0.035, 0.52), (0.91, 0.075, 0.93)],
            1: [(0.70, 0.053, 0.71), (0.94, 0.053, 0.95)], 
            2: [(0.86, 0.049, 0.84), (0.95, 0.048, 0.96)]
        }
        # 0: 敏捷型; 1: 正常型; 2: 尺钝型
        # tau
        # average, std, median
        actionPara = {
            0: [0.42, 0.068, 0.4],
            1: [0.71, 0.076, 0.7],
            2: [1.13, 0.227, 1.1]
        }
        randAlpha_1 = np.random.normal(sensitivePara[a][0][0], sensitivePara[a][0][1])
        randAlpha_2 = np.random.normal(sensitivePara[a][1][0], sensitivePara[a][1][1])
        randSM_DL = np.random.normal(characterPara[b][0][0], characterPara[b][0][1])
        randSM_DH = np.random.normal(characterPara[b][1][0], characterPara[b][1][1])
        randTau = np.random.normal(actionPara[c][0], actionPara[c][1])

        DSMins = DSM(
            alpha_1 = randAlpha_1, 
            alpha_2 = randAlpha_2, 
            SM_DH=randSM_DH, 
            SM_DL = randSM_DL
        )
        vehins = vehicle(vehID, xPos, oriVelocity, DSMins, tau = randTau)
        return vehins

    @staticmethod
    def categoryDriverTest(vehTest: vehicle):
        vehAhead = genAheadVeh()
        vehTest.precedVeh = vehAhead
        xPosList_1 = []
        xPosList_2 = []
        velocityList_1 = []
        velocityList_2 = []
        for _ in range(int(3/vehAhead.dt)):
            vehAhead.DSMStep()
            vehTest.DSMStep()
            xPosList_1.append(vehAhead.xPos)
            xPosList_2.append(vehTest.xPos)
            velocityList_1.append(vehAhead.velocity)
            velocityList_2.append(vehTest.velocity)

        return xPosList_2, velocityList_2

    @staticmethod
    def categoryDriverPlot(paraList: dict) -> None:
        plt.rcParams["font.sans-serif"]=["SimHei"]   # 设置字体
        plt.rcParams["axes.unicode_minus"]=False   # 该语句解决图像中的“-”负号的乱码问题
        ax1 = plt.subplot(2, 1, 1)
        ax1.set_ylabel('x position (m)')
        ax2 = plt.subplot(2, 1, 2)
        ax2.set_xlabel('timestep (0.01s)')
        ax2.set_ylabel('velocity (m/s)')
        
        vehAhead = genAheadVeh()
        apl = []
        avl = []
        for _ in range(int(3/vehAhead.dt)):
            vehAhead.DSMStep()
            apl.append(vehAhead.xPos)
            avl.append(vehAhead.velocity)

        ax1.plot(
            list(range(len(apl))), 
            apl, color='black', linestyle='--', label='preceding vehicle'
        )
        ax2.plot(
            list(range(len(avl))),
            avl, color='black', linestyle='--', label='preceding vehicle'
        )

        for k, v in paraList.items():
            a, b, c, vehName = v[0], v[1], v[2], k
            vehinstance = categoryVeh.categoryDriver(vehName, 0, 15, a, b, c)
            pl, vl= categoryVeh.categoryDriverTest(vehinstance)
            ax1.plot(list(range(len(pl))), pl, label = vehName)
            ax2.plot(list(range(len(vl))), vl, label = vehName)

        ax1.legend()
        ax2.legend()
        plt.show()


if __name__ == '__main__':
    generalVeh.generalDriverPlot(5)

    paraList = {
        '适中、保守、敏捷': [1, 2, 0, ],
        '敏感、保守、敏捷': [2, 2, 0, ],
        '适中、冒险、迟钝': [1, 0, 2, ],
        '不敏感、中立、正常': [0, 1, 1, ],
        '敏感、中立、正常': [2, 1, 1, ]
    }
    categoryVeh.categoryDriverPlot(paraList)