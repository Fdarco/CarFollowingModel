# created at: [2022-04-14 08:58]
# fudaocheng@pjlab.org.cn 
#######################################################
# 对自动驾驶模型进行检测范围、敏感程度的分级
# 并从车头间距、磁滞回线、最小安全距离等方面检验分级结果


from vehicle import vehicle
from MEVF import MEVF
import random
import numpy as np


class Model():
    def __init__(self, numVeh: int) -> None:
        self.numVeh = numVeh
        self.vehList = []
        self.xPosList = []
        self.velocityList = []
        self.acceList = []
        self.headwayList = []
        self.RSHWList = []


    def genVehs(self):
        cfm = MEVF(alpha=2.0, connRange=150)
        gap = 0
        for i in range(self.numVeh):
            veh = vehicle(
                    id='veh_%i'%i, xPos=gap, yPos=0, 
                    velocity=random.uniform(7, 13), CFModel=cfm
                )
            self.vehList.append(veh)
            gap += random.uniform(5, 15)
        
        
        for j in range(len(self.vehList) - 1):
            self.vehList[j].precedVeh = self.vehList[j+1]
            self.headwayList.append([self.vehList[j+1].xPos - self.vehList[j].xPos])
            v_r = self.vehList[j+1].velocity - self.vehList[j].velocity
            timeres = 1.5 - 0.05 * v_r
            RSHW = timeres * self.vehList[j].velocity
            self.RSHWList.append([RSHW])
            

        for k in range(len(self.vehList)):
            self.xPosList.append([self.vehList[k].xPos])
            self.velocityList.append([self.vehList[k].velocity])
            self.acceList.append([self.vehList[k].acceleration])
        

    def updatexPosList(self):
        for i in range(len(self.vehList)):
            self.xPosList[i].append(self.vehList[i].xPos)
            self.velocityList[i].append(self.vehList[i].velocity)
            self.acceList[i].append(self.vehList[i].acceleration)

        for j in range(len(self.vehList) - 1):
            self.headwayList[j].append(self.vehList[j+1].xPos - self.vehList[j].xPos)
            v_r = self.vehList[j+1].velocity - self.vehList[j].velocity
            timeres = 1.5 - 0.05 * v_r
            RSHW = timeres * self.vehList[j].velocity
            self.RSHWList[j].append(RSHW)


    def run(self):
        for veh in self.vehList:
            veh.MEVEStep()
        self.updatexPosList()


if __name__ == '__main__':
    from matplotlib import pyplot as plt

    model = Model(numVeh=20)
    model.genVehs()
    # print(model.xPosList)
    # for veh in model.vehList:
    #     print(veh)

    TotalStep = 1000
    timestep = 0
    while timestep < TotalStep:
        model.run()
        timestep += 1

    for i in range(3):
        # print(model.xPosList[i][-10:])
        print('velocity:')
        print(model.velocityList[i][-10:])
        print(np.mean(model.velocityList[i][-10:]))
        print(np.std(model.velocityList[i][-10:]))
        # print(model.acceList[i][-10:])
        print('headway:')
        print(model.headwayList[i][-10:])
        print(np.mean(model.headwayList[i][-10:]))
        print(np.std(model.headwayList[i][-10:]))
        print('required safety headway:')
        print(model.RSHWList[i][-10:])
        print(np.mean(model.RSHWList[i][-10:]))
        print(np.std(model.RSHWList[i][-10:]))
        print('='*20)

    # for i in range(5, 10):
    #     plt.plot(range(len(model.headwayList[i])), model.headwayList[i], label='veh_{}'.format(i))

    # plt.legend(loc='best')
    # plt.show()


    