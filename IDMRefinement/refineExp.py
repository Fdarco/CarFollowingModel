from CarFactory import CarFactory
from IDMExp import IDM
from Street import Street, StreetRamp, StreetAuto
from sensors import sensor


import numpy as np


stage1 = [10, 0.001, 10, 2]
stage2 = [20, 0.0005, 5, 1.5]
stage3 = [50, 0.0002, 2, 1]
stage4 = [100, 0.0001, 1, 0.5]
stages = [stage1, stage2, stage3, stage4]


prob_auto, prob_car = 0, 1
num_lane = 1
road_length = 500 # 5 km
flow_in = float(input('flow in:')) # vehicle per second


def calTTC(st):
    if not st:
        return None
    posList = [c.pos for c in st]
    velList = [c.vel for c in st]
    TTCList = []
    for i in range(len(posList)-1):
        if velList[i+1] > velList[i]:
            relDis = posList[i] - posList[i+1]
            TTC = relDis / (velList[i+1] - velList[i])
            TTCList.append(TTC)
        else:
            TTCList.append(None)

    cnt = 0
    for T in TTCList:
        if T and T <= 2:
            cnt += 1

    return cnt


def getPI(sID: int):
    sensorT = sensor(stages[sID][0], stages[sID][1])
    HVModel = IDM(
        v0 = 120/3.6, a = 0.5, b = 3.0, 
        s0 = stages[sID][2], T = stages[sID][3], sensor=sensorT
        )
    cf = CarFactory(prob_auto, prob_car, HVModel)
    road = Street(num_lane, road_length, cf)
    for i in range(1000):
        road.update(flow_in)
        
        if i % 100 == 0:
            print('density (veh/m):', len(road.street)/road.road_length)
            print('TTC (times):', calTTC(road.street))

            if road.street:
                print('average velocity (m/s):', np.average([c.vel for c in road.street]))

            # for c in road.street: 
            #     print(c.pos, c.lane, c.vel)
            
            print('='*20)
            # print(road.street)
            # road.report()


sID = int(input('stage ID (0, 1, 2, 3) :'))
getPI(sID)