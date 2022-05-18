from CarFactory import CarFactory
from IDM import IDM
from Street import Street, StreetRamp, StreetAuto
import seaborn as sns
from matplotlib import pyplot as plt


import numpy as np


prob_car = 1
num_lane = 3
road_length = 500 # 5 km


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


def get_flow_diff(flow_in, density, AveVel):
    if flow_in >= (density * AveVel):
        return (flow_in - (density * AveVel)) / flow_in
    else:
        return 0


def getPI(prob_auto, flow_in):
    cf = CarFactory(prob_auto, prob_car)
    road = Street(num_lane, road_length, cf)
    for i in range(1000):
        road.update(flow_in)
    
    density = len(road.street)/road.road_length
    TTCsum = calTTC(road.street)
    averageVel = np.average([c.vel for c in road.street])

    return density, TTCsum, averageVel
        
        # if i % 100 == 0:
        #     print('density (veh/m):', len(road.street)/road.road_length)
        #     print('TTC (times):', calTTC(road.street))

        #     if road.street:
        #         print('average velocity (m/s):', np.average([c.vel for c in road.street]))

            # for c in road.street: 
            #     print(c.pos, c.lane, c.vel)
            
            # print('='*20)
            # print(road.street)
            # road.report()

heat_flow_diff = []
heat_TTC = []
heat_AveVel = []
for i in range(10):
    flow_in = 0.2 * (i + 1)
    row_flow_diff = []
    row_TTC = []
    row_AveVel = []
    for j in range(10):
        prob_auto = 0.1 * (j + 1)
        density, TTC, AveVel = getPI(prob_auto, flow_in)
        print(density, TTC, AveVel)
        if i < 2:
            row_flow_diff.append(0)
        else:
            row_flow_diff.append(get_flow_diff(flow_in, density, AveVel) * 100)
        row_TTC.append(TTC)
        row_AveVel.append(AveVel)
        print('='*20)
    heat_flow_diff.append(row_flow_diff)
    heat_TTC.append(row_TTC)
    heat_AveVel.append(row_AveVel)


heat_maps = [heat_flow_diff, heat_TTC, heat_AveVel]
heat_name = ['Unsatisfied Demand (%)', 'Potential Collisions', 'Average Velocity']
fmt_list = ['.1f', 'd', '.2f']
for i in range(len(heat_maps)):
    x_ticks = [round((j + 1) * 0.1, 2) for j in range(10)]
    y_ticks = [round((k + 1) * 0.1, 2) for k in range(10)]
    ax = sns.heatmap(
        np.array(heat_maps[i]), 
        xticklabels=x_ticks, yticklabels=y_ticks, 
        annot=True, fmt=fmt_list[i]
    )
    ax.set_title(heat_name[i])
    ax.set_xlabel('Penetration of CAVs')
    ax.set_ylabel('Demand (veh/s)')
    plt.show()