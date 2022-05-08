from CarFactory import CarFactory
from IDMExp import IDM
from Street import Street, StreetRamp, StreetAuto
from sensors import sensor


import numpy as np


prob_auto, prob_car = 0, 1
num_lane = 1
road_length = 500 # 5 km
flow_in = 0.5 # vehicle per second

sensorT = sensor(100, 0.0001)
HVModel = IDM(v0 = 120/3.6, a = 0.5, b = 3.0, s0 = 1, T = 0.5, sensor=sensorT)
cf = CarFactory(prob_auto, prob_car, HVModel)
road = Street(num_lane, road_length, cf)
for i in range(1000):
    road.update(flow_in)
    
    if i % 100 == 0:
        print(np.average([c.vel for c in road.street]))
        # for c in road.street: 
            #print(c.pos, c.lane, c.vel)
        print('='*20)
        # print(road.street)
        # road.report()