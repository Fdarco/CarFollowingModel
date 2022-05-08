# 构建检测精度与检测距离之间的关系，用来为驾驶行为分级

from numpy import real
import random


class sensor:
    def __init__(self, th, coef) -> None:
        self.th = th
        # coef: 0.0001, 0.0002, 0.0005, 0.001
        self.coef = coef

    def getError(self, val, ratio):
        ran = val * ratio
        return random.uniform(-ran, ran)


    def getDistance(self, nowCar, precedCar):
        realDis = precedCar.pos - nowCar.pos
        if realDis <= self.th:
            return realDis + self.getError(realDis, 0.001)
        else:
            ratio = (realDis - self.th) * 0.001 + 0.001
            return realDis + self.getError(realDis, ratio)


    def getSpeed(self, nowCar, precedCar):
        realDis = precedCar.pos - nowCar.pos
        rat = pow(realDis, 4) * self.coef
        return precedCar.vel + self.getError(precedCar.vel, rat)
