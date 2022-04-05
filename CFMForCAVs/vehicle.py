# created at: [2022-03-22 14:56]
# fudaocheng@pjlab.org.cn
import math
import queue


class vehicle:
    def __init__(
        self, id, xPos, velocity, CFModel,
        dt = 0.01, tau = 0.7, maxVelocity = 20.5, 
        desireVelocity = 33, deceleration = 7.5, length = 5
    ):
        self.id = id
        self.xPos = xPos
        self.velocity = velocity
        self.CFModel = CFModel
        self.precedVeh = None
        self.dt = dt
        self.maxVelocity = maxVelocity
        self.deceleration = deceleration   # 制动减速度 7.5~8 m/s 或 0.75g
        self.length = length
        # 期望速度，参考 M. Treiber, A. Kesting, and C. Thiemann, 
        # Traffic flow dynamics : data, models and simulation. Springer, 2013. （第11.3节）
        # 对不同的驾驶员，这个数值可以修改，来表现驾驶员特性
        self.desireVelocity = desireVelocity
        # 驾驶员的反应时间
        self.tau = tau
        # 因为加速度的变化是在一定时间之后的，所有用一个栈来存储计算的加速度
        self.acceSeq = queue.Queue(int(self.tau/self.dt))
        for i in range(int(self.tau/self.dt)):
            self.acceSeq.put(0)

    # 根据 HCM ，车辆加速度范围为 0.9~4 m/s2
    # 而舒适加速度不超过 1.5 m/s2
    # 但是对于激进驾驶员这个数值可以高一些，对构建人驾模型具有一定帮助
    def accelerationCst(self, acceleration: float) -> float:
        if acceleration > 1.5:
            return 1.5
        elif acceleration < -8:
            return -8
        else:
            return acceleration


    def velocityCst(self, velocity: float) -> None:
        if velocity < 0:
            self.velocity = 0
        elif velocity > self.maxVelocity:
            self.velocity = self.maxVelocity
        else:
            self.velocity = velocity


    def getFreeAcce(self):
        acceFM = 1.5
        delta = 4
        acceFree = acceFM * (1 - math.pow(self.velocity/self.desireVelocity, delta))
        return acceFree


    def getMEVEStep(self):
        if self.precedVeh:
            acceDSM = self.CFModel.getAcceleration(self, self.precedVeh)
            acceFree = self.getFreeAcce()
            tempAcceleration = min(acceDSM, acceFree)
        else:
            tempAcceleration = self.getFreeAcce()

        acceleration= self.accelerationCst(tempAcceleration)
        # 先计算位置，再更新速度，这样位移的计算算的面积就是梯形的
        Vdis = self.velocity * self.dt
        Adis = acceleration * pow(acceleration, 2) /2
        self.xPos = self.xPos + Vdis + Adis

        tempVelocity = self.velocity + acceleration
        self.velocityCst(tempVelocity)
        

    def __str__(self) -> str:
        return '{}: xPos = {}; velocity = {}'.format(self.id, self.xPos, self.velocity)