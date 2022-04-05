# created at: [2022-04-02 11:29]
# fudaocheng@pjlab.org.cn 
# DFI: discrete following interval


import math
import numpy as np

# h_c0: 
class DFI:
    def __init__(
        self, h_c0 = 2, t_0 = 1
        ) -> None:
        self.h_c0 = h_c0
        self.t_0 = t_0


    def getHeadwayBound(self):
        raise NotImplementedError