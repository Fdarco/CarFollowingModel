import math
from vehicle import vehicle


class LC_tools:
    @staticmethod
    def getS_v(Vego: vehicle, Obs_af: vehicle) -> float:
        v_rel = Vego.velocity - Obs_af.velocity
        return (Vego.tau - Vego.c_v * v_rel) * Vego.velocity


    @staticmethod
    def getDelta(Vego: vehicle, Obs_af: vehicle) -> float:
        S_v = LC_tools.getS_v(Vego, Obs_af)
        v_rel = Obs_af.velocity - Vego.velocity
        x_af = Obs_af.xPos - Vego.xPos
        return 8 * math.pow(v_rel, 2) + 16 * Vego.acce * (x_af - S_v)


    @staticmethod
    def getT(Vego: vehicle, Obs_af: vehicle):
        Delta = LC_tools.getDelta(Vego, Obs_af)
        if Delta < 0:
            return False
        else:
            T_1 = (Vego.V_max - Vego.velocity) / Vego.acce
            T_2 = (Vego.V_max - Obs_af.velocity) / Vego.acce
            return T_1, T_2


    @staticmethod
    def getT_3(Vego: vehicle, Obs_af: vehicle, v_lim):
        x_af = Obs_af.xPos - Vego.xPos
        v_rel = Obs_af.velocity - Vego.velocity
        v_diff = v_lim - Obs_af.velocity
        S_v = LC_tools.getS_v(Vego, Obs_af)
        res = (2 * math.pow(v_diff, 2) - math.pow(v_rel, 2)) / (2 * Vego.acce)
        return (x_af - S_v - res) / v_diff


    @staticmethod
    def getc_k(veh: vehicle):
        return (veh.V_des - veh.velocity) / veh.V_des


    # t: t_0 < t < t_1
    # t_diff: t_1 - t
    @staticmethod
    def getMSS(ego: vehicle, Obs_af: vehicle, t_diff: int):
        v_rel_re = ego.velocity - Obs_af.velocity
        S_v = LC_tools.getS_v(ego, Obs_af)
        return v_rel_re * t_diff + 0.5 * ego.acce * math.pow(t_diff, 2) + ego.width * ego.theta + S_v
        

def main():
    ego = vehicle('veh_1', 'real', 100, 1.5, 27.78)
    Obs_cf = vehicle('veh_2', 'real', 180, 1.5, 20)
    Obs_af = vehicle('veh_3', 'real', 125, 4.5, 25)
    adj_fol = vehicle('veh_4', 'real', 30, 4.5, 25)
    timestep = 0
    while True:
        Obs_af.run()
        Obs_cf.run()
        adj_fol.run()
        ego.C += LC_tools.getc_k(ego)
        if ego.C > ego.C_thr:
            Vego = vehicle('Vego', 'virtual', ego.xPos, 4.5, ego.velocity)
            if LC_tools.getT(Vego, Obs_af):
                T1, T2 = LC_tools.getT(Vego, Obs_af)
                C_vir = 0
                for _ in range(int(T1/Vego.dt)):
                    Vego.velocity += Vego.acce * Vego.dt
                    C_vir += LC_tools.getc_k(Vego)

        timestep += 1