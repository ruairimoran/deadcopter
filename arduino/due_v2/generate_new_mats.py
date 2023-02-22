import math

import numpy as np
import control as ctrl

# general params
number_of_motors = 4  # count
total_quad_mass = 2  # Kg
arm_length = 0.225  # mts
air_density = 1.225  # Kg/m^3
acc_due_to_g = 9.81  # m/s^2
moi_xx = 0.01788  # Kg.m^2
moi_yy = 0.03014  # Kg.m^2
moi_zz = 0.04614  # Kg.m^2

# motor params
motor_speed_const = 1000  # rpm/V
motor_time_const = 0.05  # s
rotor_mass = 0.04  # Kg
rotor_radius = 0.019  # mts
total_motor_mass = 0.112  # Kg

# propeller params
thrust_coefficient = 0.112  # no units
power_coefficient = 0.044  # no units
propeller_mass = 0.009  # Kg
inch_to_meter = 0.0254
propeller_diameter = 10 * inch_to_meter

vmax = 16.8
vmin = 15

motor_moi = rotor_mass * (rotor_radius ** 2)
prop_moi = (propeller_mass * propeller_diameter ** 2) / 12  # kg.m^2           # prop moment of inertia
hover_rps = np.sqrt((total_quad_mass * acc_due_to_g) / (
            number_of_motors * thrust_coefficient * air_density * (propeller_diameter ** 4)))
k1 = (vmax - vmin) * motor_speed_const / 60
k2 = 1 / motor_time_const
k3_x = (2 * hover_rps * thrust_coefficient * air_density * (propeller_diameter ** 4)
        * number_of_motors * arm_length) / ((2 ** 0.5) * moi_xx)
k3_y = (2 * hover_rps * thrust_coefficient * air_density * (propeller_diameter ** 4)
        * number_of_motors * arm_length) / ((2 ** 0.5) * moi_yy)
k3_z = (2 * hover_rps * power_coefficient * air_density * (propeller_diameter ** 5)
        * number_of_motors) / (2 * math.pi * moi_zz)
k4_x = 0
k4_y = 0
k4_z = 2*math.pi*number_of_motors*(prop_moi + motor_moi)/moi_zz

gamma_n = np.diagflat([k3_x, k3_y, -k4_z*k2 + k3_z])
gamma_u = np.diagflat([0, 0, k4_z*k2*k1])

A = np.concatenate([
    np.concatenate([np.zeros((3,3)), 0.5*np.eye(3, 3), np.zeros((3, 3))], axis=1),  # how is it 0.5*I? it should be zero for linearised model, see eq 2.22
    np.concatenate([np.zeros((3, 3)), np.zeros((3, 3)), gamma_n], axis=1),
    np.concatenate([np.zeros((3, 3)), np.zeros((3, 3)), -k2*np.eye(3, 3)], axis=1)
], axis=0)
B = np.concatenate([
    np.zeros((3, 3)),
    gamma_u,
    k2*k1*np.eye(3, 3)
], axis=0)
C = np.concatenate([
    np.eye(6, 6),
    np.zeros((6, 3))
], axis=1)
D = np.zeros((6, 3))

state_space = ctrl.ss(A, B, C, D)
discretization_sampling_time = 1/125

def controllability(a, b, n):
    ctrb_rank = np.linalg.matrix_rank(ctrl.ctrb(a, b))
    if ctrb_rank < n:
        raise Exception(f"System not controllable. Ctrb Matrix Rank ({ctrb_rank}) < States ({n})")


def observability(a, c, n):
    obsv_rank = np.linalg.matrix_rank(ctrl.obsv(a, c))
    if obsv_rank < n:
        raise Exception(f"System not observable. Obsv Matrix Rank ({obsv_rank}) < Measured States ({n})")


def discretize(A, B, C, D):
    return ctrl.c2d(state_space, discretization_sampling_time)


def get_LQR_gain_K(A, B):
    Q_lqr = np.diagflat([2000, 2000, 10, 2, 2, 10, 0, 0, 0])
    R_lqr = np.diagflat([1, 1, 10])
    solution_P_lqr, eigenvalues_cl_lqr, negative_gain_K_lqr = ctrl.dare(A, B, Q_lqr, R_lqr)
    return -negative_gain_K_lqr


def get_kalman_gain_L(A, C):
    Q_Kf = np.diagflat([1, 1, 1, 1, 1, 1, 500, 500, 500])
    R_Kf = 2 * np.diagflat([1, 1, 1, 1, 1, 1])
    solution_P_Kf, eigenvalues_cl_Kf, negative_gain_L_Kf = ctrl.dare(A.T, C.T, Q_Kf, R_Kf)
    return -negative_gain_L_Kf.T

controllability(A, B, A.shape[0])
observability(A, C, C.shape[0])



discrete_system = discretize(A, B, C, D)
Ad = discrete_system.A
Bd = discrete_system.B
Cd = discrete_system.C
Dd = discrete_system.D
K = get_LQR_gain_K(Ad, Bd)
L = get_kalman_gain_L(Ad, Cd)
np.set_printoptions(linewidth=np.inf)


ip_to_mot = np.array(
    [
        [1.0, 1.0, 1.0, 1.0],
        [1.0, -1.0, 1.0, -1.0],
        [1.0, 1.0, -1.0, -1.0],
        [1.0, -1.0, -1.0, 1.0]
    ]
)

with open("ss_and_mats.h", 'w') as fp:
    print(f"const float Ad[{Ad.shape[0]}][{Ad.shape[1]}] = \n" + np.array2string(Ad, separator=", ").replace('[', '{').replace(']', '}') + ";\n", file=fp)
    print(f"const float Bd[{Bd.shape[0]}][{Bd.shape[1]}] = \n" + np.array2string(Bd, separator=", ").replace('[', '{').replace(']', '}') + ";\n", file=fp)
    print(f"const float Cd[{Cd.shape[0]}][{Cd.shape[1]}] = \n" + np.array2string(Cd, separator=", ").replace('[', '{').replace(']', '}') + ";\n", file=fp)
    print(f"const float Dd[{Dd.shape[0]}][{Dd.shape[1]}] = \n" + np.array2string(Dd, separator=", ").replace('[', '{').replace(']', '}') + ";\n", file=fp)
    print(f"const float K[{K.shape[0]}][{K.shape[1]}] = \n" + np.array2string(K, separator=", ").replace('[', '{').replace(']', '}') + ";\n", file=fp)
    print(f"const float L[{L.shape[0]}][{L.shape[1]}] = \n" + np.array2string(L, separator=", ").replace('[', '{').replace(']', '}') + ";\n", file=fp)
    print(f"const float ip_to_mot[{ip_to_mot.shape[0]}][{ip_to_mot.shape[1]}] = \n" + np.array2string(ip_to_mot, separator=", ").replace('[', '{').replace(']', '}') + ";\n", file=fp)


