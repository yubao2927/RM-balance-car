# -*- coding: utf-8 -*-
"""
LQR 增益离线设计脚本（两轮自平衡小车）

用途：
  1. 按小车的物理参数建立"直立平衡点"附近的线性化状态空间模型；
  2. 用离散 LQR（里卡提迭代）离线解出状态反馈增益 K；
  3. 把 K 换算成固件里可直接使用的增益 LQR_K0..LQR_K4，并做稳定性检查。

状态量（固件与脚本一致）：
  x = [倾角θ(rad), 角速度θ̇(rad/s), 车轮位移x(m), 车速ẋ(m/s), 位置积分∫x dt]
输入：两轮合力矩 T (N·m)
控制律：T = -K·x ，固件里 ctrl_sum = LQR_K0*angle + ... + LQR_K4*pos_int

用法：
  python lqr_design.py
改完参数重跑，把打印出来的 LQR_K0..LQR_K4 抄进平衡车代码.ino 顶部。
"""

import numpy as np

# ---------------- 机械参数（典型值，务必按实车测量后修改） ----------------
M_BODY   = 0.50    # 车身(不含轮)质量 kg
L_COM    = 0.08    # 轮轴到车身质心的距离 m
I_BODY   = M_BODY * L_COM**2 / 3.0   # 车身绕质心转动惯量 kg·m²（近似均匀杆）
MW       = 0.03    # 单个车轮质量 kg
R_WHEEL  = 0.033   # 车轮半径 m
IW       = 0.5 * MW * R_WHEEL**2     # 单轮转动惯量（近似实心圆盘）
GRAV     = 9.81

# ---------------- 电机/编码器参数（典型值，务必按实车测量后修改） ----------------
KT_MOTOR = 0.025   # 电机转矩常数 N·m/A（堵转转矩/堵转电流）
RM_MOTOR = 2.0     # 电机电枢电阻 Ω
GEAR_RATIO = 30.0  # 减速比
V_BAT    = 7.4     # 电机供电电压 V
PWM_MAX  = 255.0
PPR      = 13.0    # 电机每转霍尔脉冲数
EDGES    = 2.0     # CHANGE 中断=每个脉冲计 2 个边沿
TS       = 0.005   # 控制周期 5ms，与固件 MsTimer2 一致

# ---------------- LQR 权重矩阵（先调这里，比改模型简单） ----------------
Q_THETA  = 8000.0  # 惩罚倾角（rad）
Q_OMEGA  = 60.0    # 惩罚角速度
Q_X      = 2.0     # 惩罚位置
Q_V      = 6.0     # 惩罚速度
Q_XI     = 6.0     # 惩罚位置误差积分（决定回中/前进响应）
R_U      = 1.0     # 惩罚输入力矩

# ======================= 模型与求解 =======================
# 化简后的线性化模型（θ=0 附近，状态 [θ, θ̇, x, ẋ]）：
#   (m+M_eff)·ẍ + m·l·θ̈ = T/R
#   m·l·ẍ + (I+m·l²)·θ̈ - m·g·l·θ = -T
M_EFF = 2.0 * MW + 2.0 * IW / R_WHEEL**2   # 轮子折算到平动的等效质量
a1 = M_BODY + M_EFF
b1 = M_BODY * L_COM
c1 = I_BODY + M_BODY * L_COM**2
d1 = M_BODY * GRAV * L_COM
det = a1 * c1 - b1 * b1

A = np.zeros((4, 4))
A[0, 1] = 1.0
A[1, 0] = a1 * d1 / det
A[2, 3] = 1.0
A[3, 0] = -b1 * d1 / det
B = np.zeros((4, 1))
B[1, 0] = -(b1 / R_WHEEL + a1) / det
B[3, 0] = (c1 / R_WHEEL + b1) / det

# 增广位置积分状态 xi̇ = x （与固件 pos_int += positions*dt 一致）
Aa = np.zeros((5, 5))
Aa[:4, :4] = A
Aa[4, 2] = 1.0
Ba = np.zeros((5, 1))
Ba[:4, 0] = B[:, 0]

# 离散化（5ms，二阶泰勒近似）
Ad = np.eye(5) + Aa * TS + (Aa @ Aa) * (TS * TS / 2.0)
Bd = Ba * TS

Q = np.diag([Q_THETA, Q_OMEGA, Q_X, Q_V, Q_XI])
Rm = np.array([[R_U]])

# 离散代数里卡提方程迭代求解
P = Q.copy()
for _ in range(2000):
    S = Rm + Bd.T @ P @ Bd
    Pn = Q + Ad.T @ P @ Ad - Ad.T @ P @ Bd @ np.linalg.inv(S) @ Bd.T @ P @ Ad
    if np.max(np.abs(Pn - P)) < 1e-12:
        P = Pn
        break
    P = Pn

S = Rm + Bd.T @ P @ Bd
K = np.linalg.inv(S) @ Bd.T @ P @ Ad   # T = -K·x

# ---------------- 换算到固件单位 ----------------
counts_per_wheel_rev = PPR * EDGES * GEAR_RATIO
m_per_pulse = 2.0 * np.pi * R_WHEEL / counts_per_wheel_rev
pwm_per_nm = PWM_MAX * RM_MOTOR / (KT_MOTOR * GEAR_RATIO * V_BAT)

# 固件里各状态单位：angle=度, angle_speed=度/s, positions=脉冲,
# speeds_filter=脉冲/40ms, pos_int=脉冲·s
# 注意：本车固件中 positions/speeds_filter 与模型位移方向相反
# （前进时位置积分为负积累，与串级PID外环 -ki*positions 一致），
# 因此位置/速度/积分的换算带上负号。
scale = np.array([
    np.pi / 180.0,              # θ rad/度
    np.pi / 180.0,              # ω (rad/s)/(度/s)
    -m_per_pulse,               # x m/脉冲（方向翻转）
    -m_per_pulse / 0.04,        # v (m/s)/(脉冲/40ms)（方向翻转）
    -m_per_pulse,               # xi (m·s)/(脉冲·s)（方向翻转）
])

# 固件约定 ctrl_sum = LQR_K·x_固件 与串级PID的 PD_pwm+PI_pwm 同号
# （倾角为正时 ctrl_sum 为正），因此 LQR_K = -K * scale * pwm_per_nm
lqr_k = -K.flatten() * scale * pwm_per_nm

# ---------------- 结果输出 ----------------
eigv = np.linalg.eigvals(Ad - Bd @ K)
print("模型 det = %.6f" % det)
print("连续开环极点: ", np.round(np.linalg.eigvals(Aa), 4))
print("离散闭环极点模长: ", np.round(np.abs(eigv), 5))
if np.max(np.abs(eigv)) < 1.0:
    print("闭环稳定: OK")
else:
    print("警告: 闭环不稳定，请加大 Q_THETA 或 Q_XI")
print("K(力矩, SI单位) =", np.round(K.flatten(), 5))
print("m_per_pulse = %.8f m, pwm_per_nm = %.3f PWM/(N·m)" % (m_per_pulse, pwm_per_nm))
print()
print("固件增益（直接复制到 .ino）:")
for i, name in enumerate(["LQR_K0(角度)", "LQR_K1(角速度)", "LQR_K2(位置)",
                          "LQR_K3(速度)", "LQR_K4(位置积分)"]):
    print("const float %s = %.6f;" % (name.replace("(", "").replace(")", ""), lqr_k[i]))
print()
print("期望符号检查: K0、K1 应 >0；K2、K3、K4 应 <0（与串级PID外环 -ki*positions 同号）")

# ---------------- 闭环仿真自检（连续模型，RK1 近似足够看趋势） ----------------
def sim(x0, t=5.0):
    x = np.array(x0, float)
    out = [x.copy()]
    for _ in range(int(t / TS)):
        u = float(-K.flatten() @ x)
        x = x + (Aa @ x + Ba[:, 0] * u) * TS
        out.append(x.copy())
    return np.array(out)

checks = [("倾斜0.1rad", [0.1, 0, 0, 0, 0]),
          ("位置0.1m", [0, 0, 0.1, 0, 0]),
          ("速度0.5m/s", [0, 0, 0, 0.5, 0])]
print()
for name, x0 in checks:
    o = sim(x0)
    print("%s -> 终值 %s, 最大|倾角| %.4f rad" %
          (name, np.round(o[-1], 4), float(np.max(np.abs(o[:, 0])))))
