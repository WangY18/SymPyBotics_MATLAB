import sympy
import math
import SymPyBotics.sympybotics as sympybotics

rbtdef = sympybotics.RobotDef('Example Robot', # robot name
                              [('-pi/2', 0, 0, 'q+pi/2'),  # list of tuples with  Denavit-Hartenberg parameters
                              ( 'pi/2', 0, 0, 'q-pi/2')], # (alpha, a, d, theta)
                              dh_convention='standard' # either 'standard' or 'modified'
                              )

rbtdef.frictionmodel = {'Coulomb', 'offset'} # options are None or a combination of 'Coulomb', 'viscous' and 'offset'
# rbtdef.gravityacc = sympy.Matrix([0.0, 0.0, -9.81]) # optional, this is the default value
print(rbtdef.dynparms())
rbt = sympybotics.RobotDynCode(rbtdef, verbose=True)
tau_str = sympybotics.robotcodegen.robot_code_to_func('matlab', rbt.invdyn_code, 'tau_out', 'tau', rbtdef)
print(tau_str)

rbt.calc_base_parms()
print(rbt.dyn.baseparms)

# H_code 表示 tau = Y * theta_b 的回归表达式
H_code = rbt.H_code      # 回归表达式中的 H(q,dq,ddq)
baseparms = rbt.dyn.baseparms  # θ_b

q_subs = {q: f'q[{i}]' for i, q in enumerate(rbt.rbtdef.q)}
q_subs.update({dq: f'dq[{i}]' for i, dq in enumerate(rbt.rbtdef.dq)})
q_subs.update({ddq: f'ddq[{i}]' for i, ddq in enumerate(rbt.rbtdef.ddq)})

from SymPyBotics.sympybotics.symcode import code_to_func

regressor_code = code_to_func(
    'python',
    rbt.H_code,
    'regressor',
    'regressor_func',
    ['q', 'dq', 'ddq'],
    q_subs
)

# 动态执行，构造 regressor_func 函数
exec(regressor_code, globals())
# we can use: regressor_func([1]*2,[1]*2,[1]*2)

## 收集真实数据：q, dq, ddq, tau
## 堆叠构建完整回归系统

# import numpy as np
#
# Y_total = []
# tau_total = []
#
# for q, dq, ddq, tau in zip(q_log, dq_log, ddq_log, tau_log):
#     Y = np.array(regressor_func(q, dq, ddq))  # Y: dof x n_baseparms
#     Y_total.append(Y)
#     tau_total.append(np.array(tau))
#
# Y_total = np.vstack(Y_total)      # shape: (N*dof, n_baseparms)
# tau_total = np.hstack(tau_total)  # shape: (N*dof,)

## 利用最小二乘法求解基参数
# from numpy.linalg import lstsq
#
# theta_hat, residuals, rank, s = lstsq(Y_total, tau_total, rcond=None)

## 可选：对比拟合效果
# tau_pred = Y_total @ theta_hat
# error = tau_total - tau_pred
# mse = np.mean(error**2)
# print('拟合误差 (MSE):', mse)
