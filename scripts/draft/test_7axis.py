import sympy
import math
import SymPyBotics.sympybotics as sympybotics
import numpy as np
from SymPyBotics.sympybotics.symcode import code_to_func
from SymPyBotics.sympybotics._compatibility_ import exec_


rbtdef = sympybotics.RobotDef('Franka Robot', # robot name
                              [(0, 0, 0.333, 'q+0'),
                              ( '-pi/2', 0, 0, 'q+0'),
                              ( 'pi/2', 0, 0.316, 'q+0'),
                              ( 'pi/2', 0.0825, 0, 'q+0'),
                              ( '-pi/2', -0.0825, 0.384, 'q+0'),
                              ( 'pi/2', 0, 0, 'q+0'),
                              ( 'pi/2', 0.088, 0, 'q+0')], # list of tuples with  Denavit-Hartenberg parameters# (alpha, a, d, theta)
                              dh_convention='standard' # either 'standard' or 'modified'
                              )

rbtdef.frictionmodel = {'Coulomb', 'offset'} # options are None or a combination of 'Coulomb', 'viscous' and 'offset'
# rbtdef.gravityacc = sympy.Matrix([0.0, 0.0, -9.81]) # optional, this is the default value
print(rbtdef.dynparms())
rbt = sympybotics.RobotDynCode(rbtdef, verbose=True)

# Dynamic code generation
tau_str = sympybotics.robotcodegen.robot_code_to_func('matlab', rbt.invdyn_code, 'tau_out', 'tau', rbtdef)
print(tau_str)

rbt.calc_base_parms()
print(rbt.dyn.baseparms)

# H_code: tau = Y * theta_b
H_code = rbt.H_code      # H(q,dq,ddq) in the regression expression
baseparms = rbt.dyn.baseparms  # theta_b

q_subs = {q: f'q[{i}]' for i, q in enumerate(rbt.rbtdef.q)}
q_subs.update({dq: f'dq[{i}]' for i, dq in enumerate(rbt.rbtdef.dq)})
q_subs.update({ddq: f'ddq[{i}]' for i, ddq in enumerate(rbt.rbtdef.ddq)})



regressorb_code = code_to_func(
    'python',
    rbt.Hb_code,
    'regressor',
    'regressorb_func',
    ['q', 'dq', 'ddq'],
    q_subs
)

exec(regressorb_code, globals())
# we can use: regressor_func([1]*7,[1]*7,[1]*7) -> (the Y_matrix)

## 收集真实数据：q, dq, ddq, tau
# 先假定一个数据测试一波
# 1. 替换符号为变量名字符串（q[0]、dq[1]等）
# 1. 获取 invdyn_code 中的符号集合
# all_symbols = rbt.invdyn_code[1].free_symbols  # 只取最终表达式中用到的符号（可加上中间变量定义里的）
# for i in range(len(rbt.invdyn_code[0])):
#     all_symbols = all_symbols | set(rbt.invdyn_code[0][i][1].free_symbols)  # 加上中间变量定义中的符号
# # 2. 定义通用变量集合
# common_syms = set(rbt.rbtdef.q)|set(rbt.rbtdef.dq)|set(rbt.rbtdef.ddq)
# # 获取中间变量定义中的所有变量名（如 x0, x1, ..., xn）
# intermediate_syms = set(var for var, _ in rbt.invdyn_code[0])
# # 3. 获取额外动力学参数符号
# dyn_symbols = sorted(all_symbols - common_syms - intermediate_syms, key=lambda x: str(x))  # 排除掉通用变量
# # [L_1zz, L_2yy, L_3xx, L_3xy, L_3xz, L_3yy, L_3yz, L_3zz, L_4xx, L_4xy, L_4xz, L_4yy, L_4yz, L_4zz, L_5xx, L_5xy, L_5xz, L_5yy, L_5yz, L_5zz, L_6xx, L_6xy, L_6xz, L_6yy, L_6yz, L_6zz, L_7xx, L_7xy, L_7xz, L_7yy, L_7yz, L_7zz, fc_1, fc_2, fc_3, fc_4, fc_5, fc_6, fc_7, fo_1, fo_2, fo_3, fo_4, fo_5, fo_6, fo_7, l_3x, l_3y, l_3z, l_4x, l_4y, l_4z, l_5x, l_5y, l_5z, l_6x, l_6y, l_6z, l_7x, l_7y, l_7z, m_3, m_4, m_5, m_6, m_7]
# 构造参数替换表
param_values = {sym: 1.0+0.1*i for i, sym in enumerate(rbt.dyn.dynparms)}
subexprs, exprs = rbt.invdyn_code
new_subexprs = [(s, e.subs(param_values)) for s, e in subexprs]
new_exprs = exprs.subs(param_values)
# 构造替换参数后的新代码对象
invdyn_code_param_subs = (new_subexprs, new_exprs)

subs_dict = {
    **{q: f'q[{i}]' for i, q in enumerate(rbt.rbtdef.q)},
    **{dq: f'dq[{i}]' for i, dq in enumerate(rbt.rbtdef.dq)},
    **{ddq: f'ddq[{i}]' for i, ddq in enumerate(rbt.rbtdef.ddq)},
}

# 2. 使用 invdyn_code 生成 Python 函数字符串
tau_func_str = code_to_func(
    'python',
    invdyn_code_param_subs,     # 这个就是 τ 表达式
    'tau',               # 输出名
    'tau_func',          # 函数名
    ['q', 'dq', 'ddq'],  # 输入参数列表
    subs_dict            # 符号替换表
)
# 创建 tau_func 函数
global_dict = {'numpy': np}
local_dict = {}

import math
global sin, cos, sign
sin = math.sin
cos = math.cos
sign = np.sign

# exec_(func_def_regressor, globals(), l)
exec_(tau_func_str, {'math': math, 'sin': sin, 'cos': cos, 'sign': sign}, local_dict)

# exec(tau_func_str, global_dict, local_dict)
tau_func = local_dict['tau_func']
# q_val   = np.array([0.1, 0.2, 0.3, 0.1, 0.0, 0.0, 0.0])
# dq_val  = np.zeros(7)
# ddq_val = np.zeros(7)
#
# tau_val = np.array(tau_func(q_val, dq_val, ddq_val)).tolist()
# print('tau =', tau_val)

# 计算每一组的 tau 并保存
N = 1000
q_log = np.random.rand(N, 7) * 2 * np.pi - np.pi
dq_log = np.random.rand(N, 7) * 2 - 1
ddq_log = np.random.rand(N, 7) * 2 - 1
q_log = q_log.tolist()
dq_log = dq_log.tolist()
ddq_log = ddq_log.tolist()
tau_log = []
for q, dq, ddq in zip(q_log, dq_log, ddq_log):
    tau_log.append(np.array(tau_func(q, dq, ddq)).tolist())


## 堆叠构建完整回归系统

Y_total = []
tau_total = []

for q, dq, ddq, tau in zip(q_log, dq_log, ddq_log, tau_log):
    Y = np.array(regressorb_func(q, dq, ddq))  # Y: dof x n_baseparms
    Y = Y.reshape(7, -1)
    Y_total.append(Y)
    tau_total.append(np.array(tau))

Y_total = np.vstack(Y_total)      # shape: (N*dof, n_baseparms)
tau_total = np.hstack(tau_total)  # shape: (N*dof,)

## 利用最小二乘法求解基参数
theta_hat, residuals, rank, s = np.linalg.lstsq(Y_total, tau_total, rcond=None)

theta_hat = theta_hat.tolist()
theta_real = [k.subs(param_values) for k in rbt.dyn.baseparms]

## 可选：对比拟合效果
print(np.array([[a,b] for a, b in zip(theta_hat, theta_real)]))

tau_pred = Y_total @ theta_hat
error = tau_total - tau_pred
mse = np.mean(error**2)
print('拟合误差 (MSE):', mse)




# 使用 invdyn_code 生成 Python 函数字符串
tau_func_str = code_to_func(
    'matlab',
    invdyn_code_param_subs,     # 这个就是 τ 表达式
    'tau',               # 输出名
    'get_Torque',          # 函数名
    ['q', 'dq', 'ddq'],  # 输入参数列表
    subs_dict            # 符号替换表
)

with open(r"../../results/dyn_franka_Coulomb/get_Torque_test.m", "w", encoding="utf-8") as f:
    f.write(tau_func_str)
