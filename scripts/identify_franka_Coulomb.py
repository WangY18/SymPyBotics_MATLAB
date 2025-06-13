import SymPyBotics.sympybotics as sympybotics
import numpy as np
import sympy
from SymPyBotics.sympybotics.symcode import code_to_func
import math
import matplotlib.pyplot as plt

rbtdef = sympybotics.RobotDef('Franka Robot', # robot name
                              [(0, 0, 0.333, 'q'),
                              ( '-pi/2', 0, 0, 'q'),
                              ( 'pi/2', 0, 0.316, 'q'),
                              ( 'pi/2', 0.0825, 0, 'q'),
                              ( '-pi/2', -0.0825, 0.384, 'q'),
                              ( 'pi/2', 0, 0, 'q'),
                              ( 'pi/2', 0.088, 0, 'q')], # list of tuples with  Denavit-Hartenberg parameters# (alpha, a, d, theta)
                              dh_convention='modified' # either 'standard' or 'modified'
                              )

rbtdef.frictionmodel = {'Coulomb', 'offset'} # options are None or a combination of 'Coulomb', 'viscous' and 'offset'
rbtdef.gravityacc = sympy.Matrix([0.0, 0.0, -9.81]) # optional, this is the default value
# print(rbtdef.dynparms())
rbt = sympybotics.RobotDynCode(rbtdef, verbose=True)
print(rbt.g_code)

rbt.calc_base_parms()

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

print("Begin reading data...")

## Collect Real-World Data: q, dq, ddq, tau
import pandas as pd
data = pd.read_csv(r"..\data\Exp_20250612\feedback_filter.csv")
# q_log = [[float(data[f'q{i+1}'][j]) for i in range(7)] for j in range(100)]
# dq_log = [[float(data[f'dq{i+1}'][j]) for i in range(7)] for j in range(100)]
# ddq_log = [[float(data[f'ddq{i+1}'][j]) for i in range(7)] for j in range(100)]
# tau_log = [[float(data[f'torque{i+1}'][j]) for i in range(7)] for j in range(100)]
q_log = [[float(data[f'q{i+1}'][j]) for i in range(7)] for j in range(len(data['q1']))]
dq_log = [[float(data[f'dq{i+1}'][j]) for i in range(7)] for j in range(len(data['q1']))]
ddq_log = [[float(data[f'ddq{i+1}'][j]) for i in range(7)] for j in range(len(data['q1']))]
tau_log = [[float(data[f'torque{i+1}'][j]) for i in range(7)] for j in range(len(data['q1']))]

print("Data reading complete.")

## Construct the regressor matrix Y and the torque vector tau

Y_total = []
tau_total = []

for q, dq, ddq, tau in zip(q_log, dq_log, ddq_log, tau_log):
    Y = np.array(regressorb_func(q, dq, ddq))  # Y: dof x n_baseparms
    Y = Y.reshape(7, -1)
    Y_total.append(Y)
    tau_total.append(np.array(tau))

Y_total = np.vstack(Y_total)      # shape: (N*dof, n_baseparms)
tau_total = np.hstack(tau_total)  # shape: (N*dof,)


print("Regressor matrix and torque vector constructed.")

## Regression to find base parameters based on least squares method. theta_hat is the estimated rbt.dyn.baseparms
theta_hat, residuals, rank, s = np.linalg.lstsq(Y_total, tau_total, rcond=None)

print("Least squares regression complete.")

## Optional: compare the MSE error
tau_pred = Y_total @ theta_hat
error = tau_total - tau_pred
mse = np.mean(error**2)
print('MSE error:', mse)

## Now we can obtain the matlab code for the inverse dynamics function
param_values = {sym: theta_hat[i] for i, sym in enumerate(rbt.dyn.baseparms)}
subexprs, exprs = rbt.invdyn_code
new_subexprs = [(s, e.subs(param_values)) for s, e in subexprs]
new_exprs = exprs.subs(param_values)
invdyn_code_param_subs = (new_subexprs, new_exprs)


## Output the code for the regressor and torque function
subs_dict = {
    **{q: f'q[{i}]' for i, q in enumerate(rbt.rbtdef.q)},
    **{dq: f'dq[{i}]' for i, dq in enumerate(rbt.rbtdef.dq)},
    **{ddq: f'ddq[{i}]' for i, ddq in enumerate(rbt.rbtdef.ddq)},
}

Regressorb_func_str = code_to_func(
    'matlab',
    rbt.Hb_code,
    'Y',
    'get_Regressorb',
    ['q', 'dq', 'ddq'],
    subs_dict
)

import re
id = Regressorb_func_str.find(r"Y(:,")
Regressorb_func_str = Regressorb_func_str[:id] + \
    "Y = zeros([size(q,1), 357]);\n    " + \
    Regressorb_func_str[id:]
Regressorb_func_str = re.sub(r'^\s*Y\(:,.*?\)\s*=\s*0;\s*\n', '', Regressorb_func_str, flags=re.MULTILINE)

tau_func_str = \
    "function torque = get_Torque(q, dq, ddq)\n" + \
    f"    theta = {theta_hat.tolist()}';\n" + \
    "    Y = get_Regressorb(q, dq, ddq);\n" + \
    "    torque = [" + \
    ", ".join([f"Y(:,{i*rbt.dyn.n_base+1}:{(i+1)*rbt.dyn.n_base}) * theta" for i in range(7)]) + \
    "];\nend\n"

# tau_func_str = code_to_func(
#     'matlab',
#     invdyn_code_param_subs,
#     'tau',
#     'get_Torque',
#     ['q', 'dq', 'ddq'],
#     subs_dict
# )

print("Matlab code for torque function generated.")

with open(r"../results/dyn_franka_Coulomb\get_Regressorb.m", "w", encoding="utf-8") as f:
    f.write(Regressorb_func_str)

with open(r"../results/dyn_franka_Coulomb\get_Torque.m", "w", encoding="utf-8") as f:
    f.write(tau_func_str)