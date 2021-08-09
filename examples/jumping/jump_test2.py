import numpy as np
import sys
from scipy.optimize import minimize, LinearConstraint, NonlinearConstraint
import matplotlib.pyplot as plt
np.set_printoptions(threshold=sys.maxsize)

N = 10
total_time = 2.0
T = total_time/float(N)

n_pos = 3  # x,y, pitch
n_vel = 3
n_states = n_pos + n_vel
n_forces = 4
n_vars = n_states + n_forces
n_vars_tot = n_vars*N
x0 = [0.0]*(n_vars_tot)
ub = [1e6]*(n_vars_tot)
lb = [-1e-6]*(n_vars_tot)

lower_states = [-10.0]*n_states
lower_forces = [-20.0]*n_forces
lower = np.hstack([lower_states, lower_forces])

upper_states = [10.0]*n_states
upper_forces = [20.0]*n_forces
upper = np.hstack([upper_states, upper_forces])

bounds = [(lower[s], upper[s]) for i in range(0, N) for s in range(0, n_vars)]


def objective_function(x):
    return np.sum(x)


def minimize_vel(x, w, N):
    cost = 0.0
    for i in range(0, N):
        cost += np.sum(x[i*10+3:i*10+6])
    return cost*w


# Linear constraints: lb <= A.dot(x) <= ub

# 1) boundary conditions
initial_state = [1.0]*n_pos
final_state = [5.0]*n_pos
boundary_constr = np.zeros([n_vars_tot, n_vars_tot])
boundary_constr[0:n_pos, 0:n_pos] = np.eye(n_pos)
boundary_constr[n_vars_tot - n_vars:n_vars_tot - n_vars + n_pos,
                n_vars_tot - n_vars:n_vars_tot - n_vars + n_pos] = np.eye(n_pos)
ub[0:n_pos] = initial_state
x0[0:n_pos] = initial_state
ub[n_vars_tot - n_vars:n_vars_tot - n_vars + n_pos] = final_state
x0[n_vars_tot - n_vars:n_vars_tot - n_vars + n_pos] = final_state
lb = [u - 1e-8 for u in ub]
print('initial state', np.shape(boundary_constr))
print('LB', len(lb))
print('UB', ub)
print('x0', len(x0))
initial_cond = LinearConstraint(boundary_constr, lb=lb, ub=ub)

# 2) dynamic constraint:
# f = m*a  with a = (v_k - v_{k-1})/T
A_dyn = np.zeros([2*N, n_vars_tot])
A = np.zeros([2, 2*n_vars])
mass = 1.0
A[0:2, 3:5] = -np.eye(2)*T*mass
A[0:2, 6+n_vars:10+n_vars] = [[1.0, 0.0, 1.0, 0.0], [0.0, 1.0, 0.0, 1.0]]  # forces
A[0:2, 3+n_vars:5+n_vars] = np.eye(2)*T*mass
for i in range(0, N-1):
    A_dyn[i*2:i*2+2, i*n_vars:(i+2)*n_vars] = A

eps = [1.0e-6]*(2*N)
minus_eps = [-1.0e-3]*(2*N)
dyn_constr = LinearConstraint(A_dyn, lb=minus_eps, ub=eps)


# 3) velocity constraint
A_vel = np.zeros([n_vars_tot, n_vars_tot])
A = np.zeros([n_vars, 2*n_vars])
A[0:n_pos, 0:n_pos] = -np.eye(n_pos)*T
A[0:n_pos, n_vars:n_pos+n_vars] = np.eye(n_pos)*T
A[0:n_vel, n_pos:n_pos + n_vel] = np.eye(n_pos)

for i in range(0, N-1):
    A_vel[i*n_vars:(i+1)*n_vars, i*n_vars:(i+2)*n_vars] = A

eps = [1.0e-6]*(n_vars_tot)
minus_eps = [-1.0e-3]*(n_vars_tot)
vel_constr = LinearConstraint(A_vel, lb=minus_eps, ub=eps)

res = minimize(
    minimize_vel,
    x0=x0,
    args=(1.0, N,),
    constraints={initial_cond, vel_constr, dyn_constr},
    bounds=bounds)

print("Solution:", res.x)
pos_x = [res.x[i*n_vars] for i in range(0, N)]
pos_y = [res.x[i*n_vars+1] for i in range(0, N)]
pitch = [res.x[i*n_vars+2] for i in range(0, N)]
vel_x = [res.x[i*n_vars+3] for i in range(0, N)]
vel_y = [res.x[i*n_vars+4] for i in range(0, N)]
pitch_d = [res.x[i*n_vars+5] for i in range(0, N)]
f_x_front = [res.x[i*n_vars+6] for i in range(0, N)]
f_y_front = [res.x[i*n_vars+7] for i in range(0, N)]
f_x_hind = [res.x[i*n_vars+8] for i in range(0, N)]
f_y_hind = [res.x[i*n_vars+9] for i in range(0, N)]

print('pos_x', pos_x)
print('pos_y', pos_y)
print('pitch', pitch)
print('vel_x', vel_x)
print('vel_y', vel_y)
print('pitch_d', pitch_d)
print('force front X', f_x_front)
print('force front Y', f_y_front)
print('force hind X', f_x_hind)
print('force hind Y', f_y_hind)


fig, axs = plt.subplots(10)
fig.suptitle('Trajectories')

axs[0].plot(pos_x)
axs[1].plot(pos_y)
axs[2].plot(pitch)
axs[3].plot(vel_x)
axs[4].plot(vel_y)
axs[5].plot(f_x_front)
axs[5].plot(f_x_front)
axs[5].plot(f_x_front)
axs[5].plot(f_x_front)

axs[6].plot(f_x_front)
axs[7].plot(f_y_front)
axs[8].plot(f_x_hind)
axs[9].plot(f_y_hind)

axs[0].grid()
axs[1].grid()
axs[2].grid()
axs[3].grid()
axs[4].grid()
axs[5].grid()
axs[6].grid()
axs[7].grid()
axs[8].grid()
axs[9].grid()

plt.show()
