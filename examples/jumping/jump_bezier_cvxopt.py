from cvxopt import solvers
import matplotlib.pyplot as plt
import numpy as np
import sys

from cvxopt import matrix

np.set_printoptions(threshold=sys.maxsize)

N = 20
total_time = 5.0
T = total_time/float(N)
print('T', T)
mass = 30.0
grav = 9.81
ang_inertia = 100.0

n_pos = 3  # x,y, pitch
n_vel = 3
n_states = n_pos + n_vel
n_vars = n_states
n_ampl = 8
n_vars_tot = n_vars*N+n_ampl

lower_states = [-1000.0]*n_states
# lower_ampl = [-10000.0, 0.0, -10000.0, 0.0, -10000.0, 0.0, -10000.0, 0.0]
lower_ampl = [-10000.0]*n_ampl
lower = np.hstack([lower_states, lower_ampl])

upper_states = [1000.0]*n_states
upper_ampl = [10000.0]*n_ampl
upper = np.hstack([upper_states, upper_ampl])

tol = 1e-3
z_init = 10.0
x_init = 1.0
x_final = 1.0
avg_vel_x = (x_final - x_init)/total_time
vel_z_init = 1.0

vel_z_final = -vel_z_init
# initial_state = [x_init, z_init, 0.0, avg_vel_x, vel_z_init, 0.0]
initial_state = [0.0]*n_states
# final_state = [x_final, z_init, 0.0, avg_vel_x, vel_z_final, 0.0]
final_state = [0.0]*n_states
lb_init = [initial_state[s] for s in range(0, n_vars)]
ub_init = [initial_state[s] for s in range(0, n_vars)]
lb = [lower_states[s] for i in range(1, N-1) for s in range(0, n_vars)]
ub = [upper_states[s] for i in range(1, N-1) for s in range(0, n_vars)]
lb_final = [final_state[s] for s in range(0, n_vars)]
ub_final = [final_state[s] for s in range(0, n_vars)]
lb_ampl = [lower_ampl[s] for s in range(0, n_ampl)]
ub_ampl = [upper_ampl[s] for s in range(0, n_ampl)]
lb = np.hstack([lb_init, lb, lb_final, lb_ampl])
ub = np.hstack([ub_init, ub, ub_final, ub_ampl])
print('lower bounds', lb)
print('upper bounds', ub)

base2hip_dist_x = 0.3
front_foot_0 = [x_init+base2hip_dist_x, 0.0]
front_foot_1 = [x_final+base2hip_dist_x, 0.0]

hind_foot_0 = [x_init-base2hip_dist_x, 0.0]
hind_foot_1 = [x_final-base2hip_dist_x, 0.0]


def minimize_vel(n_states, N, n_vars_tot):
    A = np.zeros([n_vars_tot, n_vars_tot])
    for i in range(0, N):
        A[i*n_states+3:(i+1)*n_states, i*n_states+3:(i+1)*n_states] = np.eye(3)
    return A


def quadratic_bezier_curve(t, p0, p1, p2):
    return (1.0-t)*((1.0-t)*p0 + t*p1) + t*((1.0-t)*p1+t*p2)


def bezier_curve(amplitude, time, td_time, lo_time):
    td_force = 0.0
    lo_force = 0.0
    mid_force = 1.0
    t = (time - td_time)/(lo_time-td_time)
    return quadratic_bezier_curve(t, td_force, mid_force, lo_force)


def compute_force_basis(i, lift_off_front, touch_down_front, lift_off_hind, touch_down_hind, tot):
    t = i*T
    ampl = 1.0
    if t < lift_off_front:
        ff0 = bezier_curve(ampl, t, 0.0, lift_off_front)
        ff1 = 0.0
    elif (t >= lift_off_front) and (t < touch_down_front):
        ff0 = 0.0
        ff1 = 0.0
    else:
        ff0 = 0.0
        ff1 = bezier_curve(ampl, t, touch_down_front, tot)

    if t < lift_off_hind:
        fh0 = bezier_curve(ampl, t, 0.0, lift_off_hind)
        fh1 = 0.0
    elif (t >= lift_off_hind) and (t < touch_down_hind):
        fh0 = 0.0
        fh1 = 0.0
    else:
        fh0 = 0.0
        fh1 = bezier_curve(ampl, t, touch_down_hind, tot)
    return [ff0, fh0, ff1, fh1]


def compute_force_matrix(i, lift_off_front, touch_down_front, lift_off_hind, touch_down_hind, tot):
    [ff0, fh0, ff1, fh1] = compute_force_basis(
        i, lift_off_front, touch_down_front, lift_off_hind, touch_down_hind, tot)
    A_force = [[-ff0, 0.0, -fh0, 0.0, -ff1, 0.0, -fh1, 0.0],
               [0.0, -ff0, 0.0, -fh0, 0.0, -ff1, 0.0, -fh1]]  # forces
    return A_force


def compute_torque_matrix(i, lift_off_front, touch_down_front, lift_off_hind, touch_down_hind, tot, front_foot, hind_foot):
    [pf_x, pf_z] = front_foot
    [ph_x, ph_z] = hind_foot
    [ff0, fh0, ff1, fh1] = compute_force_basis(
        i, lift_off_front, touch_down_front, lift_off_hind, touch_down_hind, tot)
    return [-pf_z*ff0, pf_x*ff0, -ph_z*fh0, ph_x*fh0, -pf_z*ff1, pf_x*ff1, -ph_z*fh1, ph_x*fh1]


# Linear constraints: lb <= A.dot(x) <= ub
# inequality constraints
bounds_constr = np.vstack([np.eye(n_vars_tot), 1.0*np.eye(n_vars_tot)])
bounds_b = np.hstack([ub, -lb])

print('bounds', bounds_constr)
print('b bounds', bounds_b)

# equality constraints
bc = np.zeros([2*n_pos, n_vars_tot])
bc[0:n_pos, 0:n_pos] = np.eye(n_pos)
bc[n_pos:2*n_pos, n_pos*(N-1):n_pos*N] = np.eye(n_pos)
bounds_eq_constr = bc
bounds_eq_b = np.hstack([0.0]*n_states)

print('bounds_eq_constr', bounds_eq_constr)
print('bounds_eq_b', bounds_eq_b)


# 2) dynamic constraint:
# f = m*a with a = (v_k - v_{k-1})/T
A_dyn_lin = np.zeros([2*N, n_vars_tot])
A_vel = np.zeros([2, 2*n_vars])
A_vel[0:2, 3:5] = -mass*np.eye(2)/T
A_vel[0:2, 3+n_vars:5+n_vars] = mass*np.eye(2)/T
# print('A vel', A_vel)

lift_off_front = total_time*1.0/5.0
touch_down_front = 3.0/5.0*total_time
lift_off_hind = total_time*2.0/5.0
touch_down_hind = 4.0/5.0*total_time

for i in range(0, N-1):
    A_force = compute_force_matrix(
        i, lift_off_front, touch_down_front, lift_off_hind, touch_down_hind, total_time)
    # print('A force', A_force)
    A_dyn_lin[i*2:i*2+2, n_states*N:n_vars_tot] = A_force
    A_dyn_lin[i*2:i*2+2, i*n_vars:(i+2)*n_vars] = A_vel

A_dyn_lin[(N-1)*2:(N-1)*2+2, n_states*N:n_vars_tot] = A_force
A_dyn_lin[(N-1)*2:(N-1)*2+2, (N-1)*n_vars:N*n_vars] = A_vel[:, 0:n_states]

# print('A dyn', A_dyn_lin)

dyn_constr_b = [0.0, -grav*mass]*(N)

# print('Dynamic bound', dyn_constr_b)

# angular part
A_dyn_ang = np.zeros([N, n_vars_tot])
A_ang_vel = np.zeros([1, 2*n_vars])
A_ang_vel[0, 2] = -ang_inertia/T
A_ang_vel[0, 2+n_states] = ang_inertia/T
for i in range(0, N-1):
    t = i*T
    if t < lift_off_front:
        p_front = front_foot_0
    elif (t >= lift_off_front) and (t < touch_down_front):
        p_front = [1000000.0, 1000000.0]
    else:
        p_front = front_foot_1

    if t < lift_off_hind:
        p_hind = hind_foot_0
    elif (t >= lift_off_hind) and (t < touch_down_hind):
        p_hind = [1000000.0, 1000000.0]
    else:
        p_hind = hind_foot_1

    A_tau = compute_torque_matrix(
        i, lift_off_front, touch_down_front, lift_off_hind, touch_down_hind, total_time, p_front, p_hind)
    # print('A angular dynamics', A_tau)
    A_dyn_ang[i, n_states*N:n_vars_tot] = A_tau
    A_dyn_ang[i, i*n_vars:(i+2)*n_vars] = A_ang_vel
    # print('A dyn ang', A_dyn_ang)

eps = [1.0e-6]*(N)
minus_eps = [-1.0e-6]*(N)

# 3) velocity constraint
A_vel = np.zeros([3*N, n_vars_tot])
A = np.zeros([3, 2*n_vars])
A[0:n_pos, 0:n_pos] = -np.eye(n_pos)/T
A[0:n_pos, n_vars:n_pos+n_vars] = np.eye(n_pos)/T
A[0:n_pos, n_pos+n_vars:n_pos + n_vars + n_vel] = -np.eye(n_pos)

for i in range(0, N-1):
    A_vel[i*n_pos:(i+1)*n_pos, i*n_vars:(i+2)*n_vars] = A

A_vel[(N-1)*n_pos:(N)*n_pos, (N-1)*n_vars:(N)*n_vars] = A[:, 0:n_states]


vel_b = [0.0]*(3*N)

print('A vel', A_vel)

# CVXOPT
n = n_vars_tot
Q = matrix(minimize_vel(n_states, N, n))
p = matrix(np.zeros([n, 1]))

G = matrix(bounds_constr)
q = matrix(bounds_b)

A_tot = np.vstack([bounds_eq_constr, A_dyn_lin, A_vel])
nc = np.shape(A_tot)[0]
A = matrix(A_tot, (nc, n))
b_tot = np.hstack([bounds_eq_b, dyn_constr_b, vel_b])
b = matrix(b_tot)

print('number of constraints', nc)
sol = solvers.qp(Q, p, A=A, b=b)

print("Solution:", sol['x'])
pos_x = [sol['x'][i*n_vars] for i in range(0, N)]
pos_y = [sol['x'][i*n_vars+1] for i in range(0, N)]
pitch = [sol['x'][i*n_vars+2] for i in range(0, N)]
vel_x = [sol['x'][i*n_vars+3] for i in range(0, N)]
vel_y = [sol['x'][i*n_vars+4] for i in range(0, N)]
pitch_d = [sol['x'][i*n_vars+5] for i in range(0, N)]
amplitudes = sol['x'][N*n_states:n_vars_tot]
print('pos_x', pos_x)
print('pos_y', pos_y)
print('pitch', pitch)
print('vel_x', vel_x)
print('vel_y', vel_y)
print('pitch_d', pitch_d)
print('Amplitudes', amplitudes)

f_x_front = [0.0]*N
f_y_front = [0.0]*N
f_x_hind = [0.0]*N
f_y_hind = [0.0]*N

for i in range(0, N):
    A_force = compute_force_matrix(
        i, lift_off_front, touch_down_front, lift_off_hind, touch_down_hind, total_time)
    f_x_front[i] = -A_force[0][0]*sol['x'][n_states*N] - \
        A_force[0][4]*sol['x'][n_states*N+4]
    f_y_front[i] = -A_force[1][1]*sol['x'][n_states*N+1] - \
        A_force[1][5]*sol['x'][n_states*N+5]
    f_x_hind[i] = -A_force[0][2]*sol['x'][n_states*N+2] - \
        A_force[0][6]*sol['x'][n_states*N+6]
    f_y_hind[i] = -A_force[1][3]*sol['x'][n_states*N+3] - \
        A_force[1][7]*sol['x'][n_states*N+7]

print('force front X', f_x_front)
print('force front Z', f_y_front)
print('force hind X', f_x_hind)
print('force hind Z', f_y_hind)

# check total momentum
Mx = (sum(f_x_front) + sum(f_x_hind))*T
print('total momentum along X', Mx)
Mz = (sum(f_y_front) + sum(f_y_hind))*T - mass*grav*total_time
print('total momentum along Z', Mz, 'Gravity component is', -mass*grav *
      total_time, 'Force component is', (sum(f_y_front) + sum(f_y_hind))*T)


fig, axs = plt.subplots(10)
fig.suptitle('Trajectories')

axs[0].plot(pos_x)
axs[1].plot(pos_y)
axs[2].plot(pitch)
axs[3].plot(vel_x)
axs[4].plot(vel_y)
axs[5].plot(pitch_d)
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
