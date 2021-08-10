import numpy as np
import sys
from scipy.optimize import minimize, LinearConstraint, NonlinearConstraint
import matplotlib.pyplot as plt
np.set_printoptions(threshold=sys.maxsize)

N = 20
total_time = 5.0
T = total_time/float(N)
mass = 300.0
grav = 9.81

n_pos = 3  # x,y, pitch
n_vel = 3
n_states = n_pos + n_vel
n_vars = n_states
n_ampl = 4
n_vars_tot = n_vars*N+n_ampl

lower_states = [-100.0]*n_states
lower_ampl = [-1000.0, 0.0, -1000.0, 0.0]
lower = np.hstack([lower_states, lower_ampl])

upper_states = [100.0]*n_states
upper_ampl = [1000.0]*n_ampl
upper = np.hstack([upper_states, upper_ampl])

tol = 1e-6
x_init = 0.0
x_final = 10.0
avg_vel_x = (x_final - x_init)/total_time
initial_state = [x_init, 0.0, 0.0, avg_vel_x, 0.0, 0.0]
final_state = [x_final, 0.0, 0.0, avg_vel_x, 0.0, 0.0]
bounds_init = [(initial_state[s]-tol, initial_state[s]+tol)
               for s in range(0, n_vars)]
bounds = [(lower_states[s], upper_states[s]) for i in range(1, N-1)
          for s in range(0, n_vars)]
bounds_final = [(final_state[s] - tol, final_state[s]+tol)
                for s in range(0, n_vars)]
bounds_ampl = [(lower_ampl[s], upper_ampl[s])
               for s in range(0, n_ampl)]
bounds = np.vstack([bounds_init, bounds, bounds_final, bounds_ampl])
print(len(bounds))


def objective_function(x):
    return np.sum(x)


def minimize_vel(x, w, N):
    cost = 0.0
    for i in range(0, N):
        cost += np.sum(x[i*10+3:i*10+6])
    return cost*w


def quadratic_bezier_curve(t, p0, p1, p2):
    return (1.0-t)*((1.0-t)*p0 + t*p1) + t*((1.0-t)*p1+t*p2)


def bezier_curve(amplitude, time, td_time, lo_time):
    td_force = 0.0
    lo_force = 0.0
    mid_force = 1.0
    t = (time - td_time)/(lo_time-td_time)
    return quadratic_bezier_curve(t, td_force, mid_force, lo_force)


def compute_force_matrix(i, lift_off_front, touch_down_front, lift_off_hind, touch_down_hind, tot):
    t = i*T
    if t < lift_off_front:
        ff = bezier_curve(ampl, t, 0.0, lift_off_front)
    elif (t >= lift_off_front) and (t <= touch_down_front):
        ff = 0.0
    else:
        ff = bezier_curve(ampl, t, touch_down_front, tot)

    if t < lift_off_hind:
        fh = bezier_curve(ampl, t, 0.0, lift_off_hind)
    elif (t >= lift_off_hind) and (t <= touch_down_hind):
        fh = 0.0
    else:
        fh = bezier_curve(ampl, t, touch_down_hind, tot)

    A_force = [[-ff, 0.0, -fh, 0.0],
               [0.0, -ff, 0.0, -fh]]  # forces
    return A_force


# Linear constraints: lb <= A.dot(x) <= ub

x0 = [((final_state[s]*n*T + (N-n)*T*initial_state[s])/total_time)
      for n in range(0, N) for s in range(0, n_states)]
x0_ampl = [0.0, 10, 0.0, 10]
x0 = np.hstack([x0, x0_ampl])

print(len(x0))
print('x0', x0)

# 2) dynamic constraint:
# f = m*a with a = (v_k - v_{k-1})/T
A_dyn = np.zeros([2*N, n_vars_tot])
A_vel = np.zeros([2, 2*n_vars])
A_vel[0:2, 3:5] = -mass*np.eye(2)/T
A_vel[0:2, 3+n_vars:5+n_vars] = mass*np.eye(2)/T

lift_off_front = total_time*1.0/5.0
touch_down_front = 3.0/5.0*total_time
lift_off_hind = total_time*2.0/5.0
touch_down_hind = 4.0/5.0*total_time
ampl = 1.0

for i in range(0, N-1):
    A_force = compute_force_matrix(
        i, lift_off_front, touch_down_front, lift_off_hind, touch_down_hind, total_time)
    print('A force', A_force)
    A_dyn[i*2:i*2+2, n_states*N:n_vars_tot] = A_force
    A_dyn[i*2:i*2+2, i*n_vars:(i+2)*n_vars] = A_vel

print('A dyn', A_dyn)

eps = [1.0e-6, -grav*mass+1.0e-6]*(N)
minus_eps = [-1.0e-6, -grav*mass-1.0e-6]*(N)
dyn_constr = LinearConstraint(A_dyn, lb=minus_eps, ub=eps)

# 3) velocity constraint
A_vel = np.zeros([3*N, n_vars_tot])
A = np.zeros([3, 2*n_vars])
A[0:n_pos, 0:n_pos] = -np.eye(n_pos)/T
A[0:n_pos, n_vars:n_pos+n_vars] = np.eye(n_pos)/T
A[0:n_pos, n_pos+n_vars:n_pos + n_vars + n_vel] = -np.eye(n_pos)

for i in range(0, N-1):
    A_vel[i*n_pos:(i+1)*n_pos, i*n_vars:(i+2)*n_vars] = A

eps = [1.0e-3]*(3*N)
minus_eps = [-1.0e-3]*(3*N)
vel_constr = LinearConstraint(A_vel, lb=minus_eps, ub=eps)

res = minimize(
    minimize_vel,
    x0=x0,
    args=(1.0, N,),
    constraints={vel_constr, dyn_constr},
    bounds=bounds)

print("Solution:", res.x)
pos_x = [res.x[i*n_vars] for i in range(0, N)]
pos_y = [res.x[i*n_vars+1] for i in range(0, N)]
pitch = [res.x[i*n_vars+2] for i in range(0, N)]
vel_x = [res.x[i*n_vars+3] for i in range(0, N)]
vel_y = [res.x[i*n_vars+4] for i in range(0, N)]
pitch_d = [res.x[i*n_vars+5] for i in range(0, N)]
amplitudes = res.x[N*n_states:n_vars_tot]
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
    f_x_front[i] = -A_force[0][0]*res.x[n_states*N]
    f_y_front[i] = -A_force[1][1]*res.x[n_states*N+1]
    f_x_hind[i] = -A_force[0][2]*res.x[n_states*N+2]
    f_x_hind[i] = -A_force[1][3]*res.x[n_states*N+3]

print('force front X', f_x_front)
print('force front Z', f_y_front)
print('force hind X', f_x_hind)
print('force hind Z', f_y_hind)


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
