import numpy as np
import sys
from scipy.optimize import minimize, LinearConstraint, NonlinearConstraint
import matplotlib.pyplot as plt
np.set_printoptions(threshold=sys.maxsize)

N = 20
total_time = 2.0
T = total_time/float(N)
mass = 30.0
grav = 9.81

n_pos = 3  # x,y, pitch
n_vel = 3
n_states = n_pos + n_vel
n_forces = 4
n_vars = n_states + n_forces
n_vars_tot = n_vars*N

lower_states = [-100.0]*n_states
lower_forces = [-2000.0, 0.0, -2000.0, 0.0]
lower = np.hstack([lower_states, lower_forces])

upper_states = [100.0]*n_states
upper_forces = [2000.0]*n_forces
upper = np.hstack([upper_states, upper_forces])

tol = 1e-6
initial_state = [0.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, mass/2.0, 0.0, mass/2.0]
final_state = [2.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, mass/2.0, 0.0, mass/2.0]
bounds_init = [(initial_state[s]-tol, initial_state[s]+tol)
               for s in range(0, n_vars)]
bounds = [(lower[s], upper[s]) for i in range(1, N-2)
          for s in range(0, n_vars)]
before_final_state_lb = np.hstack([[final_state[0]-tol, final_state[1]-tol, final_state[2]-tol],
                                   [-100.0]*3, [-2000, 0.0, -2000.0, 0.0]])
before_final_state_ub = np.hstack([[final_state[0]+tol, final_state[1]+tol, final_state[2]+tol], [
    100.0]*3, [2000, 2000.0, 2000.0, 2000.0]])
print('before_final_state_ub', before_final_state_ub)
bounds_before_final = [(before_final_state_lb[s], before_final_state_ub[s])
                       for s in range(0, n_vars)]
bounds_final = [(final_state[s] - tol, final_state[s]+tol)
                for s in range(0, n_vars)]
bounds = np.vstack([bounds_init, bounds, bounds_before_final, bounds_final])


def objective_function(x):
    return np.sum(x)


def minimize_vel(x, w, N):
    cost = 0.0
    for i in range(0, N):
        cost += np.sum(x[i*10+3:i*10+6])
    return cost*w


def quadratic_bezier_curve(t, p0, p1, p2):
    return (1-t)*[(1-t)*p0 + t*p1] + t*[(1-t)*p1+t*p2]


def bezier_curve(amplitude, time, td_time, lo_time, td_location, lo_location):
    p1_x = (td_location[0]+lo_location[0])/2.0
    p1_y = amplitude
    t = (time - td_time)/(lo_time-td_time)
    x = quadratic_bezier_curve(
        t, td_location[0], p1_x, lo_location[0])
    y = quadratic_bezier_curve(
        t, td_location[1], p1_y, lo_location[1])
    return [x, y]


# Linear constraints: lb <= A.dot(x) <= ub

x0 = [0.0]*(n_vars_tot)
x0[0:n_vars] = initial_state
x0[n_vars_tot-n_vars:n_vars_tot] = final_state

# 2) dynamic constraint:
# f = m*a  with a = (v_k - v_{k-1})/T
A_dyn = np.zeros([2*N, n_vars_tot])
A = np.zeros([2, 2*n_vars])
A[0:2, 3:5] = -np.eye(2)/T*mass
A[0:2, 6+n_vars:10+n_vars] = [[-1.0, 0.0, -1.0, 0.0],
                              [0.0, -1.0, 0.0, -1.0]]  # forces
A[0:2, 3+n_vars:5+n_vars] = np.eye(2)/T*mass
for i in range(0, N-1):
    A_dyn[i*2:i*2+2, i*n_vars:(i+2)*n_vars] = A

eps = [1.0e-6, -grav+1.0e-6]*(N)
minus_eps = [-1.0e-6, -grav-1.0e-6]*(N)
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

# 4) swing constraint

lift_off_front = total_time*1.0/5.0
touch_down_front = 3.0/5.0*total_time
lift_off_hind = total_time*2.0/5.0
touch_down_hind = 4.0/5.0*total_time
timings = [[lift_off_front, touch_down_front],
           [lift_off_hind, touch_down_hind]]

A_swing = np.zeros([0, n_vars_tot])
print('timngs', timings[0])
for i in range(0, N):
    t = T*i
    for leg in range(0, 2):
        print('leg', leg)
        if (t >= timings[leg][0]) and (t <= timings[leg][1]):
            A_new = np.zeros([2, n_vars_tot])
            A_new[0, i*n_vars+n_states+2*leg] = 1.0
            A_new[1, i*n_vars+n_states+2*leg+1] = 1.0
            A_swing = np.vstack([A_swing, A_new])
print('A_swing', np.shape(A_swing))
print('A_swing', A_swing)
n_swing_constr = np.shape(A_swing)[0]
eps = [1.0e-6]*(n_swing_constr)
minus_eps = [-1.0e-3]*(n_swing_constr)
swing_constr = LinearConstraint(A_swing, lb=minus_eps, ub=eps)

res = minimize(
    minimize_vel,
    x0=x0,
    args=(1.0, N,),
    constraints={vel_constr, dyn_constr, swing_constr},
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
