from cvxopt import solvers
import matplotlib.pyplot as plt
import numpy as np
import sys

from cvxopt import matrix

from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters

np.set_printoptions(threshold=sys.maxsize)

N = 40
total_time = 5.0
T = total_time/float(N)
print('T', T)
mass = 45.0
grav = 9.81

n_pos = 3  # x,y, pitch
n_vel = 3
n_states = n_pos + n_vel
n_vars = n_states
n_ampl = 8
n_vars_tot = n_vars*N+n_ampl

max_ampl = 1e8
max_ampl_pos = 0
lower_ampl = [-max_ampl, max_ampl_pos, -max_ampl,
              max_ampl_pos, -max_ampl, max_ampl_pos, -max_ampl, max_ampl_pos]
upper_ampl = [max_ampl]*n_ampl

robot_height = 0.4
z_init = robot_height
z_final = robot_height
x_init = 0.0
x_final = 0.0

base2hip_dist_x = 0.3
front_foot_0 = [x_init+base2hip_dist_x, 0.0]
front_foot_1 = [x_final+base2hip_dist_x, 0.0]

hind_foot_0 = [x_init-base2hip_dist_x, 0.0]
hind_foot_1 = [x_final-base2hip_dist_x, 0.0]


def minimize_vel(n_states, N, n_vars_tot):
    Q = np.eye(n_vars_tot)
    return Q


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
ampl_constr = np.zeros([2*n_ampl, n_vars_tot])
ampl_constr[0:n_ampl, N*n_states:n_vars_tot] = np.eye(n_ampl)
ampl_constr[n_ampl:2*n_ampl, N*n_states:n_vars_tot] = -np.eye(n_ampl)

ampl_b = np.hstack([upper_ampl, -np.array(lower_ampl)])

# print('ampl_constr', ampl_constr)
# print('ampl b', ampl_b)

# equality constraints
bounds_eq_constr = np.zeros([2*n_pos, n_vars_tot])
bounds_eq_constr[0:n_pos, 0:n_pos] = np.eye(n_pos)
bounds_eq_constr[n_pos:2*n_pos, n_states *
                 (N-1):n_states*(N-1)+n_pos] = np.eye(n_pos)
bounds_eq_b = np.hstack([x_init, z_init, 0.0, x_final, z_final, 0.0])

# bounds_eq_constr = np.zeros([2*n_pos, n_vars_tot])
# bounds_eq_constr[3:n_states, 3:n_states] = np.eye(n_pos)
# bounds_eq_constr[n_pos:2*n_pos, n_pos*(N-1):n_pos*N] = np.eye(n_pos)
# bounds_eq_b = np.hstack([x_init, z_init, 0.0, x_final, z_final, 0.0])

# print('bounds_eq_constr', bounds_eq_constr)
# print('bounds_eq_b', bounds_eq_b)


# 2) dynamic constraint:
# f = m*a with a = (v_k - v_{k-1})/T
A_dyn_lin = np.zeros([2*N, n_vars_tot])
A_vel = np.zeros([2, 2*n_vars])
A_vel[0:2, 3:5] = -mass*np.eye(2)/T
A_vel[0:2, 3+n_vars:5+n_vars] = mass*np.eye(2)/T

lift_off_front = total_time*1.0/5.0
touch_down_front = 3.0/5.0*total_time
lift_off_hind = total_time*2.0/5.0
touch_down_hind = 4.0/5.0*total_time

for i in range(0, N-1):
    A_force = compute_force_matrix(
        i, lift_off_front, touch_down_front, lift_off_hind, touch_down_hind, total_time)
    A_dyn_lin[i*2:i*2+2, n_states*N:n_vars_tot] = A_force
    A_dyn_lin[i*2:i*2+2, i*n_vars:(i+2)*n_vars] = A_vel

A_dyn_lin[(N-1)*2:(N-1)*2+2, n_states*N:n_vars_tot] = A_force
A_dyn_lin[(N-1)*2:(N-1)*2+2, (N-1)*n_vars:N*n_vars] = A_vel[:, 0:n_states]

dyn_constr_b = [0.0, -grav*mass]*(N)

# 3) velocity constraint
A_vel = np.zeros([3*(N-1), n_vars_tot])
A = np.zeros([3, 2*n_vars])
A[0:n_pos, 0:n_pos] = -np.eye(n_pos)/T
A[0:n_pos, n_vars:n_pos+n_vars] = np.eye(n_pos)/T
A[0:n_pos, n_pos+n_vars:n_pos + n_vars + n_vel] = -np.eye(n_pos)

for i in range(0, N-1):
    A_vel[i*n_pos:(i+1)*n_pos, i*n_vars:(i+2)*n_vars] = A
# A_vel[(N-1)*n_pos:(N)*n_pos, (N-1)*n_vars:(N)*n_vars] = A[:, 0:n_states]
vel_b = [0.0]*(3*(N-1))

# print('A vel', A_vel)

# CVXOPT: we can use a QP solver here because we do not consider the angular dynamics
n = n_vars_tot
Q = matrix(minimize_vel(n_states, N, n))
p = matrix(np.zeros([n, 1]))

G = matrix(ampl_constr)
q = matrix(ampl_b)

# A_tot = np.vstack([bounds_eq_constr, A_vel])
A_tot = np.vstack([bounds_eq_constr, A_dyn_lin, A_vel])
nc = np.shape(A_tot)[0]
print('constraints number', nc)
print('A_tot', A_tot)
A = matrix(A_tot, (nc, n))
b_tot = np.hstack([bounds_eq_b, dyn_constr_b, vel_b])
# b_tot = np.hstack([bounds_eq_b, vel_b])
b = matrix(b_tot)

sol = solvers.qp(Q, p, G, q, A=A, b=b)

if sol['status'] is 'optimal':
    print("Solution is", sol['status'])
    pos_x = [sol['x'][i*n_vars] for i in range(0, N)]
    # pos_z = [sol['x'][i*n_vars+1]/80+robot_height for i in range(0, N)]
    pos_z = [sol['x'][i*n_vars+1] for i in range(0, N)]
    pitch = [sol['x'][i*n_vars+2] for i in range(0, N)]
    vel_x = [sol['x'][i*n_vars+3] for i in range(0, N)]
    vel_y = [sol['x'][i*n_vars+4] for i in range(0, N)]
    pitch_d = [sol['x'][i*n_vars+5] for i in range(0, N)]
    amplitudes = sol['x'][N*n_states:n_vars_tot]
    print('pos_x', pos_x)
    print('pos_z', pos_z)
    print('pitch', pitch)
    print('vel_x', vel_x)
    print('vel_y', vel_y)
    print('pitch_d', pitch_d)
    print('Amplitudes', amplitudes)

    f_x_front = [0.0]*N
    f_z_front = [0.0]*N
    f_x_hind = [0.0]*N
    f_z_hind = [0.0]*N

    for i in range(0, N):
        [ff0, fh0, ff1, fh1] = compute_force_basis(
            i, lift_off_front, touch_down_front, lift_off_hind, touch_down_hind, total_time)
        f_x_front[i] = ff0*sol['x'][n_states*N] + ff1*sol['x'][n_states*N+4]
        f_z_front[i] = ff0*sol['x'][n_states*N+1] + ff1*sol['x'][n_states*N+5]
        f_x_hind[i] = fh0*sol['x'][n_states*N+2] + fh1*sol['x'][n_states*N+6]
        f_z_hind[i] = fh0*sol['x'][n_states*N+3] + fh1*sol['x'][n_states*N+7]

    print('force front X', f_x_front)
    print('force front Z', f_z_front)
    print('force hind X', f_x_hind)
    print('force hind Z', f_z_hind)

    # check total momentum
    Mx = (sum(f_x_front) + sum(f_x_hind))*T
    print('total momentum along X', Mx)
    f = (sum(f_z_front) + sum(f_z_hind))*T
    g = mass*grav*total_time
    Mz = f - g
    print('total momentum along Z', Mz,
          'Gravity component is', -g, 'Force component is', f)

    # f_z_front = np.array(f_z_front)/f*g
    # f_z_hind = [f_z_hind[s]/f*g for s in range(0, N)]
    # f = (sum(f_z_front) + sum(f_z_hind))*T
    # g = mass*grav*total_time
    # Mz = f - g
    # print('total momentum along Z', Mz,
    #       'Gravity component is', -g, 'Force component is', f)

    fig, axs = plt.subplots(10)
    fig.suptitle('Trajectory')

    time = np.linspace(0, total_time-T, N)
    print('time', time)
    axs[0].plot(time, pos_x)
    axs[0].set_ylabel('pos X [m]')
    axs[1].plot(time, pos_z)
    axs[1].set_ylabel('pos Z [m]')
    axs[2].plot(time, pitch)
    axs[2].set_ylabel('pitch [rad]')
    axs[3].plot(time, vel_x)
    axs[3].set_ylabel('vel X [m/s]')
    axs[4].plot(time, vel_y)
    axs[4].set_ylabel('vel Z [m/s]')
    axs[5].plot(time, pitch_d)
    axs[5].set_ylabel('pitch_dot [rad/s]')
    axs[6].plot(time, f_x_front)
    axs[6].set_ylabel('front f X [N]')
    axs[7].plot(time, f_z_front)
    axs[7].set_ylabel('front f Z [N]')
    axs[8].plot(time, f_x_hind)
    axs[8].set_ylabel('hind f X [N]')
    axs[9].plot(time, f_z_hind)
    axs[9].set_ylabel('front f X [N]')
    axs[9].set_xlabel('time [s]')

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

    # load robot model
    ''' Set the robot's name (current options: 'hyq', 'hyqreal', 'anymal_boxy', 'anymal_coyote' or 'lemo_EP0')'''
    robot = sys.argv[1]
    print('robot name:', robot)

    params = IterativeProjectionParameters(robot)
    params.setDefaultValuesWrtWorld(params.pin)
    in_contact = [True]*2

    tau_hy_front = [0.0]*N
    tau_kn_front = [0.0]*N
    tau_hy_hind = [0.0]*N
    tau_kn_hind = [0.0]*N
    for i in range(0, N):
        params.comPositionWF = [pos_x[i], 0., pos_z[i]]
        params.eurlerAngles = [0.0, 0.0, 0.0]

        t = i*T
        if t < lift_off_front:
            in_contact[0] = True
            lift_off_front
            LF_foot = [front_foot_0[0],  0.092, front_foot_0[1]]
            RF_foot = [front_foot_0[0], -0.092, front_foot_0[1]]
        elif t > touch_down_front:
            in_contact[0] = True
            LF_foot = [front_foot_1[0],  0.092, front_foot_1[1]]
            RF_foot = [front_foot_1[0], -0.092, front_foot_1[1]]
        else:
            in_contact[0] = False
            LF_foot = [0.0]*3
            RF_foot = [0.0]*3

        if t < lift_off_hind:
            in_contact[1] = True
            LH_foot = [hind_foot_0[0],  0.092, hind_foot_0[1]]
            RH_foot = [hind_foot_0[0], -0.092, hind_foot_0[1]]
        elif t > touch_down_hind:
            in_contact[1] = True
            LH_foot = [hind_foot_1[0],  0.092, hind_foot_1[1]]
            RH_foot = [hind_foot_1[0], -0.092, hind_foot_1[1]]
        else:
            in_contact[1] = False
            LH_foot = [0.0]*3
            RH_foot = [0.0]*3

        contactsWF = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
        params.setContactsPosWF(contactsWF)
        params.setActiveContacts(
            [in_contact[0], in_contact[0], in_contact[1], in_contact[1]])

        C, d, isIKoutOfWorkSpace, actuation_polygons, q_pos, knee_pos, hips_pos = params.compDyn.constr.getInequalities(
            params)

        J_LF, J_RF, J_LH, J_RH, isOutOfWorkspace = params.compDyn.kin.get_jacobians()

        if in_contact[0]:
            f_front = [f_x_front[i], 0.0, f_z_front[i]]
            tau_front = np.dot(np.transpose(J_LF), f_front)
        else:
            tau_front = [0.0]*3

        if in_contact[1]:
            f_hind = [f_x_hind[i], 0.0, f_z_hind[i]]
            tau_hind = np.dot(np.transpose(J_LH), f_hind)
        else:
            tau_hind = [0.0]*3

        tau_hy_front[i] = tau_front[1]
        tau_kn_front[i] = tau_front[2]
        tau_hy_hind[i] = tau_hind[1]
        tau_kn_hind[i] = tau_hind[2]

    fig2, axs2 = plt.subplots(4)
    fig2.suptitle('Torques')

    # print('tau_hy_front', tau_hy_front)
    # print('tau_kn_front', tau_kn_front)
    # print('tau_hy_hind', tau_hy_hind)
    # print('tau_kn_hind', tau_kn_hind)

    axs2[0].plot(time, tau_hy_front)
    axs2[0].set_ylabel('front HY [Nm]')
    axs2[1].plot(time, tau_kn_front)
    axs2[1].set_ylabel('front KN [Nm]')
    axs2[2].plot(time, tau_hy_hind)
    axs2[2].set_ylabel('hind HY [Nm]')
    axs2[3].plot(time, tau_kn_hind)
    axs2[3].set_ylabel('hind KN [Nm]')
    axs2[3].set_xlabel('time [s]')

    axs2[0].grid()
    axs2[1].grid()
    axs2[2].grid()
    axs2[3].grid()

    plt.show()
