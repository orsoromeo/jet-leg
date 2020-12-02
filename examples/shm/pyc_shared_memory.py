import sysv_ipc as ipc
import ctypes
import numpy as np
import time
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics

'''
Addresses:

0: cycle_number_cpp
1: cycle_number_py
2: requested_new_value
3: computation_completed
4: margin

'''


def setValues(values, params):

    constraint_mode_IP = ['FRICTION_AND_ACTUATION',
                          'FRICTION_AND_ACTUATION',
                          'FRICTION_AND_ACTUATION',
                          'FRICTION_AND_ACTUATION']

    comWF = np.array([0.0, 0.0, 0.0])
    comWF_lin_acc = np.array([.0, .0, .0])
    comWF_ang_acc = np.array([.0, .0, .0])

    ''' extForceW is an optional external pure force (no external torque for now) applied on the CoM of the robot.'''
    extForce = np.array([0., .0, 0.0 * 9.81])  # units are N
    extCentroidalTorque = np.array([.0, .0, .0])  # units are Nm
    extCentroidalWrench = np.hstack([extForce, extCentroidalTorque])

    ''' parameters to be tuned'''
    mu = 0.5

    ''' stanceFeet vector contains 1 is the foot is on the ground and 0 if it is in the air'''
    stanceFeet = [values[37], values[38], values[39], values[40]]

    print "stance feet ", stanceFeet

    ''' now I define the normals to the surface of the contact points. By default they are all vertical now'''
    axisZ = np.array([[0.0], [0.0], [1.0]])

    n1 = np.transpose(np.transpose(params.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # LF
    n2 = np.transpose(np.transpose(params.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # RF
    n3 = np.transpose(np.transpose(params.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # LH
    n4 = np.transpose(np.transpose(params.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # RH
    normals = np.vstack([n1, n2, n3, n4])

    ''' extForceW is an optional external pure force (no external torque for now) applied on the CoM of the robot.'''
    extForceW = np.array([0.0, 0.0, 0.0])  # units are Nm

    '''You now need to fill the 'params' object with all the relevant 
        informations needed for the computation of the IP'''

    """ contact points in the World Frame"""
    LF_foot = values[24:27]
    RF_foot = values[27:30]
    LH_foot = values[30:33]
    RH_foot = values[33:36]

    contactsWF = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
    print 'contacts WF ', contactsWF
    
    params.setContactsPosWF(contactsWF)

    params.useContactTorque = True
    params.useInstantaneousCapturePoint = True
    params.externalCentroidalWrench = extCentroidalWrench
    params.setCoMPosWF(comWF)
    params.comLinVel = [0., 0.0, 0.0]
    params.setCoMLinAcc(comWF_lin_acc)
    params.setTorqueLims(params.compDyn.robotModel.robotModel.joint_torque_limits)
    params.setActiveContacts(stanceFeet)
    params.setConstraintModes(constraint_mode_IP)
    params.setContactNormals(normals)
    params.setFrictionCoefficient(mu)
    params.setTotalMass(params.compDyn.robotModel.robotModel.trunkMass)
    params.externalForceWF = extForceW  # params.externalForceWF is actually used anywhere at the moment

def main():
    path = "/tmp"
    key = ipc.ftok(path, 2337)
    buffer_size = 53
    shm = ipc.SharedMemory(key, flags=0, size=ctypes.sizeof(ctypes.c_double) * buffer_size)
    write_shm = ipc.SharedMemory(key, flags = 0, size = ctypes.sizeof(ctypes.c_double) * buffer_size)
    new_values = [0.0]*buffer_size
    print "new values", new_values
    c_buffer = (ctypes.c_double * len(new_values))(*new_values)
    write_shm.write(c_buffer)

    robot ="anymal_coyote"
    params = IterativeProjectionParameters(robot)
    comp_dyn = ComputationalDynamics(robot)
    shm.attach(long(0), 0)
    cycle_number_py = 0
    while True:
        values = np.frombuffer(shm.read())
        requested_new_value = values[2]
        cycle_number_cpp = values[0]
        #print "cycle n. CPP: ", cycle_number_cpp, " cycle n. Python: ", cycle_number_py, "requested new computation ", requested_new_value, " computation completed: ", values[3]
        if cycle_number_cpp == 0 and cycle_number_py == 0:
            previous_cycle_was_received = True
        elif cycle_number_cpp == cycle_number_py - 1:
            previous_cycle_was_received = True
        else:
            previous_cycle_was_received = False
        if requested_new_value and previous_cycle_was_received:
            setValues(values, params)
            isPointFeasible, margin = comp_dyn.compute_IP_margin(params, "COM")
            print "margin: ", margin
            computation_completed = 1
            new_values = [cycle_number_cpp, cycle_number_py, 0, computation_completed, margin, 5, 6, 7, 8, 9, 10, 11]
            c_buffer = (ctypes.c_double * len(new_values))(*new_values)
            write_shm.write(c_buffer)
            print "cycle_number_py: ", cycle_number_py
            cycle_number_py = cycle_number_py + 1

    shm.detach()

if __name__ == '__main__':
    main()