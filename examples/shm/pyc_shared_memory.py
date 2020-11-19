import sysv_ipc as ipc
import ctypes
import numpy as np
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
def main():
    path = "/tmp"
    key = ipc.ftok(path, 2336)
    shm = ipc.SharedMemory(key, flags=0, size=ctypes.sizeof(ctypes.c_double) * 12)
    write_shm = ipc.SharedMemory(key, flags = 0, size = ctypes.sizeof(ctypes.c_double) * 12)
    new_values = [0.0]*12
    print "new values", new_values
    c_buffer = (ctypes.c_double * len(new_values))(*new_values)
    write_shm.write(c_buffer)

    robot ="anymal_boxy"
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
            params.setDefaultValues()
            isPointFeasible, margin = comp_dyn.compute_IP_margin(params, "COM")
            print "margin: ", margin
            computation_completed = 1
            new_values = [cycle_number_cpp, cycle_number_py, 0, computation_completed, margin, 5, 6, 7, 8, 9, 10, 11]
            c_buffer = (ctypes.c_double * len(new_values))(*new_values)
            write_shm.write(c_buffer)
            print "computation_completed: ", computation_completed
            print "cycle_number_py: ", cycle_number_py
            cycle_number_py = cycle_number_py + 1

    shm.detach()

if __name__ == '__main__':
    main()