from os.path import join
import pinocchio as se3
#from mobilerobot import MobileRobotWrapper
from pinocchio.utils import *

from pinocchio.robot_wrapper import RobotWrapper
PKG = '/opt/openrobots/share/example-robot-data'
URDF = '/opt/openrobots/share/example-robot-data/ur_description/urdf/ur5_robot.urdf'
robot = RobotWrapper.BuildFromURDF(URDF, [PKG])

#PKG = '/opt/openrobots/share'
#URDF = join(PKG, 'ur5_description/urdf/ur5_gripper.urdf')
#robot = MobileRobotWrapper(URDF, [PKG])
#robot.initDisplay(loadModel=True)
# robot.viewer.gui.addFloor('world/floor')
NQ, NV = robot.model.nq, robot.model.nv
q = rand(NQ)
print (q)
#robot.display(q)
IDX_TOOL = 22
IDX_BASIS = 21
JOINT_ID = 6
#se3.framesKinematics(robot.model, robot.data)
print (len(robot.data.oMf))
Mtool = robot.data.oMf[IDX_TOOL]
Mbasis = robot.data.oMf[IDX_BASIS]
print ('finished')
print (Mtool)
print (Mbasis)
J = se3.jointJacobian(robot.model, robot.data, q, JOINT_ID)
print (J)