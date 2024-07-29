import pybullet as p
import pybullet_data
import numpy as np
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

robot_urdf_file = "../../Robots/digit-v3/digit-v3-armfixedspecific-floatingbase-springfixed.urdf"
robot_id = p.loadURDF(robot_urdf_file,
                      basePosition=[0, 0, 0],
                      baseOrientation=[0, 0, 0, 1])

trajectory = np.loadtxt("data/trajectory-digit.txt")
trajectory = trajectory.T

for q in trajectory:
    # xyz = q[:3]
    # rpy = q[3:6]
    # quat = p.getQuaternionFromEuler(rpy)
    # p.resetBasePositionAndOrientation(robot_id, xyz, quat)
    
    id = 0
    for joint in range(p.getNumJoints(robot_id)):
        joint_info = p.getJointInfo(robot_id, joint)
        if joint_info[2] != p.JOINT_FIXED:
            p.resetJointState(robot_id, joint, q[id])
            id += 1
    
    p.stepSimulation()
    time.sleep(0.05)

input("Press Enter to exit...")
p.disconnect()