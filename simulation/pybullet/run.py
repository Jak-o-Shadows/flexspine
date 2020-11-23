

import enum
import time
import atexit
import os
import pprint

import numpy as np

import pybullet as p
import pybullet_data

#
# Initialize robot and sim
#
GRAVITY = -9.81
atexit.register(p.disconnect)
try:
    p.disconnect()
except Exception:
    pass
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.resetDebugVisualizerCamera(cameraDistance=0.6,
                             cameraYaw=20,
                             cameraPitch=-40,
                             cameraTargetPosition=[0, 0, 0.1])
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
p.setRealTimeSimulation(0)
p.setPhysicsEngineParameter(numSubSteps=4)
p.setPhysicsEngineParameter(numSolverIterations=10)
p.setPhysicsEngineParameter(enableFileCaching=0)

p.setGravity(0, 0, GRAVITY)
StartPos = [0, 0, 0.3]
StartOrientation = p.getQuaternionFromEuler([0, 0, 0])
planeId = p.loadURDF("plane.urdf")
botId = p.loadURDF("robot2.urdf", StartPos, StartOrientation)

print("Have %d joints" % p.getNumJoints(botId))

# Get mapping of joint ID to joint name
joints = {}
for jointIdx in range(p.getNumJoints(botId)):
    jointInfo = p.getJointInfo(botId, jointIdx)
    name = jointInfo[1].decode("utf-8")
    joints[name] = jointIdx

pprint.pprint(joints)
    
joint = enum.Enum('joint', list(joints.keys()))

inputOrder = [joint.ForeStarboardServo,
              joint.ForeStarboardHip,
              joint.ForeStarboardKnee,
              joint.ForePortServo,
              joint.ForePortHip,
              joint.ForePortKnee,
              joint.ForePlateGut,
              joint.AftStarboardServo,
              joint.AftStarboardHip,
              joint.AftStarboardKnee,
              joint.AftPortServo,
              joint.AftPortHip,
              joint.AftPortKnee]
inputOrder = {k:i for i, k in enumerate(inputOrder)}

enum2urdf_id = {k:joints[k.name] for k in joint}

# Get mapping of links to names`
links = {}
# Have to loop over joints - seems kinda awkward
#   Also jointIdx = linkIdx
#   See https://github.com/bulletphysics/bullet3/pull/1082
for jointIdx in range(p.getNumJoints(botId)):
    jointInfo = p.getJointInfo(botId, jointIdx)
    name = jointInfo[12].decode("utf-8")
    links[name] = jointIdx
pprint.pprint(links)

link = enum.Enum('link', links)


# Disable the motors for torque control:
p.setJointMotorControlArray(
    botId,
    [j for j in range(p.getNumJoints(botId))],
    p.VELOCITY_CONTROL,
    forces=[0.0 for j in range(p.getNumJoints(botId))]
)


dt_actual = 1. / 240.
last_time = time.time()
start_time = time.time()
step_counter = 0
joint_pos = [0.0 for _ in range(p.getNumJoints(botId))]
last_pos = [0.0 for _ in range(p.getNumJoints(botId))]
joint_vel = [0.0 for _ in range(p.getNumJoints(botId))]
joint_accel = [0.0 for _ in range(p.getNumJoints(botId))]

def GetTimeStep(TargetDT=None, MinDT=1./240.):
    global last_time
    global dt_actual
    global step_counter
    global joint_pos
    global joint_vel
    global joint_accel
    step_counter += 1
    dt_actual = 0.0
    t0 = time.time()
    dt_actual = t0 - last_time

    # enforce a minumum dt limit and/or a user provided time limit
    if TargetDT is not None or dt_actual < MinDT:
        if TargetDT is None:
            TargetDT = MinDT
        remaining_time = TargetDT - dt_actual
        if remaining_time > 0.00001:
            time.sleep(remaining_time)
            t0 = time.time()
            dt_actual = t0 - last_time
    last_time = t0

    p.setPhysicsEngineParameter(fixedTimeStep=dt_actual)
    p.stepSimulation()
    for jnt_indx in range(p.getNumJoints(botId)):
        jointPosition, jointVelocity, _, _ = p.getJointState(
            botId, jnt_indx)
        joint_pos[jnt_indx] = jointPosition
        joint_accel[jnt_indx] = (
            jointVelocity - joint_vel[jnt_indx])/dt_actual
        joint_vel[jnt_indx] = jointVelocity
    elapsed_time = time.time() - start_time
    return dt_actual, elapsed_time

BodyCount = 9
torso_index = -1
state_snapshot = [[{}, 0.1234] for _ in range(BodyCount)]

def _anglevec(u, v): return np.arccos(
    np.array(u).dot(v)/np.linalg.norm(u)/np.linalg.norm(v))

class TipTap():

    @staticmethod
    def OnHardware():
        return False

    @staticmethod
    def GetBodyState(body_indx=-1):
        global step_counter, dt_actual, state_snapshot

        state_indx = body_indx + 1
        if step_counter != state_snapshot[state_indx][1] and dt_actual > 0:
            if body_indx == -1:
                pos_linkcom_world, world_rot_linkcom = \
                    p.getBasePositionAndOrientation(botId)
                linVel_linkcom_world, rotVel_linkcom_world = \
                    p.getBaseVelocity(botId)
            else:
                pos_linkcom_world, world_rot_linkcom, _, _, _, _, \
                    linVel_linkcom_world, rotVel_linkcom_world \
                    = p.getLinkState(botId, body_indx,
                                     computeLinkVelocity=1,
                                     computeForwardKinematics=1,
                                     physicsClientId=0)
            old_vel = state_snapshot[state_indx][0].get(
                "pos_vel", linVel_linkcom_world)
            position_accel = [0, 0, 0]
            position_accel[0] = (
                linVel_linkcom_world[0] - old_vel[0])/dt_actual
            position_accel[1] = (
                linVel_linkcom_world[1] - old_vel[1])/dt_actual
            position_accel[2] = (
                linVel_linkcom_world[2] - old_vel[2])/dt_actual
            mat = np.array(p.getMatrixFromQuaternion(
                world_rot_linkcom)).reshape((3, 3))
            T = np.array([
                [mat[0][0], mat[0][1], mat[0][2], pos_linkcom_world[0]],
                [mat[1][0], mat[1][1], mat[1][2], pos_linkcom_world[1]],
                [mat[2][0], mat[2][1], mat[2][2], pos_linkcom_world[2]],
                [0, 0, 0, 1]
            ])
            state_snapshot[state_indx][0] = {
                "pos": pos_linkcom_world,
                "quat": world_rot_linkcom,
                "mat": mat,
                "T": T,
                "ang": p.getEulerFromQuaternion(world_rot_linkcom),
                "pos_vel": linVel_linkcom_world,
                "ang_vel": rotVel_linkcom_world,
                "accel": position_accel
            }
            state_snapshot[state_indx][1] = step_counter
        return state_snapshot[state_indx][0]


    @staticmethod
    def setPosition(joint_enum, angle, force=30, debug=False):
        global joint_vel, joint_accel
        joint_id = enum2urdf_id[joint_enum]
        p.setJointMotorControl2(
            botId, joint_id, p.POSITION_CONTROL,
            targetPosition=angle, force=force)
        return {"Angle": angle,
                "Vel": joint_vel[joint_id],
                "Accel": joint_accel[joint_id],
                "Code": "OK",
                "ID": np.nan}

    @staticmethod
    def getState(joint_enum):
        global joint_pos, joint_vel, joint_accel
        joint_id = enum2urdf_id[joint_enum]
        return {"Angle": joint_pos[joint_id],
                "Vel": joint_vel[joint_id],
                "Accel": joint_accel[joint_id],
                "Code": "OK",
                "ID": np.nan}

    @staticmethod
    def getIMU():
        base_state = TipTap.GetBodyState(torso_index)
        mat = base_state["mat"]
        u = mat.dot([0, 0, 1])
        pitch = _anglevec(u, [u[0], u[1], 0])*np.sign(u[2])
        u = mat.dot([1, 0, 0])
        roll = _anglevec(u, [u[0], u[1], 0])*np.sign(u[2])
        u = mat.dot([0, 0, -1])
        yaw = np.arctan2(u[1], u[0])
        return {
            "quat": base_state["quat"],
            "yaw": yaw,
            "pitch": pitch,
            "roll": roll,
            "accel": base_state["accel"]}

    @staticmethod
    def setStates(qin):
        states = [TipTap.setPosition(j, q) for j, q in zip(inputOrder.keys(), qin)]
        
        q = [state["Angle"] for state in states]
        dq = [state["Vel"] for state in states]
        ddq = [state["Accel"] for state in states]
        return q, dq, ddq

    @staticmethod
    def getStates():
    
        states = [TipTap.getState(j) for j in inputOrder.keys()]
        
        q = [state["Angle"] for state in states]
        dq = [state["Vel"] for state in states]
        ddq = [state["Accel"] for state in states]
        return q, dq, ddq

    @staticmethod
    def calibrate(FailoverTorqueRatio=0,
                  SearchingTorqueRatio=0.35,
                  FailAtLimitOffsetDeg=15*8,
                  Timeout=15):
        pass

    @staticmethod
    def setZeroAngles():
        pass

    @staticmethod
    def GetTimeStep(TargetDT=None, MinDT=1./240.):
        return GetTimeStep(TargetDT, MinDT)
            
if __name__ == "__main__":
                        
    ControlSignal = [0 for _ in range(6 + 1 + 6)]
    
    
    line_ids = [p.addUserDebugLine([0, 0, 0], [0, 0, 0]) for _ in range(3)]


    # gutAngles = np.radians(np.linspace(-10, 10, 100))
    # gutAngleIdx = 0
    # gutAngleDt = 0.1
    # gutAngleLastTime = 0
    
    
    # Use inverse kinematics from PyBullet to determine the joint coordinates for a leg
    legControlIndices = [inputOrder[joint.ForeStarboardServo],
                         inputOrder[joint.ForeStarboardHip],
                         inputOrder[joint.ForeStarboardKnee]]
    currentJointPositions = [ControlSignal[x] for x in legControlIndices]
    targetLocalPosition = [0, 1, 0]
    jointPositions = p.calculateInverseKinematics(botId, link.fore_starboard_foot.value, targetLocalPosition)
    print(currentJointPositions)
    print(jointPositions)
    for controlSignalIdx, value in zip(legControlIndices, jointPositions):
        ControlSignal[controlSignalIdx] = value



    # main sim/control loop
    while True:

        # get a dictionary of the current IMU states
        imu_data = TipTap.getIMU()

        # feel free to do simulation specific things, using not TipTap.OnHardware()
        bs = TipTap.GetBodyState()  # (only in sim)
        pos = np.array(bs["pos"])
        mat = bs["mat"]
        # draw some debugging lines (+x = red,+y = green,+z = blue)
        for bvi in range(3):
            base_vect = [bvi == 0, bvi == 1, bvi == 2]
            p.addUserDebugLine(pos, pos+mat.dot(base_vect)*0.3,
                               lineColorRGB=base_vect,
                               replaceItemUniqueId=line_ids[bvi])

        # TODO:
        # Put your controller here, which uses "imu_data" and
        # crafts a new "ControlSignal" list

        # apply controller torques and positions, and get the new joint states
        q, dq, ddq = TipTap.setStates(ControlSignal)

        # step the simulation if not on hw, or just gets the elapsed time info
        dt, elapsed_time = TipTap.GetTimeStep()
        
        # if (elapsed_time - gutAngleLastTime) > gutAngleDt:
            # gutAngleLastTime = elapsed_time
            # gutAngleIdx += 1
            # if gutAngleIdx >= len(gutAngles):
                # gutAngleIdx = 0
                
                
            # ControlSignal[6] = gutAngles[gutAngleIdx]
        
    
    
    
    
    