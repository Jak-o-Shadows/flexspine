import os
import atexit
import enum
import pprint

import numpy as np

import gym
import pybullet as p
import pybullet_data


class MyGym(gym.Env):
    metadata = {"render.modes": ["human"]}
    
    def __init__(self, pathUrdf):
        self.pathUrdf = pathUrdf
        self.start(pathUrdf)
        
    def step(self, action):
    
        # Apply the action
        self.setPositions(action)
    
        truthObs = self._get_truth_observation()
        obs = self._get_observation()
    
        # Add some axis lines
        basePos, baseOrientation = p.getBasePositionAndOrientation(self.botId)
        baseOrientationMatrix = np.array(p.getMatrixFromQuaternion(baseOrientation)).reshape((3, 3))
        for bvi in range(3):
            base_vect = [bvi == 0, bvi == 1, bvi == 2]
            p.addUserDebugLine(basePos, basePos+baseOrientationMatrix.dot(base_vect)*0.3,
                               lineColorRGB=base_vect,
                               replaceItemUniqueId=self.line_ids[bvi])
                               
                               
                               
        # Step the actual simulation
        p.setPhysicsEngineParameter(fixedTimeStep = 1/240)
        p.stepSimulation()
       
       
        # Prepare output
                               
        reward = 0
        episodeOver = False
        extra = {}
                               
                               
                               
                               
    
        return obs, reward, episodeOver, extra
        
    def reset(self):
        self.close()
        self.start(self.pathUrdf)
        self.mapJoints()
        self.mapLinks()
        
        # Define the mapping between the 'action' vector, and the joints
        self.actionMapping = [self.JOINT.ForeStarboardServo,
                              self.JOINT.ForeStarboardHip,
                              self.JOINT.ForeStarboardKnee,
                              self.JOINT.ForePortServo,
                              self.JOINT.ForePortHip,
                              self.JOINT.ForePortKnee,
                              self.JOINT.ForePlateGut,
                              self.JOINT.AftStarboardServo,
                              self.JOINT.AftStarboardHip,
                              self.JOINT.AftStarboardKnee,
                              self.JOINT.AftPortServo,
                              self.JOINT.AftPortHip,
                              self.JOINT.AftPortKnee]
        self.actionMapping = {k:i for i, k in enumerate(self.actionMapping)}

                               
        defaultValues = [0]*len(self.actionMapping)
        
        self.setPositions(defaultValues)
        
    
        return self._get_observation()
        
    def render(self, mode="human"):
        pass
        
    def close(self):
        pass
       
        
     # More Helper Functions
    def start(self, pathUrdf):
        # Setup the Pybullet system
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
        self.botId = p.loadURDF(pathUrdf, StartPos, StartOrientation)
        
        # Debug lines
        #   Must store the Id's so they can be replaced
        self.line_ids = [p.addUserDebugLine([0, 0, 0], [0, 0, 0]) for _ in range(3)]
        
    # Generic Robot Things
    def mapJoints(self):
        # Get mapping of joint ID to joint name
        joints = {}
        for jointIdx in range(p.getNumJoints(self.botId)):
            jointInfo = p.getJointInfo(self.botId, jointIdx)
            name = jointInfo[1].decode("utf-8")
            joints[name] = jointIdx
            
        self.JOINT = enum.Enum('joint', list(joints.keys()))

    def mapLinks(self):
        # Get mapping of links to names`
        links = {}
        # Have to loop over joints - seems kinda awkward
        #   Also jointIdx = linkIdx
        #   See https://github.com/bulletphysics/bullet3/pull/1082
        for jointIdx in range(p.getNumJoints(self.botId)):
            jointInfo = p.getJointInfo(self.botId, jointIdx)
            name = jointInfo[12].decode("utf-8")
            links[name] = jointIdx
        pprint.pprint(links)

        self.LINK = enum.Enum('link', links)
        
    def setPositions(self, action):
       
        def _setPosition(joint, value):
            p.setJointMotorControl2(self.botId,
                                    joint.value,
                                    p.POSITION_CONTROL,
                                    targetPosition=value,
                                    force=30)
        [_setPosition(j, v) for j, v in zip(self.actionMapping.keys(), action)]

    # Robot Specific Things
    def _get_truth_observation(self):
        return None
        
    def _get_observation(self):
        return self._get_truth_observation()
        