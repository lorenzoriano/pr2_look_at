import roslib
roslib.load_manifest("pr2_look_at")
import rospy
import openravepy
import numpy as np
import pr2_control_utilities
import tf
from geometry_msgs.msg import PoseStamped

env = openravepy.Environment()
robot = env.ReadRobotXMLFile("robots/pr2-beta-sim.robot.xml");
env.Add(robot)

class WristCameraIK (object):
    def __init__(self):
        if type(self) is WristCameraIK:
            raise ValueError("Cannot instantiate a CameraIK directly!")
        
        self.env = None
        self.robot = None
        self.manip = None
        self.ikmodel =  None
        
        self.state = pr2_control_utilities.RobotState()
        self.controller = pr2_control_utilities.PR2JointMover(
                                                             robot_state=self.state, 
                                                             name="CamerIK Contoller", 
                                                             time_to_reach=1.0)
        
        self.listener = tf.TransformListener()

    def lookat_posestamped(self, pose) :
        assert isinstance(pose, PoseStamped)
        self.listener.waitForTransform("/base_link", pose.header.frame_id,
                                       rospy.Time.now(), rospy.Duration(1))
        target = self.listener.transformPose("/base_link", pose)
        tx = target.pose.position.x
        ty = target.pose.position.y
        tz = target.pose.position.z
        
        return self.lookat_array([tx, ty, tz])
    
    def lookat_array(self, dest_pose):
        """Points the camera towards a 3D position.
        
        dest_pose: an array_like list of 3 coordinates (x,y,z)
        """
        dest_pose = np.array(dest_pose)
        par = openravepy.IkParameterization(dest_pose,
                                            self.ikmodel.iktype)        
        sol = self.manip.FindIKSolution(par,
                                        openravepy.IkFilterOptions.CheckEnvCollisions)
        if sol is None:
            return False
        
        self.execute_sol(sol)
        
        return True

class RightWristCameraIK(WristCameraIK):
    def __init__ (self):
        super(RightWristCameraIK, self).__init__()
        self.env = env
        self.robot = robot
        self.manip = robot.SetActiveManipulator("rightarm_camera")
        ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot,
                                                                                iktype=openravepy.IkParameterization.Type.Lookat3D,
                                                                                freeindices=[27, 28, 30])
        if not ikmodel.load():
            rospy.logwarn("IK model  needs to be created, this might take some time")
            ikmodel.autogenerate()
        
        self.ikmodel = ikmodel
        
    def execute_sol(self, sol):
        
        target_dofs = self.state.right_arm_positions[:]        
        target_dofs[:5] = sol
        target_dofs[6] = 0  #putting the wrist straight
        
        self.controller.set_arm_state(target_dofs, "right_arm", wait=True)
        
        
class LeftWristCameraIK(WristCameraIK):
    def __init__ (self):
        super(LeftWristCameraIK, self).__init__()
        self.env = env
        self.robot = robot
        self.manip = robot.SetActiveManipulator("leftarm_camera")
        ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot,
                                                                                iktype=openravepy.IkParameterization.Type.Lookat3D,
                                                                                freeindices=[15, 16, 18])
        if not ikmodel.load():
            rospy.logwarn("IK model  needs to be created, this might take some time")
            ikmodel.autogenerate()
        
        self.ikmodel = ikmodel
    
    def execute_sol(self, sol):
        target_dofs = self.state.left_arm_positions[:]
        target_dofs[:5] = sol
        target_dofs[6] = 0  #putting the wrist straight
            
        self.controller.set_arm_state(target_dofs, "left_arm", wait=True)    