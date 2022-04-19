
from pybullet_utils import bullet_client as bc
from pybullet_utils import urdfEditor as ed
import pybullet
import pybullet_data
import time
import numpy as np

p0 = bc.BulletClient(connection_mode=pybullet.DIRECT)
p0.setAdditionalSearchPath(pybullet_data.getDataPath())
p1 = bc.BulletClient(connection_mode=pybullet.DIRECT)
p1.setAdditionalSearchPath(pybullet_data.getDataPath())

#can also connect using different modes, GUI, SHARED_MEMORY, TCP, UDP, SHARED_MEMORY_SERVER, GUI_SERVER

ur10_hong = p1.loadURDF("assets/ur5/ur10.urdf")
soft_hybrid_gripper = p0.loadURDF("assets/ur5/soft_hybrid_gripper_URDF.urdf")
ed0 = ed.UrdfEditor()
ed0.initializeFromBulletBody(ur10_hong, p1._client)
ed1 = ed.UrdfEditor()
ed1.initializeFromBulletBody(soft_hybrid_gripper, p0._client)
'''
ur10_kong = p1.loadURDF("assets/ur5/ur10_kong.urdf")
soft_hybrid_gripper_kong = p0.loadURDF("assets/ur5/soft_hybrid_gripper_URDF_kong.urdf")
ed0 = ed.UrdfEditor()
ed0.initializeFromBulletBody(ur10_kong, p1._client)
ed1 = ed.UrdfEditor()
ed1.initializeFromBulletBody(soft_hybrid_gripper_kong, p0._client)
'''
#ed1.saveUrdf("combined.urdf")

parentLinkIndex = 7

jointPivotXYZInParent = [0, 0, 0]
jointPivotRPYInParent = [np.pi/2, 0, np.pi/2]

jointPivotXYZInChild = [0, 0, 0]
jointPivotRPYInChild = [0, 0, 0]

newjoint = ed0.joinUrdf(ed1, parentLinkIndex, jointPivotXYZInParent, jointPivotRPYInParent,
                        jointPivotXYZInChild, jointPivotRPYInChild, p0._client, p1._client)
newjoint.joint_type = p0.JOINT_FIXED

ed0.saveUrdf("assets/ur5/ur10_hong_gripper.urdf")
#ed0.saveUrdf("assets/ur5/ur10_kong_gripper.urdf")
print(p0._client)
print(p1._client)

print("p0.getNumBodies()=", p0.getNumBodies())
print("p1.getNumBodies()=", p1.getNumBodies())

pgui = bc.BulletClient(connection_mode=pybullet.GUI)
pgui.configureDebugVisualizer(pgui.COV_ENABLE_RENDERING, 0)

orn = [0, 0, 0, 1]
ed0.createMultiBody([0, 0, 0], orn, pgui._client)

pgui.setRealTimeSimulation(1)

pgui.configureDebugVisualizer(pgui.COV_ENABLE_RENDERING, 1)

while (pgui.isConnected()):
  pgui.getCameraImage(320, 200, renderer=pgui.ER_BULLET_HARDWARE_OPENGL)
  time.sleep(1. / 240.)
